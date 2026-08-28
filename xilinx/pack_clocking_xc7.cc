/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019  David Shah <david@symbioticeda.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <algorithm>
#include <cmath>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>
#include <iterator>
#include <queue>
#include <unordered_set>
#include "cells.h"
#include "chain_utils.h"
#include "design_utils.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"
#include "pins.h"

NEXTPNR_NAMESPACE_BEGIN

void XC7Packer::prepare_clocking()
{
    log_info("Preparing clocking...\n");
    std::unordered_map<IdString, IdString> upgrade;
    upgrade[ctx->id("MMCME2_BASE")] = ctx->id("MMCME2_ADV");
    upgrade[ctx->id("PLLE2_BASE")] = ctx->id("PLLE2_ADV");

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        // Whenever this loop rewrites ci->type, preserve the original under
        // X_ORIG_TYPE so downstream consumers (json2dcp / fasm.cc) know what
        // primitive name to re-emit.  Without this, a packed BUFG looks like a
        // BUFGCTRL with no provenance, and RapidWright can't reconstruct the
        // BUFG EDIF cell — it gets silently dropped, leaving clk driverless.
        auto save_orig_type = [&]() {
            if (!ci->attrs.count(ctx->id("X_ORIG_TYPE")))
                ci->attrs[ctx->id("X_ORIG_TYPE")] = ci->type.str(ctx);
        };
        // Tag a port rename so downstream consumers (json2dcp / fasm.cc)
        // can reconstruct the source-RTL port name.  The general xform_cell
        // path in pack.cc:70 does this automatically; this lambda mirrors
        // that behaviour for the rename_port() calls in this loop that
        // bypass xform_cell.  Without these tags the json2dcp EDIF leaves
        // the packed cell's pins floating (e.g. clk_raw had no sink because
        // BUFG.I → BUFGCTRL.I0 had no X_ORIG_PORT_I0 mapping).
        auto save_orig_port = [&](const std::string &new_name, const std::string &orig_name) {
            ci->attrs[ctx->id("X_ORIG_PORT_" + new_name)] = orig_name;
        };
        if (upgrade.count(ci->type)) {
            save_orig_type();
            IdString new_type = upgrade.at(ci->type);
            ci->type = new_type;
        } else if (ci->type == ctx->id("BUFG")) {
            save_orig_type();
            ci->type = ctx->id("BUFGCTRL");
            rename_port(ctx, ci, ctx->id("I"), ctx->id("I0"));
            save_orig_port("I0", "I");
            save_orig_port("O",  "O");
            tie_port(ci, "CE0", true, true);
            tie_port(ci, "S0", true, true);
            tie_port(ci, "S1", false, true);
            tie_port(ci, "IGNORE0", true, true);
        } else if (ci->type == ctx->id("BUFGCE")) {
            save_orig_type();
            ci->type = ctx->id("BUFGCTRL");
            rename_port(ctx, ci, ctx->id("I"), ctx->id("I0"));
            rename_port(ctx, ci, ctx->id("CE"), ctx->id("CE0"));
            save_orig_port("I0",  "I");
            save_orig_port("CE0", "CE");
            save_orig_port("O",   "O");
            tie_port(ci, "S0", true, true);
            tie_port(ci, "S1", false, true);
            tie_port(ci, "IGNORE0", true, true);
        } else if (ci->type == ctx->id("BUFGMUX") || ci->type == ctx->id("BUFGMUX_CTRL")) {
            save_orig_type();
            // I0/I1 unchanged; S → S1 direct and S → S0 inverted (one
            // source feeding two ports).
            save_orig_port("I0", "I0");
            save_orig_port("I1", "I1");
            save_orig_port("S1", "S");
            save_orig_port("S0", "S");
            save_orig_port("O",  "O");
            // I same, S->S1 direct, S->S0 inverted, CE to VCC, IGNORE to GND
            ci->type = ctx->id("BUFGCTRL");
            rename_port(ctx, ci, ctx->id("S"), ctx->id("S1"));
            NetInfo *s_net = ci->ports.at(ctx->id("S1")).net;
            if (s_net != nullptr) {
                ci->ports[ctx->id("S0")].name = ctx->id("S0");
                ci->ports[ctx->id("S0")].type = PORT_IN;
                connect_port(ctx, s_net, ci, ctx->id("S0"));
                ci->params[ctx->id("IS_S0_INVERTED")] = Property(1);
            }
            tie_port(ci, "CE0", true, true);
            tie_port(ci, "CE1", true, true);
            tie_port(ci, "IGNORE0", false, false);
            tie_port(ci, "IGNORE1", false, false);
        } else if (ci->type == id_BUFH || ci->type == id_BUFHCE) {
            ci->type = id_BUFHCE_BUFHCE;
            tie_port(ci, "CE", true, true);
        } else if (ci->type == id_BUFR) {
            // BUFR site holds a single BEL of type BUFR_BUFR (per the
            // virtex7 meta tree, ported from artix7).  Rename the cell
            // type to match the BEL.  No port renames needed (BUFR pin
            // names are identical: I/CE/CLR/O), but json2dcp drops any
            // connection that lacks an X_ORIG_PORT_<name> attribute,
            // so emit identity mappings for the four ports.
            save_orig_type();
            ci->type = id_BUFR_BUFR;
            save_orig_port("I",   "I");
            save_orig_port("CE",  "CE");
            save_orig_port("CLR", "CLR");
            save_orig_port("O",   "O");
        } else if (ci->type == ctx->id("BUFIO")) {
            // BUFIO is the undivided I/O clock buffer and sits in its own
            // site, one BEL of type BUFIO_BUFIO carrying just two pins --
            // I and O.  No CE and no CLR, which is the difference from the
            // BUFR case above; there is nothing to tie off.  Same
            // rename-to-match-the-BEL treatment and the same identity port
            // tags, for the json2dcp reason given there.
            //
            // Without this branch the cell reached the placer still typed
            // BUFIO, no bel of that type exists to bind it to, and the run
            // died with "no Bels remaining of type 'BUFIO'" while twenty
            // sat unused -- the resource table counts bels by site type and
            // so listed them as available the whole time.
            save_orig_type();
            ci->type = id_BUFIO_BUFIO;
            save_orig_port("I", "I");
            save_orig_port("O", "O");
        }
    }
}

void XC7Packer::pack_plls()
{
    log_info("Packing PLLs...\n");

    auto set_default = [](CellInfo *ci, IdString param, const Property &value) {
        if (!ci->params.count(param))
            ci->params[param] = value;
    };

    std::unordered_map<IdString, XFormRule> pll_rules;
    pll_rules[ctx->id("MMCME2_ADV")].new_type = ctx->id("MMCME2_ADV_MMCME2_ADV");
    pll_rules[ctx->id("PLLE2_ADV")].new_type = ctx->id("PLLE2_ADV_PLLE2_ADV");
    generic_xform(pll_rules);
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        // Preplace PLLs to make use of dedicated/short routing paths
        if (ci->type == ctx->id("MMCME2_ADV_MMCME2_ADV") || ci->type == ctx->id("PLLE2_ADV_PLLE2_ADV"))
            try_preplace(ci, ctx->id("CLKIN1"));
        if (ci->type == ctx->id("MMCME2_ADV_MMCME2_ADV")) {
            // Fixup parameters
            for (int i = 1; i <= 2; i++)
                set_default(ci, ctx->id("CLKIN" + std::to_string(i) + "_PERIOD"), Property("0.0"));
            for (int i = 0; i <= 6; i++) {
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_CASCADE"), Property("FALSE"));
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_DIVIDE"), Property(1));
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_DUTY_CYCLE"), Property("0.5"));
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_PHASE"), Property(0));
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_USE_FINE_PS"), Property("FALSE"));
            }
            set_default(ci, ctx->id("COMPENSATION"), Property("INTERNAL"));

            // Fixup routing.  An MMCM with a REAL CLKFBOUT->CLKFBIN loop in
            // the netlist must keep it: golden bitstreams realise INTERNAL
            // compensation via the dedicated CLKFBOUT2IN pip (same-tile,
            // always routable), and a VCC-tied CLKFBIN never locks on
            // hardware.  The old guard kept the loop only for BEL-pinned
            // (imported-placement) MMCMs; a FREE MMCM (SVS-placed flow) got
            // its feedback tied to VCC -> never locks -> totally dead design
            // (VC707 eth-arp: no LEDs until the fasm was hand-patched back
            // to CLKFBOUT2IN).  Tie only when CLKFBIN is genuinely undriven.
            if (str_or_default(ci->params, ctx->id("COMPENSATION"), "INTERNAL") == "INTERNAL" &&
                !ci->attrs.count(ctx->id("BEL"))) {
                NetInfo *fb = get_net_or_empty(ci, ctx->id("CLKFBIN"));
                bool has_real_fb = fb != nullptr && fb->driver.cell != nullptr;
                if (!has_real_fb) {
                    disconnect_port(ctx, ci, ctx->id("CLKFBIN"));
                    connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, ctx->id("CLKFBIN"));
                }
            }
        }
    }

    // Vivado leaves the PLLE2 DRP / static-control pins UNROUTED (the DRP port defaults
    // to inactive); the open flow instead routes each const-tied pin from a fabric IMUX,
    // and DCLK lands on a clock net -- a spuriously-clocked DRP can corrupt the PLL's
    // config at runtime, yielding a clock clean enough for a free counter but too
    // jittery for synchronous logic (open-flow PLL designs were silent on HW while
    // Vivado's worked).  Disconnect the const-tied DRP/control pins to match Vivado.
    // RST / CLKIN* / CLKFB* are real signals and are left intact.  Runs after the final
    // pack_constants, so the ties are not reapplied.
    {
        IdString gnd = ctx->id("$PACKER_GND_NET"), vcc = ctx->id("$PACKER_VCC_NET");
        std::vector<std::string> drp = {"DEN", "DWE", "DCLK", "PWRDWN", "CLKINSEL",
                                        "DADDR0", "DADDR1", "DADDR2", "DADDR3", "DADDR4", "DADDR5", "DADDR6"};
        for (int i = 0; i < 16; i++)
            drp.push_back("DI" + std::to_string(i));
        // MMCME2 additionally has the phase-shift port; a fabric-routed
        // PSCLK/PSEN with random IMUX levels walks the output phase away.
        std::vector<std::string> mmcm_extra = {"PSCLK", "PSEN", "PSINCDEC", "RST"};
        int n = 0;
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            bool is_pll = ci->type == ctx->id("PLLE2_ADV_PLLE2_ADV");
            bool is_mmcm = ci->type == ctx->id("MMCME2_ADV_MMCME2_ADV");
            if (!is_pll && !is_mmcm)
                continue;
            for (auto &p : drp) {
                NetInfo *nn = get_net_or_empty(ci, ctx->id(p));
                if (nn != nullptr && (nn->name == gnd || nn->name == vcc)) {
                    disconnect_port(ctx, ci, ctx->id(p));
                    ++n;
                }
            }
            if (is_mmcm) {
                for (auto &p : mmcm_extra) {
                    NetInfo *nn = get_net_or_empty(ci, ctx->id(p));
                    if (nn != nullptr && (nn->name == gnd || nn->name == vcc)) {
                        disconnect_port(ctx, ci, ctx->id(p));
                        ++n;
                    }
                }
            }
        }
        if (n)
            log_info("    PLL/MMCM: disconnected %d const DRP/control pins (Vivado leaves them unrouted)\n", n);
    }
}

void XC7Packer::pack_gbs()
{
    log_info("Packing global buffers...\n");

    // Make sure prerequisites are set up first
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("PS7_PS7"))
            preplace_unique(ci);
        if (ci->type == ctx->id("PCIE_2_1_PCIE_2_1"))
            preplace_unique(ci);
    }

    // Preplace global buffers to make use of dedicated/short routing
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == id_BUFGCTRL)
            try_preplace(ci, id_I0);
        if (ci->type == id_BUFG_BUFG)
            try_preplace(ci, id_I);
        if (ci->type == id_BUFHCE_BUFHCE)
            try_preplace(ci, id_I);
        // Fallback for a global buffer driven from FABRIC rather than an I/O pin
        // (e.g. a divided/gated clock generated by a FF — a management MDC clock,
        // a clock enable divider, etc.). Such a net has no dedicated input route
        // for try_preplace() to follow, so it stays unplaced and the placer later
        // aborts with "Unable to find legal placement for cell". Bind it to any
        // free buffer BEL of its type; its fabric input reaches the buffer through
        // the BUFGCTRL input mux, exactly as Vivado routes it.
        //
        // PLL/MMCM-driven BUFGs get an extra rule on top of that fallback: the
        // chipdb's SOLVED dedicated PLL->BUFG exits are the BOTTOM-region ones
        // (HCLK_CMT MUX_CLK_PLL* -> CK_IN pips in the lower HCLK_CMT tile).  The
        // top-region numbered muxes (MUX_CLK_<m> -> CK_IN) have no prjxray
        // segbits except three GT-validated combos (see setup_pip_blacklist in
        // arch.cc), so a TOP-region BUFG can only ever receive 3 PLL clocks and
        // the 4th+ starves -- litex-ddr-arty-s7: "Failed to route arc 0 of net
        // 'main_crg_clkout4'" (PLLE2_ADV_X1Y0/CLKOUT4 -> BUFGCTRL_X0Y19/I0).
        // preplace_unique() walks getBels() in chipdb order and lands these on
        // TOP-region sites first, so pin PLL/MMCM-driven buffers to the lowest
        // free BOTTOM-region site (CLK_BUFG_BOT_R), whose dedicated exits are
        // fully solved (matches the working golden path via CLK_HROW_BOT_R +
        // CK_BUFG_CASCO into the bottom BUFG tile).
        if ((ci->type == id_BUFGCTRL || ci->type == id_BUFG_BUFG || ci->type == id_BUFHCE_BUFHCE) &&
            !ci->attrs.count(ctx->id("BEL"))) {
            bool pll_driven = false;
            if (ci->type == id_BUFGCTRL) {
                NetInfo *n = get_net_or_empty(ci, id_I0);
                if (n != nullptr && n->driver.cell != nullptr) {
                    IdString dt = n->driver.cell->type;
                    pll_driven = dt == id_PLLE2_ADV_PLLE2_ADV || dt == id_MMCME2_ADV_MMCME2_ADV;
                }
            }
            if (pll_driven) {
                for (auto bel : ctx->getBels()) {
                    if (ctx->getBelType(bel) != id_BUFGCTRL || used_bels.count(bel))
                        continue;
                    if (ctx->getBelTileType(bel) != ctx->id("CLK_BUFG_BOT_R"))
                        continue;
                    if (!ctx->checkBelAvail(bel))
                        continue;
                    used_bels.insert(bel);
                    ci->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(bel));
                    log_info("    Constrained BUFGCTRL '%s' to bel '%s' (PLL-driven, bottom region)\n", ctx->nameOf(ci),
                             ctx->nameOfBel(bel));
                    break;
                }
            }
            if (!ci->attrs.count(ctx->id("BEL")))
                preplace_unique(ci);
        }
    }

    // pack_gbs() runs in the XC7 pack() sequence AFTER the final pack_constants, so
    // disconnect const-tied BUFGCTRL control pins here (doing it earlier just gets
    // undone when pack_constants re-applies the get_tied_pins/invertible_pins ties).
    // CE0/CE1/S0/S1/IGNORE0/IGNORE1 are realised by the BUFGCTRL config bits
    // (ZINV_CE0/ZINV_S0/IS_IGNORE1_INVERTED, emitted by fasm.cc when the BUFG's I0/I1
    // output pip is used) and must NOT also be routed: ZINV_CE0 inverts CE0, so a
    // routed CE0=VCC becomes CE=0 and DISABLES the buffer -> no clock on HW (this is
    // why the open-flow USER_CLOCK build was silent).  A real control signal (e.g. a
    // BUFGMUX select) is on a non-const net and is left untouched.
    if (getenv("NEXTPNR_BUFG_CONST_DISCONNECT")) {
        IdString gnd = ctx->id("$PACKER_GND_NET"), vcc = ctx->id("$PACKER_VCC_NET");
        int n = 0;
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type != id_BUFGCTRL)
                continue;
            for (auto p : {"CE0", "CE1", "S0", "S1", "IGNORE0", "IGNORE1"}) {
                NetInfo *nn = get_net_or_empty(ci, ctx->id(p));
                if (nn != nullptr && (nn->name == gnd || nn->name == vcc)) {
                    disconnect_port(ctx, ci, ctx->id(p));
                    ++n;
                }
            }
        }
        log_info("    BUFGCTRL: disconnected %d const control pins (config-tied, not routed)\n", n);
    }

    // A BUFIO is a regional I/O clock buffer rather than a global one, but this
    // is where the clock buffers get their bels and it has to run after
    // pack_io() has placed the pads it reads.
    constrain_bufios();
}

// A BUFIO is not placed wherever there is room: it is placed where the pad
// says.  Its I pin has no fabric input at all -- the only wire that reaches it
// is the I2IOCLK leg its own clock-capable pad drives into the HCLK_IOI tile --
// so of the four BUFIO_BUFIO bels of a tile exactly ONE is reachable from a
// given pad.  Sweeping the four master clock-capable pads of bank 35 of an
// xc7a35tcsg324-1 against the four BUFIO sites of their tile gives a diagonal:
// E2 routes only on BUFIO_X1Y4, F4 only on X1Y5, E3 only on X1Y6, D5 only on
// X1Y7 -- the CCIO->BUFIO bijection prjxray's 039-hclk-config campaign measured
// on silicon bitstreams, here read back out of our own routing graph.
//
// Nothing told the placer that.  #157 packs the cell onto a BUFIO_BUFIO bel and
// the placer takes any free one, so a pad-fed BUFIO dies in the router:
//
//   ERROR: Failed to route arc 0 of net 'clk_ibuf',
//          from SITEWIRE/IOB_X1Y76/INBUF_EN_OUT to SITEWIRE/BUFIO_X1Y5/I.
//
// (0 of those 4 probes routed; the placer picked X1Y5 for three of the pads.)
//
// So ask the routing graph which bel the pad reaches -- the same pip BFS
// try_preplace() runs for a BUFG -- and constrain the cell to it.  A table of
// pad->site pairs would answer the same question for artix7 today and be wrong
// for the next family; the graph is per-part data and already knows.
void XC7Packer::constrain_bufios()
{
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type != id_BUFIO_BUFIO)
            continue;
        NetInfo *clk = get_net_or_empty(ci, id_I);
        if (clk == nullptr || clk->driver.cell == nullptr)
            continue;
        CellInfo *drv = clk->driver.cell;
        // Only a pad fixes the site.  A BUFIO driven by an MMCM/PLL output, or
        // by anything else, enters the tile through a different DMUX leg and is
        // left exactly as it was.  pack_io() has already given every input
        // buffer its bel by now (decompose_iob() writes <site>/IOB33/INBUF_EN),
        // which is what makes the pad knowable this early.
        if (!boost::contains(drv->type.str(ctx), "INBUF") || !drv->attrs.count(id_BEL))
            continue;
        BelId pad_bel = ctx->getBelByName(ctx->id(drv->attrs.at(id_BEL).as_string()));
        if (pad_bel == BelId())
            continue;
        BelId dedicated =
                find_bel_with_short_route(ctx->getBelPinWire(pad_bel, clk->driver.port), ci->type, id_I);
        // No BUFIO reachable: the pad is not clock-capable.  Leave that to the
        // router, whose message names the two ends of the arc it could not
        // build; guessing a site here would only move the failure.
        if (dedicated == BelId())
            continue;
        if (ci->attrs.count(id_BEL)) {
            // A site the user wrote wins, but check it against the graph rather
            // than let a wrong one turn into a routing failure an hour later.
            // A name that does not resolve is not ours to report: the placer
            // already says "No Bel named ..." for that.
            const std::string want = ci->attrs.at(id_BEL).as_string();
            BelId want_bel = ctx->getBelByName(ctx->id(want));
            if (want_bel == dedicated)
                used_bels.insert(dedicated);
            else if (want_bel != BelId())
                log_error("BUFIO '%s' is constrained to bel '%s', which the pad driving it cannot reach; "
                          "the BUFIO of that pad (%s) is '%s'.\n",
                          ctx->nameOf(ci), want.c_str(), ctx->nameOfBel(pad_bel), ctx->nameOfBel(dedicated));
            continue;
        }
        used_bels.insert(dedicated);
        ci->attrs[id_BEL] = std::string(ctx->nameOfBel(dedicated));
        log_info("    Constrained BUFIO '%s' to bel '%s' (dedicated site of the pad at %s)\n", ctx->nameOf(ci),
                 ctx->nameOfBel(dedicated), ctx->nameOfBel(pad_bel));
    }
}

void XC7Packer::pack_clocking()
{
    pack_plls();
    pack_gbs();
}


// -----------------------------------------------------------------------
// Clock constraint propagation (openXC7/nextpnr-xilinx#155)
//
// create_clock attaches a ClockConstraint to the net the XDC names -- for
// `[get_ports clk100]` that is the pad-side net, which no flop is clocked
// by. The timing walk keys its domains by the net that actually reaches
// the flops (the IBUF output, the BUFG output, a PLL output...), and none of
// those inherited the constraint, so every domain but the pad's was
// analysed against the --freq default and reported "PASS at 12.00 MHz" for
// designs constrained at 100 that do not meet it.
//
// Same scheme as the ecp5/ice40 packers: walk from every constrained net
// through the clock buffers (ratio 1, or 1/N for a dividing BUFR) and
// through PLLE2/MMCME2 (period_out = period_in * DIVCLK_DIVIDE *
// CLKOUTn_DIVIDE / CLKFBOUT_MULT), to a fixpoint. A constraint the user put
// on a downstream net themselves wins over a derived one.
// -----------------------------------------------------------------------
void XC7Packer::propagate_clock_constraints()
{
    log_info("Propagating clock constraints...\n");

    auto MHz = [&](delay_t period) { return 1000.0 / ctx->getDelayNS(period); };
    auto param_double = [&](CellInfo *ci, const std::string &name, double def) {
        IdString p = ctx->id(name);
        if (!ci->params.count(p))
            return def;
        const Property &prop = ci->params.at(p);
        if (prop.is_string) {
            try {
                return std::stod(prop.as_string());
            } catch (...) {
                return def;
            }
        }
        return double(prop.as_int64());
    };

    std::unordered_set<IdString> user_constrained;
    std::unordered_set<IdString> changed_nets;
    for (auto &net : ctx->nets) {
        if (net.second->clkconstr) {
            user_constrained.insert(net.first);
            changed_nets.insert(net.first);
        }
    }
    if (changed_nets.empty())
        return;

    // Give `to` the constraint of `from` scaled by ratio (f_to = f_from * ratio).
    auto set_derived = [&](CellInfo *ci, NetInfo *from, NetInfo *to, double ratio) {
        if (from == nullptr || from->clkconstr == nullptr || to == nullptr || ratio <= 0)
            return;
        delay_t period = delay_t(from->clkconstr->period.delay / ratio);
        if (to->clkconstr != nullptr) {
            if (user_constrained.count(to->name) && std::abs(to->clkconstr->period.delay - period) > 1)
                log_warning("    net '%s' is constrained to %.1f MHz by the XDC; the %.1f MHz derived through "
                            "'%s' is not applied\n",
                            to->name.c_str(ctx), MHz(to->clkconstr->period.delay), MHz(period), ci->name.c_str(ctx));
            return;
        }
        to->clkconstr = std::unique_ptr<ClockConstraint>(new ClockConstraint);
        to->clkconstr->period.delay = period;
        to->clkconstr->high.delay = period / 2;
        to->clkconstr->low.delay = period / 2;
        log_info("    derived %.1f MHz for net '%s' (through %s '%s')\n", MHz(period), to->name.c_str(ctx),
                 ci->type.c_str(ctx), ci->name.c_str(ctx));
        changed_nets.insert(to->name);
    };
    auto copy_through = [&](CellInfo *ci, const std::string &in, const std::string &out, double ratio = 1.0) {
        NetInfo *from = get_net_or_empty(ci, ctx->id(in));
        NetInfo *to = get_net_or_empty(ci, ctx->id(out));
        set_derived(ci, from, to, ratio);
    };

    const IdString t_IBUF = ctx->id("IBUF"), t_IBUFDS = ctx->id("IBUFDS"), t_IBUFG = ctx->id("IBUFG"),
                   t_IBUFGDS = ctx->id("IBUFGDS"), t_BUFG = ctx->id("BUFG"), t_BUFGCTRL = ctx->id("BUFGCTRL"),
                   t_BUFGCE = ctx->id("BUFGCE"), t_BUFH = ctx->id("BUFH"), t_BUFHCE = ctx->id("BUFHCE"),
                   t_BUFR = ctx->id("BUFR"), t_BUFMR = ctx->id("BUFMR"), t_BUFMRCE = ctx->id("BUFMRCE"),
                   t_BUFIO = ctx->id("BUFIO"), t_PLLE2_ADV = ctx->id("PLLE2_ADV"), t_PLLE2_BASE = ctx->id("PLLE2_BASE"),
                   t_MMCME2_ADV = ctx->id("MMCME2_ADV"), t_MMCME2_BASE = ctx->id("MMCME2_BASE");

    // Iterate while constraints keep appearing (buffer after PLL after buffer...);
    // the bound guards against a loop through a self-fed PLL.
    int iter = 0;
    const int itermax = 1000;
    while (!changed_nets.empty() && iter < itermax) {
        ++iter;
        std::unordered_set<IdString> changed_cells;
        for (auto net : changed_nets)
            for (auto &user : ctx->nets.at(net)->users)
                changed_cells.insert(user.cell->name);
        changed_nets.clear();
        for (auto cell : sorted(changed_cells)) {
            CellInfo *ci = ctx->cells.at(cell).get();
            IdString t = ci->type;
            if (t == t_IBUF || t == t_IBUFDS || t == t_IBUFG || t == t_IBUFGDS || t == t_BUFG || t == t_BUFGCE ||
                t == t_BUFH || t == t_BUFHCE || t == t_BUFMR || t == t_BUFMRCE || t == t_BUFIO) {
                copy_through(ci, "I", "O");
            } else if (t == t_BUFGCTRL) {
                copy_through(ci, "I0", "O");
                copy_through(ci, "I1", "O");
            } else if (t == t_BUFR) {
                std::string div = str_or_default(ci->params, ctx->id("BUFR_DIVIDE"), "BYPASS");
                double ratio = 1.0;
                if (div != "BYPASS") {
                    try {
                        ratio = 1.0 / std::stod(div);
                    } catch (...) {
                        log_warning("    BUFR '%s': unrecognised BUFR_DIVIDE '%s', assuming BYPASS for the "
                                    "constraint\n",
                                    ci->name.c_str(ctx), div.c_str());
                    }
                }
                copy_through(ci, "I", "O", ratio);
            } else if (t == t_PLLE2_ADV || t == t_PLLE2_BASE || t == t_MMCME2_ADV || t == t_MMCME2_BASE) {
                bool is_mmcm = (t == t_MMCME2_ADV || t == t_MMCME2_BASE);
                NetInfo *in = get_net_or_empty(ci, ctx->id("CLKIN1"));
                if (in == nullptr || in->clkconstr == nullptr)
                    in = get_net_or_empty(ci, ctx->id("CLKIN2"));
                if (in == nullptr || in->clkconstr == nullptr)
                    continue;
                double mult = is_mmcm && ci->params.count(ctx->id("CLKFBOUT_MULT_F"))
                                      ? param_double(ci, "CLKFBOUT_MULT_F", 1.0)
                                      : param_double(ci, "CLKFBOUT_MULT", 1.0);
                double divclk = param_double(ci, "DIVCLK_DIVIDE", 1.0);
                if (mult <= 0 || divclk <= 0)
                    continue;
                // f_vco = f_in * mult / divclk ; f_out_n = f_vco / CLKOUTn_DIVIDE
                double vco_ratio = mult / divclk;
                log_info("    %s '%s': input %.1f MHz, VCO %.1f MHz\n", ci->type.c_str(ctx), ci->name.c_str(ctx),
                         MHz(in->clkconstr->period.delay), MHz(in->clkconstr->period.delay) * vco_ratio);
                int nout = is_mmcm ? 7 : 6;
                for (int n = 0; n < nout; n++) {
                    std::string port = "CLKOUT" + std::to_string(n);
                    double div;
                    if (is_mmcm && n == 0 && ci->params.count(ctx->id("CLKOUT0_DIVIDE_F")))
                        div = param_double(ci, "CLKOUT0_DIVIDE_F", 1.0);
                    else
                        div = param_double(ci, port + "_DIVIDE", 1.0);
                    // MMCM: CLKOUT4 may be cascaded through CLKOUT6's divider
                    if (is_mmcm && n == 4 &&
                        str_or_default(ci->params, ctx->id("CLKOUT4_CASCADE"), "FALSE") == "TRUE")
                        div *= param_double(ci, "CLKOUT6_DIVIDE", 1.0);
                    if (div <= 0)
                        continue;
                    set_derived(ci, in, get_net_or_empty(ci, ctx->id(port)), vco_ratio / div);
                    // the inverted outputs (MMCM CLKOUT0B..3B) run at the same rate
                    if (is_mmcm && n <= 3)
                        set_derived(ci, in, get_net_or_empty(ci, ctx->id(port + "B")), vco_ratio / div);
                }
                // CLKFBOUT runs at the VCO rate
                set_derived(ci, in, get_net_or_empty(ci, ctx->id("CLKFBOUT")), vco_ratio);
                if (is_mmcm)
                    set_derived(ci, in, get_net_or_empty(ci, ctx->id("CLKFBOUTB")), vco_ratio);
            }
        }
    }
    if (iter >= itermax)
        log_warning("    clock constraint propagation did not settle after %d iterations (loop through a PLL?)\n",
                    itermax);
}

NEXTPNR_NAMESPACE_END
