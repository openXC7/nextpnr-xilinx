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
    // Set the configuration rules of BUFGMUX. BUFGMUX and BUFGMUX_1 and
    // apply all the configuration information of the S port to CE0 and CE1
    std::unordered_map<IdString, XFormRule> bufgctrl_rules;
    bufgctrl_rules[ctx->id("BUFGMUX")].new_type = ctx->id("BUFGCTRL");
    bufgctrl_rules[ctx->id("BUFGMUX")].port_multixform[ctx->id("S")] = {ctx->id("CE0"),ctx->id("CE1")};
    bufgctrl_rules[ctx->id("BUFGMUX_1")] = bufgctrl_rules[ctx->id("BUFGMUX")];

    bufgctrl_rules[ctx->id("BUFGMUX_CTRL")].new_type = ctx->id("BUFGCTRL");
    bufgctrl_rules[ctx->id("BUFGMUX_CTRL")].port_multixform[ctx->id("S")] = {ctx->id("S0"),ctx->id("S1")};

    bufgctrl_rules[ctx->id("BUFGCE")].new_type = ctx->id("BUFGCTRL");
    bufgctrl_rules[ctx->id("BUFGCE")].port_xform[ctx->id("I")] = ctx->id("I0");
    bufgctrl_rules[ctx->id("BUFGCE")].port_xform[ctx->id("CE")] = ctx->id("CE0");
    bufgctrl_rules[ctx->id("BUFGCE_1")] = bufgctrl_rules[ctx->id("BUFGCE")];

    bufgctrl_rules[ctx->id("BUFG")].new_type = ctx->id("BUFGCTRL");
    bufgctrl_rules[ctx->id("BUFG")].port_xform[ctx->id("I")] = ctx->id("I0");

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
            // Retype BUFG -> BUFGCTRL and rename I -> I0 here rather than
            // leaving it to the trailing generic_xform(): xform_cell()
            // records X_ORIG_TYPE plus X_ORIG_PORT_I0 / X_ORIG_PORT_O, which
            // json2dcp / fasm.cc need to rebuild the BUFG EDIF cell.  Doing
            // it *before* the tie_port()s below is deliberate - the tied
            // CE0/S0/S1/IGNORE0 pins must stay untagged, because they do not
            // exist on the source BUFG.
            xform_cell(bufgctrl_rules, ci);
            tie_port(ci, "CE0", true, true);
            tie_port(ci, "S0", true, true);
            tie_port(ci, "S1", false, true);
            tie_port(ci, "IGNORE0", true, true);
        } else if (ci->type == ctx->id("BUFGCE") || ci->type == ctx->id("BUFGCE_1")) {
            fold_inverter(ci,"CE");
            int inverter = int_or_default(ci->params,ctx->id("IS_CE_INVERTED"));
            if (inverter)
            {
                ci->params[ctx->id("IS_CE0_INVERTED")] = Property(1);
                ci->params.erase(ctx->id("IS_CE_INVERTED"));
            }
            // I -> I0, CE -> CE0, plus the X_ORIG_TYPE / X_ORIG_PORT_* tags
            // master needs for json2dcp.  Must run after fold_inverter()
            // (which looks the port up under its source name "CE") and
            // before the tie_port()s (tied pins must stay untagged).
            xform_cell(bufgctrl_rules, ci);
            tie_port(ci, "S0", true, true);
            tie_port(ci, "S1", false, true);
            tie_port(ci, "IGNORE0", true, true);
            tie_port(ci, "IGNORE1", false, true);
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
        } else if (ci->type == ctx->id("BUFGMUX") || ci->type == ctx->id("BUFGMUX_1")) {
            // insert inverter before the destination port
            fold_inverter(ci,"S");
            int inverter = int_or_default(ci->params,ctx->id("IS_S_INVERTED"));
            if (inverter)
            {
                ci->params[ctx->id("IS_CE0_INVERTED")] = Property(0);
                ci->params[ctx->id("IS_CE1_INVERTED")] = Property(1);
                ci->params.erase(ctx->id("IS_S_INVERTED"));
            } else {
                ci->params[ctx->id("IS_CE0_INVERTED")] = Property(1);
                ci->params[ctx->id("IS_CE1_INVERTED")] = Property(0);
            }

            // S -> {CE0, CE1} (port_multixform), type -> BUFGCTRL, and the
            // X_ORIG_TYPE / X_ORIG_PORT_* provenance master added.  Same
            // reason as above: run it before the tie_port()s.
            xform_cell(bufgctrl_rules, ci);

            std::string sel_type = str_or_default(ci->params, ctx->id("CLK_SEL_TYPE"), "SYNC");
            if (sel_type == "ASYNC") {
                tie_port(ci, "S0", true, true);
                tie_port(ci, "S1", true, true);
                tie_port(ci, "IGNORE0", false, true);
                tie_port(ci, "IGNORE1", false, true);
            } else if (sel_type == "SYNC") {
                /*These settings ensure that BUFGCTRL behaves like BUFGMUX:
                    1. S0 and S1 are set high so that CE0 and CE1 control input selection.
                    2. IGNORE0 and IGNORE1 are set high, and the internal default is inverted,
                       so no inversion is required.
                */
                tie_port(ci, "S0", true, true);
                tie_port(ci, "S1", true, true);
                tie_port(ci, "IGNORE0", true, false);
                tie_port(ci, "IGNORE1", true, false);
            }

        } else if (ci->type == ctx->id("BUFGMUX_CTRL")) {
            // Inverter before the absorption port
            fold_inverter(ci, "S");
            for (auto& it : ci->params)
            {
                std::string str = it.first.c_str(ctx);
            }
            int inverter = int_or_default(ci->params,ctx->id("IS_S_INVERTED"));
            if (inverter)
            {
                ci->params[ctx->id("IS_S0_INVERTED")] = Property(0);
                ci->params[ctx->id("IS_S1_INVERTED")] = Property(1);
                ci->params.erase(ctx->id("IS_S_INVERTED"));
            } else {
                ci->params[ctx->id("IS_S0_INVERTED")] = Property(1);
                ci->params[ctx->id("IS_S1_INVERTED")] = Property(0);
            }

            xform_cell(bufgctrl_rules, ci);
            /* These settings ensure that BUFGCTRL behaves like BUFGMUX_CTRL:
                1. CE0 and CE1 are set high so that S0 and S1 control input selection.
                2. IGNORE0 and IGNORE1 are set high, and the internal inversion is the default
                   so no inversion is needed.
            */
            tie_port(ci, "CE0", true, true);
            tie_port(ci, "CE1", true, true);
            // configure IGNORE0 and IGNORE1, keep high level,
            // do not enter "IS_IGNOREX_INVERTED" to indicate inversion
            tie_port(ci, "IGNORE0", true, false);
            tie_port(ci, "IGNORE1", true, false);
        }
    }
    generic_xform(bufgctrl_rules);
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
}

void XC7Packer::pack_clocking()
{
    pack_plls();
    pack_gbs();
}

NEXTPNR_NAMESPACE_END
