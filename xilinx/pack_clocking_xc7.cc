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

            // Fixup routing
            if (str_or_default(ci->params, ctx->id("COMPENSATION"), "INTERNAL") == "INTERNAL") {
                disconnect_port(ctx, ci, ctx->id("CLKFBIN"));
                connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, ctx->id("CLKFBIN"));
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
        int n = 0;
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type != ctx->id("PLLE2_ADV_PLLE2_ADV"))
                continue;
            for (auto &p : drp) {
                NetInfo *nn = get_net_or_empty(ci, ctx->id(p));
                if (nn != nullptr && (nn->name == gnd || nn->name == vcc)) {
                    disconnect_port(ctx, ci, ctx->id(p));
                    ++n;
                }
            }
        }
        if (n)
            log_info("    PLLE2: disconnected %d const DRP/control pins (Vivado leaves them unrouted)\n", n);
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
