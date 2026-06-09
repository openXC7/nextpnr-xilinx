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

bool XC7Packer::has_illegal_fanout(NetInfo *carry)
{
    // FIXME: sometimes we can feed out of the chain
    if (carry->users.size() > 2)
        return true;
    CellInfo *muxcy = nullptr, *xorcy = nullptr;
    for (auto &usr : carry->users) {
        if (usr.cell->type == ctx->id("MUXCY")) {
            if (muxcy != nullptr)
                return true;
            else if (usr.port != ctx->id("CI"))
                return true;
            else
                muxcy = usr.cell;
        } else if (usr.cell->type == ctx->id("XORCY")) {
            if (xorcy != nullptr)
                return true;
            else if (usr.port != ctx->id("CI"))
                return true;
            else
                xorcy = usr.cell;
        } else {
            return true;
        }
    }
    if (muxcy && xorcy) {
        NetInfo *muxcy_s = get_net_or_empty(muxcy, ctx->id("S"));
        NetInfo *xorcy_li = get_net_or_empty(xorcy, ctx->id("LI"));
        if (muxcy_s != xorcy_li)
            return true;
    }
    return false;
}

void XilinxPacker::split_carry4s()
{
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type != ctx->id("CARRY4"))
            continue;
        NetInfo *cin = get_net_or_empty(ci, ctx->id("CI"));
        if (cin == nullptr || cin->name == ctx->id("$PACKER_GND_NET")) {
            cin = get_net_or_empty(ci, ctx->id("CYINIT"));
        }
        disconnect_port(ctx, ci, ctx->id("CI"));
        disconnect_port(ctx, ci, ctx->id("CYINIT"));

        for (int i = 0; i < 4; i++) {
            std::unique_ptr<CellInfo> xorcy =
                    create_cell(ctx, ctx->id("XORCY"), ctx->id(ci->name.str(ctx) + "$split$xorcy" + std::to_string(i)));
            std::unique_ptr<CellInfo> muxcy =
                    create_cell(ctx, ctx->id("MUXCY"), ctx->id(ci->name.str(ctx) + "$split$muxcy" + std::to_string(i)));
            connect_port(ctx, cin, muxcy.get(), ctx->id("CI"));
            connect_port(ctx, cin, xorcy.get(), ctx->id("CI"));
            replace_port(ci, ctx->id("DI[" + std::to_string(i) + "]"), muxcy.get(), ctx->id("DI"));
            connect_port(ctx, get_net_or_empty(ci, ctx->id("S[" + std::to_string(i) + "]")), muxcy.get(), ctx->id("S"));
            replace_port(ci, ctx->id("S[" + std::to_string(i) + "]"), xorcy.get(), ctx->id("LI"));
            replace_port(ci, ctx->id("O[" + std::to_string(i) + "]"), xorcy.get(), ctx->id("O"));
            NetInfo *co = get_net_or_empty(ci, ctx->id("CO[" + std::to_string(i) + "]"));
            disconnect_port(ctx, ci, ctx->id("CO[" + std::to_string(i) + "]"));
            if (co == nullptr)
                co = create_internal_net(ci->name, "$split$co" + std::to_string(i), false);
            connect_port(ctx, co, muxcy.get(), ctx->id("O"));
            cin = co;
            new_cells.push_back(std::move(xorcy));
            new_cells.push_back(std::move(muxcy));
        }
        packed_cells.insert(ci->name);
    }
    flush_cells();
}

// Atomic CARRY4 packer: keep each Vivado CARRY4 as a single nextpnr cell
// (no split into MUXCY/XORCY pieces).  Each cell is constrained to a
// SLICE/CARRY4 BEL; chains are linked head-to-tail via CO[3] -> CI.
// Cell names are preserved verbatim from Vivado synth, which lets the
// hybrid flow's json_to_loc_tcl pin every CARRY4 at the slice nextpnr
// chose — eliminating the dominant chunk of the direct-vs-hybrid FASM
// gap that was a placement-mismatch artifact rather than a real chipdb
// shortfall.
//
// Set XC7_LEGACY_CARRY4_SPLIT=1 in the environment to fall back to the
// historical split-and-repack path (kept for emergency comparison).
void XC7Packer::pack_carries_atomic()
{
    log_info("Packing carries (atomic CARRY4)..\n");

    static const std::unordered_set<IdString> lut_types{
        ctx->id("LUT1"), ctx->id("LUT2"), ctx->id("LUT3"),
        ctx->id("LUT4"), ctx->id("LUT5")};

    // Find chain roots: a CARRY4 whose CI input is not driven by another
    // CARRY4's CO[3] output.
    std::vector<CellInfo *> roots;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type != ctx->id("CARRY4"))
            continue;
        NetInfo *ci_net = get_net_or_empty(ci, ctx->id("CI"));
        bool is_root = (ci_net == nullptr) ||
                       (ci_net->driver.cell == nullptr) ||
                       (ci_net->driver.cell->type != ctx->id("CARRY4"));
        if (is_root)
            roots.push_back(ci);
    }

    int chain_count = 0, cell_count = 0;
    for (auto *root : roots) {
        // Place root at absolute CARRY4 BEL position.
        root->constr_abs_z = true;
        root->constr_z = BEL_CARRY4;

        CellInfo *prev = root;
        int idx_in_chain = 0;
        while (true) {
            // Constrain LUTs driving this CARRY4's DI[i] / S[i] inputs
            // into the adjacent BEL_5LUT / BEL_6LUT slots.  Insert feed-
            // through LUTs if the input nets aren't LUT-driven.
            int constr_y = -(idx_in_chain + idx_in_chain / 25);
            for (int z = 0; z < 4; z++) {
                std::string zs = std::to_string(z);
                NetInfo *c4_s  = get_net_or_empty(prev, ctx->id("S["  + zs + "]"));
                NetInfo *c4_di = get_net_or_empty(prev, ctx->id("DI[" + zs + "]"));

                // Constant-driven DI inputs ($PACKER_GND_NET / $PACKER_VCC_NET)
                // don't need a feed-through LUT: the CARRY4 DI mux defaults to 0
                // when the DI route is simply omitted, so a const DI is tied
                // internally.  (Spurious DI feed-throughs were what hung the
                // heap placer when we tried to constrain them.)
                //
                // S inputs are different: the carry's S (propagate) input MUST
                // be driven by a real LUT output -- there is no internal tie.  A
                // constant S therefore needs a feed-through LUT that buffers the
                // VCC/GND net into S; otherwise routeVcc() is asked to route the
                // constant to a LUT *output* sitewire, which no fabric pip can
                // reach (arch.cc:744 assert).  So DON'T null a const S here --
                // let the s_lut==nullptr path below insert the feed-through.
                auto is_const = [&](NetInfo *n) {
                    return n && (n->name == ctx->id("$PACKER_GND_NET") ||
                                 n->name == ctx->id("$PACKER_VCC_NET"));
                };
                // A constant-driven CARRY4 DI must NOT be left on $PACKER_GND_NET: the
                // global pseudo-constant router does not negotiate congestion, so the
                // const DI fought real signals for the scarce SLICE bypass inputs and the
                // route stalled.  A DI cannot be driven from a *remote* LUT either -- a
                // 5LUT's O5 output only reaches its own slice's DI mux, not the fabric.
                // So let the const DI fall through to the normal di_lut path below, which
                // inserts a feed-through LUT (a local ground reference, O5=const) and
                // CONSTRAINS it into THIS slice's BEL_5LUT -> the internal O5->DI path,
                // using no fabric bypass at all.  (The earlier code nulled const DI here
                // to skip that, on a stale worry about the HeAP placer; the constrained S
                // feed-throughs prove slice-constrained feed-throughs place fine.)

                std::unordered_set<IdString> unique_lut_inputs;
                int s_inputs = 0;
                CellInfo *s_lut = nullptr, *di_lut = nullptr;

                if (c4_s && c4_s->users.size() == 1 &&
                    c4_s->driver.cell != nullptr &&
                    lut_types.count(c4_s->driver.cell->type)) {
                    s_lut = c4_s->driver.cell;
                    for (int j = 0; j < 5; j++) {
                        NetInfo *ix = get_net_or_empty(s_lut, ctx->id("I" + std::to_string(j)));
                        if (ix) { unique_lut_inputs.insert(ix->name); s_inputs++; }
                    }
                }
                if (c4_di && c4_di->users.size() == 1 &&
                    c4_di->driver.cell != nullptr &&
                    lut_types.count(c4_di->driver.cell->type)) {
                    di_lut = c4_di->driver.cell;
                    for (int j = 0; j < 5; j++) {
                        NetInfo *ix = get_net_or_empty(di_lut, ctx->id("I" + std::to_string(j)));
                        if (ix) unique_lut_inputs.insert(ix->name);
                    }
                }
                int lut_inp_count = int(unique_lut_inputs.size());
                if (!s_lut)  ++lut_inp_count;
                if (!di_lut) ++lut_inp_count;
                if (lut_inp_count > 5) {
                    di_lut = nullptr;
                    if (s_inputs > 4) s_lut = nullptr;
                }
                if (!s_lut && c4_s) {
                    PortRef pr; pr.cell = prev;
                    pr.port = ctx->id("S[" + zs + "]");
                    auto s_feed = feed_through_lut(c4_s, {pr});
                    s_lut = s_feed.get();
                    new_cells.push_back(std::move(s_feed));
                }
                if (!di_lut && c4_di) {
                    PortRef pr; pr.cell = prev;
                    pr.port = ctx->id("DI[" + zs + "]");
                    auto di_feed = feed_through_lut(c4_di, {pr});
                    di_lut = di_feed.get();
                    new_cells.push_back(std::move(di_feed));
                }
                if (s_lut) {
                    root->constr_children.push_back(s_lut);
                    s_lut->constr_parent = root;
                    s_lut->constr_x = 0;
                    s_lut->constr_y = constr_y;
                    s_lut->constr_abs_z = true;
                    s_lut->constr_z = (z << 4 | BEL_6LUT);
                }
                if (di_lut) {
                    root->constr_children.push_back(di_lut);
                    di_lut->constr_parent = root;
                    di_lut->constr_x = 0;
                    di_lut->constr_y = constr_y;
                    di_lut->constr_abs_z = true;
                    di_lut->constr_z = (z << 4 | BEL_5LUT);
                }
            }

            // Walk to the next CARRY4 in the chain.  Vivado's CARRY4 spec
            // says CO[3] is the chain output, but JSON bus-expansion in
            // nextpnr places the chain output at CO[0] for an EDIF
            // declared "output [3:0] CO" (because list index 0 maps to
            // the FIRST declared bit, which is bit 3 / the MSB).  Be
            // robust: scan CO[0..3] and pick whichever bit drives a
            // CARRY4's CI input.
            CellInfo *next = nullptr;
            for (int co_bit = 0; co_bit < 4 && !next; co_bit++) {
                NetInfo *co_n = get_net_or_empty(
                    prev, ctx->id("CO[" + std::to_string(co_bit) + "]"));
                if (!co_n) continue;
                for (auto &u : co_n->users) {
                    if (u.cell->type == ctx->id("CARRY4") &&
                        u.port == ctx->id("CI")) {
                        next = u.cell;
                        break;
                    }
                }
            }
            if (!next) break;
            ++idx_in_chain;
            // Constrain next CARRY4 relative to the root.
            next->constr_parent = root;
            root->constr_children.push_back(next);
            next->constr_x = 0;
            // Skip every 25th tile (no CARRY4 BEL on tiles where
            // grid_y is a multiple of 26 - HCLK rows etc.).
            next->constr_y = -(idx_in_chain + idx_in_chain / 25);
            next->constr_abs_z = true;
            next->constr_z = BEL_CARRY4;
            prev = next;
        }
        ++chain_count;
        cell_count += idx_in_chain + 1;
    }
    log_info("   Packed %d CARRY4 cells into %d chains (atomic).\n",
             cell_count, chain_count);

    // feed_through_lut() pushes newly-created LUT cells into new_cells
    // (not ctx->cells).  Flush them in now so pack_luts (which runs after
    // pack_carries and iterates ctx->cells) sees and xforms them to
    // SLICE_LUTX; otherwise they survive as unplaceable LUT1 cells and
    // the placer either hangs (HeAP) or asserts (SA).
    flush_cells();

    // Bus-style ports DI[i]/S[i]/O[i]/CO[i] -> chipdb pin names DIi/Si/Oi/COi.
    // Also CI -> CIN per the chipdb's BEL pin naming.
    std::unordered_map<IdString, XFormRule> c4_rules;
    c4_rules[ctx->id("CARRY4")].new_type = ctx->id("CARRY4");
    c4_rules[ctx->id("CARRY4")].port_xform[ctx->id("CI")] = ctx->id("CIN");
    for (int i = 0; i < 4; i++) {
        std::string is = std::to_string(i);
        c4_rules[ctx->id("CARRY4")].port_xform[ctx->id("DI[" + is + "]")] = ctx->id("DI" + is);
        c4_rules[ctx->id("CARRY4")].port_xform[ctx->id("S["  + is + "]")] = ctx->id("S"  + is);
        c4_rules[ctx->id("CARRY4")].port_xform[ctx->id("O["  + is + "]")] = ctx->id("O"  + is);
        c4_rules[ctx->id("CARRY4")].port_xform[ctx->id("CO[" + is + "]")] = ctx->id("CO" + is);
    }
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type != ctx->id("CARRY4"))
            continue;
        xform_cell(c4_rules, ci);
    }

    // Any leftover MUXCY/XORCY in RTL (not part of CARRY4 chains) -> soft logic.
    int remaining_muxcy = 0, remaining_xorcy = 0;
    for (auto &cell : ctx->cells) {
        if (cell.second->type == ctx->id("MUXCY"))
            ++remaining_muxcy;
        else if (cell.second->type == ctx->id("XORCY"))
            ++remaining_xorcy;
    }
    std::unordered_map<IdString, XFormRule> softlogic_rules;
    softlogic_rules[ctx->id("MUXCY")].new_type = ctx->id("LUT3");
    softlogic_rules[ctx->id("MUXCY")].port_xform[ctx->id("DI")] = ctx->id("I0");
    softlogic_rules[ctx->id("MUXCY")].port_xform[ctx->id("CI")] = ctx->id("I1");
    softlogic_rules[ctx->id("MUXCY")].port_xform[ctx->id("S")]  = ctx->id("I2");
    softlogic_rules[ctx->id("MUXCY")].set_params.emplace_back(ctx->id("INIT"), Property(0xCA));
    softlogic_rules[ctx->id("XORCY")].new_type = ctx->id("LUT2");
    softlogic_rules[ctx->id("XORCY")].port_xform[ctx->id("CI")] = ctx->id("I0");
    softlogic_rules[ctx->id("XORCY")].port_xform[ctx->id("LI")] = ctx->id("I1");
    softlogic_rules[ctx->id("XORCY")].set_params.emplace_back(ctx->id("INIT"), Property(0x6));
    generic_xform(softlogic_rules, false);
    if (remaining_muxcy || remaining_xorcy)
        log_info("   Blasted %d non-chain MUXCYs and %d non-chain XORCYs to soft logic\n",
                 remaining_muxcy, remaining_xorcy);

    // router2 fix: realize a CONSTANT CARRY4 carry-in (a chain ROOT whose
    // CIN/CYINIT is $PACKER_GND_NET/$PACKER_VCC_NET) via the PRECYINIT.C0/.C1
    // config bit (see fasm.cc) instead of routing the GND/VCC pseudo-net into
    // the CIN/CYINIT pins.  Leaving those constant pins connected makes router2
    // fail ("Unrouteable $PACKER_GND_NET sink ...CIN"); router1 tolerated it.
    // Record the constant value, then disconnect the constant carry pins so they
    // are no longer routing sinks.  A real cascade CIN (mid-chain) and a dynamic
    // CYINIT (routed in via AX) are left untouched.
    {
        IdString gnd = ctx->id("$PACKER_GND_NET");
        IdString vcc = ctx->id("$PACKER_VCC_NET");
        int nfix = 0, ncarry = 0;
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type != ctx->id("CARRY4"))
                continue;
            ++ncarry;
            NetInfo *cin    = get_net_or_empty(ci, ctx->id("CIN"));
            NetInfo *cyinit = get_net_or_empty(ci, ctx->id("CYINIT"));
            bool cin_chain    = cin    != nullptr && cin->name    != gnd && cin->name != vcc;
            bool cin_const    = cin    != nullptr && (cin->name    == gnd || cin->name == vcc);
            bool cyinit_const = cyinit != nullptr && (cyinit->name == gnd || cyinit->name == vcc);
            if (!cin_chain && (cyinit_const || cin_const)) {
                NetInfo *konst = cyinit_const ? cyinit : cin;
                int val = (konst->name == vcc) ? 1 : 0;
                ci->params[ctx->id("PRECYINIT_CONST")] = Property(val, 1);
            }
            if (cin_const) { disconnect_port(ctx, ci, ctx->id("CIN")); ++nfix; }
            if (cyinit_const) { disconnect_port(ctx, ci, ctx->id("CYINIT")); ++nfix; }
        }
        log_info("router2 PRECYINIT fix: %d CARRY4 cells, %d constant carry pins disconnected\n", ncarry, nfix);
    }
}

void XC7Packer::pack_carries()
{
    const char *legacy = std::getenv("XC7_LEGACY_CARRY4_SPLIT");
    if (legacy == nullptr || legacy[0] == '0') {
        pack_carries_atomic();
        return;
    }
    log_info("Packing carries (legacy split + repack)..\n");
    split_carry4s();
    std::vector<CellInfo *> root_muxcys;
    // Find MUXCYs
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type != ctx->id("MUXCY"))
            continue;
        NetInfo *ci_net = get_net_or_empty(ci, ctx->id("CI"));
        if (ci_net == nullptr || ci_net->driver.cell == nullptr || ci_net->driver.cell->type != ctx->id("MUXCY") ||
            has_illegal_fanout(ci_net)) {
            root_muxcys.push_back(ci);
        }
    }

    // Create chains from root MUXCYs
    std::unordered_set<IdString> processed_muxcys;
    std::vector<CarryGroup> groups;
    int muxcy_count = 0, xorcy_count = 0;
    for (auto root : root_muxcys) {
        CarryGroup group;

        CellInfo *muxcy = root;
        NetInfo *mux_ci = nullptr;
        while (true) {

            group.muxcys.push_back(muxcy);
            ++muxcy_count;
            mux_ci = get_net_or_empty(muxcy, ctx->id("CI"));
            NetInfo *mux_s = get_net_or_empty(muxcy, ctx->id("S"));
            group.xorcys.push_back(nullptr);
            if (mux_s != nullptr) {
                for (auto &user : mux_s->users) {
                    if (user.cell->type == ctx->id("XORCY") && user.port == ctx->id("LI")) {
                        CellInfo *xorcy = user.cell;
                        NetInfo *xor_ci = get_net_or_empty(xorcy, ctx->id("CI"));
                        if (xor_ci == mux_ci) {
                            group.xorcys.back() = xorcy;
                            ++xorcy_count;
                            break;
                        }
                    }
                }
            }

            mux_ci = get_net_or_empty(muxcy, ctx->id("O"));
            if (mux_ci == nullptr)
                break;
            if (has_illegal_fanout(mux_ci))
                break;
            muxcy = nullptr;
            for (auto &user : mux_ci->users) {
                if (user.cell->type == ctx->id("MUXCY")) {
                    muxcy = user.cell;
                    break;
                }
            }
            if (muxcy == nullptr)
                break;
        }
        if (mux_ci != nullptr) {
            if (mux_ci->users.size() == 1 && mux_ci->users.at(0).cell->type == ctx->id("XORCY") &&
                mux_ci->users.at(0).port == ctx->id("CI")) {
                // Trailing XORCY at end, can pack into chain.
                CellInfo *xorcy = mux_ci->users.at(0).cell;
                std::unique_ptr<CellInfo> dummy_muxcy =
                        create_cell(ctx, ctx->id("MUXCY"), ctx->id(xorcy->name.str(ctx) + "$legal_muxcy$"));
                connect_port(ctx, mux_ci, dummy_muxcy.get(), ctx->id("CI"));
                connect_port(ctx, get_net_or_empty(xorcy, ctx->id("LI")), dummy_muxcy.get(), ctx->id("S"));
                group.muxcys.push_back(dummy_muxcy.get());
                group.xorcys.push_back(xorcy);
                new_cells.push_back(std::move(dummy_muxcy));
            } else if (mux_ci->users.size() > 0) {
                // Users other than a MUXCY
                // Feed out with a zero-driving LUT and a XORCY
                // (creating a zero-driver using Vcc and an inverter for now...)
                std::unique_ptr<CellInfo> zero_lut =
                        create_lut(ctx, mux_ci->name.str(ctx) + "$feed$zero",
                                   {ctx->nets[ctx->id("$PACKER_VCC_NET")].get()}, nullptr, Property(1));
                std::unique_ptr<CellInfo> feed_xorcy =
                        create_cell(ctx, ctx->id("XORCY"), ctx->id(mux_ci->name.str(ctx) + "$feed$xor"));
                std::unique_ptr<CellInfo> dummy_muxcy =
                        create_cell(ctx, ctx->id("MUXCY"), ctx->id(mux_ci->name.str(ctx) + "$feed$muxcy"));

                CellInfo *last_muxcy = mux_ci->driver.cell;

                disconnect_port(ctx, last_muxcy, ctx->id("O"));

                connect_ports(ctx, zero_lut.get(), ctx->id("O"), feed_xorcy.get(), ctx->id("LI"));
                connect_ports(ctx, zero_lut.get(), ctx->id("O"), dummy_muxcy.get(), ctx->id("S"));
                connect_ports(ctx, last_muxcy, ctx->id("O"), feed_xorcy.get(), ctx->id("CI"));
                connect_ports(ctx, last_muxcy, ctx->id("O"), dummy_muxcy.get(), ctx->id("CI"));

                connect_port(ctx, mux_ci, feed_xorcy.get(), ctx->id("O"));

                group.muxcys.push_back(dummy_muxcy.get());
                group.xorcys.push_back(feed_xorcy.get());
                new_cells.push_back(std::move(zero_lut));
                new_cells.push_back(std::move(feed_xorcy));
                new_cells.push_back(std::move(dummy_muxcy));
            }
        }

        groups.push_back(group);
    }
    flush_cells();

    log_info("   Grouped %d MUXCYs and %d XORCYs into %d chains.\n", muxcy_count, xorcy_count, int(root_muxcys.size()));

    // N.B. LUT6 is not a valid type here, as CARRY requires dual outputs
    std::unordered_set<IdString> lut_types{ctx->id("LUT1"), ctx->id("LUT2"), ctx->id("LUT3"), ctx->id("LUT4"),
                                           ctx->id("LUT5")};

    std::unordered_set<IdString> folded_nets;

    for (auto &grp : groups) {
        std::vector<std::unique_ptr<CellInfo>> carry4s;
        for (int i = 0; i < int(grp.muxcys.size()); i++) {
            int z = i % 4;
            CellInfo *muxcy = grp.muxcys.at(i), *xorcy = grp.xorcys.at(i);
            if (z == 0)
                carry4s.push_back(
                        create_cell(ctx, ctx->id("CARRY4"), ctx->id(muxcy->name.str(ctx) + "$PACKED_CARRY4$")));
            CellInfo *c4 = carry4s.back().get();
            CellInfo *root = carry4s.front().get();
            if (i == 0) {
                // Constrain initial CARRY4, forcing it to the CARRY4 of a logic tile
                c4->constr_abs_z = true;
                c4->constr_z = BEL_CARRY4;
            } else if (z == 0) {
                // Constrain relative to the root carry4
                c4->constr_parent = root;
                root->constr_children.push_back(c4);
                c4->constr_x = 0;
                // Looks no CARRY4 on the tile of which grid_y is a multiple of 26. Skip them
                c4->constr_y = -(i / 4 + i / (4*25));
                c4->constr_abs_z = true;
                c4->constr_z = BEL_CARRY4;
            }
            // Fold CI->CO connections into the CARRY4, except for those external ones every 8 units
            if (z == 0 && i == 0) {
                replace_port(muxcy, ctx->id("CI"), c4, ctx->id("CYINIT"));
            } else if (z == 0 && i > 0) {
                replace_port(muxcy, ctx->id("CI"), c4, ctx->id("CI"));
            } else {
                NetInfo *muxcy_ci = get_net_or_empty(muxcy, ctx->id("CI"));
                if (muxcy_ci)
                    folded_nets.insert(muxcy_ci->name);
                disconnect_port(ctx, muxcy, ctx->id("CI"));
            }
            if (z == 3) {
                replace_port(muxcy, ctx->id("O"), c4, ctx->id("CO[3]"));
            } else {
                NetInfo *muxcy_o = get_net_or_empty(muxcy, ctx->id("O"));
                if (muxcy_o)
                    folded_nets.insert(muxcy_o->name);
                disconnect_port(ctx, muxcy, ctx->id("O"));
            }
            // Replace connections into the MUXCY with external CARRY4 ports
            replace_port(muxcy, ctx->id("S"), c4, ctx->id("S[" + std::to_string(z) + "]"));
            replace_port(muxcy, ctx->id("DI"), c4, ctx->id("DI[" + std::to_string(z) + "]"));
            packed_cells.insert(muxcy->name);
            // Fold MUXCY->XORCY into the CARRY4, if there is a XORCY
            if (xorcy) {
                // Replace XORCY output with external CARRY4 output
                replace_port(xorcy, ctx->id("O"), c4, ctx->id("O[" + std::to_string(z) + "]"));
                // Disconnect internal XORCY connectivity
                disconnect_port(ctx, xorcy, ctx->id("LI"));
                disconnect_port(ctx, xorcy, ctx->id("DI"));
                packed_cells.insert(xorcy->name);
            }
            // Check legality of LUTs driving CARRY4, making them legal if they aren't already
            NetInfo *c4_s = get_net_or_empty(c4, ctx->id("S[" + std::to_string(z) + "]"));
            NetInfo *c4_di = get_net_or_empty(c4, ctx->id("DI[" + std::to_string(z) + "]"));
            // Keep track of the total LUT input count; cannot exceed five or the LUTs cannot be packed together
            std::unordered_set<IdString> unique_lut_inputs;
            int s_inputs = 0, d_inputs = 0;
            // Check that S and DI are validy and unqiuely driven by LUTs
            // FIXME: in multiple fanout cases, cell duplication will probably be cheaper
            // than feed-throughs
            CellInfo *s_lut = nullptr, *di_lut = nullptr;
            if (c4_s) {
                if (c4_s->users.size() == 1 && c4_s->driver.cell != nullptr &&
                    lut_types.count(c4_s->driver.cell->type)) {
                    s_lut = c4_s->driver.cell;
                    for (int j = 0; j < 5; j++) {
                        NetInfo *ix = get_net_or_empty(s_lut, ctx->id("I" + std::to_string(j)));
                        if (ix) {
                            unique_lut_inputs.insert(ix->name);
                            s_inputs++;
                        }
                    }
                }
            }
            if (c4_di) {
                if (c4_di->users.size() == 1 && c4_di->driver.cell != nullptr &&
                    lut_types.count(c4_di->driver.cell->type)) {
                    di_lut = c4_di->driver.cell;
                    for (int j = 0; j < 5; j++) {
                        NetInfo *ix = get_net_or_empty(di_lut, ctx->id("I" + std::to_string(j)));
                        if (ix) {
                            unique_lut_inputs.insert(ix->name);
                            d_inputs++;
                        }
                    }
                }
            }
            int lut_inp_count = int(unique_lut_inputs.size());
            if (!s_lut)
                ++lut_inp_count; // for feedthrough
            if (!di_lut)
                ++lut_inp_count; // for feedthrough
            if (lut_inp_count > 5) {
                // Must use feedthrough for at least one LUT
                di_lut = nullptr;
                if (s_inputs > 4)
                    s_lut = nullptr;
            }
            // If LUTs are nullptr, that means we need a feedthrough lut
            if (!s_lut && c4_s) {
                PortRef pr;
                pr.cell = c4;
                pr.port = ctx->id("S[" + std::to_string(z) + "]");
                auto s_feed = feed_through_lut(c4_s, {pr});
                s_lut = s_feed.get();
                new_cells.push_back(std::move(s_feed));
            }
            if (!di_lut && c4_di) {
                PortRef pr;
                pr.cell = c4;
                pr.port = ctx->id("DI[" + std::to_string(z) + "]");
                auto di_feed = feed_through_lut(c4_di, {pr});
                di_lut = di_feed.get();
                new_cells.push_back(std::move(di_feed));
            }
            // Constrain LUTs relative to root CARRY4
            if (s_lut) {
                root->constr_children.push_back(s_lut);
                s_lut->constr_parent = root;
                s_lut->constr_x = 0;
                s_lut->constr_y = -(i / 4 + i / (4*25));
                s_lut->constr_abs_z = true;
                s_lut->constr_z = (z << 4 | BEL_6LUT);
            }
            if (di_lut) {
                root->constr_children.push_back(di_lut);
                di_lut->constr_parent = root;
                di_lut->constr_x = 0;
                di_lut->constr_y = -(i / 4 + i / (4*25));
                di_lut->constr_abs_z = true;
                di_lut->constr_z = (z << 4 | BEL_5LUT);
            }
        }
        for (auto &c4 : carry4s)
            new_cells.push_back(std::move(c4));
    }
    flush_cells();

    for (auto net : folded_nets)
        ctx->nets.erase(net);

    // XORCYs and MUXCYs not part of any chain (and therefore not packed into a CARRY4) must now be blasted
    // to boring soft logic (LUT2 or LUT3 - these will become SLICE_LUTXs later in the flow.)
    int remaining_muxcy = 0, remaining_xorcy = 0;
    for (auto &cell : ctx->cells) {
        if (cell.second->type == ctx->id("MUXCY"))
            ++remaining_muxcy;
        else if (cell.second->type == ctx->id("XORCY"))
            ++remaining_xorcy;
    }
    std::unordered_map<IdString, XFormRule> softlogic_rules;
    softlogic_rules[ctx->id("MUXCY")].new_type = ctx->id("LUT3");
    softlogic_rules[ctx->id("MUXCY")].port_xform[ctx->id("DI")] = ctx->id("I0");
    softlogic_rules[ctx->id("MUXCY")].port_xform[ctx->id("CI")] = ctx->id("I1");
    softlogic_rules[ctx->id("MUXCY")].port_xform[ctx->id("S")] = ctx->id("I2");
    // DI 1010 1010
    // CI 1100 1100
    //  S 1111 0000
    //  O 1100 1010
    softlogic_rules[ctx->id("MUXCY")].set_params.emplace_back(ctx->id("INIT"), Property(0xCA));

    softlogic_rules[ctx->id("XORCY")].new_type = ctx->id("LUT2");
    softlogic_rules[ctx->id("XORCY")].port_xform[ctx->id("CI")] = ctx->id("I0");
    softlogic_rules[ctx->id("XORCY")].port_xform[ctx->id("LI")] = ctx->id("I1");
    // CI 1100
    // LI 1010
    //  O 0110
    softlogic_rules[ctx->id("XORCY")].set_params.emplace_back(ctx->id("INIT"), Property(0x6));

    generic_xform(softlogic_rules, false);
    log_info("   Blasted %d non-chain MUXCYs and %d non-chain XORCYs to soft logic\n", remaining_muxcy,
             remaining_xorcy);

    // Finally, use generic_xform to remove the [] from bus ports; and set up the logical-physical mapping for
    // RapidWright
    std::unordered_map<IdString, XFormRule> c4_rules;
    c4_rules[ctx->id("CARRY4")].new_type = ctx->id("CARRY4");
    c4_rules[ctx->id("CARRY4")].port_xform[ctx->id("CI")] = ctx->id("CIN");

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type != ctx->id("CARRY4"))
            continue;
        xform_cell(c4_rules, ci);
    }
}

NEXTPNR_NAMESPACE_END