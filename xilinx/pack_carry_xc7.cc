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
#include <fstream>
#include <iterator>
#include <queue>
#include <set>
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
    // FF primitives whose D may be a CARRY4 sum output (O[i]).  Vivado packs
    // a registered adder/counter with each sum FF in the carry's OWN slice
    // (O[i] -> FFMUX.XOR -> AFF/BFF/CFF/DFF); we co-locate them the same way.
    static const std::unordered_set<IdString> ff_types{
        ctx->id("FDRE"), ctx->id("FDSE"), ctx->id("FDCE"), ctx->id("FDPE"),
        ctx->id("FDRSE")};

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
        // A BEL-pinned root (imported placement) already has its z fixed by
        // the BEL attribute; the CARRY4 bel z is NOT uniformly BEL_CARRY4
        // (0xF) across the device -- some columns (e.g. X43 on xc7vx485t)
        // encode it at 0x4F -- so hardcoding constr_z = BEL_CARRY4 makes the
        // absolute-z constraint distance non-zero and the placer aborts
        // ("constraint satisfaction check failed").  Leave a pinned root's z
        // unconstrained (the BEL attr binds it); its S/DI/FF children keep
        // their own absolute-z constraints, which are unaffected.
        root->constr_abs_z = true;
        if (!root->attrs.count(ctx->id("BEL")))
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
                // CARRY4 DI=GND: OMIT the route entirely.  The DI mux defaults to 0
                // when its input is left unconnected (golden sets no DImux feature
                // for a GND DI), so the carry is tied low internally with no fabric
                // route, no AX bypass and no feed-through LUT.  Routing GND to these
                // DIs (via the sparse GND backbone -> AX) is exactly what produced
                // the unroutable holdouts in congested slots.  (VCC DIs still fall
                // through to the feed-through/AX path below -- omitting would give 0,
                // not 1.)
                // FIX (DI-delivery): an unrouted carry-DI bypass does NOT default
                // to 0 -- it defaults to VCC (the BYP mux VCC fallback), so omitting a
                // GND DI silently delivers 1 and corrupts the carry (counters oscillate).
                // Vivado drives these to 0 explicitly (GFAN).  Keep the GND DI connected
                // so the di_feed path below buffers it to a const-0 (O5=0 -> DI=O5=0).
                // The legacy omit (which relied on the false 0-default) is available via
                // NEXTPNR_CARRY_OMIT_GND_DI for the old behaviour.
                if (getenv("NEXTPNR_CARRY_OMIT_GND_DI") &&
                    c4_di != nullptr && c4_di->name == ctx->id("$PACKER_GND_NET")) {
                    disconnect_port(ctx, prev, ctx->id("DI[" + zs + "]"));
                    c4_di = nullptr;
                }

                // ---- Counter "+1" re-encode (robust replacement for the I5
                // trick) ----
                // yosys encodes a counter's bottom +1 as CYINIT=0, DI[0]=VCC,
                // S[0]=~cnt[0] (an inverter).  Delivering DI[0]=1 needs O5=1 in
                // the A5LUT, but that O5 SHARES INIT bits with the co-located
                // S[0] inverter's O6 (forcing O5=1 corrupts S[0]); the AX/VCC
                // route hits the const-network's CARRY4-DI delivery weakness;
                // and pinning the inverter onto physical A6 (the literal I5
                // trick) fights nextpnr's LUT-input permuter.  Use the
                // ALGEBRAICALLY IDENTICAL encoding CYINIT=1, S[0]=cnt[0],
                // DI[0]=0:  O[0]=cnt[0]^1=~cnt[0] and CI[1]=cnt[0]?1:0=cnt[0].
                // CYINIT=1 is the PRECYINIT.C1 config bit (no routing), DI[0]=0
                // ties internally, and S[0]=cnt[0] becomes a plain routethru
                // like S[1..3] -- no O5/O6 sharing, no constant delivery.
                if (getenv("NEXTPNR_CARRY_COUNTER_FIX") &&
                    z == 0 && prev == root && c4_di != nullptr &&
                    c4_di->name == ctx->id("$PACKER_VCC_NET") && c4_s != nullptr) {
                    NetInfo *cyi = get_net_or_empty(prev, ctx->id("CYINIT"));
                    NetInfo *cin = get_net_or_empty(prev, ctx->id("CI"));
                    bool cyinit_zero = (cyi == nullptr || cyi->name == ctx->id("$PACKER_GND_NET"));
                    bool cin_zero    = (cin == nullptr || cin->name == ctx->id("$PACKER_GND_NET"));
                    CellInfo *inv = c4_s->driver.cell;
                    IdString inv_in_port;
                    bool is_inv = false;
                    if (inv != nullptr) {
                        if (inv->type == ctx->id("INV")) {
                            is_inv = true; inv_in_port = ctx->id("I");
                        } else if (inv->type == ctx->id("LUT1")) {
                            // LUT1 inverter O=~I0 has INIT="01" = value 1
                            // (INIT param is stored as bits, not a string).
                            if (int_or_default(inv->params, ctx->id("INIT"), -1) == 1) {
                                is_inv = true; inv_in_port = ctx->id("I0");
                            }
                        }
                    }
                    NetInfo *inv_in = is_inv ? get_net_or_empty(inv, inv_in_port) : nullptr;
                    bool sole = is_inv && c4_s->users.size() == 1;
                    if (is_inv && sole && inv_in != nullptr && cyinit_zero && cin_zero) {
                        // S[0] <- cnt[0] (bypass the inverter)
                        disconnect_port(ctx, prev, ctx->id("S[0]"));
                        connect_port(ctx, inv_in, prev, ctx->id("S[0]"));
                        // DI[0] <- 0 (omit; tied internally)
                        disconnect_port(ctx, prev, ctx->id("DI[0]"));
                        c4_di = nullptr;
                        // CYINIT <- 1 -> the PRECYINIT pass below emits PRECYINIT.C1
                        disconnect_port(ctx, prev, ctx->id("CYINIT"));
                        connect_port(ctx, ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get(),
                                     prev, ctx->id("CYINIT"));
                        c4_s = get_net_or_empty(prev, ctx->id("S[0]"));
                        if (getenv("DBG_CARRYFF"))
                            log_info("CARRY +1 re-encode %s: CYINIT=1, DI[0]=0, S[0]<-%s\n",
                                     ctx->nameOf(prev), ctx->nameOf(inv_in));
                    }
                }
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

                // A BEL-pinned LUT driver (imported placement) is adopted
                // regardless of fanout: Vivado already placed it at this
                // slot's 6LUT, where O6 legally feeds the carry S AND the
                // fabric simultaneously.  Creating a feed-through here
                // would collide with the pinned cell.
                if (c4_s && c4_s->driver.cell != nullptr &&
                    ((lut_types.count(c4_s->driver.cell->type) && c4_s->users.size() == 1) ||
                     // pinned import: any LUT1-6 Vivado placed at this slot
                     ((lut_types.count(c4_s->driver.cell->type) || c4_s->driver.cell->type == ctx->id("LUT6")) &&
                      c4_s->driver.cell->attrs.count(ctx->id("BEL"))))) {
                    s_lut = c4_s->driver.cell;
                    for (int j = 0; j < 5; j++) {
                        NetInfo *ix = get_net_or_empty(s_lut, ctx->id("I" + std::to_string(j)));
                        if (ix) { unique_lut_inputs.insert(ix->name); s_inputs++; }
                    }
                }
                if (c4_di && c4_di->driver.cell != nullptr &&
                    ((lut_types.count(c4_di->driver.cell->type) && c4_di->users.size() == 1) ||
                     (lut_types.count(c4_di->driver.cell->type) &&
                      c4_di->driver.cell->attrs.count(ctx->id("BEL"))))) {
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
                    // BEL-pinned LUTs (imported placement) stay adopted --
                    // Vivado already proved the input sharing legal.
                    if (!(di_lut && di_lut->attrs.count(ctx->id("BEL"))))
                        di_lut = nullptr;
                    if (s_inputs > 4 && !(s_lut && s_lut->attrs.count(ctx->id("BEL"))))
                        s_lut = nullptr;
                }
                if (!s_lut && c4_s) {
                    PortRef pr; pr.cell = prev;
                    pr.port = ctx->id("S[" + zs + "]");
                    auto s_feed = feed_through_lut(c4_s, {pr});
                    s_lut = s_feed.get();
                    if (ctx->debug)
                        log_info("feedthru %s for %s.S[%s]\n", ctx->nameOf(s_lut), ctx->nameOf(prev), zs.c_str());
                    if (getenv("DBG_SFEED"))
                        log_info("SFEED s_lut=%s for %s.S[%d] prevBEL=%s\n", ctx->nameOf(s_lut),
                                 ctx->nameOf(prev), z,
                                 prev->attrs.count(ctx->id("BEL")) ?
                                   prev->attrs.at(ctx->id("BEL")).as_string().c_str() : "NONE");
                    new_cells.push_back(std::move(s_feed));
                }
                // Imported placement: DI comes from O5 (feed-through in
                // the free 5LUT, golden's xCY0=O5) UNLESS the slot's 6LUT
                // is a pinned LUT using all six inputs -- then the 5LUT
                // is physically unusable and golden routes DI via the AX
                // bypass (xCY0=AX) instead.
                // A dynamic carry-in (CYINIT driven by a real net) is routed
                // into the slice via the AX bypass pin -- the SAME pin DI[0]'s
                // ACY0 mux would use for an AX-routed const DI[0].  They cannot
                // share AX, so for z==0 with a dynamic CYINIT the const DI[0]
                // must NOT go via AX; fall through to the O5 feed-through path.
                // (Symptom: "Failed to route ... AQ -> PRECYINIT_OUT" -- the
                // dynamic CYINIT could not reach AX because DI[0] had taken it.)
                NetInfo *cyi = get_net_or_empty(prev, ctx->id("CYINIT"));
                bool dyn_cyinit = cyi != nullptr && !is_const(cyi);
                bool di_via_ax = false;
                // A DI driven by a cell OTHER than this slot's own S-LUT is a
                // "remote" operand (golden's xCY0=AX).  It cannot be sourced from
                // the slot's free 5LUT O5: a feed-through LUT for it carries a
                // distinct input net, which the slot's shared-input packing must
                // place on a 6th pin (A6) -- a pin the 5LUT bel does not have
                // ("No wire found for port A6").  So for an imported (BEL-pinned)
                // CARRY4 with no same-slot di_lut, route the DI via the AX bypass
                // (as the const-DI case already does) whenever the DI is constant
                // OR driven by a different cell than S -- matching golden's
                // xCY0=AX encoding.  Only the genuine dual-output case (DI and S
                // from the SAME LUT, i.e. golden's xCY0=O5) keeps the feed-through.
                bool di_remote = c4_di && c4_di->driver.cell != nullptr &&
                                 (c4_s == nullptr || c4_s->driver.cell != c4_di->driver.cell);
                // A genuinely remote (non-const) DI prefers the O5 feed-through
                // over AX whenever this slot has an S-LUT to anchor it AND the
                // feed-through's single input still fits the 5LUT alongside the
                // S-LUT's inputs (<=5 unique nets, so the feed-through input
                // never spills onto the nonexistent 5LUT A6 pin).  Routing DI
                // via O5 leaves the column's X bypass free for a co-packed
                // main-FF whose D can ONLY arrive through X (its 6LUT is the
                // carry S-LUT, so no O6 routethru).  This is golden's encoding
                // for the 72 columns where a CARRY4 DI and a regfile FF.D
                // contend for the same X pin -- forcing DI onto AX there made
                // router1 fail ~2099 FF.D arcs.  AX is still used when the
                // feed-through is infeasible (no S-LUT anchor, or the DI input
                // would need A6) -- the original "No wire found for port A6"
                // guard -- and unconditionally for a constant DI.
                bool di_remote_real = di_remote && !is_const(c4_di);
                int di_feed_inputs = int(unique_lut_inputs.size());
                if (c4_di && !unique_lut_inputs.count(c4_di->name))
                    di_feed_inputs++;
                // A full 6-input S-LUT (I5 connected) occupies all of A1..A6, so
                // the paired 5LUT has no free pin for a feed-through's input --
                // the unique-input count (loop above only tallies I0..I4) can
                // undercount and call it a fit, but golden delivers such a DI via
                // AX, not O5.  Treat a 6-input S-LUT as "won't fit".
                bool s_lut_full6 = s_lut != nullptr &&
                                   get_net_or_empty(s_lut, ctx->id("I5")) != nullptr;
                bool di_feed_fits = (s_lut != nullptr) && di_feed_inputs <= 5 && !s_lut_full6;
                // When no O5 feed-through can fit (!di_feed_fits: the slot has a
                // full 6-input S-LUT, a thin LUT1 S-routethru, or no S-LUT to
                // anchor it), an O5->DI path is physically impossible -- forcing
                // one spills the feed-through's input onto the nonexistent 5LUT
                // A6 pin ("No wire found for port A6", e.g. the SGMII elastic-
                // buffer wr_occupancy carries).  Route such a DI via the AX
                // bypass too, not just constant/remote DIs.
                if (prev->attrs.count(ctx->id("BEL")) && !di_lut && c4_di &&
                    (is_const(c4_di) || di_remote || !di_feed_fits) &&
                    !(z == 0 && dyn_cyinit) &&
                    !(di_remote_real && di_feed_fits)) {
                    // Imported (BEL-pinned) placement with a CONSTANT DI
                    // (GND/VCC).  Realise it by routing the constant in via the
                    // slot's bypass pin (xCY0=AX) -- the reliable path the
                    // wide-LUT slots already use successfully.
                    //
                    // The alternative (a const-0 feed-through LUT constrained
                    // into THIS slot's free 5LUT, O5->DI internal) is fragile:
                    // whenever the slot's 5LUT can't legally share inputs with a
                    // const-0 LUT -- a BEL-pinned 6-input S-LUT, a thin LUT1
                    // S-routethru, OR simply no S-LUT to anchor the slot -- the
                    // validity check relocates the feed-through to a REMOTE
                    // slice, and the intended O5->DI path becomes an unroutable
                    // cross-slice arc ("Failed to route ... C6LUT_O6 ->
                    // DCY0_OUT").  Going via AX needs no same-slot 5LUT at all,
                    // so it is immune to every one of those collisions.  A
                    // const DI is don't-care-routed-to-0 either way, so this is
                    // functionally identical to golden's xCY0=O5 encoding.
                    di_via_ax = true;
                }
                if (!di_lut && c4_di && !di_via_ax) {
                    PortRef pr; pr.cell = prev;
                    pr.port = ctx->id("DI[" + zs + "]");
                    auto di_feed = feed_through_lut(c4_di, {pr});
                    di_lut = di_feed.get();
                    if (ctx->debug || getenv("DBG_DIFEED"))
                        log_info("DIFEED %s for %s.DI[%s] di_net=%s prevBEL=%s\n", ctx->nameOf(di_lut),
                                 ctx->nameOf(prev), zs.c_str(), ctx->nameOf(c4_di),
                                 prev->attrs.count(ctx->id("BEL")) ?
                                   prev->attrs.at(ctx->id("BEL")).as_string().c_str() : "NONE");
                    new_cells.push_back(std::move(di_feed));
                }
                // Cells with absolute BEL pins (imported Vivado placement)
                // must not be chain-constrained: Vivado legally places some
                // carry feeder LUTs in NEIGHBOUR slices, so a relative
                // constraint can never be satisfied and the legaliser would
                // relocate the whole pinned chain.
                if (s_lut && s_lut->attrs.count(ctx->id("BEL")))
                    s_lut = nullptr;
                if (di_lut && di_lut->attrs.count(ctx->id("BEL")))
                    di_lut = nullptr;
                // When the segment carry itself is BEL-pinned (imported
                // placement), anchor created feed-throughs to IT with zero
                // offset -- the root-relative row arithmetic (idx + idx/25)
                // mis-phases across HCLK rows depending on the root's
                // position within the clock region.
                CellInfo *anchor = prev->attrs.count(ctx->id("BEL")) ? prev : root;
                int anchor_y = (anchor == prev) ? 0 : constr_y;
                if (getenv("DBG_SFEED") && s_lut == nullptr && c4_s)
                    log_info("SFEED-NULLED %s.S[%d] (s_lut nulled before constrain) prevBEL=%s\n",
                             ctx->nameOf(prev), z,
                             prev->attrs.count(ctx->id("BEL")) ?
                               prev->attrs.at(ctx->id("BEL")).as_string().c_str() : "NONE");
                // A BEL-pinned child (imported placement) already has its z
                // fixed by its BEL attr; adding an ABSOLUTE constr_z assumes the
                // standard carry-slice z base (BEL_CARRY4 = 0xF), but some
                // columns (X43 on xc7vx485t) encode the whole slice at 0x40|z
                // (carry = 0x4F), so the abs-z distance is non-zero and the
                // legaliser aborts ("constraint satisfaction check failed").
                // Leave a pinned child's z UNCONSTR (its BEL binds it); only
                // impose abs-z on nextpnr-inserted (un-pinned) feed-throughs.
                if (s_lut) {
                    anchor->constr_children.push_back(s_lut);
                    s_lut->constr_parent = anchor;
                    s_lut->constr_x = 0;
                    s_lut->constr_y = anchor_y;
                    if (!s_lut->attrs.count(ctx->id("BEL"))) {
                        s_lut->constr_abs_z = true;
                        s_lut->constr_z = (z << 4 | BEL_6LUT);
                    }
                }
                if (di_lut) {
                    anchor->constr_children.push_back(di_lut);
                    di_lut->constr_parent = anchor;
                    di_lut->constr_x = 0;
                    di_lut->constr_y = anchor_y;
                    if (!di_lut->attrs.count(ctx->id("BEL"))) {
                        di_lut->constr_abs_z = true;
                        di_lut->constr_z = (z << 4 | BEL_5LUT);
                    }
                }

                // Co-locate the sum FF into this carry slice (Vivado's pattern
                // for registered adders/counters: O[i] -> FFMUX.XOR -> the
                // AFF/BFF/CFF/DFF in the carry's OWN slice).  Without this,
                // nextpnr places the sum FF in a foreign slice and exports the
                // sum via xOUTMUX -> fabric -> bypass-X, which on V7 leaves the
                // carry DI/sum mis-delivered and the counter non-functional
                // (the sum never returns to drive S correctly).  Constrain the
                // single FF whose D is driven by this O[z] with the SAME anchor
                // / abs-z pattern as the S/DI LUTs above -> BEL_FF of lane z.
                // (Constraint is set on the FDRE here; pack_ffs xforms it in
                // place to SLICE_FFX preserving constr_*, and pack_lutffs skips
                // already-constrained FFs.)
                // Sum-FF co-location is DEFAULT-ON: it is a no-op for imported
                // (BEL-pinned) placements -- the guard below only adopts a plain,
                // un-pinned, un-constrained, single-O[z]-sink FF -- so it never
                // perturbs the stamped flows, and it is exactly what fresh-synth
                // registered counters/adders need (sum returns intra-slice to
                // drive S).  Opt out with NEXTPNR_NO_CARRY_COUNTER_FIX.
                NetInfo *c4_o = get_net_or_empty(prev, ctx->id("O[" + zs + "]"));
                if (!getenv("NEXTPNR_NO_CARRY_COUNTER_FIX") && c4_o != nullptr) {
                    CellInfo *sum_ff = nullptr;
                    bool ambiguous = false;
                    for (auto &u : c4_o->users) {
                        if (u.port != ctx->id("D"))
                            continue;
                        if (!ff_types.count(u.cell->type))
                            continue;
                        if (sum_ff != nullptr && sum_ff != u.cell) {
                            ambiguous = true;
                            break;
                        }
                        sum_ff = u.cell;
                    }
                    // Only adopt a plain, un-pinned, un-constrained FF whose D
                    // is THIS O[z] (single FF sink).  Leave anything exotic
                    // (multiple FF sinks, pinned, already clustered) to the
                    // generic packer.
                    //
                    // Control-set guard: all FFs co-located into one carry
                    // slice share the slice's LSR (the legaliser rejects a
                    // half-tile whose FFs carry different clk/sr/ce).  An
                    // adder whose sum FFs use different resets would make the
                    // anchor slice permanently invalid and the whole carry
                    // chain unplaceable; skip such FFs (the generic packer
                    // then places them in a control-set-compatible slice).
                    if (sum_ff != nullptr && !ambiguous &&
                        !sum_ff->attrs.count(ctx->id("BEL")) &&
                        sum_ff->constr_parent == nullptr &&
                        sum_ff->constr_children.empty()) {
                        // Full control-set key: FFs sharing a carry slice
                        // must agree on clk, ce, set/reset nets AND their
                        // inversion/latch/sync flavours (the legaliser checks
                        // all of them per half-tile).
                        auto ctrl_key = [&](CellInfo *ff) {
                            std::vector<IdString> key;
                            for (IdString p : {ctx->id("C"), ctx->id("CLK"), ctx->id("CE"), ctx->id("R"),
                                               ctx->id("S"), ctx->id("CLR"), ctx->id("PRE")}) {
                                NetInfo *n = get_net_or_empty(ff, p);
                                key.push_back(n ? n->name : IdString());
                            }
                            for (IdString p : {ctx->id("IS_CLK_INVERTED"), ctx->id("IS_R_INVERTED"),
                                               ctx->id("IS_S_INVERTED"), ctx->id("IS_PRE_INVERTED"),
                                               ctx->id("IS_CLR_INVERTED")})
                                key.push_back(bool_or_default(ff->params, p, false) ? ctx->id("$inv") : IdString());
                            if (ff->attrs.count(ctx->id("X_FF_AS_LATCH")))
                                key.push_back(ctx->id("$latch"));
                            if (ff->attrs.count(ctx->id("X_FFSYNC")))
                                key.push_back(ctx->id("$sync"));
                            return key;
                        };
                        bool sr_ok = true;
                        auto new_key = ctrl_key(sum_ff);
                        for (auto child : anchor->constr_children)
                            if ((child->constr_z & 0xF) == BEL_FF && ctrl_key(child) != new_key)
                                sr_ok = false;
                        if (!sr_ok) {
                            if (getenv("DBG_CARRYFF"))
                                log_info("CARRYFF SKIP %s for %s.O[%d] ctrlset-mismatch\n",
                                         ctx->nameOf(sum_ff), ctx->nameOf(prev), z);
                        } else {
                        anchor->constr_children.push_back(sum_ff);
                        sum_ff->constr_parent = anchor;
                        sum_ff->constr_x = 0;
                        sum_ff->constr_y = anchor_y;
                        sum_ff->constr_abs_z = true;
                        sum_ff->constr_z = (z << 4 | BEL_FF);
                        if (getenv("DBG_CARRYFF"))
                            log_info("CARRYFF %s -> %s.O[%d] @ lane %d (z<<4|FF)\n",
                                     ctx->nameOf(sum_ff), ctx->nameOf(prev), z, z);
                        }
                    } else if (getenv("DBG_CARRYFF") && sum_ff != nullptr) {
                        log_info("CARRYFF SKIP %s for %s.O[%d] amb=%d pinned=%d constr=%d\n",
                                 ctx->nameOf(sum_ff), ctx->nameOf(prev), z, ambiguous,
                                 (int)sum_ff->attrs.count(ctx->id("BEL")),
                                 (int)(sum_ff->constr_parent != nullptr));
                    }
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
            if (next->attrs.count(ctx->id("BEL"))) {
                // pinned chain segment: placement comes from the import
                prev = next;
                continue;
            }
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

    // (CARRY4 DI=GND is omitted at the per-bit handling below: the DI mux defaults
    // to 0 when left unrouted, so no GND route/LUT is needed -- matches golden.)

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
            bool cyinit_dyn   = cyinit != nullptr && cyinit->name != gnd && cyinit->name != vcc;
            // A Vivado chain root often has BOTH CIN=GND and CYINIT=<real net>
            // (the PRECYINIT mux selects the AX-routed CYINIT; the GND CIN is
            // vestigial).  Treating that GND CIN as a "constant carry-in" here
            // stamped PRECYINIT_CONST=0, which fasm.cc prioritises over the
            // dynamic-CYINIT AX path -> silicon carry-in tied to 0 -> every
            // Vivado down-counter / borrow-comparator chain dead (uart baud
            // counters, busy-polls, branch compares...).  Only realize a
            // constant when there is NO dynamic CYINIT.
            if (!cin_chain && !cyinit_dyn && (cyinit_const || cin_const)) {
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

// Carry-O fabric-fanout relocation for ALU chains.
//
// A chain sub-slice whose carry SUM (O) AND carry-out (CO) both have fabric
// fanout is unplaceable: both need the subslice's single output path
// (xc7_logic_tile_valid's mux_output_used / co_uses_mux contention check;
// router-confirmed unroutable -- 51 overused wires when relaxed).  The
// yosys/ABC $alu decomposition produces such taps (e.g. the MAC datapath of
// ddr3-test-arty-s7).  Restructure:
//
//  - the sum is S ^ CIN; for a non-head bit its CIN is an internal carry
//    (not fabric-reachable), so SPLIT the CARRY4 at the contended bit: the
//    bit becomes the head of a new CARRY4 whose CIN is the previous carry
//    (now a fabric net);
//  - duplicate the sum into an ordinary LUT (S ^ CIN, 2-input XOR) and
//    re-point the O net's fabric users at it; the CARRY4 O pin keeps only
//    the local sum-FF (XOR-direct path).
//
// The duplicate LUT is unconstrained, so the general placer puts it in any
// slice with a free output mux.  Opt out with NEXTPNR_NO_CARRY_O_RELOC.
void XC7Packer::relocate_carry_o_fabric()
{
    if (getenv("NEXTPNR_NO_CARRY_O_RELOC"))
        return;
    if (!ctx->nets.count(ctx->id("$PACKER_GND_NET")) || !ctx->nets.count(ctx->id("$PACKER_VCC_NET")))
        return;
    NetInfo *gnd = ctx->nets.at(ctx->id("$PACKER_GND_NET")).get();
    NetInfo *vcc = ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get();
    static const std::unordered_set<IdString> ff_types{
        ctx->id("FDRE"), ctx->id("FDSE"), ctx->id("FDCE"), ctx->id("FDPE"),
        ctx->id("FDRSE")};
    int relocated = 0, split_carries = 0, skipped = 0;

    // bit = GLOBAL bit index within the original 4-bit CARRY4.  Every cell
    // (original or split) keeps global port indices 0..3: the COUT->CIN
    // spine only exists between physical CO3 and CIN, so a split cell with
    // ports renumbered from 0 puts its carry exit on CO(3-off), which has
    // no route to the next cell's CIN (seen as an unroutable CO2->CIN arc
    // on litex ddr3).  Lanes outside a cell's real bit range are
    // pass-through filled instead.
    auto pname = [&](const char *p, int bit) {
        return ctx->id(std::string(p) + std::to_string(bit));
    };
    auto o_has_fabric = [&](CellInfo *c4, int bit) {
        // ANY user (sum FFs included): a lone FF is only mux-free when it
        // lands in the carry's own subslice, which the placer is not
        // guaranteed to honour after a split -- and the validity check
        // re-evaluates the carry bit against both eighths of the pair.
        NetInfo *o = get_net_or_empty(c4, pname("O", bit));
        return o != nullptr && !o->users.empty();
    };
    auto co_has_fabric = [&](CellInfo *c4, int bit) {
        NetInfo *co = get_net_or_empty(c4, pname("CO", bit));
        if (co == nullptr)
            return false;
        if (bit == 3) { // physical COUT: CIN users ride the spine
            for (auto &u : co->users)
                if (u.port != ctx->id("CIN"))
                    return true;
            return false;
        }
        return !co->users.empty();
    };
    auto next_in_chain = [&](CellInfo *c4) -> CellInfo * {
        for (int b = 0; b < 4; b++) {
            NetInfo *co = get_net_or_empty(c4, ctx->id("CO" + std::to_string(b)));
            if (!co)
                continue;
            for (auto &u : co->users)
                if (u.cell->type == ctx->id("CARRY4") && u.port == ctx->id("CIN"))
                    return u.cell;
        }
        return nullptr;
    };

    // Duplicate the sum (S ^ CIN) into a LUT; the O net keeps only FF sinks.
    auto relocate_sum = [&](CellInfo *c4, int bit, NetInfo *cin_net, CellInfo *root) {
        NetInfo *s = get_net_or_empty(c4, pname("S", bit));
        NetInfo *o = get_net_or_empty(c4, pname("O", bit));
        if (s == nullptr || cin_net == nullptr || o == nullptr)
            return;
        std::unique_ptr<NetInfo> sum_copy{new NetInfo};
        sum_copy->name = ctx->id(o->name.str(ctx) + "$sum$" + std::to_string(++autoidx));
        std::unique_ptr<CellInfo> lut =
                create_lut(ctx, o->name.str(ctx) + "$sumLUT$" + std::to_string(++autoidx),
                           {s, cin_net}, sum_copy.get(), Property(6));
        // Re-point EVERY user (including any sum-FF: xc7_logic_tile_valid
        // re-evaluates the carry bits against both slice halves (eighths
        // 0..3 and 4..7), and a lone FF on the O net claims the output mux
        // against the other half's null ff1 -- leaving the CARRY4 O pin
        // with an empty net avoids all claims).  The sum LUT computes the
        // identical S^CIN value.
        std::vector<PortRef> users(o->users.begin(), o->users.end());
        int moved = 0;
        for (auto &u : users) {
            disconnect_port(ctx, u.cell, u.port);
            connect_port(ctx, sum_copy.get(), u.cell, u.port);
            moved++;
        }
        // A moved sum-FF must leave the cluster too: anchored in the
        // carry's own lane, its D would now have to arrive from the fabric
        // sum LUT through the slice's input mux, whose only fabric path
        // (the lane's X bypass) can be claimed by the carry's DI --
        // leaving the arc unroutable (seen on litex ddr3 as a failed
        // route into DFFMUX_OUT).  Like the sum LUT itself, the general
        // placer finds them a routable home.
        for (auto &u : users) {
            if (!(u.port == ctx->id("D") && ff_types.count(u.cell->type)))
                continue;
            CellInfo *ff = u.cell;
            if (ff->constr_parent != root)
                continue;
            root->constr_children.erase(
                    std::remove(root->constr_children.begin(), root->constr_children.end(), ff),
                    root->constr_children.end());
            ff->constr_parent = nullptr;
            ff->constr_x = ff->UNCONSTR;
            ff->constr_y = ff->UNCONSTR;
            ff->constr_z = ff->UNCONSTR;
            ff->constr_abs_z = false;
        }
        // Leave the sum LUT unconstrained: the general placer puts it in any
        // slice with a free output mux (the validity check rejects carry
        // tiles), while a fixed relative row can collide with another chain's
        // absolute position and displace one of its LUTs.
        IdString nname = sum_copy->name;
        ctx->nets[nname] = std::move(sum_copy);
        new_cells.push_back(std::move(lut));
        relocated++;
        if (getenv("DBG_CARRYO"))
            log_info("CARRO %s.O%d -> sum LUT (S^CIN), %d fabric users moved\n",
                     ctx->nameOf(c4), bit, moved);
    };

    // Split c4 at global bit i: bits i..3 move to a new full-width CARRY4
    // whose CIN carries the old CO[i-1] value via a dedicated COUT->CIN
    // link.  Both cells pass the carry through their empty lanes with
    // S=VCC: in the MUXCY, CO = S ? CIN : DI, so S must be HIGH to
    // propagate (the earlier S=GND fill selected DI and silently zeroed
    // the carry -- O, not CO, is what passes CIN when S is low).  Moved
    // LUT/FF constraints are re-anchored onto the new CARRY4 (rows
    // assigned by the chain renumber).
    auto split_at = [&](CellInfo *c4, int i, CellInfo *root) -> CellInfo * {
        std::unique_ptr<CellInfo> nb{new CellInfo};
        nb->name = ctx->id(c4->name.str(ctx) + "$split$" + std::to_string(++autoidx));
        nb->type = ctx->id("CARRY4");
        auto declare = [&](IdString name, PortType t) {
            nb->ports[name].name = name;
            nb->ports[name].type = t;
        };
        declare(ctx->id("CIN"), PORT_IN);
        for (int b = 0; b < 4; b++) {
            for (const char *p : {"S", "DI"})
                declare(pname(p, b), PORT_IN);
            for (const char *p : {"O", "CO"})
                declare(pname(p, b), PORT_OUT);
        }
        // Move global bits i..3 FIRST -- the range includes CO3, so the
        // COUT->CIN link must only be created once the move is done.
        // (Creating it before let the move loop steal the link net back
        // onto the new cell: a CO->CIN self-loop, a driverless chain net,
        // and a broken CO->CIN walk that left every split row-less -- the
        // legaliser livelock on the litex ddr3 designs.  On a re-split the
        // steal becomes the WANTED effect: CO3 holds the link to the upper
        // split, and moving it re-chains it through the new cell.)
        for (int b = i; b < 4; b++) {
            for (const char *p : {"S", "DI", "O", "CO"}) {
                IdString port = pname(p, b);
                NetInfo *net = get_net_or_empty(c4, port);
                disconnect_port(ctx, c4, port);
                if (net != nullptr)
                    connect_port(ctx, net, nb.get(), port);
            }
        }
        // Dedicated COUT->CIN spine link: the model has no fabric pips into
        // CIN, so a link via the old CO[i-1] fabric net is unroutable.
        std::unique_ptr<NetInfo> link{new NetInfo};
        link->name = ctx->id(c4->name.str(ctx) + "$carry$" + std::to_string(++autoidx));
        connect_port(ctx, link.get(), c4, ctx->id("CO3"));
        connect_port(ctx, link.get(), nb.get(), ctx->id("CIN"));
        IdString lname = link->name;
        ctx->nets[lname] = std::move(link);
        // pass-through fills: c4's freed head i..3 carries CO[i-1] up to its
        // CO3; nb's empty tail 0..i-1 carries its CIN up to bit i
        for (int b = i; b < 4; b++) {
            connect_port(ctx, vcc, c4, pname("S", b));
            connect_port(ctx, gnd, c4, pname("DI", b));
        }
        for (int b = 0; b < i; b++) {
            connect_port(ctx, vcc, nb.get(), pname("S", b));
            connect_port(ctx, gnd, nb.get(), pname("DI", b));
        }
        // re-anchor the moved LUT/FF constraints onto the new CARRY4
        for (int b = i; b < 4; b++) {
            for (const char *p : {"S", "DI"}) {
                NetInfo *net = get_net_or_empty(nb.get(), pname(p, b));
                if (net == nullptr || net->driver.cell == nullptr)
                    continue;
                CellInfo *drv = net->driver.cell;
                if (drv->constr_parent == nullptr)
                    continue;
                drv->constr_z = (b << 4) | (drv->constr_z & 0xF);
                drv->constr_y = 0;
            }
            NetInfo *o = get_net_or_empty(nb.get(), pname("O", b));
            if (o != nullptr) {
                for (auto &u : o->users) {
                    if (!(u.port == ctx->id("D") && ff_types.count(u.cell->type)))
                        continue;
                    if (u.cell->constr_parent == nullptr)
                        continue;
                    // Force the FF slot: the pre-split anchor may carry an
                    // unrelated low nibble (e.g. a RSTINV slot from an early
                    // adoption) that must not leak into the new lane.
                    u.cell->constr_z = (b << 4) | BEL_FF;
                    u.cell->constr_y = 0;
                }
            }
        }
        nb->constr_parent = root;
        root->constr_children.push_back(nb.get());
        nb->constr_x = 0;
        nb->constr_abs_z = true;
        nb->constr_z = BEL_CARRY4;
        CellInfo *ret = nb.get();
        new_cells.push_back(std::move(nb));
        return ret;
    };

    // Find chain roots (CIN not driven by another CARRY4's CO).
    std::vector<CellInfo *> roots;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type != ctx->id("CARRY4"))
            continue;
        NetInfo *cin = get_net_or_empty(ci, ctx->id("CIN"));
        bool is_root = (cin == nullptr) || (cin->driver.cell == nullptr) ||
                       (cin->driver.cell->type != ctx->id("CARRY4"));
        if (is_root)
            roots.push_back(ci);
    }

    for (auto *root : roots) {
        if (root->attrs.count(ctx->id("BEL")))
            continue; // imported (Vivado-proven) placement: leave alone
        // NOTE: roots deliberately keep constr_x/y UNCONSTR.  An earlier
        // revision pinned each root to a distinct row via
        // constr_y = -(chain_seq + chain_seq/25), but on a parentless cell
        // constr_y is an ABSOLUTE grid position (place_common's
        // get_constraints_distance) -- a negative row can never be
        // satisfied, so the final constraint check failed the moment
        // placement completed.  The overlap scenario the pin targeted
        // (a LUT shared between two chains, adopted by one and bound by
        // the other's BFS) is handled by the legaliser's leaf relocation
        // instead.
        // work items: (CARRY4, global bit offset)
        std::vector<std::pair<CellInfo *, int>> work;
        for (CellInfo *c = root; c != nullptr; c = next_in_chain(c))
            work.emplace_back(c, 0);
        while (!work.empty()) {
            auto [c4, off] = work.back();
            work.pop_back();
            if (c4->attrs.count(ctx->id("BEL")))
                continue;
            for (int i = 3; i >= off; i--) {
                if (!o_has_fabric(c4, i) || !co_has_fabric(c4, i))
                    continue;
                if (i == off) {
                    NetInfo *cin = get_net_or_empty(c4, ctx->id("CIN"));
                    if (cin != nullptr)
                        relocate_sum(c4, i, cin, root);
                } else {
                    // Includes i == 3: a one-bit split makes bit 3 a new chain
                    // head whose CIN (the old CO[2]) becomes fabric-reachable.
                    NetInfo *co_prev = get_net_or_empty(c4, pname("CO", i - 1));
                    if (co_prev == nullptr)
                        continue;
                    CellInfo *c4b = split_at(c4, i, root);
                    split_carries++;
                    relocate_sum(c4b, i, co_prev, root);
                    work.emplace_back(c4b, i);
                }
            }
        }
        // renumber the chain's row offsets (HCLK-skip formula, as pack_carries)
        int idx = 0;
        for (CellInfo *c = root; c != nullptr; c = next_in_chain(c)) {
            if (c == root) {
                idx++;
                continue; // root keeps UNCONSTR x/y (only z is pinned)
            }
            if (c->attrs.count(ctx->id("BEL"))) {
                idx++;
                continue;
            }
            c->constr_y = -(idx + idx / 25);
            idx++;
        }
        // Lane members follow their carry by NETLIST relation (the S/DI
        // driver LUTs and O-sink FFs clustered under this root), never by
        // matching y values: split lanes and original chain rows live in
        // the same small negative integers, so any value-matching shift
        // cross-assigns lanes between owners (observed on litex ddr3:
        // lanes pinned to rows -7/-8 where no CARRY4 sits, and two LUTs
        // booked onto one (x,y,z) -- unroutable carry S inputs).
        for (CellInfo *c = root; c != nullptr; c = next_in_chain(c)) {
            if (c->attrs.count(ctx->id("BEL")))
                continue;
            int row_y = (c == root) ? 0 : c->constr_y;
            for (int b = 0; b < 4; b++) {
                for (const char *p : {"S", "DI"}) {
                    NetInfo *net = get_net_or_empty(c, ctx->id(std::string(p) + std::to_string(b)));
                    if (net == nullptr || net->driver.cell == nullptr)
                        continue;
                    CellInfo *drv = net->driver.cell;
                    if (drv->constr_parent != root || drv->type == ctx->id("CARRY4"))
                        continue;
                    drv->constr_y = row_y;
                }
                NetInfo *o = get_net_or_empty(c, ctx->id("O" + std::to_string(b)));
                if (o == nullptr)
                    continue;
                for (auto &u : o->users) {
                    if (!(u.port == ctx->id("D") && ff_types.count(u.cell->type)))
                        continue;
                    if (u.cell->constr_parent != root)
                        continue;
                    u.cell->constr_y = row_y;
                }
            }
        }
    }

    if (relocated || split_carries || skipped)
        log_info("   Carry-O relocation: %d sum(s) duplicated, %d CARRY4 split(s), %d bit(s) skipped (unrelocatable)\n",
                 relocated, split_carries, skipped);

    flush_cells();
    if (getenv("DBG_CARRYO")) {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type != ctx->id("CARRY4"))
                continue;
            std::string ports;
            for (int b = 0; b < 4; b++) {
                NetInfo *o = get_net_or_empty(ci, ctx->id("O" + std::to_string(b)));
                NetInfo *co = get_net_or_empty(ci, ctx->id("CO" + std::to_string(b)));
                ports += " [b" + std::to_string(b) + " O=";
                if (o) {
                    ports += std::to_string(o->users.size());
                    for (auto &u : o->users)
                        ports += u.cell->type.str(ctx) + " ";
                } else
                    ports += "-";
                ports += " CO=";
                if (co) {
                    ports += std::to_string(co->users.size());
                    for (auto &u : co->users)
                        ports += u.cell->type.str(ctx) + " ";
                } else
                    ports += "-";
                ports += "]";
            }
            log_info("CARRO-C4 %s%s\n", ctx->nameOf(ci), ports.c_str());
            log_info("CARRO-C4P %s parent=%d nchild=%d y=%d\n", ctx->nameOf(ci),
                     int(ci->constr_parent != nullptr), int(ci->constr_children.size()), ci->constr_y);
            for (int b = 0; b < 4; b++) {
                NetInfo *o = get_net_or_empty(ci, ctx->id("O" + std::to_string(b)));
                if (!o) continue;
                for (auto &u : o->users) {
                    if (!(u.port == ctx->id("D") && (u.cell->type == ctx->id("FDRE") || u.cell->type == ctx->id("FDSE") ||
                                                      u.cell->type == ctx->id("FDCE") || u.cell->type == ctx->id("FDPE"))))
                        continue;
                    log_info("CARRO-FF %s.O%d -> %s parent=%d y=%d z=%x\n", ctx->nameOf(ci), b, ctx->nameOf(u.cell),
                             int(u.cell->constr_parent != nullptr), u.cell->constr_y, u.cell->constr_z);
                }
            }
        }
    }

}

NEXTPNR_NAMESPACE_END