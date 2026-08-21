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

#include <boost/algorithm/string.hpp>
#include <queue>
#include "design_utils.h"
#include "log.h"
#include <cstdlib>
#include "nextpnr.h"
#include "timing.h"
#include "util.h"
NEXTPNR_NAMESPACE_BEGIN

inline NetInfo *port_or_nullptr(const CellInfo *cell, IdString name)
{
    auto found = cell->ports.find(name);
    if (found == cell->ports.end())
        return nullptr;
    return found->second.net;
}

//#define DEBUG_VALIDITY
// Set NEXTPNR_DBG_VALIDITY=1 to have every rejected slice name the rule that
// refused it (DBG()/DBG_RT() print __LINE__).  The flag existed but nothing
// ever set it, so "post-placement validity check failed for Bel X (no cell)"
// was unactionable -- it says a tile is illegal without saying which of the
// dozen rules in this file objected.
bool dbg_validity_runtime = getenv("NEXTPNR_DBG_VALIDITY") != nullptr;
static bool packing_oracle();
#define DBG_RT()                                                                                                       \
    do {                                                                                                               \
        if (dbg_validity_runtime)                                                                                      \
            log_info("invalid-arm: line %d\n", __LINE__);                                                             \
    } while (0)
#ifdef DEBUG_VALIDITY
#define DBG() log_info("invalid: %s %d\n", __FILE__, __LINE__)
#else
#define DBG() DBG_RT()
#endif

bool Arch::xcu_logic_tile_valid(IdString tileType, LogicTileStatus &lts) const
{
    bool is_slicem = (tileType == id_CLEM) || (tileType == id_CLEM_R);
    bool tile_is_memory = false;
    if (lts.cells[(7 << 4) | BEL_6LUT] != nullptr && lts.cells[(7 << 4) | BEL_6LUT]->lutInfo.is_memory)
        tile_is_memory = true;
    bool small_memory = false;
    if (lts.cells[(7 << 4) | BEL_5LUT] != nullptr && lts.cells[(7 << 4) | BEL_5LUT]->lutInfo.is_memory)
        small_memory = true;
    // Check eight-tiles (mostly LUT-related validity)
    for (int i = 0; i < 8; i++) {
        if (lts.eights[i].dirty) {
            lts.eights[i].dirty = false;
            lts.eights[i].valid = false;

            CellInfo *lut6 = lts.cells[(i << 4) | BEL_6LUT];
            CellInfo *lut5 = lts.cells[(i << 4) | BEL_5LUT];

            // Check 6LUT
            if (lut6 != nullptr) {
                if (!is_slicem && (lut6->lutInfo.is_memory || lut6->lutInfo.is_srl))
                    return false; // Memory and SRLs only valid in SLICEMs
                if (lut5 != nullptr) {
                    // Can't mix memory and non-memory
                    if (lut6->lutInfo.is_memory != lut5->lutInfo.is_memory ||
                        lut6->lutInfo.is_srl != lut5->lutInfo.is_srl)
                        return false;
                    // If all 6 inputs or 2 outputs are used, 5LUT can't also be present
                    if (lut6->lutInfo.input_count == 6 || lut6->lutInfo.output_count == 2)
                        return false;
                    // If more than 5 total inputs are used, need to check number of shared input
                    if ((lut6->lutInfo.input_count + lut5->lutInfo.input_count) > 5) {
                        int shared = 0, need_shared = (lut6->lutInfo.input_count + lut5->lutInfo.input_count - 5);
                        for (int j = 0; j < lut6->lutInfo.input_count; j++) {
                            for (int k = 0; k < lut5->lutInfo.input_count; k++) {
                                if (lut6->lutInfo.input_sigs[j] == lut5->lutInfo.input_sigs[k])
                                    shared++;
                                if (shared >= need_shared)
                                    break;
                            }
                        }
                        if (shared < need_shared) {
                            // Imported (Vivado/golden) CARRY4 slot: the operand LUT
                            // (O6->S) and the DI feed-through (O5->DI) are ONE
                            // dual-output LUT on real hardware -- O5 and O6 of the
                            // same 5-input LUT -- so they share all physical inputs
                            // by construction.  SVS/pack represents that as two cells
                            // whose lone feed-through input is a renamed net copy, so
                            // input_sigs no longer match by identity (shared=0 here)
                            // even though the placement is physically legal (golden
                            // uses it and runs).  When the slot's 6LUT is BEL-pinned
                            // (imported) and the 5LUT only drives the carry, trust the
                            // import instead of rejecting a valid slot.
                            bool imported_carry_feedthrough =
                                lut6->attrs.count(id_BEL) && lut5->lutInfo.only_drives_carry;
                            if (!imported_carry_feedthrough) {
                                DBG();
                                return false;
                            }
                        }
                    }
                }
            }
            if (lut5 != nullptr) {
                if (!is_slicem && (lut5->lutInfo.is_memory || lut5->lutInfo.is_srl)) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
                // 5LUT can use at most 5 inputs and 1 output
                if (lut5->lutInfo.input_count > 5 || lut5->lutInfo.output_count == 2) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
            }

            // Check (over)usage of DI and X inputs
            NetInfo *i_net = nullptr, *x_net = nullptr;
            if (lut6 != nullptr) {
                i_net = lut6->lutInfo.di1_net;
                x_net = lut6->lutInfo.di2_net;
            }
            if (lut5 != nullptr) {
                if (lut5->lutInfo.di1_net != nullptr) {
                    if (i_net == nullptr)
                        i_net = lut5->lutInfo.di1_net;
                    else if (i_net != lut5->lutInfo.di1_net) {
                        DBG();
                        return false; // Memory and SRLs only valid in SLICEMs
                    }
                }
                // DI2 not available for 5LUT
                if (lut5->lutInfo.di2_net != nullptr) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
            }

            CellInfo *mux = nullptr;
            // Eights A, C, E, G: F7MUX uses X input
            if (i == 0 || i == 2 || i == 4 || i == 6)
                mux = lts.cells[i << 4 | BEL_F7MUX];
            // Eights B, F: F8MUX uses X input
            if (i == 1 || i == 5)
                mux = lts.cells[(i - 1) << 4 | BEL_F8MUX];
            // Eights D: F9MUX uses X input
            if (i == 3)
                mux = lts.cells[BEL_F9MUX];

            if (mux != nullptr) {
                if (x_net == nullptr)
                    x_net = mux->muxInfo.sel;
                else if (x_net != mux->muxInfo.sel) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
            }

            CellInfo *out_fmux = nullptr;
            // Eights B, D, F, H: F7MUX connects to F7F8 out
            if (i == 1 || i == 3 || i == 5 || i == 7)
                out_fmux = lts.cells[(i - 1) << 4 | BEL_F7MUX];
            // Eights C, G: F8MUX connects to F7F8 out
            if (i == 2 || i == 6)
                out_fmux = lts.cells[(i - 2) << 4 | BEL_F8MUX];
            // Eights E: F9MUX connects to F7F8 out
            if (i == 4)
                out_fmux = lts.cells[BEL_F9MUX];

            CellInfo *carry8 = lts.cells[BEL_CARRY8];
            // CARRY8 might use X
            if (carry8 != nullptr && carry8->carryInfo.x_sigs[i] != nullptr) {
                if (x_net == nullptr)
                    x_net = carry8->carryInfo.x_sigs[i];
                else if (x_net != carry8->carryInfo.x_sigs[i]) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
            }

            // FF1 might use X, if it isn't driven directly
            CellInfo *ff1 = lts.cells[i << 4 | BEL_FF];
            if (ff1 != nullptr && ff1->ffInfo.d != nullptr && ff1->ffInfo.d->driver.cell != nullptr) {
                auto &drv = ff1->ffInfo.d->driver;
                if ((drv.cell == lut6 && drv.port != id_MC31) || drv.cell == lut5 || drv.cell == out_fmux) {
                    // Direct, OK
                    // FIXME: CARRY8 direct
                } else {
                    // Indirect, must use X input
                    if (x_net == nullptr)
                        x_net = ff1->ffInfo.d;
                    else if (x_net != ff1->ffInfo.d) {
                        DBG();
                        return false; // Memory and SRLs only valid in SLICEMs
                    }
                }
            }

            // FF2 might use I, if it isn't driven directly
            CellInfo *ff2 = lts.cells[i << 4 | BEL_FF2];
            if (ff2 != nullptr && ff2->ffInfo.d != nullptr && ff2->ffInfo.d->driver.cell != nullptr) {
                auto &drv = ff2->ffInfo.d->driver;
                if ((drv.cell == lut6 && drv.port != id_MC31) || drv.cell == lut5 || drv.cell == out_fmux) {
                    // Direct, OK
                    // FIXME: CARRY8 direct
                } else {
                    // Indirect, must use X input
                    if (i_net == nullptr)
                        i_net = ff2->ffInfo.d;
                    else if (i_net != ff2->ffInfo.d) {
                        DBG();
                        return false; // Memory and SRLs only valid in SLICEMs
                    }
                }
            }

            // Collision with top address bits
            if (tile_is_memory && !small_memory) {
                CellInfo *top_lut = lts.cells[(7 << 4) | BEL_6LUT];
                if (top_lut != nullptr) {
                    if ((i == 6) && x_net != top_lut->lutInfo.address_msb[0])
                        return false;
                    if ((i == 5) && x_net != top_lut->lutInfo.address_msb[1])
                        return false;
                    if ((i == 3) && x_net != top_lut->lutInfo.address_msb[2])
                        return false;
                }
            }

            bool mux_output_used = false;
            NetInfo *out5 = nullptr;
            if (lut6 != nullptr && lut6->lutInfo.output_count == 2)
                out5 = lut6->lutInfo.output_sigs[1];
            else if (lut5 != nullptr && !lut5->lutInfo.only_drives_carry)
                out5 = lut5->lutInfo.output_sigs[0];
            if (out5 != nullptr && (out5->users.size() > 1 || ((ff1 == nullptr || out5 != ff1->ffInfo.d) &&
                                                               (ff2 == nullptr || out5 != ff2->ffInfo.d)))) {
                mux_output_used = true;
            }

            if (carry8 != nullptr && carry8->carryInfo.out_sigs[i] != nullptr) {
                // FIXME: direct connections to FF
                if (mux_output_used) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
                mux_output_used = true;
            }
            if (out_fmux != nullptr) {
                NetInfo *f7f8 = out_fmux->muxInfo.out;
                if (f7f8 != nullptr && (f7f8->users.size() > 1 || ((ff1 == nullptr || f7f8 != ff1->ffInfo.d) &&
                                                                   (ff2 == nullptr || f7f8 != ff2->ffInfo.d)))) {
                    if (mux_output_used) {
                        DBG();
                        return false; // Memory and SRLs only valid in SLICEMs
                    }
                    mux_output_used = true;
                }
            }

            lts.eights[i].valid = true;
        } else if (!lts.eights[i].valid) {
            return false;
        }
    }
    // Check half-tiles
    for (int i = 0; i < 2; i++) {
        if (lts.halfs[i].dirty) {
            lts.halfs[i].valid = false;
            bool found_ff[2] = {false, false};
            NetInfo *clk = nullptr, *sr = nullptr, *ce[2] = {nullptr};
            bool clkinv = false, srinv = false, islatch = false;
            for (int z = 4 * i; z < 4 * (i + 1); z++) {
                for (int k = 0; k < 2; k++) {
                    CellInfo *ff = lts.cells[z << 4 | (BEL_FF + k)];
                    if (ff == nullptr)
                        continue;
                    if (found_ff[0] || found_ff[1]) {
                        if (ff->ffInfo.clk != clk)
                            return false;
                        if (ff->ffInfo.sr != sr)
                            return false;
                        if (ff->ffInfo.is_clkinv != clkinv)
                            return false;
                        if (ff->ffInfo.is_srinv != srinv)
                            return false;
                        if (ff->ffInfo.is_latch != islatch)
                            return false;
                    } else {
                        clk = ff->ffInfo.clk;
                        sr = ff->ffInfo.sr;
                        clkinv = ff->ffInfo.is_clkinv;
                        srinv = ff->ffInfo.is_srinv;
                        islatch = ff->ffInfo.is_latch;
                    }
                    if (found_ff[k]) {
                        if (ff->ffInfo.ce != ce[k])
                            return false;
                    } else {
                        ce[k] = ff->ffInfo.ce;
                    }
                    found_ff[k] = true;
                }
            }
            lts.halfs[i].valid = true;
        } else if (!lts.halfs[i].valid) {
            return false;
        }
    }
    return true;
}

bool Arch::xc7_logic_tile_valid(IdString tileType, LogicTileStatus &lts) const
{
    // Fully-frozen tile fast-path: if every occupied BEL holds a cell that is
    // still BEL-pinned (imported from a proven Vivado placement), the tile is
    // legal by construction -- Vivado already validated it on silicon.  The
    // downstream heuristics (LUT input/output packing, X-input usage, control
    // sets) model nextpnr's OWN packer and cannot see Vivado's input-pin
    // permutation / shared-IMUX packing, so they reject legal frozen tiles.
    // The bbox exclusion keeps user (non-imported) cells out of macro tiles,
    // so a tile is either all-imported (trust) or contains user logic (full
    // checks).  Any cell WITHOUT a BEL attr forces the full validation.
    {
        bool any = false, all_frozen = true;
        for (int z = 0; z < 128; z++) {
            CellInfo *c = lts.cells[z];
            if (c == nullptr)
                continue;
            any = true;
            // frozen == stamped: bound STRENGTH_USER by the BEL-attr placer.
            // (A BEL *attribute* is NOT a reliable marker -- place_initial
            //  back-annotates one onto every nextpnr-placed user cell too.)
            //
            // The bar is LOCKED, not STRONG.  STRENGTH_STRONG is what nextpnr's
            // OWN cluster/macro placement binds at (place_common.cc place_macro,
            // the radius search below), so accepting it let a self-placed macro
            // count as "imported, trust it".  A dist-RAM macro landing in a tile
            // whose FFs were stamped by place_lef then skipped every control-set
            // check: RAMD64E on one clock co-packed with FDREs on another, in a
            // slice with ONE CLK pin.  Unroutable by construction -- it surfaced
            // as permanently skipped clock arcs (eth-arp: 4 slices of the txf
            // CDC FIFO, u_txf_wr WCLK vs u_txf_rd CK).  Imported placements bind
            // USER (attributesToArchInfo / HeAP / placer1), so they still hit
            // this fast path; nextpnr's own STRONG bindings now get validated.
            if (c->belStrength < STRENGTH_LOCKED) {
                all_frozen = false;
                break;
            }
        }
        if (any && all_frozen)
            return true;
    }
    bool is_slicem = (tileType == id_CLBLM_L) || (tileType == id_CLBLM_R);
    // SLICEM-only guard, run UNCONDITIONALLY (not behind the per-eight dirty
    // cache in the loop below).  A distributed-RAM / SRL LUT (is_memory /
    // is_srl) is legal only in a SLICEM.  The dirty-gated loop leaves this
    // unchecked whenever a tile is validated while clean, so SA / legalisation
    // can strand a memory/SRL LUT in a SLICEL -- surfacing much later as an
    // unroutable "Pin 'DI1'/'WE' of bel ... has no associated wire".  A cheap
    // O(16) scan here closes that hole for both the 6LUT and 5LUT bels.
    if (!is_slicem) {
        for (int i = 0; i < 8; i++) {
            CellInfo *l6 = lts.cells[(i << 4) | BEL_6LUT];
            CellInfo *l5 = lts.cells[(i << 4) | BEL_5LUT];
            if ((l6 != nullptr && (l6->lutInfo.is_memory || l6->lutInfo.is_srl)) ||
                (l5 != nullptr && (l5->lutInfo.is_memory || l5->lutInfo.is_srl))) {
                DBG();
                return false;
                }
        }
    }
    // A 5LUT bel has neither an A6 input nor an O6 output wire; a cell there
    // must not use either.  Enforce UNCONDITIONALLY -- the dirty-gated loop
    // below caches past this, letting legalisation strand a 6-input LUT on a
    // 5LUT ("No wire found for port A6/O6").  CRUCIAL: at placement time an
    // ABC-mapped LUT still carries LOGICAL pin names (I0..I5), so neither a
    // physical id_A6 lookup nor lutInfo.input_count (which scans A1..A6) sees
    // its 6th input -- we must COUNT connected input ports, exactly as
    // isValidBelForCell does.
    for (int i = 0; i < 8; i++) {
        CellInfo *l5 = lts.cells[(i << 4) | BEL_5LUT];
        if (l5 == nullptr)
            continue;
        // Memory/SRL LUTs legitimately draw many ports (address + DI + WE) and
        // are packed to specific 5LUT bels -- the SLICEM guard above already
        // constrains them, so the plain-LUT input-count rule must not apply.
        if (l5->lutInfo.is_memory || l5->lutInfo.is_srl)
            continue;
        int in_used = 0;
        bool drives_o6 = false;
        for (auto &port : l5->ports) {
            if (port.second.net == nullptr)
                continue;
            if (port.second.type == PORT_IN)
                ++in_used;
            else if (port.first == id_O6)
                drives_o6 = true;
        }
        if (in_used > 5 || drives_o6 || l5->lutInfo.output_count == 2 ||
            get_net_or_empty(l5, id_A6) != nullptr || get_net_or_empty(l5, id_O6) != nullptr) {
            DBG();
            return false;
            }
    }
    bool tile_is_memory = false;
    if (lts.cells[(3 << 4) | BEL_6LUT] != nullptr && lts.cells[(3 << 4) | BEL_6LUT]->lutInfo.is_memory)
        tile_is_memory = true;
    bool small_memory = false;
    if (lts.cells[(3 << 4) | BEL_5LUT] != nullptr && lts.cells[(3 << 4) | BEL_5LUT]->lutInfo.is_memory)
        small_memory = true;
    NetInfo *wclk = nullptr;
    // A MUXF8 belongs on an F8MUX bel and a MUXF7 on F7AMUX/F7BMUX.  This
    // repeats the gate in isValidBelForCell ON PURPOSE: that one only filters
    // INITIAL candidates, and SA swaps and legalisation go around it -- exactly
    // as the 5LUT and SLICEM comments in this file record.  With the rule
    // stated only there, johnson's MUXF8 roots drifted back onto F7BMUX bels
    // during annealing and took their clusters apart across three slices each:
    //   root SLICE_X124Y216/F7BMUX kids=6 -> X122Y216, X124Y216, X125Y216
    // which the router then cannot close, e.g.
    //   Failed to route arc 0 ... from SLICE_X124Y215/F7BMUX_OUT
    //                              to SLICE_X122Y215/C6LUT_O6
    // (a MUXF7 has to be fed by the two LUTs of its own slice).
    // Only xc7 needs this: other families keep MUXF7/MUXF8 as distinct cell
    // types, so the type check alone is sufficient there.
    if (xc7) {
        // A mux cluster must not straddle slices.  The packer builds the whole
        // F7/F7/F8 tree plus its four LUTs as one constrained cluster, and the
        // constraints themselves are right (X=0 Y=0, Z relative: -1 for the
        // F7AMUX, +31 for the F7BMUX, off the F8MUX's z=8).  Even so ONE child
        // was observed escaping -- SLICE_X125Y217/F7BMUX under a parent at
        // SLICE_X123Y217/F8MUX, five of six siblings correct -- with no
        // "chain child pre-bind failed" warning, so the placer believed it was
        // legal.  A MUXF7 can only be fed by the two LUTs of its own slice, so
        // the router then fails:
        //   Failed to route arc 0 ... from SLICE_X123Y217/D6LUT_O6
        //                              to SLICE_X125Y217/D6LUT_O6
        // Rejecting the straddle here makes the state unreachable rather than
        // relying on every placement path to maintain it.
        // Every slot, not just the mux ones.  A LUT->FF pair is a cluster too
        // (constr_x/y both 0, so the flop must share its LUT's slice for the
        // O6->D path), and SA's LEGALISATION pass moves constrained cells
        // individually on purpose -- try_swap_position only refuses them while
        // `!require_legal`.  With the rule stated for muxes alone, a LUT+FF
        // pair split across rows survived to the router:
        //   child  SLICE_FFX  SLICE_X125Y217/AFF
        //   parent SLICE_LUTX SLICE_X125Y216/A6LUT
        // johnson had merely been lucky in the ordering until now.
        for (int i = 0; i < 8; i++) {
            for (int mt : {BEL_F7MUX, BEL_F8MUX, BEL_6LUT, BEL_5LUT, BEL_FF, BEL_FF2}) {
                CellInfo *mc = lts.cells[(i << 4) | mt];
                if (mc == nullptr || mc->belStrength >= STRENGTH_USER)
                    continue;
                // UNCONSTR means NO positional constraint -- it does NOT mean
                // "offset zero".  Conflating the two made this rule demand that
                // any child with a parent share the parent's tile, even when
                // nothing had asked for that, and it rejected a perfectly good
                // carry slice: SLICE_X169Y172 held a complete, correctly packed
                // CARRY4 (count_cycle LUTs, GND di_luts on the 5LUT slots, FFs,
                // CARRY4 at z=79) and was refused anyway.  Only enforce
                // co-location where the constraint EXPLICITLY says offset 0.
                auto tile_local = [](const CellInfo *c) {
                    return c->constr_x == 0 && c->constr_y == 0;
                };
                CellInfo *par = mc->constr_parent;
                if (par != nullptr && par->bel != BelId() && mc->bel != BelId() && tile_local(mc)) {
                    Loc pl = getBelLocation(par->bel), cl = getBelLocation(mc->bel);
                    if (pl.x != cl.x || pl.y != cl.y) {
                        DBG();
                        return false;
                    }
                }
                for (auto ch : mc->constr_children) {
                    if (ch->bel == BelId() || mc->bel == BelId() || !tile_local(ch))
                        continue;
                    Loc chl = getBelLocation(ch->bel), cl = getBelLocation(mc->bel);
                    if (chl.x != cl.x || chl.y != cl.y) {
                        DBG();
                        return false;
                    }
                }
            }
        }
        for (int i = 0; i < 8; i++) {
            CellInfo *m7 = lts.cells[(i << 4) | BEL_F7MUX];
            if (m7 != nullptr && m7->belStrength < STRENGTH_USER) {
                auto o = m7->attrs.find(id("X_ORIG_TYPE"));
                if (o != m7->attrs.end() && o->second.as_string() == "MUXF8") {
                    DBG();
                    return false;
                }
            }
            CellInfo *m8 = lts.cells[(i << 4) | BEL_F8MUX];
            if (m8 != nullptr && m8->belStrength < STRENGTH_USER) {
                auto o = m8->attrs.find(id("X_ORIG_TYPE"));
                if (o != m8->attrs.end() && o->second.as_string() == "MUXF7") {
                    DBG();
                    return false;
                }
            }
        }
    }
    // Check eight-tiles (mostly LUT-related validity)
    for (int i = 0; i < 8; i++) {
        if (lts.eights[i].dirty) {
            lts.eights[i].dirty = false;
            lts.eights[i].valid = false;

            CellInfo *lut6 = lts.cells[(i << 4) | BEL_6LUT];
            CellInfo *lut5 = lts.cells[(i << 4) | BEL_5LUT];

            // Check 6LUT
            if (lut6 != nullptr) {
                if (!is_slicem && (lut6->lutInfo.is_memory || lut6->lutInfo.is_srl)) {
                    DBG_RT();
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
                if (lut6->lutInfo.is_srl && (i >= 4)) {
                    DBG_RT();
                    DBG();
                    return false;
                }
                if (lut6->lutInfo.is_memory || lut6->lutInfo.is_srl) {
                    if (wclk == nullptr)
                        wclk = lut6->lutInfo.wclk;
                    else if (lut6->lutInfo.wclk != wclk) {
                        DBG();
                        return false;
                    }
                }
                if (lut5 != nullptr) {
                    // Can't mix memory and non-memory
                    if (lut6->lutInfo.is_memory != lut5->lutInfo.is_memory ||
                        lut6->lutInfo.is_srl != lut5->lutInfo.is_srl) {
                        DBG();
                        return false;
                    }
                    bool srl_pair = lut6->lutInfo.is_srl && lut5->lutInfo.is_srl;
                    // Imported (Vivado/golden) CARRY4 slot: a full 6-input
                    // operand LUT (O6->S) coexisting with a DI feed-through LUT
                    // (O5->DI, only_drives_carry) is physically legal on silicon
                    // -- golden uses it and runs.  Trust a BEL-pinned (imported)
                    // 6LUT whose companion 5LUT only drives the carry, for BOTH
                    // the all-inputs/two-outputs check here and the shared-inputs
                    // check below (the operand often uses all 6 inputs, so the
                    // input_count==6 arm fires first and must honour the same
                    // exemption).
                    bool imported_carry_feedthrough =
                            lut6->attrs.count(id_BEL) && lut5->lutInfo.only_drives_carry;
                    // General imported-slot trust: BOTH LUTs BEL-pinned means a
                    // frozen Vivado placement that already runs on silicon.
                    // The LUT-pack heuristics below (6-input coexistence,
                    // shared-input count) cannot see Vivado's input-pin
                    // permutation / shared-IMUX packing and would reject legal
                    // slots.  Trust the import; only user (non-BEL) cells get
                    // the full checks.  (Frozen macros come from a proven bit,
                    // so no illegal slot can enter this way.)
                    bool imported_slot =
                            lut6->attrs.count(id_BEL) && lut5->attrs.count(id_BEL);
                    // If all 6 inputs or 2 outputs are used, 5LUT can't also be
                    // present.  SRL16E pairs are exempt: their address pins are
                    // constant-tied and shared by construction (Vivado packs
                    // two SRL16Es per slot routinely).
                    if (!srl_pair && !imported_carry_feedthrough && !imported_slot &&
                        (lut6->lutInfo.input_count == 6 || lut6->lutInfo.output_count == 2)) {
                        if (dbg_validity_runtime)
                            log_info("  invalid-arm-387: lut6=%s(BEL=%d) in=%d out=%d lut5=%s(BEL=%d) imported_slot=%d\n",
                                     nameOf(lut6), int(lut6->attrs.count(id_BEL)), lut6->lutInfo.input_count,
                                     lut6->lutInfo.output_count, nameOf(lut5), int(lut5->attrs.count(id_BEL)),
                                     int(imported_slot));
                        DBG();
                        return false;
                    }
                    // If more than 5 total inputs are used, need to check number of shared input
                    if (!srl_pair && (lut6->lutInfo.input_count + lut5->lutInfo.input_count) > 5) {
                        int shared = 0, need_shared = (lut6->lutInfo.input_count + lut5->lutInfo.input_count - 5);
                        for (int j = 0; j < lut6->lutInfo.input_count; j++) {
                            for (int k = 0; k < lut5->lutInfo.input_count; k++) {
                                if (lut6->lutInfo.input_sigs[j] == lut5->lutInfo.input_sigs[k])
                                    shared++;
                                if (shared >= need_shared)
                                    break;
                            }
                        }
                        if (shared < need_shared) {
                            // Imported (Vivado/golden) CARRY4 slot: the operand LUT
                            // (O6->S) and the DI feed-through (O5->DI) are ONE
                            // dual-output LUT on real hardware -- O5 and O6 of the
                            // same 5-input LUT -- so they share all physical inputs
                            // by construction.  SVS/pack represents that as two cells
                            // whose lone feed-through input is a renamed net copy, so
                            // input_sigs no longer match by identity (shared=0 here)
                            // even though the placement is physically legal (golden
                            // uses it and runs).  When the slot's 6LUT is BEL-pinned
                            // (imported) and the 5LUT only drives the carry, trust the
                            // import instead of rejecting a valid slot.
                            bool imported_carry_feedthrough =
                                lut6->attrs.count(id_BEL) && lut5->lutInfo.only_drives_carry;
                            // General imported-slot trust: when BOTH LUTs are
                            // BEL-pinned (a frozen Vivado placement), Vivado has
                            // already proven the slot legal on silicon.  The
                            // shared-input heuristic under-counts when the two
                            // logical LUT inputs are distinct nets that Vivado
                            // routes to the SAME physical A-pin (LUT input-pin
                            // permutation / shared IMUX), which our net-identity
                            // comparison cannot see.  Trust the import rather
                            // than reject a placement that demonstrably works.
                            if (!imported_carry_feedthrough && !imported_slot) {
                                DBG();
                                return false;
                            }
                        }
                    }
                }
            }
            if (lut5 != nullptr) {
                if (!is_slicem && (lut5->lutInfo.is_memory || lut5->lutInfo.is_srl)) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
                if (lut5->lutInfo.is_srl) {
                    if (wclk == nullptr)
                        wclk = lut5->lutInfo.wclk;
                    else if (lut5->lutInfo.wclk != wclk) {
                        DBG();
                        return false;
                    }
                }
                // 5LUT can use at most 5 inputs and 1 output
                if (lut5->lutInfo.input_count > 5 || lut5->lutInfo.output_count == 2) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
                // The 5LUT bel has neither an A6 input nor an O6 output: a cell
                // occupying it must not use either.  input_count only counts
                // A1..A6 ports, so it misses an ABC LUT that still drives the
                // 6-output "O6" port (or draws A6) -- which passes every input
                // check above yet is unroutable ("No wire found for port O6").
                // Catch it HERE so SA swaps and the final placement check
                // enforce it, not just the initial-placement isValidBelForCell
                // gate (which SA/legalisation bypass).
                if (get_net_or_empty(lut5, id_A6) != nullptr || get_net_or_empty(lut5, id_O6) != nullptr) {
                    DBG();
                    return false;
                }
            }

            // Check (over)usage ofX inputs
            NetInfo *x_net = nullptr;
            if (lut6 != nullptr) {
                x_net = lut6->lutInfo.di2_net;
            }

            CellInfo *mux = nullptr;
            // Eights A, C, E, G: F7MUX uses X input
            if (i == 0 || i == 2 || i == 4 || i == 6)
                mux = lts.cells[i << 4 | BEL_F7MUX];
            // Eights B, F: F8MUX uses X input
            if (i == 1 || i == 5)
                mux = lts.cells[(i - 1) << 4 | BEL_F8MUX];

            if (mux != nullptr) {
                if (x_net == nullptr)
                    x_net = mux->muxInfo.sel;
                else if (x_net != mux->muxInfo.sel) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
            }

            CellInfo *out_fmux = nullptr;
            // Subslices A, C: F7MUX connects to F7F8 out
            if (i == 0 || i == 2 || i == 4 || i == 6)
                out_fmux = lts.cells[(i << 4) | BEL_F7MUX];
            // Subslices B: F8MUX connects to F7F8 out
            if (i == 1 || i == 5)
                out_fmux = lts.cells[(i - 1) << 4 | BEL_F8MUX];

            CellInfo *carry4 = lts.cells[((i / 4) << 6) | BEL_CARRY4];

            if (carry4 != nullptr && carry4->carryInfo.x_sigs[i % 4] != nullptr) {
                if (x_net == nullptr)
                    x_net = carry4->carryInfo.x_sigs[i % 4];
                else if (x_net != carry4->carryInfo.x_sigs[i % 4]) {
                    DBG();
                    return false;
                }
            }

            // FF1 might use X, if it isn't driven directly
            CellInfo *ff1 = lts.cells[i << 4 | BEL_FF];
            bool ff1_uses_x = false;
            if (ff1 != nullptr && ff1->ffInfo.d != nullptr && ff1->ffInfo.d->driver.cell != nullptr) {
                auto &drv = ff1->ffInfo.d->driver;
                if ((drv.cell == lut6 && drv.port != id_MC31) || drv.cell == lut5 || drv.cell == out_fmux ||
                    (carry4 != nullptr && drv.cell == carry4)) {
                    // Direct, OK (LUT O6/O5, F7/F8 mux out, or the slot's
                    // CARRY4 O/CO via the XOR/CY xFFMUX paths)
                } else if (ff1->belStrength >= STRENGTH_USER && (lut6 == nullptr || lut5 == nullptr)) {
                    // Imported (Vivado-placed) FF with a free LUT position in
                    // the slot: Vivado feeds the main FF through a LUT
                    // routethru (free 6LUT: INIT buffer + xFFMUX.O6; free
                    // 5LUT: lower-half INIT buffer + xFFMUX.O5) and leaves
                    // the X bypass for the 5FF -- no X usage here.  The
                    // router realises this via the route-thru pseudo pip.
                    //
                    // TEST belStrength, NOT attrs["BEL"].  This arm is reached
                    // from isBelLocationValid, i.e. AFTER binding, and placer1
                    // back-annotates `cell->attrs["BEL"]` on every cell it
                    // places (placer1.cc:585-587, bound STRENGTH_WEAK).  So the
                    // attribute is true for nextpnr's OWN placements too, and
                    // this Vivado-only escape hatch fired for them whenever the
                    // slot's 5LUT happened to be free -- skipping the X-usage
                    // accounting below and accepting a slice that cannot route.
                    // That is how johnson ended up with an F8MUX (whose select
                    // consumes BX) sharing SLICE_X122Y216 with a BFF whose D
                    // came from another slice and needed BX as well:
                    //   ERROR: Failed to route arc 1 of net 'core.prbs[26]',
                    //     from SLICE_X123Y216/AQ to SLICE_X122Y216/BFFMUX_OUT
                    // Cells stamped from the JSON are bound STRENGTH_USER by
                    // the constraints placer (placer1.cc:192), so the strength
                    // separates the two cases where the attribute cannot.  The
                    // same hazard is documented at the foot of this file for
                    // the post-placement fast path.
                } else {
                    // Indirect, must use X input
                    ff1_uses_x = true;
                    if (x_net == nullptr)
                        x_net = ff1->ffInfo.d;
                    else if (x_net != ff1->ffInfo.d) {
                        DBG();
                        return false;
                    }
                }
            }

            // FF2 might use X, if it isn't driven directly
            CellInfo *ff2 = lts.cells[i << 4 | BEL_FF2];
            if (ff2 != nullptr && ff2->ffInfo.d != nullptr && ff2->ffInfo.d->driver.cell != nullptr) {
                auto &drv = ff2->ffInfo.d->driver;
                if (drv.cell == lut5) {
                    // Direct, OK
                } else {
                    // Indirect, must use X input
                    if (x_net == nullptr)
                        x_net = ff2->ffInfo.d;
                    else if (x_net != ff2->ffInfo.d) {
#ifdef DEBUG_VALIDITY
                        log_info("%s %s %s %s %s\n", nameOf(lut6), nameOf(ff1), nameOf(lut5), nameOf(ff2),
                                 nameOf(drv.cell));
#endif
                        DBG();
                        return false;
                    }
                }
            }

            // Legalisation fix (xc7): when the main FF (BEL_FF) takes its D from
            // the X bypass (indirect) AND the column's secondary "5" FF (BEL_FF2)
            // is also placed, nextpnr's router fails to bind the main FF's input
            // MUX (xFFMUX) from X — it silently leaves the main FF's D floating
            // (observed via physical sim: the bypass-fed AFF's D = X, corrupting
            // an LFSR).  Forbid that co-pack so the placer separates the two FFs.
            // Exception: when BOTH FFs carry absolute BEL pins (imported
            // Vivado placement), trust the import -- Vivado co-packs this
            // legally, and the route-only flow cannot separate them.  Any
            // residual xFFMUX emission bug then shows up as a bit-level
            // diff against the golden bitstream instead of a place error.
            if (ff1_uses_x && ff2 != nullptr &&
                !(ff1->attrs.count(id_BEL) && ff2->attrs.count(id_BEL))) {
                DBG();
                return false;
            }

            // collision with top address bits
            if (tile_is_memory && !small_memory) {
                CellInfo *top_lut = lts.cells[(3 << 4) | BEL_6LUT];
                if (top_lut != nullptr) {
                    if ((i == 2) && x_net != top_lut->lutInfo.address_msb[0]) {
                        DBG();
                        return false;
                    }
                    if ((i == 1) && x_net != top_lut->lutInfo.address_msb[1]) {
                        DBG();
                        return false;
                    }
                }
            }

            bool mux_output_used = false;
            NetInfo *out5 = nullptr;
            if (lut6 != nullptr && lut6->lutInfo.output_count == 2)
                out5 = lut6->lutInfo.output_sigs[1];
            else if (lut5 != nullptr && !lut5->lutInfo.only_drives_carry)
                out5 = lut5->lutInfo.output_sigs[0];
            if (out5 != nullptr && (out5->users.size() > 1 || ((ff1 == nullptr || out5 != ff1->ffInfo.d) &&
                                                               (ff2 == nullptr || out5 != ff2->ffInfo.d)))) {
                mux_output_used = true;
            }

            if (carry4 != nullptr && carry4->carryInfo.out_sigs[i % 4] != nullptr) {
                // The carry O output reaches the main FF directly through
                // the XOR path of xFFMUX; only fabric fanout (or feeding
                // anything else) needs this subslice's output mux.
                NetInfo *o = carry4->carryInfo.out_sigs[i % 4];
                bool o_uses_mux = o->users.size() > 1 || ff1 == nullptr || o != ff1->ffInfo.d;
                if (o_uses_mux) {
                    if (mux_output_used) {
                        DBG();
                        return false;
                    }
                    mux_output_used = true;
                }
            }
            // This CO-fabric vs output-mux contention check stops nextpnr's own
            // placer from co-locating a CARRY4 CO and a 5FF into an unroutable
            // slot.  A *stamped* vendor placement (hybrid R0 flow: Vivado places,
            // nextpnr only routes) can legally co-locate them via muxing nextpnr
            // doesn't model, so let that flow opt out -- a wrong guess fails
            // loudly in the router, it can't produce a silently-bad bitstream.
            static const bool allow_co_5ff_contention =
                    getenv("NEXTPNR_ALLOW_CO_5FF_CONTENTION") != nullptr;
            if (!allow_co_5ff_contention && carry4 != nullptr && carry4->carryInfo.cout_sigs[i % 4] != nullptr) {
                NetInfo *co = carry4->carryInfo.cout_sigs[i % 4];
                bool co_uses_mux = false;
                if ((i % 4) == 3) {
                    // CO3 leaves on the dedicated COUT->CIN spine; only other
                    // (fabric) users need this subslice's output mux
                    for (auto &usr : co->users)
                        if (usr.port != id_CIN)
                            co_uses_mux = true;
                } else {
                    // CO0..CO2 can only reach the fabric through the output mux
                    co_uses_mux = !co->users.empty();
                }
                if (co_uses_mux) {
                    if (mux_output_used) {
                        DBG();
                        return false;
                    }
                    mux_output_used = true;
                }
            }
            if (out_fmux != nullptr) {
                NetInfo *f7f8 = out_fmux->muxInfo.out;
                if (f7f8 != nullptr && (f7f8->users.size() > 1 || ((ff1 == nullptr || f7f8 != ff1->ffInfo.d)))) {
                    if (mux_output_used) {
                        DBG();
                        return false; // Memory and SRLs only valid in SLICEMs
                    }
                    mux_output_used = true;
                }
            }
            if (ff2 != nullptr) {
                if (mux_output_used) {
                    DBG();
                    return false; // Memory and SRLs only valid in SLICEMs
                }
                mux_output_used = true;
            }

            lts.eights[i].valid = true;
        } else if (!lts.eights[i].valid) {
            if (dbg_validity_runtime)
                log_info("  invalid-arm: cached eights[%d].valid=false\n", i);
            DBG();
            return false;
        }
    }
    // Check half-tiles
    for (int i = 0; i < 2; i++) {
        if (lts.halfs[i].dirty) {
            lts.halfs[i].valid = false;
            bool found_ff[2] = {false, false};
            if (i == 0 && wclk == nullptr) {
                // Need to check wclk too
                for (int z = 4 * i; z < 4 * (i + 1); z++) {
                    for (int k = 0; k < 2; k++) {
                        CellInfo *lut = lts.cells[z << 4 | (BEL_6LUT + k)];
                        if (lut == nullptr)
                            continue;
                        if (!lut->lutInfo.is_memory && !lut->lutInfo.is_srl)
                            continue;
                        if (lut->lutInfo.wclk != nullptr) {
                            wclk = lut->lutInfo.wclk;
                            break;
                        }
                    }
                }
            }
            NetInfo *clk = nullptr, *sr = nullptr, *ce = nullptr;
            bool clkinv = false, srinv = false, islatch = false, ffsync = false;
            for (int z = 4 * i; z < 4 * (i + 1); z++) {
                for (int k = 0; k < 2; k++) {
                    CellInfo *ff = lts.cells[z << 4 | (BEL_FF + k)];
                    if (ff == nullptr)
                        continue;
                    if (ff->ffInfo.is_latch && k == 1) {
                        DBG_RT();
                        DBG();
                        return false;
                    }
                    if (found_ff[0] || found_ff[1]) {
                        if (ff->ffInfo.clk != clk) {
                            if (dbg_validity_runtime)
                                log_info("  invalid-arm: ctrlset clk %s: %s vs %s\n", nameOf(ff),
                                         ff->ffInfo.clk ? nameOf(ff->ffInfo.clk) : "-", clk ? nameOf(clk) : "-");
                            DBG();
                            return false;
                        }
                        if (ff->ffInfo.sr != sr) {
                            if (dbg_validity_runtime)
                                log_info("  invalid-arm: ctrlset sr %s: %s vs %s\n", nameOf(ff),
                                         ff->ffInfo.sr ? nameOf(ff->ffInfo.sr) : "-", sr ? nameOf(sr) : "-");
                            DBG();
                            return false;
                        }
                        if (ff->ffInfo.ce != ce) {
                            if (dbg_validity_runtime)
                                log_info("  invalid-arm: ctrlset ce %s: %s vs %s\n", nameOf(ff),
                                         ff->ffInfo.ce ? nameOf(ff->ffInfo.ce) : "-", ce ? nameOf(ce) : "-");
                            DBG();
                            return false;
                        }
                        if (ff->ffInfo.is_clkinv != clkinv) {
                            DBG_RT();
                            DBG();
                            return false;
                        }
                        if (ff->ffInfo.is_srinv != srinv) {
                            DBG_RT();
                            DBG();
                            return false;
                        }
                        if (ff->ffInfo.is_latch != islatch) {
                            DBG_RT();
                            DBG();
                            return false;
                        }
                        if (ff->ffInfo.ffsync != ffsync) {
                            DBG_RT();
                            DBG();
                            return false;
                        }
                    } else {
                        clk = ff->ffInfo.clk;
                        if (i == 0 && wclk != nullptr && clk != wclk) {
                            if (dbg_validity_runtime)
                                log_info("  invalid-arm: wclk %s: clk %s vs wclk %s\n", nameOf(ff),
                                         clk ? nameOf(clk) : "-", wclk ? nameOf(wclk) : "-");
                            DBG();
                            return false;
                        }
                        sr = ff->ffInfo.sr;
                        ce = ff->ffInfo.ce;
                        clkinv = ff->ffInfo.is_clkinv;
                        srinv = ff->ffInfo.is_srinv;
                        islatch = ff->ffInfo.is_latch;
                        ffsync = ff->ffInfo.ffsync;
                    }
                    found_ff[k] = true;
                }
            }
            lts.halfs[i].valid = true;
        } else if (!lts.halfs[i].valid) {
            if (dbg_validity_runtime)
                log_info("  invalid-arm: cached halfs[%d].valid=false\n", i);
            DBG();
            return false;
        }
    }
    return true;
}

void Arch::dumpTileStatus(BelId bel) const
{
    if (!isLogicTile(bel) || !tileStatus[bel.tile].lts)
        return;
    LogicTileStatus &lts = *(tileStatus[bel.tile].lts);
    for (int z = 0; z < 128; z++) {
        if (lts.cells[z] == nullptr)
            continue;
        CellInfo *c = lts.cells[z];
        const char *stale = (c->bel == BelId() || c->bel.tile != bel.tile) ? " [STALE: cell not bound here]" : "";
        log_info("  lts[z=%d eighth=%d slot=%d]: %s (%s)%s\n", z, z >> 4, z & 0xf, nameOf(c), c->type.c_str(this),
                 stale);
    }
    // force a full re-evaluation so the failing arm prints (cached verdicts
    // from the SA phase were computed with the runtime debug flag off)
    for (int i = 0; i < 8; i++)
        lts.eights[i].dirty = true;
    for (int i = 0; i < 2; i++)
        lts.halfs[i].dirty = true;
    log_info("  revalidating tile with arms visible:\n");
    (void)isBelLocationValid(bel);
}

// PACKING ORACLE (NEXTPNR_PACKING_ORACLE=1) ---------------------------------
// Treat an EXTERNALLY STAMPED placement as authoritative and skip our own slice
// legality check for it.
//
// The case this exists for: replaying a Vivado placement.  Vivado packs ~5.16
// primitives per slice where our recognition packer averages 1.82, and it uses
// intra-slice combinations our checker rejects outright -- e.g. an A5FF whose
// partner LUT our model does not expect ("post-placement validity check failed
// for Bel 'SLICE_X210Y29/A5FF' (no cell)").  Those placements are not illegal;
// they are legal silicon that our model is too conservative to describe, and
// the design they came from answers ARP on hardware.
//
// This is NOT --force.  --force downgrades the check globally, including for
// cells WE placed, and the result was 12306 violations and a scrambled
// placement timing 3x worse than Vivado's on the same input.  Here the check is
// waived ONLY for a tile whose every occupant carries a BEL attribute, i.e. one
// placed entirely by the external tool; any tile we have touched is still
// checked exactly as before.
static bool packing_oracle()
{
    static int cached = -1;
    if (cached < 0) {
        const char *e = getenv("NEXTPNR_PACKING_ORACLE");
        cached = (e && *e && *e != '0') ? 1 : 0;
    }
    return cached == 1;
}

// The oracle does TWO separable things: it WAIVES our slice check for a fully
// stamped tile (above), and it EXCLUDES our own cells from such tiles (below).
// Both are right for a WHOLE-DESIGN replay.  In a PARTIAL one -- place_lef
// stamps the fabric, and nextpnr's packer then invents feedthrough and constant
// LUTs -- nearly every occupied tile is fully stamped, so the exclusion pushes
// those invented cells out to distant empty tiles and the routes to reach them
// do not close: measured on ethmin, SKIPS=150 with the exclusion against a
// placement that is otherwise legal.  NEXTPNR_ORACLE_EXCLUDE=0 keeps the waiver
// and drops the exclusion.  Default stays ON so the replay flow is unchanged.
static bool packing_oracle_exclude()
{
    static int cached = -1;
    if (cached < 0) {
        const char *e = getenv("NEXTPNR_ORACLE_EXCLUDE");
        cached = (e == nullptr) ? 1 : ((*e && *e != '0') ? 1 : 0);
    }
    return cached == 1;
}

// A tile counts as externally packed if the cells that came from the DESIGN are
// all stamped.  Cells the PACKER INVENTED are ignored, because they are exactly
// the ones that must be allowed into such a tile: GND/VCC const drivers have to
// sit beside their loads, and route-through LUTs are created for carry S-pins.
// Requiring a literally untouched tile is the wrong test -- measured, it left
// one CLBLL_L rejected for holding $PACKER_GND_NET$LUT$106/$108 and a
// wr_addr[4]$LUT$104 alongside six stamped cells, and that single tile failed
// the whole run.
static bool is_packer_invented(const BaseCtx *ctx, const CellInfo *ci)
{
    std::string n = ci->name.str(ctx);
    return n.find("$PACKER_") != std::string::npos || n.find("$LUT$") != std::string::npos ||
           n.find("$intcell$") != std::string::npos;
}

bool Arch::tileIsFullyStamped(int tile) const
{
    if (!tileStatus[tile].lts)
        return false;
    const LogicTileStatus &lts = *(tileStatus[tile].lts);
    bool any_stamped = false;
    for (int z = 0; z < 128; z++) {
        const CellInfo *ci = lts.cells[z];
        if (ci == nullptr)
            continue;
        if (ci->attrs.count(id("BEL"))) {
            any_stamped = true;
            continue;
        }
        // A cell CONSTRAINED to a stamped one belongs to the imported
        // placement too: repacking a macro (RAM256X1S -> 4 x RAMS64E + a
        // MUXF7/F8 tree) creates children that carry no BEL of their own but
        // are constr_abs_z children of a base that does.  Treating them as
        // foreign would make every repacked tile fail the waiver.
        bool derived = false;
        for (const CellInfo *p = ci->constr_parent; p != nullptr; p = p->constr_parent)
            if (p->attrs.count(id("BEL"))) {
                derived = true;
                break;
            }
        if (derived) {
            any_stamped = true;
            continue;
        }
        if (is_packer_invented(this, ci))
            continue; // ours by necessity, not a foreign packing decision
        return false; // a real design cell we placed -- check the tile normally
    }
    return any_stamped;
}

bool Arch::isBelLocationValid(BelId bel) const
{
    IdString belTileType = getBelTileType(bel);
    if (isLogicTile(bel)) {
        // Logic Tile
        if (!tileStatus[bel.tile].lts)
            return true;
        if (packing_oracle() && tileIsFullyStamped(bel.tile))
            return true;
        LogicTileStatus &lts = *(tileStatus[bel.tile].lts);
        if (xc7) {
            bool v = xc7_logic_tile_valid(belTileType, lts);
            if (!v && dbg_validity_runtime) {
                log_info("  invalid-check: xc7_logic_tile_valid=false (type %s)\n", belTileType.c_str(this));
                // Was that a CACHED verdict or a fresh one?  A cached false
                // prints no invalid-arm, which is indistinguishable from a
                // fresh false whose arm we failed to log.  Re-evaluate with
                // every section forced dirty: if it now comes back TRUE the
                // cache was stale, and if it comes back false the arm that
                // fires is the real reason.
                for (int i = 0; i < 8; i++) {
                    lts.eights[i].dirty = true;
                    lts.halfs[i].dirty = true;
                }
                bool v2 = xc7_logic_tile_valid(belTileType, lts);
                log_info("  invalid-check: forced re-evaluation says %s\n",
                         v2 ? "VALID -- the cached verdict was STALE" : "still invalid (arm above is the reason)");
            }
            return v;
        } else
            return xcu_logic_tile_valid(belTileType, lts);
    } else if (belTileType == id_BRAM || belTileType == id_BRAM_L || belTileType == id_BRAM_R) {
        if (!tileStatus[bel.tile].bts)
            return true;
        BRAMTileStatus *bts = tileStatus[bel.tile].bts;
        auto onehot = [&](CellInfo *a, CellInfo *b, CellInfo *c) {
            return (((a != nullptr) ? 1 : 0) + ((b != nullptr) ? 1 : 0) + ((c != nullptr) ? 1 : 0)) <= 1;
        };
        // Only one type of BRAM cell at any given location
        if (!onehot(bts->cells[BEL_RAMFIFO36], bts->cells[BEL_RAM36], bts->cells[BEL_FIFO36])) {
            DBG();
            return false;
        }
        if (!onehot(bts->cells[BEL_RAMFIFO18_L], bts->cells[BEL_RAM18_L], bts->cells[BEL_FIFO18_L])) {
            DBG();
            return false;
        }
        // 18-bit BRAMs cannot be used whilst 36-bit is used
        if (bts->cells[BEL_RAMFIFO36] || bts->cells[BEL_RAM36] || bts->cells[BEL_FIFO36]) {
            for (int i = 4; i < 12; i++)
                if (bts->cells[i]) {
                    DBG();
                    return false;
                }
        }
    } else {
        for (auto bel : getBelsByTile(bel.tile % chip_info->width, bel.tile / chip_info->width))
            if (getBoundBelCell(bel) != nullptr && usp_bel_hard_unavail(bel))
                return false;
    }
    return true;
}

bool Arch::isValidBelForCell(CellInfo *cell, BelId bel) const
{
    if (usp_bel_hard_unavail(bel))
        return false;
    // CONTROL SETS, at candidate-selection time.  Every flop in a slice half
    // shares one clock, one clock-enable and one set/reset, and that rule lived
    // ONLY in the tile-level check -- which the placer consults when it moves a
    // cell, but not when it first chooses one.  Initial placement was therefore
    // control-set blind: it scattered flops of different domains into the same
    // half and left the annealer to repair thousands of conflicts, and any it
    // failed to repair surfaced only at the final sweep:
    //   invalid-arm: ctrlset clk core.dma.tx_run_FDRE_Q: clk_mac vs clk_sys
    // Rejecting the candidate is far cheaper than un-picking it later, and it
    // is the same reasoning the 5LUT and SLICEM gates below are built on.
    // A cell that arrived with a BEL attribute is trusted, as everywhere else.
    if (xc7 && cell->type == id_SLICE_FFX && !cell->attrs.count(id("BEL"))) {
        int z = locInfo(bel).bel_data[bel.index].z;
        int half = (z >> 4) / 4;
        LogicTileStatus *lts = tileStatus[bel.tile].lts;
        if (lts != nullptr) {
            for (int i = half * 4; i < half * 4 + 4; i++) {
                for (int slot : {BEL_FF, BEL_FF2}) {
                    CellInfo *other = lts->cells[(i << 4) | slot];
                    if (other == nullptr || other == cell || other->type != id_SLICE_FFX)
                        continue;
                    if (other->ffInfo.clk != cell->ffInfo.clk || other->ffInfo.ce != cell->ffInfo.ce ||
                        other->ffInfo.sr != cell->ffInfo.sr || other->ffInfo.is_clkinv != cell->ffInfo.is_clkinv ||
                        other->ffInfo.is_srinv != cell->ffInfo.is_srinv ||
                        other->ffInfo.is_latch != cell->ffInfo.is_latch ||
                        other->ffInfo.ffsync != cell->ffInfo.ffsync)
                        return false;
                }
            }
        }
    }
    // A MUXF8 may only sit on an F8MUX bel, and a MUXF7 only on F7AMUX/F7BMUX.
    //
    // On every other family that is enforced by the TYPE: pack.cc gives
    // MUXF7 -> F7MUX and MUXF8 -> F8MUX.  On xc7 alone both are rewritten to
    // the single type SELMUX2_1 (pack.cc: `ctx->xc7 ? id("SELMUX2_1") : ...`),
    // which erases the distinction the placer would otherwise honour -- so
    // nothing stops an F8 landing on an F7 bel.  It happened in two of
    // johnson's three mux trees (MUXF8 cells bound to SLICE_X124Y214/F7BMUX
    // and SLICE_X124Y216/F7BMUX), and the router then cannot reach the LUT
    // that should have shared the slice:
    //   ERROR: Failed to route arc 0 ... from SITEWIRE/SLICE_X124Y215/F7BMUX_OUT
    //                                      to SITEWIRE/SLICE_X124Y214/C6LUT_O6
    // That is the whole reason johnson and calc cannot be placed by nextpnr
    // and have to come in pre-stamped by place_lef.
    //
    // The original type survives packing as X_ORIG_TYPE, so the rule can be
    // restated here without touching the packer or the chipdb.  Enforced at
    // candidate-selection time for the same reason as the 5LUT and SLICEM
    // gates above: a post-hoc tile check rejects it too late for the placer to
    // choose differently.
    if (xc7 && cell->type == id("SELMUX2_1") && !cell->attrs.count(id("BEL"))) {
        auto orig = cell->attrs.find(id("X_ORIG_TYPE"));
        if (orig != cell->attrs.end()) {
            const std::string &ot = orig->second.as_string();
            int bel_z = getBelLocation(bel).z & 0xF;
            if (ot == "MUXF8" && bel_z != BEL_F8MUX)
                return false;
            if (ot == "MUXF7" && bel_z != BEL_F7MUX)
                return false;
        }
    }
    // A distributed-RAM / SRL LUT (is_memory / is_srl) is only legal in a
    // SLICEM.  Like the 6LUT-on-5LUT gate below, this MUST be enforced at
    // candidate-selection time: the tile-level isBelLocationValid() rejects
    // it only post-hoc, and legalisation can slip a memory/SRL LUT onto a
    // SLICEL bel that the cached tile check then misses -- surfacing much
    // later as an unroutable "Pin 'DI1'/'WE' of bel ... has no associated
    // wire".  This is exactly what happens to the RAMD32/RAMD64 sub-cells
    // pack_dram splits out of a RAM32X1D/RAM64M: they are created UNSTAMPED
    // (no BEL attr), so the placer chooses their bels here.  A cell already
    // carrying a BEL attr is trusted (Vivado/SVS-stamped to a proven SLICEM).
    if (xc7 && cell->type == id_SLICE_LUTX &&
        (cell->lutInfo.is_memory || cell->lutInfo.is_srl) &&
        !cell->attrs.count(id("BEL"))) {
        IdString tt = getBelTileType(bel);
        if (tt != id_CLBLM_L && tt != id_CLBLM_R)
            return false;
        // ...and it must be the SLICEM ITSELF, not merely a slice in a CLBLM
        // TILE.  A CLBLM holds TWO slices, one SLICEM and one SLICEL, so the
        // tile-type test above passes for both and a distributed-RAM cell
        // landing in the SLICEL half has no write port at all:
        //   ERROR: No wire found for port CLK on destination cell
        //          core.soc.cpu.cpuregs.0.2/DPR0_0
        // (that cell was at SLICE_X149Y150/A5LUT).  Ask the bel whether it
        // actually has the pins, rather than inferring capability from the
        // tile: the device model knows, and this cannot drift the way a
        // site-naming convention can.
        if (getBelPinWire(bel, id_CLK) == WireId())
            return false;
    }
    // A LUT that uses any 6LUT-only physical pin -- the A6 input or the O6
    // output -- cannot sit on a 5LUT bel, which has neither (a 5LUT bel is
    // A1..A5 in, O5 out).  The tile-level isBelLocationValid() also rejects the
    // A6 case, but only post-hoc; the placer uses THIS per-candidate gate to
    // choose bels, and legalisation can slip such a LUT onto a 5LUT bel that
    // the cached tile check then misses -- surfacing much later as an
    // unroutable "No wire found for port A6/O6".  A 5LUT bel is identified by
    // the absence of an A6 pin.  The cell needs a 6LUT bel if it draws 6
    // inputs, drives both outputs, or has a connected A6/O6 physical port
    // (the latter covers a LUT whose output was mapped to O6 while at a 6LUT
    // bel and then relocated).
    // (Only gates USER cells: an imported BEL-pinned cell is fixed at a
    // Vivado-proven bel -- e.g. a $trt routethru LUT1 deliberately placed at a
    // free 5LUT -- so it is trusted, not re-validated by this heuristic.)
    // Marker note: at candidate-selection time a STAMPED cell already carries
    // its JSON BEL attr (but isn't bound yet, so belStrength is still NONE),
    // whereas a nextpnr-placed USER cell has no BEL attr until place_initial
    // back-annotates it post-bind.  So the BEL attr is the correct
    // "is-this-a-frozen-cell" test HERE (unlike the post-placement fast-path,
    // which must use belStrength because back-annotation pollutes the attr).
    if (cell->type == id_SLICE_LUTX && !cell->attrs.count(id("BEL")) &&
        getBelPinWire(bel, id_A6) == WireId()) {
        // Count CONNECTED input ports directly rather than lutInfo.input_count:
        // ABC-mapped LUTs still carry logical pin names (I0..I5) at placement
        // time, for which lutInfo (which scans A1..A6) reports 0 -- so an
        // input_count test silently misses them and the placer drops a
        // 6-input LUT onto a 5LUT bel nondeterministically.
        int in_used = 0;
        bool drives_o6 = false;
        for (auto &p : cell->ports) {
            if (p.second.net == nullptr)
                continue;
            if (p.second.type == PORT_IN)
                ++in_used;
            else if (p.first == id_O6)
                drives_o6 = true;
        }
        if (getenv("NEXTPNR_DBG_LUTGATE") && cell->name.str(this).find("47890") != std::string::npos) {
            log_info("LUTGATE %s at 5LUT bel: in_used=%d drives_o6=%d out_count=%d in_count=%d\n",
                     cell->name.c_str(this), in_used, int(drives_o6), cell->lutInfo.output_count,
                     cell->lutInfo.input_count);
            for (auto &p : cell->ports)
                log_info("   port %s type=%d net=%d\n", p.first.c_str(this), int(p.second.type),
                         int(p.second.net != nullptr));
        }
        if (in_used == 6 || cell->lutInfo.output_count == 2 || drives_o6 ||
            get_net_or_empty(cell, id_A6) != nullptr)
            return false;
    }
    // NEXTPNR_PACKING_ORACLE: keep OUR cells out of a tile the external tool
    // has already packed.  The oracle waives our slice check for a fully
    // stamped tile, but only while it STAYS fully stamped -- drop one of our
    // own cells into a Vivado-packed slice and the tile is mixed, the waiver
    // correctly no longer applies, and the run dies on that single tile
    // ("post-placement validity check failed for Bel 'SLICE_X206Y21/A5FF'").
    // The bbox exclusion above cannot do this job here: in a whole-design
    // replay the stamped cells span the entire die, so its bounding box leaves
    // nowhere at all for the handful of cells the packer invents.  This is
    // per-TILE instead, which leaves every untouched tile free.
    if (packing_oracle() && packing_oracle_exclude() && !cell->attrs.count(id("BEL")) &&
        cell->name.str(this).find("$PACKER_") == std::string::npos) {
        // packer const drivers must stay placeable beside their loads
        if (isLogicTile(bel) && tileIsFullyStamped(bel.tile))
            return false;
    }
    // NEXTPNR_EXCLUDE_STAMPED_BBOX: keep unstamped cells out of the bounding
    // box of the pre-placed (BEL-attr) fabric cells -- the frozen macro's
    // region, whose LOCKED routing makes it unroutable for foreign logic.
    // Lazy: computed on the first query for an unstamped cell, at which point
    // constraint placement has already bound all stamped cells.
    static bool excl_en = getenv("NEXTPNR_EXCLUDE_STAMPED_BBOX") != nullptr;
    if (excl_en && !cell->attrs.count(id("BEL")) &&
        cell->name.str(this).find("$PACKER_") == std::string::npos) {
        // packer const drivers (GND/VCC legalisation LUTs) must be placeable
        // next to their loads, including inside the locked macro region
        static int ex0 = -2, ey0 = 0, ex1 = 0, ey1 = 0;
        if (ex0 == -2) {
            ex0 = -1;
            for (auto &cp : cells) {
                CellInfo *ci = cp.second.get();
                if (ci->bel == BelId() || !ci->attrs.count(id("BEL")))
                    continue;
                std::string t = ci->type.str(this);
                if (t.substr(0, 6) != "SLICE_" && t.substr(0, 4) != "RAMD" && t != "CARRY4")
                    continue;
                Loc l = getBelLocation(ci->bel);
                if (ex0 < 0) {
                    ex0 = ex1 = l.x;
                    ey0 = ey1 = l.y;
                } else {
                    ex0 = std::min(ex0, l.x);
                    ex1 = std::max(ex1, l.x);
                    ey0 = std::min(ey0, l.y);
                    ey1 = std::max(ey1, l.y);
                }
            }
            if (ex0 >= 0)
                log_info("isValidBelForCell: excluding unstamped cells from bbox (%d,%d)-(%d,%d)\n", ex0, ey0, ex1,
                         ey1);
        }
        if (ex0 >= 0) {
            Loc l = getBelLocation(bel);
            if (l.x >= ex0 && l.x <= ex1 && l.y >= ey0 && l.y <= ey1)
                return false;
        }
    }
    return true;
}

void Arch::fixupPlacement()
{
    if (getenv("DBG_SFEED")) {
        for (auto cell : sorted(cells)) {
            CellInfo *ci = cell.second;
            std::string nm = ci->name.str(this);
            if (nm.find("$LUT$24") != std::string::npos || nm.find("$LUT$26") != std::string::npos)
                log_info("FEEDPOS %s bel=%s strength=%d constr_parent=%s\n", nm.c_str(),
                         ci->bel == BelId() ? "UNBOUND" : getBelName(ci->bel).c_str(this),
                         int(ci->belStrength),
                         ci->constr_parent ? ci->constr_parent->name.c_str(this) : "none");
        }
    }
    log_info("Running post-placement legalisation...\n");
    // Validity-repair pass: some bind paths (heap solution-apply, detailed
    // refine) strand a cell at an invalid bel -- a LUT6 on a 5LUT (its 6th
    // input is invisible at bind time because ABC LUTs carry logical I0..I5
    // pins) or a memory/SRL LUT in a SLICEL.  Relocate each such standalone,
    // non-stamped cell to the nearest valid free bel of its type before the
    // pin-merge fixup below (which assumes a legal placement) runs.
    {
        int gw = chip_info->width, gh = chip_info->height;
        // Collect stranded cluster roots / standalone cells (a LUT+FF pair has
        // the LUT as root and the FF as a constr_z child; the placer can leave
        // the LUT6 root on a 5LUT).  A cell is a root here if it has no parent.
        std::vector<CellInfo *> roots;
        for (auto &cp : cells) {
            CellInfo *ci = cp.second.get();
            if (ci->bel == BelId() || ci->constr_parent != nullptr)
                continue; // only cluster roots / standalone
            if (ci->attrs.count(id("BEL")))
                continue; // stamped: trusted at a proven bel
            // gather the full cluster tree (root + all transitive children)
            std::vector<CellInfo *> tree;
            tree.push_back(ci);
            for (size_t i = 0; i < tree.size(); i++)
                for (auto c : tree[i]->constr_children)
                    tree.push_back(c);
            // CARRY chains span multiple slices/tiles: nextpnr's dedicated carry
            // placer handles them, and this single-tile repair would corrupt
            // their multi-tile geometry -- leave them entirely alone.
            bool has_carry = false;
            for (auto m : tree)
                if (m->type == id_CARRY4) {
                    has_carry = true;
                    break;
                }
            if (has_carry)
                continue;
            bool invalid = false;
            for (auto m : tree)
                if (m->bel != BelId() && !isValidBelForCell(m, m->bel))
                    invalid = true;
            if (invalid)
                roots.push_back(ci);
        }
        int fixed = 0;
        for (CellInfo *root : roots) {
            const bool abs_z = root->constr_abs_z;
            // Full cluster tree (root + all transitive constr_children), root first.
            std::vector<CellInfo *> members;
            members.push_back(root);
            for (size_t i = 0; i < members.size(); i++)
                for (auto c : members[i]->constr_children)
                    members.push_back(c);
            // Compute the bel of every member for a chosen root bel rb, mirroring
            // the placer's cluster BFS: a child's z is abs (constr_z) or relative
            // to its IMMEDIATE parent (parent_z + constr_z) -- this handles nested
            // MUXF7/MUXF8 trees, not just single-level LUT+FF / DRAM clusters.
            auto cluster_bels = [&](BelId rb, std::vector<BelId> &out) -> bool {
                std::vector<std::pair<CellInfo *, Loc>> work;
                work.emplace_back(root, getBelLocation(rb));
                out.clear();
                for (size_t i = 0; i < work.size(); i++) {
                    CellInfo *vc = work[i].first;
                    Loc ploc = work[i].second;
                    BelId tb = getBelByLocation(ploc);
                    if (tb == BelId() || !checkBelAvail(tb) || getBelType(tb) != vc->type ||
                        !isValidBelForCell(vc, tb))
                        return false;
                    out.push_back(tb);
                    for (auto child : vc->constr_children) {
                        Loc cloc = ploc;
                        if (child->constr_z != child->UNCONSTR)
                            cloc.z = child->constr_abs_z ? child->constr_z : (ploc.z + child->constr_z);
                        work.emplace_back(child, cloc);
                    }
                }
                return out.size() == members.size();
            };
            Loc oloc = getBelLocation(root->bel);
            for (auto m : members)
                if (m->bel != BelId())
                    unbindBel(m->bel);
            bool placed = false;
            std::vector<BelId> mbels;
            for (int r = 0; r <= std::max(gw, gh) && !placed; r++)
                for (int dx = -r; dx <= r && !placed; dx++)
                    for (int dy = -r; dy <= r && !placed; dy++) {
                        if (std::max(std::abs(dx), std::abs(dy)) != r)
                            continue;
                        int x = oloc.x + dx, y = oloc.y + dy;
                        if (x < 0 || y < 0 || x >= gw || y >= gh)
                            continue;
                        for (BelId rb : getBelsByTile(x, y)) {
                            if (abs_z && getBelLocation(rb).z != root->constr_z)
                                continue; // abs-z root must sit at its fixed z
                            if (getBelType(rb) != root->type || !checkBelAvail(rb) ||
                                !isValidBelForCell(root, rb))
                                continue;
                            if (!cluster_bels(rb, mbels))
                                continue;
                            for (size_t i = 0; i < members.size(); i++)
                                bindBel(mbels[i], members[i], STRENGTH_STRONG);
                            bool allok = true;
                            for (BelId b : mbels)
                                if (!isBelLocationValid(b)) {
                                    allok = false;
                                    break;
                                }
                            if (allok) {
                                placed = true;
                                break;
                            }
                            for (size_t i = 0; i < members.size(); i++)
                                unbindBel(mbels[i]);
                        }
                    }
            if (!placed) {
                // give up: restore members at their original positions
                std::vector<BelId> obels;
                if (cluster_bels(getBelByLocation(oloc), obels))
                    for (size_t i = 0; i < members.size(); i++) {
                        if (checkBelAvail(obels[i]))
                            bindBel(obels[i], members[i], STRENGTH_WEAK);
                    }
                log_warning("post-place repair: no valid placement for '%s'\n", root->name.c_str(this));
            } else
                ++fixed;
        }
        if (fixed)
            log_info("post-place repair: relocated %d stranded cluster(s)/cell(s) to valid bels\n", fixed);
    }
    for (auto &ts : tileStatus) {
        if (ts.lts == nullptr)
            continue;
        auto &lt = *(ts.lts);
        for (int z = 0; z < 8; z++) {
            // Fixup LUT connectivity - applies whenever a LUT5 is used
            CellInfo *lut5 = lt.cells[z << 4 | BEL_5LUT];
            if (lut5 == nullptr)
                continue;
            // Skip imported (BEL-pinned) LUT slots: this fixup re-merges the
            // 5LUT/6LUT inputs into <=6 shared physical pins assuming nextpnr's
            // own packing.  Vivado-frozen LUTs route distinct nets to shared
            // A-pins (input-pin permutation), so the by-net unique-input set can
            // exceed 6 and overrun the 6-entry ports[] array (garbage IdString
            // -> out_of_range).  Their pin mapping is already correct from the
            // import; leave it untouched.
            {
                CellInfo *l6 = lt.cells[z << 4 | BEL_6LUT];
                if (lut5->belStrength >= STRENGTH_STRONG || (l6 && l6->belStrength >= STRENGTH_STRONG)) {
                    // Skip the INPUT remerge (it would overwrite Vivado's
                    // input-pin permutation), but the cell at the 5LUT bel
                    // still drives O5, not O6: if the imported netlist named
                    // its output O6, rename it to O5 to match the bel, else the
                    // router fails with "No wire found for port O6".
                    if (lut5->ports.count(id_O6)) {
                        rename_port(getCtx(), lut5, id_O6, id_O5);
                        std::string orig = lut5->attrs.count(id("X_ORIG_PORT_O6"))
                                                   ? lut5->attrs.at(id("X_ORIG_PORT_O6")).as_string()
                                                   : std::string("O");
                        lut5->attrs.erase(id("X_ORIG_PORT_O6"));
                        lut5->attrs[id("X_ORIG_PORT_O5")] = orig;
                    }
                    continue;
                }
            }
            std::unordered_map<IdString, std::vector<int>> lut5Inputs, lut6Inputs;
            for (int i = 0; i < lut5->lutInfo.input_count; i++)
                if (lut5->lutInfo.input_sigs[i])
                    lut5Inputs[lut5->lutInfo.input_sigs[i]->name].push_back(i);
            CellInfo *lut6 = lt.cells[z << 4 | BEL_6LUT];
            if (lut6) {
                for (int i = 0; i < lut6->lutInfo.input_count; i++)
                    if (lut6->lutInfo.input_sigs[i])
                        lut6Inputs[lut6->lutInfo.input_sigs[i]->name].push_back(i);
            }
            if (lut5->lutInfo.is_memory || lut5->lutInfo.is_srl) {
                if (lut6) {
                    if (!lut6->ports.count(id_A6)) {
                        lut6->ports[id_A6].name = id_A6;
                        lut6->ports[id_A6].type = PORT_IN;
                    }
                    // pack_srls already ties A6 for 6LUT SRLs -- only
                    // connect when still floating (connect_port asserts
                    // on an already-driven port)
                    if (lut6->ports.at(id_A6).net == nullptr)
                        connect_port(getCtx(), nets[id("$PACKER_VCC_NET")].get(), lut6, id_A6);
                }
                continue;
            }
            std::set<IdString> uniqueInputs;
            for (auto i5 : lut5Inputs)
                uniqueInputs.insert(i5.first);
            for (auto i6 : lut6Inputs)
                uniqueInputs.insert(i6.first);
            // A slot cannot hold more than 6 distinct inputs; if it does the
            // input-remerge below would overrun ports[6] (garbage IdString ->
            // out_of_range).  Never legal for a properly-packed slot, so log
            // the offenders and skip rather than crash.
            if (uniqueInputs.size() > 6) {
                CellInfo *l6 = lt.cells[z << 4 | BEL_6LUT];
                log_warning("fixupPlacement: slot with %d unique inputs (>6): lut5=%s(str%d) lut6=%s(str%d); skip\n",
                            int(uniqueInputs.size()), nameOf(lut5), int(lut5->belStrength),
                            l6 ? nameOf(l6) : "-", l6 ? int(l6->belStrength) : -1);
                continue;
            }
            // Disconnect LUT inputs, and re-connect them to not overlap
            IdString ports[6] = {id_A1, id_A2, id_A3, id_A4, id_A5, id_A6};
            for (auto p : ports) {
                disconnect_port(getCtx(), lut5, p);
                lut5->attrs.erase(id("X_ORIG_PORT_" + p.str(this)));
                if (lut6) {
                    lut6->attrs.erase(id("X_ORIG_PORT_" + p.str(this)));
                    disconnect_port(getCtx(), lut6, p);
                }
            }
            int index = 0;
            for (auto i : uniqueInputs) {
                if (lut5Inputs.count(i)) {
                    if (!lut5->ports.count(ports[index])) {
                        lut5->ports[ports[index]].name = ports[index];
                        lut5->ports[ports[index]].type = PORT_IN;
                    }
                    connect_port(getCtx(), nets.at(i).get(), lut5, ports[index]);
                    lut5->attrs[id("X_ORIG_PORT_" + ports[index].str(this))] = std::string("");
                    bool first = true;
                    for (auto inp : lut5Inputs[i]) {
                        lut5->attrs[id("X_ORIG_PORT_" + ports[index].str(this))].str +=
                                (first ? "I" : " I") + std::to_string(inp);
                        first = false;
                    }
                }
                if (lut6 && lut6Inputs.count(i)) {
                    if (!lut6->ports.count(ports[index])) {
                        lut6->ports[ports[index]].name = ports[index];
                        lut6->ports[ports[index]].type = PORT_IN;
                    }
                    connect_port(getCtx(), nets.at(i).get(), lut6, ports[index]);

                    lut6->attrs[id("X_ORIG_PORT_" + ports[index].str(this))] = std::string("");
                    bool first = true;
                    for (auto inp : lut6Inputs[i]) {
                        lut6->attrs[id("X_ORIG_PORT_" + ports[index].str(this))].str +=
                                (first ? "I" : " I") + std::to_string(inp);
                        first = false;
                    }
                }
                ++index;
            }
            rename_port(getCtx(), lut5, id_O6, id_O5);
            lut5->attrs.erase(id("X_ORIG_PORT_O6"));
            lut5->attrs[id("X_ORIG_PORT_O5")] = std::string("O");

            if (lut6) {
                if (!lut6->ports.count(id_A6)) {
                    lut6->ports[id_A6].name = id_A6;
                    lut6->ports[id_A6].type = PORT_IN;
                }
                // Only tie A6 to VCC when still floating: the input de-dup above
                // may already have routed a shared input onto A6 (connect_port
                // asserts on an already-driven port -- same guard as the SRL A6 tie).
                if (lut6->ports.at(id_A6).net == nullptr)
                    connect_port(getCtx(), nets[id("$PACKER_VCC_NET")].get(), lut6, id_A6);
            }
        }
    }
    for (auto cell : sorted(cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == id("PSS_ALTO_CORE")) {
            log_info("Tieing unused PSS inputs to constants...\n");
            for (IdString pname : getBelPins(ci->bel)) {
                if (ci->ports.count(pname) && ci->ports.at(pname).net != nullptr &&
                    ci->ports.at(pname).net->driver.cell != nullptr)
                    continue;
                if (getBelPinType(ci->bel, pname) == PORT_OUT)
                    continue;
                std::string name = pname.str(this);
                if (name.find("_PAD_") != std::string::npos)
                    continue;
                if (name.find("PSVERSION") != std::string::npos || name.find("PSSGTS") != std::string::npos ||
                    name == "PSSGPWRDWNB" || name == "PSSGHIGHB" || name == "PSSFSTCFGB" || name == "PSSCFGRESETB" ||
                    name == "PCFGPORB" || name.find("IDCODE") != std::string::npos ||
                    name.find("BSCAN") != std::string::npos)
                    continue;
                bool constval = false;
                if (name == "NIRQ1LPDRPU" || name == "NIRQ0LPDRPU" || name == "NFIQ1LPDRPU" || name == "NFIQ0LPDRPU")
                    constval = true;
                if (boost::ends_with(name, "SSIN") || name == "EMIOSDIO1WP" || name == "EMIOSDIO0WP" ||
                    boost::ends_with(name, "RSOP") || boost::ends_with(name, "REOP") ||
                    boost::ends_with(name, "GMIITXCLK") || name == "DPVIDEOINCLK" || name == "DPSAXISAUDIOCLK")
                    constval = true;
                ci->ports[pname].name = pname;
                ci->ports[pname].type = PORT_IN;
                if (ci->ports[pname].net != nullptr) {
                    disconnect_port(getCtx(), ci, pname);
                    ci->attrs.erase(id("X_ORIG_PORT_" + name));
                }
                connect_port(getCtx(), nets[constval ? id("$PACKER_VCC_NET") : id("$PACKER_GND_NET")].get(), ci, pname);
            }
        } else if (ci->type == id_PS7_PS7) {
            log_info("Tieing unused PS7 inputs to constants...\n");
            for (IdString pname : getBelPins(ci->bel)) {
                if (ci->ports.count(pname) && ci->ports.at(pname).net != nullptr &&
                    ci->ports.at(pname).net->driver.cell != nullptr)
                    continue;
                if (getBelPinType(ci->bel, pname) != PORT_IN)
                    continue;
                std::string name = pname.str(this);
                if (name.find("_PAD_") != std::string::npos)
                    continue;
                if (boost::starts_with(name, "TEST") || boost::starts_with(name, "DEBUGSELECT") ||
                    boost::starts_with(name, "MIO") || boost::starts_with(name, "DDR"))
                    continue;
                bool constval = false;
                ci->ports[pname].name = pname;
                ci->ports[pname].type = PORT_IN;
                if (ci->ports[pname].net != nullptr) {
                    disconnect_port(getCtx(), ci, pname);
                    ci->attrs.erase(id("X_ORIG_PORT_" + name));
                }
                connect_port(getCtx(), nets[constval ? id("$PACKER_VCC_NET") : id("$PACKER_GND_NET")].get(), ci, pname);
            }
        } else if (ci->type == id("BITSLICE_CONTROL_BEL")) {
            std::unordered_map<IdString, bool> constpins;
            constpins[id("EN_VTC")] = true;
            constpins[id("DLY_TEST_IN")] = false;
            constpins[id("RIU_NIBBLE_SEL")] = false;
            constpins[id("TBYTE_IN0")] = false;
            constpins[id("TBYTE_IN1")] = false;
            constpins[id("TBYTE_IN2")] = false;
            constpins[id("TBYTE_IN3")] = false;

            for (auto p : constpins) {
                if (get_net_or_empty(ci, p.first) != nullptr)
                    continue;
                ci->ports[p.first].name = p.first;
                ci->ports[p.first].type = PORT_IN;
                connect_port(getCtx(), nets[p.second ? id("$PACKER_VCC_NET") : id("$PACKER_GND_NET")].get(), ci,
                             p.first);
            }
        }
    }
    // Update the set of reserved wires
    reserved_wires.clear();

    auto get_bouncewire = [&](int tile, IdString swname) {
        auto &td = chip_info->tile_types[chip_info->tile_insts[tile].type];
        WireId sitewire;
        for (int i = 0; i < td.num_wires; i++) {
            auto &w = td.wire_data[i];
            if (w.site != -1 && w.name == swname.index) {
                sitewire.tile = tile;
                sitewire.index = i;
                break;
            }
        }
        NPNR_ASSERT(sitewire != WireId());
        WireId cursor = sitewire;
        while (wireIntent(cursor) != ID_NODE_PINBOUNCE) {
            auto uh = getPipsUphill(cursor);
            NPNR_ASSERT(uh.begin() != uh.end());
            cursor = getPipSrcWire(*(uh.begin()));
        }
        return cursor;
    };
    if (!xc7) {
        for (int tile = 0; tile < chip_info->num_tiles; ++tile) {
            auto &ts = tileStatus[tile];
            if (ts.lts == nullptr)
                continue;
            auto &lts = *(ts.lts);
            for (int z = 0; z < 8; z++) {
                CellInfo *lut5 = lts.cells[z << 4 | BEL_5LUT];
                CellInfo *lut6 = lts.cells[z << 4 | BEL_6LUT];
                NetInfo *i_net = nullptr, *x_net = nullptr;
                // Check usage of DI and X inputs
                if (lut6 != nullptr) {
                    i_net = lut6->lutInfo.di1_net;
                    x_net = lut6->lutInfo.di2_net;
                }
                if (lut5 != nullptr) {
                    if (lut5->lutInfo.di1_net != nullptr)
                        i_net = lut5->lutInfo.di1_net;
                }

                CellInfo *mux = nullptr;
                // Eights A, C, E, G: F7MUX uses X input
                if (z == 0 || z == 2 || z == 4 || z == 6)
                    mux = lts.cells[z << 4 | BEL_F7MUX];
                // Eights B, F: F8MUX uses X input
                if (z == 1 || z == 5)
                    mux = lts.cells[(z - 1) << 4 | BEL_F8MUX];
                // Eights D: F9MUX uses X input
                if (z == 3)
                    mux = lts.cells[BEL_F9MUX];

                if (mux != nullptr)
                    if (x_net == nullptr)
                        x_net = mux->muxInfo.sel;

                CellInfo *carry8 = lts.cells[BEL_CARRY8];
                // CARRY8 might use X
                if (carry8 != nullptr && carry8->carryInfo.x_sigs[z] != nullptr) {
                    if (x_net == nullptr)
                        x_net = carry8->carryInfo.x_sigs[z];
                }

                CellInfo *out_fmux = nullptr;
                // Eights B, D, F, H: F7MUX connects to F7F8 out
                if (z == 1 || z == 3 || z == 5 || z == 7)
                    out_fmux = lts.cells[(z - 1) << 4 | BEL_F7MUX];
                // Eights C, G: F8MUX connects to F7F8 out
                if (z == 2 || z == 6)
                    out_fmux = lts.cells[(z - 2) << 4 | BEL_F8MUX];
                // Eights E: F9MUX connects to F7F8 out
                if (z == 4)
                    out_fmux = lts.cells[BEL_F9MUX];

                // FF1 might use X, if it isn't driven directly
                CellInfo *ff1 = lts.cells[z << 4 | BEL_FF];
                if (ff1 != nullptr && ff1->ffInfo.d != nullptr && ff1->ffInfo.d->driver.cell != nullptr) {
                    auto &drv = ff1->ffInfo.d->driver;
                    if ((drv.cell == lut6 && drv.port != id_MC31) || drv.cell == lut5 || drv.cell == out_fmux) {
                        // Direct, OK
                    } else {
                        // Indirect, must use X input
                        x_net = ff1->ffInfo.d;
                    }
                }

                // FF2 might use I, if it isn't driven directly
                CellInfo *ff2 = lts.cells[z << 4 | BEL_FF2];
                if (ff2 != nullptr && ff2->ffInfo.d != nullptr && ff2->ffInfo.d->driver.cell != nullptr) {
                    auto &drv = ff2->ffInfo.d->driver;
                    if ((drv.cell == lut6 && drv.port != id_MC31) || drv.cell == lut5 || drv.cell == out_fmux) {
                        // Direct, OK
                    } else {
                        // Indirect, must use X input
                        i_net = ff2->ffInfo.d;
                    }
                }

                if (x_net != nullptr) {
                    WireId x_wire = get_bouncewire(tile, id(std::string("") + char('A' + z) + std::string("X")));
                    reserved_wires[x_wire] = x_net;
                }
                if (i_net != nullptr) {
                    WireId i_wire = get_bouncewire(tile, id(std::string("") + char('A' + z) + std::string("_I")));
                    reserved_wires[i_wire] = i_net;
                }
            }
        }
    }
}

void Arch::bindDedicatedPads()
{
    // DEDICATED PAD CONNECTIONS ARE IMPLICIT -- recognise them from the routing
    // graph, not from a list of cell or pin names.
    //
    // A transceiver's serial pins (GTXRXP/N, GTXTXP/N) and its refclk pair reach
    // the package directly: there is no fabric path between the GT site wire and
    // the IPAD/OPAD site wire, so the chipdb gives those wires NO PIPS AT ALL,
    // uphill or downhill.  A wire outside the routing graph cannot be routed to,
    // and the router should not try -- left alone it burns a full search and then
    // reports
    //   SKIP_FAILED_ARCS: failed to route arc 0 of net 'sgmii_txn'
    //     (SITEWIRE/GTXE2_CHANNEL_X1Y1/GTXTXN -> SITEWIRE/OPAD_X1Y2/OPAD_O)
    // which is noise that raises the floor under NEXTPNR_SKIP_FAILED_ARCS and
    // makes a REAL failure easy to miss.
    //
    // "No pips on the wire" is the device model stating the connection is
    // dedicated, so bind it at STRENGTH_LOCKED.  router2 already honours that:
    // its "arcs that were pre-routed strongly (e.g. clocks)" test returns
    // ARC_SUCCESS for a sink wire bound above STRENGTH_STRONG.
    auto wire_is_dedicated = [&](WireId w) {
        if (w == WireId())
            return false;
        // Uphill/DownhillPipRange are iterator ranges, not containers.
        for (auto p : getPipsUphill(w)) {
            (void)p;
            return false;
        }
        for (auto p : getPipsDownhill(w)) {
            (void)p;
            return false;
        }
        return true;
    };
    int n_implicit = 0;
    for (auto cell : sorted(cells)) {
        CellInfo *ci = cell.second;
        if (ci->type != id("PAD") && ci->type != id("IOB_PAD"))
            continue;
        if (ci->bel == BelId())
            continue;
        WireId pad_wire = getBelPinWire(ci->bel, id("PAD"));
        if (!wire_is_dedicated(pad_wire))
            continue;               // ordinary IOB: fall through to the BFS below
        NetInfo *pad_net = ci->ports.count(id("PAD")) ? ci->ports[id("PAD")].net : nullptr;
        if (pad_net == nullptr)
            continue;
        if (!getBoundWireNet(pad_wire))
            bindWire(pad_wire, pad_net, STRENGTH_LOCKED);
        // and the far end, so the arc in EITHER direction counts as pre-routed
        if (pad_net->driver.cell != nullptr) {
            WireId drv = getCtx()->getNetinfoSourceWire(pad_net);
            if (drv != WireId() && drv != pad_wire && !getBoundWireNet(drv))
                bindWire(drv, pad_net, STRENGTH_LOCKED);
        }
        for (auto &usr : pad_net->users) {
            WireId snk = getCtx()->getNetinfoSinkWire(pad_net, usr);
            if (snk != WireId() && snk != pad_wire && !getBoundWireNet(snk))
                bindWire(snk, pad_net, STRENGTH_LOCKED);
        }
        ++n_implicit;
    }
    if (n_implicit > 0)
        log_info("    %d dedicated pad net(s) pre-bound as implicit (no pips on the pad wire)\n", n_implicit);
}

void Arch::fixupRouting()
{
    log_info("Running post-routing legalisation...\n");
    /*
     * Convert LUT permutation into correct physical connections (i.e. effectively eliminating the permutation pips),
     * then specifying the permutation as a new physical-to-logical mapping using X_ORIG_PORT. This keeps RapidWright
     * and Vivado happy, preserving the original logical netlist
     */
    std::unordered_map<int, std::vector<int>> used_perm_pips; // tile -> [extra_data] for LUT perm pips

    for (auto net : sorted(nets)) {
        NetInfo *ni = net.second;
        for (auto &wire : ni->wires) {
            PipId pip = wire.second.pip;
            if (pip == PipId())
                continue;
            auto &pd = locInfo(pip).pip_data[pip.index];
            if (pd.flags != PIP_LUT_PERMUTATION)
                continue;
            used_perm_pips[pip.tile].push_back(pd.extra_data);
        }
    }

    for (size_t ti = 0; ti < tileStatus.size(); ti++) {
        if (!used_perm_pips.count(int(ti)))
            continue;
        auto &ts = tileStatus.at(ti);
        if (ts.lts == nullptr)
            continue;

        auto &lt = *(ts.lts);
        for (int z = 0; z < 8; z++) {
            CellInfo *lut5 = lt.cells[z << 4 | BEL_5LUT];
            CellInfo *lut6 = lt.cells[z << 4 | BEL_6LUT];
            if (lut5 == nullptr && lut6 == nullptr)
                continue;
            auto &pp = used_perm_pips.at(ti);
            // from -> to
            std::unordered_map<IdString, std::vector<IdString>> new_connections;
            IdString ports[6] = {id_A1, id_A2, id_A3, id_A4, id_A5, id_A6};
            for (auto pip : pp) {
                if (((pip >> 8) & 0xF) != z)
                    continue;
                new_connections[ports[(pip >> 4) & 0xF]].push_back(ports[pip & 0xF]);
            }
            std::unordered_map<IdString, NetInfo *> orig_nets;
            std::unordered_map<IdString, std::string> orig_ports_l6, orig_ports_l5;
            for (int i = 0; i < 6; i++) {
                NetInfo *l6net = lut6 ? get_net_or_empty(lut6, ports[i]) : nullptr;
                NetInfo *l5net = lut5 ? get_net_or_empty(lut5, ports[i]) : nullptr;
                orig_nets[ports[i]] = (l6net ? l6net : l5net);
                if (lut6)
                    orig_ports_l6[ports[i]] = str_or_default(lut6->attrs, id("X_ORIG_PORT_" + ports[i].str(this)));
                if (lut5)
                    orig_ports_l5[ports[i]] = str_or_default(lut5->attrs, id("X_ORIG_PORT_" + ports[i].str(this)));
            }
            for (auto &nc : new_connections) {
                if (lut6)
                    disconnect_port(getCtx(), lut6, nc.first);
                if (lut5)
                    disconnect_port(getCtx(), lut5, nc.first);
                for (auto &dst : nc.second) {
                    if (lut6)
                        disconnect_port(getCtx(), lut6, dst);
                    if (lut5)
                        disconnect_port(getCtx(), lut5, dst);
                }
            }
            for (int i = 0; i < 6; i++) {
                if (lut6)
                    lut6->attrs.erase(id("X_ORIG_PORT_" + ports[i].str(this)));
                if (lut5)
                    lut5->attrs.erase(id("X_ORIG_PORT_" + ports[i].str(this)));
            }
            for (int i = 0; i < 6; i++) {
                auto p = ports[i];
                if (!new_connections.count(p) || new_connections.at(p).empty())
                    continue;
                if (lut6) {
                    if (!lut6->ports.count(p)) {
                        lut6->ports[p].name = p;
                        lut6->ports[p].type = PORT_IN;
                    }
                    connect_port(getCtx(), orig_nets[new_connections.at(p).front()], lut6, p);
                    lut6->attrs[id("X_ORIG_PORT_" + p.str(this))] = std::string("");
                    auto &orig_attr = lut6->attrs[id("X_ORIG_PORT_" + p.str(this))].str;
                    bool first = true;
                    for (auto &nc : new_connections.at(p)) {
                        // Separator goes BETWEEN labels, not after each one: suffixing
                        // fused the pins into "I0I1 ", which Vivado rejects on import
                        // as "InstTerm I0I1 does not exist".
                        const std::string &lbl = orig_ports_l6[nc];
                        if (lbl.empty())
                            continue;
                        if (!first)
                            orig_attr += " ";
                        orig_attr += lbl;
                        first = false;
                    }
                    if (orig_attr.empty())
                        lut6->attrs.erase(id("X_ORIG_PORT_" + p.str(this)));
                }
                if (lut5) {
                    if (!lut5->ports.count(p)) {
                        lut5->ports[p].name = p;
                        lut5->ports[p].type = PORT_IN;
                    }
                    connect_port(getCtx(), orig_nets[new_connections.at(p).front()], lut5, p);
                    lut5->attrs[id("X_ORIG_PORT_" + p.str(this))] = std::string("");
                    auto &orig_attr = lut5->attrs[id("X_ORIG_PORT_" + p.str(this))].str;
                    bool first = true;
                    for (auto &nc : new_connections.at(p)) {
                        const std::string &lbl = orig_ports_l5[nc];
                        if (lbl.empty())
                            continue;
                        if (!first)
                            orig_attr += " ";
                        orig_attr += lbl;
                        first = false;
                    }
                    if (orig_attr.empty())
                        lut5->attrs.erase(id("X_ORIG_PORT_" + p.str(this)));
                }
            }
        }
    }
    /*
     * Route PAD nets which won't have been routed due to inout issues
     */
    auto route_bfs = [&](NetInfo *net, WireId src, WireId dst) {
        if (src == dst)
            return;
        std::queue<WireId> visit;
        std::unordered_map<WireId, PipId> backtrace;
        visit.push(dst);
        WireId cursor;
        while (!visit.empty()) {
            WireId curr = visit.front();
            visit.pop();
            for (auto uh : getPipsUphill(curr)) {
                if (!checkPipAvail(uh))
                    continue;
                WireId pip_src = getPipSrcWire(uh);
                if (pip_src == src) {
                    cursor = curr;
                    break;
                }
                if (backtrace.count(pip_src))
                    continue;
                if (!checkWireAvail(pip_src))
                    continue;
                backtrace[pip_src] = uh;
                visit.push(pip_src);
            }
        }
        NPNR_ASSERT(cursor != WireId());
        while (backtrace.count(cursor)) {
            auto uh = backtrace[cursor];
            cursor = getPipDstWire(uh);
            if (!getBoundWireNet(cursor))
                bindWire(cursor, net, STRENGTH_STRONG);
            bindPip(uh, net, STRENGTH_STRONG);
        }
    };

    for (auto cell : sorted(cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == id("IOB_PAD")) {
            NetInfo *pad_net = ci->ports[id("PAD")].net;
            NPNR_ASSERT(pad_net != nullptr && nets.count(pad_net->name));
            std::vector<WireId> unbind;
            for (auto w : pad_net->wires)
                if (getBoundWireNet(w.first))
                    unbind.push_back(w.first);
            for (auto w : unbind)
                unbindWire(w);
            WireId pad_wire = getBelPinWire(ci->bel, id("PAD"));
            bindWire(pad_wire, pad_net, STRENGTH_LOCKED);
            if (pad_net->driver.cell != nullptr) {
                WireId drv_wire = getCtx()->getNetinfoSourceWire(pad_net);
                if (drv_wire != pad_wire)
                    bindWire(drv_wire, pad_net, STRENGTH_LOCKED);
                route_bfs(pad_net, drv_wire, pad_wire);
            }
            for (auto &usr : pad_net->users) {
                if (usr.cell == ci)
                    continue;
                route_bfs(pad_net, pad_wire, getCtx()->getNetinfoSinkWire(pad_net, usr));
            }
        }
    }

    /*
     * Legalise route through OSERDESE3s
     */
    for (auto cell : sorted(cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == id("OSERDESE3")) {
            if (get_net_or_empty(ci, id("T_OUT")) != nullptr)
                continue;
            ci->params[id("OSERDES_T_BYPASS")] = std::string("TRUE");
        }
    }
}

NEXTPNR_NAMESPACE_END
