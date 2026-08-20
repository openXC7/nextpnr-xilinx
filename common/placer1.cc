/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2018  Clifford Wolf <clifford@symbioticeda.com>
 *  Copyright (C) 2018  David Shah <david@symbioticeda.com>
 *
 *  Simulated annealing implementation based on arachne-pnr
 *  Copyright (C) 2015-2018 Cotton Seed
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

#include "placer1.h"
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <ostream>
#include <queue>
#include <set>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "log.h"
#include "place_common.h"
#include "timing.h"
#include "util.h"

namespace std {
template <> struct hash<std::pair<NEXTPNR_NAMESPACE_PREFIX IdString, std::size_t>>
{
    std::size_t operator()(const std::pair<NEXTPNR_NAMESPACE_PREFIX IdString, std::size_t> &idp) const noexcept
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, hash<NEXTPNR_NAMESPACE_PREFIX IdString>()(idp.first));
        boost::hash_combine(seed, hash<std::size_t>()(idp.second));
        return seed;
    }
};
} // namespace std

NEXTPNR_NAMESPACE_BEGIN

class SAPlacer
{
  private:
    struct BoundingBox
    {
        // Actual bounding box
        int x0 = 0, x1 = 0, y0 = 0, y1 = 0;
        // Number of cells at each extremity
        int nx0 = 0, nx1 = 0, ny0 = 0, ny1 = 0;
        wirelen_t hpwl(const Placer1Cfg &cfg) const
        {
            return wirelen_t(cfg.hpwl_scale_x * (x1 - x0) + cfg.hpwl_scale_y * (y1 - y0));
        }
    };

  public:
    SAPlacer(Context *ctx, Placer1Cfg cfg) : ctx(ctx), cfg(cfg)
    {
        int num_bel_types = 0;
        for (auto bel : ctx->getBels()) {
            IdString type = ctx->getBelType(bel);
            if (bel_types.find(type) == bel_types.end()) {
                bel_types[type] = std::tuple<int, int>(num_bel_types++, 1);
            } else {
                std::get<1>(bel_types.at(type))++;
            }
        }
        for (auto bel : ctx->getBels()) {
            Loc loc = ctx->getBelLocation(bel);
            IdString type = ctx->getBelType(bel);
            int type_idx = std::get<0>(bel_types.at(type));
            int type_cnt = std::get<1>(bel_types.at(type));
            if (type_cnt < cfg.minBelsForGridPick)
                loc.x = loc.y = 0;
            if (int(fast_bels.size()) < type_idx + 1)
                fast_bels.resize(type_idx + 1);
            if (int(fast_bels.at(type_idx).size()) < (loc.x + 1))
                fast_bels.at(type_idx).resize(loc.x + 1);
            if (int(fast_bels.at(type_idx).at(loc.x).size()) < (loc.y + 1))
                fast_bels.at(type_idx).at(loc.x).resize(loc.y + 1);
            max_x = std::max(max_x, loc.x);
            max_y = std::max(max_y, loc.y);
            fast_bels.at(type_idx).at(loc.x).at(loc.y).push_back(bel);
        }
        diameter = std::max(max_x, max_y) + 1;

        net_bounds.resize(ctx->nets.size());
        net_arc_tcost.resize(ctx->nets.size());
        old_udata.reserve(ctx->nets.size());
        net_by_udata.reserve(ctx->nets.size());
        decltype(NetInfo::udata) n = 0;
        for (auto &net : ctx->nets) {
            old_udata.emplace_back(net.second->udata);
            net_arc_tcost.at(n).resize(net.second->users.size());
            net.second->udata = n++;
            net_by_udata.push_back(net.second.get());
        }
        for (auto &region : sorted(ctx->region)) {
            Region *r = region.second;
            BoundingBox bb;
            if (r->constr_bels) {
                bb.x0 = std::numeric_limits<int>::max();
                bb.x1 = std::numeric_limits<int>::min();
                bb.y0 = std::numeric_limits<int>::max();
                bb.y1 = std::numeric_limits<int>::min();
                for (auto bel : r->bels) {
                    Loc loc = ctx->getBelLocation(bel);
                    bb.x0 = std::min(bb.x0, loc.x);
                    bb.x1 = std::max(bb.x1, loc.x);
                    bb.y0 = std::min(bb.y0, loc.y);
                    bb.y1 = std::max(bb.y1, loc.y);
                }
            } else {
                bb.x0 = 0;
                bb.y0 = 0;
                bb.x1 = max_x;
                bb.y1 = max_y;
            }
            region_bounds[r->name] = bb;
        }
        build_port_index();
    }

    ~SAPlacer()
    {
        for (auto &net : ctx->nets)
            net.second->udata = old_udata[net.second->udata];
    }

    bool place(bool refine = false)
    {
        log_break();
        ctx->lock();

        size_t placed_cells = 0;
        std::vector<CellInfo *> autoplaced;
        std::vector<CellInfo *> chain_basis;
        if (!refine) {
            // Initial constraints placer
            for (auto &cell_entry : ctx->cells) {
                CellInfo *cell = cell_entry.second.get();
                auto loc = cell->attrs.find(ctx->id("BEL"));
                if (loc != cell->attrs.end()) {
                    std::string loc_name = loc->second.as_string();
                    BelId bel = ctx->getBelByName(ctx->id(loc_name));
                    if (bel == BelId()) {
                        log_error("No Bel named \'%s\' located for "
                                  "this chip (processing BEL attribute on \'%s\')\n",
                                  loc_name.c_str(), cell->name.c_str(ctx));
                    }

                    IdString bel_type = ctx->getBelType(bel);
                    if (bel_type != cell->type) {
                        log_error("Bel \'%s\' of type \'%s\' does not match cell "
                                  "\'%s\' of type \'%s\'\n",
                                  loc_name.c_str(), bel_type.c_str(ctx), cell->name.c_str(ctx), cell->type.c_str(ctx));
                    }
                    if (!ctx->isValidBelForCell(cell, bel)) {
                        log_error("Bel \'%s\' of type \'%s\' is not valid for cell "
                                  "\'%s\' of type \'%s\'\n",
                                  loc_name.c_str(), bel_type.c_str(ctx), cell->name.c_str(ctx), cell->type.c_str(ctx));
                    }

                    auto bound_cell = ctx->getBoundBelCell(bel);
                    if (bound_cell) {
                        log_error(
                                "Cell \'%s\' cannot be bound to bel \'%s\' since it is already bound to cell \'%s\'\n",
                                cell->name.c_str(ctx), loc_name.c_str(), bound_cell->name.c_str(ctx));
                    }

                    ctx->bindBel(bel, cell, STRENGTH_USER);
                    locked_bels.insert(bel);
                    placed_cells++;
                }
            }
            int constr_placed_cells = placed_cells;
            log_info("Placed %d cells based on constraints.\n", int(placed_cells));
            setup_exclusion_bbox();
            ctx->yield();

            // Bind packer-created chain children of BEL-pinned roots before
            // building the autoplace set, so SA doesn't scatter them
            // (imported Vivado placements: carry feed-through LUTs etc.)
            int chain_children_placed = 0;
            std::function<void(CellInfo *)> bind_children = [&](CellInfo *root) {
                Loc rootLoc = ctx->getBelLocation(root->bel);
                for (auto child : root->constr_children) {
                    if (getenv("DBG_SFEED") && std::string(ctx->nameOf(child)).find("$LUT$") != std::string::npos)
                        log_info("BINDCH child=%s root=%s childBel=%s\n", ctx->nameOf(child),
                                 ctx->getBelName(root->bel).c_str(ctx),
                                 child->bel == BelId() ? "UNBOUND" : ctx->getBelName(child->bel).c_str(ctx));
                    if (child->bel == BelId()) {
                        Loc cl;
                        cl.x = rootLoc.x + (child->constr_x == child->UNCONSTR ? 0 : child->constr_x);
                        cl.y = rootLoc.y + (child->constr_y == child->UNCONSTR ? 0 : child->constr_y);
                        // xc7: an "absolute" chain z encodes the slot within
                        // one slice half; chains never cross halves, so the
                        // root's half (z bit 6) is ALWAYS imposed on children
                        cl.z = child->constr_abs_z
                                       ? (child->constr_z | (rootLoc.z & ~0x3F))
                                       : rootLoc.z + (child->constr_z == child->UNCONSTR ? 0 : child->constr_z);
                        BelId target = ctx->getBelByLocation(cl);
                        if (target != BelId() && ctx->checkBelAvail(target)) {
                            ctx->bindBel(target, child, STRENGTH_STRONG);
                            chain_children_placed++;
                        } else {
                            log_warning("chain child '%s' pre-bind failed: root=%s loc=(%d,%d,%d) target=%s bound=%s\n",
                                        ctx->nameOf(child), ctx->getBelName(root->bel).c_str(ctx), cl.x, cl.y, cl.z,
                                        target == BelId() ? "none" : ctx->getBelName(target).c_str(ctx),
                                        target == BelId() ? "-" : ctx->nameOf(ctx->getBoundBelCell(target)));
                        }
                    }
                    if (child->bel != BelId())
                        bind_children(child);
                }
            };
            std::function<void(CellInfo *)> clear_constrs = [&](CellInfo *c) {
                c->constr_x = c->UNCONSTR;
                c->constr_y = c->UNCONSTR;
                c->constr_z = c->UNCONSTR;
                c->constr_abs_z = false;
                for (auto ch : c->constr_children) {
                    if (ch->bel == BelId())
                        log_warning("chain child '%s' of pinned root '%s' is still unbound\n", ctx->nameOf(ch),
                                    ctx->nameOf(c));
                    clear_constrs(ch);
                }
            };
            for (auto &cell : ctx->cells) {
                CellInfo *ci = cell.second.get();
                if (ci->constr_parent == nullptr && !ci->constr_children.empty() && ci->bel != BelId() &&
                    ci->attrs.count(ctx->id("BEL"))) {
                    bind_children(ci);
                    // constraints have served their purpose; the xc7 half-bit
                    // z encoding would otherwise trip the final distance sweep
                    clear_constrs(ci);
                }
            }
            // pinned chain SEGMENTS with no children (skipped from chaining)
            // may still carry the root abs-z constraint from pack_carries
            for (auto &cell : ctx->cells) {
                CellInfo *ci = cell.second.get();
                if (ci->constr_parent == nullptr && ci->bel != BelId() && ci->attrs.count(ctx->id("BEL")) &&
                    ci->constr_abs_z)
                    clear_constrs(ci);
            }
            if (chain_children_placed > 0)
                log_info("Bound %d chain children of BEL-pinned roots.\n", chain_children_placed);

            // Sort to-place cells for deterministic initial placement

            for (auto &cell : ctx->cells) {
                CellInfo *ci = cell.second.get();
                // Constrained CHILDREN are deliberately absent: `autoplaced`
                // is also the SA MOVE LIST, so listing them lets annealing
                // relocate a child on its own and split the cluster -- which is
                // how a MUXF7 ended up in a different slice from its LUTs.
                // They are seated by their root, and any the root could not
                // seat are swept up after this loop.
                if (ci->bel == BelId() && ci->constr_parent == nullptr) {
                    autoplaced.push_back(cell.second.get());
                }
            }
            std::sort(autoplaced.begin(), autoplaced.end(), [](CellInfo *a, CellInfo *b) { return a->name < b->name; });
            ctx->shuffle(autoplaced);
            // BIGGEST CLUSTERS FIRST.  A root binds its own children, so it
            // must choose its site while their bels are still free.  Ordering
            // roots merely before singletons is not enough: a LUT+FF pair is a
            // cluster too, and once enough one-LUT clusters are scattered
            // about, no slice retains the FOUR free LUT bels a MUXF7/F8 tree
            // needs.  ethmin failed exactly so, with the contested bels held by
            // other cluster roots --
            //   child ...MUXF8_O_I1_MUXF7_O_I0_LUT2_O wants SLICE_X108Y170/B6LUT,
            //   held by 'core.soc.cpu.eoi_FDRE_Q_12_D_LUT3_O'
            // -- and enlarging the region did not help, because the small
            // clusters simply spread out with it (8375 sites failed the same
            // way as 2577).  Seat the most demanding clusters while the device
            // is still empty; singletons fit in the gaps afterwards.
            {
                std::unordered_map<IdString, int> desc;
                std::function<int(CellInfo *)> count = [&](CellInfo *c) -> int {
                    auto it = desc.find(c->name);
                    if (it != desc.end())
                        return it->second;
                    int n = 0;
                    for (auto ch : c->constr_children)
                        n += 1 + count(ch);
                    desc[c->name] = n;
                    return n;
                };
                for (auto c : autoplaced)
                    count(c);
                std::stable_sort(autoplaced.begin(), autoplaced.end(),
                                 [&](CellInfo *a, CellInfo *b) { return desc[a->name] > desc[b->name]; });
            }
            auto iplace_start = std::chrono::high_resolution_clock::now();
            // Place cells randomly initially
            log_info("Creating initial placement for remaining %d cells.\n", int(autoplaced.size()));

            for (auto cell : autoplaced) {
                place_initial(cell);
                placed_cells++;
                if ((placed_cells - constr_placed_cells) % 500 == 0)
                    log_info("  initial placement placed %d/%d cells\n", int(placed_cells - constr_placed_cells),
                             int(autoplaced.size()));
            }
            if ((placed_cells - constr_placed_cells) % 500 != 0)
                log_info("  initial placement placed %d/%d cells\n", int(placed_cells - constr_placed_cells),
                         int(autoplaced.size()));
            // Orphan sweep: a root that had to fall back to placing itself
            // alone (a long CARRY4 chain that fits nowhere as a unit) leaves
            // its children unbound, and they are not in `autoplaced`.  Place
            // them individually rather than letting them reach the router
            // unplaced.
            {
                int orphans = 0;
                for (auto &cell : ctx->cells) {
                    CellInfo *ci = cell.second.get();
                    if (ci->bel == BelId()) {
                        // Inherit the ROOT's region.  A constrained child has
                        // none of its own (only roots are given one), so an
                        // orphan placed here would be free to land anywhere on
                        // the die -- 9 of them scattered out of a region at
                        // X[122..174] Y[114..166] as far as SLICE_X24Y347, and
                        // one produced a tile the validity check then refused.
                        if (ci->region == nullptr) {
                            CellInfo *root = ci;
                            while (root->constr_parent != nullptr)
                                root = root->constr_parent;
                            ci->region = root->region;
                        }
                        place_initial(ci);
                        ++orphans;
                    }
                }
                if (orphans > 0)
                    log_info("  placed %d cluster child(ren) whose root could not seat them\n", orphans);
            }
            if (cfg.budgetBased && cfg.slack_redist_iter > 0)
                assign_budget(ctx);
            ctx->yield();
            auto iplace_end = std::chrono::high_resolution_clock::now();
            log_info("Initial placement time %.02fs\n",
                     std::chrono::duration<float>(iplace_end - iplace_start).count());
            log_info("Running simulated annealing placer.\n");
        } else {
            for (auto &cell : ctx->cells) {
                CellInfo *ci = cell.second.get();
                if (ci->belStrength > STRENGTH_STRONG)
                    continue;
                else if (ci->constr_parent != nullptr)
                    continue;
                else if (!ci->constr_children.empty() || ci->constr_z != ci->UNCONSTR)
                    chain_basis.push_back(ci);
                else
                    autoplaced.push_back(ci);
            }
            require_legal = false;
            diameter = 3;
            log_info("Running simulated annealing placer for refinement.\n");
        }
        auto saplace_start = std::chrono::high_resolution_clock::now();

        // Invoke timing analysis to obtain criticalities
        if (!cfg.budgetBased)
            get_criticalities(ctx, &net_crit);

        // Calculate costs after initial placement
        setup_costs();
        moveChange.init(this);
        curr_wirelen_cost = total_wirelen_cost();
        curr_timing_cost = total_timing_cost();
        last_wirelen_cost = curr_wirelen_cost;
        last_timing_cost = curr_timing_cost;

        if (cfg.netShareWeight > 0)
            setup_nets_by_tile();

        wirelen_t avg_wirelen = curr_wirelen_cost;
        wirelen_t min_wirelen = curr_wirelen_cost;

        int n_no_progress = 0;
        temp = refine ? 1e-7 : cfg.startTemp;

        // Main simulated annealing loop
        for (int iter = 1;; iter++) {
            n_move = n_accept = 0;
            improved = false;

            if (iter % 5 == 0 || iter == 1)
                log_info("  at iteration #%d: temp = %f, timing cost = "
                         "%.0f, wirelen = %.0f\n",
                         iter, temp, double(curr_timing_cost), double(curr_wirelen_cost));

            for (int m = 0; m < 15; ++m) {
                // Loop through all automatically placed cells
                for (auto cell : autoplaced) {
                    // Find another random Bel for this cell
                    BelId try_bel = random_bel_for_cell(cell);
                    // If valid, try and swap to a new position and see if
                    // the new position is valid/worthwhile
                    if (try_bel != BelId() && try_bel != cell->bel)
                        try_swap_position(cell, try_bel);
                }
                // Also try swapping chains, if applicable
                for (auto cb : chain_basis) {
                    Loc chain_base_loc = ctx->getBelLocation(cb->bel);
                    BelId try_base = random_bel_for_cell(cb, chain_base_loc.z);
                    if (try_base != BelId() && try_base != cb->bel)
                        try_swap_chain(cb, try_base);
                }
            }

            if (ctx->debug) {
                // Verify correctness of incremental wirelen updates
                for (size_t i = 0; i < net_bounds.size(); i++) {
                    auto net = net_by_udata[i];
                    if (ignore_net(net))
                        continue;
                    auto &incr = net_bounds.at(i), gold = get_net_bounds(net);
                    NPNR_ASSERT(incr.x0 == gold.x0);
                    NPNR_ASSERT(incr.x1 == gold.x1);
                    NPNR_ASSERT(incr.y0 == gold.y0);
                    NPNR_ASSERT(incr.y1 == gold.y1);
                    NPNR_ASSERT(incr.nx0 == gold.nx0);
                    NPNR_ASSERT(incr.nx1 == gold.nx1);
                    NPNR_ASSERT(incr.ny0 == gold.ny0);
                    NPNR_ASSERT(incr.ny1 == gold.ny1);
                }
            }

            if (curr_wirelen_cost < min_wirelen) {
                min_wirelen = curr_wirelen_cost;
                improved = true;
            }

            // Heuristic to improve placement on the 8k
            if (improved)
                n_no_progress = 0;
            else
                n_no_progress++;

            if (temp <= 1e-7 && n_no_progress >= (refine ? 1 : 5)) {
                log_info("  at iteration #%d: temp = %f, timing cost = "
                         "%.0f, wirelen = %.0f \n",
                         iter, temp, double(curr_timing_cost), double(curr_wirelen_cost));
                break;
            }

            double Raccept = double(n_accept) / double(n_move);

            int M = std::max(max_x, max_y) + 1;

            if (ctx->verbose)
                log("iter #%d: temp = %f, timing cost = "
                    "%.0f, wirelen = %.0f, dia = %d, Ra = %.02f \n",
                    iter, temp, double(curr_timing_cost), double(curr_wirelen_cost), diameter, Raccept);

            if (curr_wirelen_cost < 0.95 * avg_wirelen && curr_wirelen_cost > 0) {
                avg_wirelen = 0.8 * avg_wirelen + 0.2 * curr_wirelen_cost;
            } else {
                double diam_next = diameter * (1.0 - 0.44 + Raccept);
                diameter = std::max<int>(1, std::min<int>(M, int(diam_next + 0.5)));
                if (Raccept > 0.96) {
                    temp *= 0.5;
                } else if (Raccept > 0.8) {
                    temp *= 0.9;
                } else if (Raccept > 0.15 && diameter > 1) {
                    temp *= 0.95;
                } else {
                    temp *= 0.8;
                }
            }
            // Once cooled below legalise threshold, run legalisation and start requiring
            // legal moves only
            if (diameter < legalise_dia && require_legal) {
                if (legalise_relative_constraints(ctx)) {
                    // Only increase temperature if something was moved
                    autoplaced.clear();
                    chain_basis.clear();
                    for (auto cell : sorted(ctx->cells)) {
                        if (cell.second->belStrength <= STRENGTH_STRONG && cell.second->constr_parent == nullptr &&
                            !cell.second->constr_children.empty())
                            chain_basis.push_back(cell.second);
                        else if (cell.second->belStrength < STRENGTH_STRONG)
                            autoplaced.push_back(cell.second);
                    }
                    // temp = post_legalise_temp;
                    // diameter = std::min<int>(M, diameter * post_legalise_dia_scale);
                    ctx->shuffle(autoplaced);

                    // Legalisation is a big change so force a slack redistribution here
                    if (cfg.slack_redist_iter > 0 && cfg.budgetBased)
                        assign_budget(ctx, true /* quiet */);
                }
                require_legal = false;
            } else if (cfg.budgetBased && cfg.slack_redist_iter > 0 && iter % cfg.slack_redist_iter == 0) {
                assign_budget(ctx, true /* quiet */);
            }

            // Invoke timing analysis to obtain criticalities
            if (!cfg.budgetBased && cfg.timing_driven)
                get_criticalities(ctx, &net_crit);
            // Need to rebuild costs after criticalities change
            setup_costs();
            // Reset incremental bounds
            moveChange.reset(this);
            moveChange.new_net_bounds = net_bounds;

            // Recalculate total metric entirely to avoid rounding errors
            // accumulating over time
            curr_wirelen_cost = total_wirelen_cost();
            curr_timing_cost = total_timing_cost();
            last_wirelen_cost = curr_wirelen_cost;
            last_timing_cost = curr_timing_cost;
            // Let the UI show visualization updates.
            ctx->yield();
        }

        auto saplace_end = std::chrono::high_resolution_clock::now();
        log_info("SA placement time %.02fs\n", std::chrono::duration<float>(saplace_end - saplace_start).count());

        // Final post-pacement validitiy check
        extern bool dbg_validity_runtime;
        if (getenv("NEXTPNR_DUMP_INVALID_TILE") != nullptr)
            dbg_validity_runtime = true;
        ctx->yield();
        for (auto bel : ctx->getBels()) {
            CellInfo *cell = ctx->getBoundBelCell(bel);
            if (!ctx->isBelLocationValid(bel)) {
                std::string cell_text = "no cell";
                if (cell != nullptr)
                    cell_text = std::string("cell '") + ctx->nameOf(cell) + "'";
                if (getenv("NEXTPNR_DUMP_INVALID_TILE") != nullptr) {
                    Loc l = ctx->getBelLocation(bel);
                    for (auto tb : ctx->getBelsByTile(l.x, l.y)) {
                        CellInfo *tc = ctx->getBoundBelCell(tb);
                        if (tc != nullptr)
                            log_info("  tile occupant: %s = %s (%s)%s\n", ctx->getBelName(tb).c_str(ctx),
                                     ctx->nameOf(tc), tc->type.c_str(ctx),
                                     tc->attrs.count(ctx->id("BEL")) ? " [stamped]" : "");
                    }
                    ctx->dumpTileStatus(bel);
                }
                if (ctx->force) {
                    log_warning("post-placement validity check failed for Bel '%s' "
                                "(%s)\n",
                                ctx->getBelName(bel).c_str(ctx), cell_text.c_str());
                } else {
                    log_error("post-placement validity check failed for Bel '%s' "
                              "(%s)\n",
                              ctx->getBelName(bel).c_str(ctx), cell_text.c_str());
                }
            }
        }
        for (auto cell : sorted(ctx->cells))
            if (get_constraints_distance(ctx, cell.second) != 0)
                log_error("constraint satisfaction check failed for cell '%s' at Bel '%s'\n", cell.first.c_str(ctx),
                          ctx->getBelName(cell.second->bel).c_str(ctx));
        // NEXTPNR_CRIT_PATH_REPORT=1: also dump the per-stage critical path
        // breakdown (logic vs routing ns).  router2 never calls timing_analysis,
        // and only router1 passes print_path, so without this the open flow can
        // report "43 MHz" with no way to see WHICH path is slow.
        timing_analysis(ctx, true /* slack_histogram */, true /* print_fmax */,
                        getenv("NEXTPNR_CRIT_PATH_REPORT") != nullptr /* print_path */,
                        false /* warn_on_failure */);
        ctx->unlock();
        return true;
    }

  private:
    // Initial random placement
    // Where a constrained child lands if its parent sits at parentLoc.
    Loc constr_child_loc(const CellInfo *child, const Loc &parentLoc) const
    {
        Loc cl;
        cl.x = parentLoc.x + (child->constr_x == child->UNCONSTR ? 0 : child->constr_x);
        cl.y = parentLoc.y + (child->constr_y == child->UNCONSTR ? 0 : child->constr_y);
        cl.z = child->constr_abs_z ? (child->constr_z | (parentLoc.z & ~0x3F))
                                   : parentLoc.z + (child->constr_z == child->UNCONSTR ? 0 : child->constr_z);
        return cl;
    }

    // Could every constrained descendant be placed, if `cell` went to cellLoc?
    //
    // place_initial otherwise places cells ONE AT A TIME with no knowledge of
    // constraints: only BEL-pinned roots get their children bound up front
    // (see bind_children above, gated on attrs["BEL"]), so for a design placed
    // from scratch every cluster member lands at an independent random bel and
    // the cluster is only reassembled afterwards, if SA's chain moves happen to
    // manage it.  On johnson most did and one did not -- an F7BMUX two slices
    // from its F8MUX root -- which is unroutable, because a MUXF7 can only be
    // fed by the two LUTs of its own slice.
    bool constr_children_fit(CellInfo *cell, Loc cellLoc, std::unordered_set<BelId> &claimed)
    {
        for (auto child : cell->constr_children) {
            Loc cl = constr_child_loc(child, cellLoc);
            BelId tb = ctx->getBelByLocation(cl);
            if (tb == BelId() || ctx->getBelType(tb) != child->type)
                return false;
            if (!ctx->checkBelAvail(tb) || !claimed.insert(tb).second)
                return false;
            // Deliberately NOT isValidBelForCell(child, tb).  This predicate
            // exists to predict whether bind_constr_children can seat the
            // cluster, and that binder tests only type and availability -- so
            // adding a legality test here makes the gate STRICTER THAN THE
            // BINDER IT GUARDS and rejects placements that would work.  On
            // ethmin it refused every CARRY4 site in the region over one
            // packer-made VCC LUT:
            //   at SLICE_X155Y191/CARRY4: child '$PACKER_VCC_NET$LUT$1696'
            //      (SLICE_LUTX) -> isValidBelForCell refused the child
            // Genuine illegality is still caught afterwards by the tile-level
            // isBelLocationValid, which is where slice rules belong.
            if (!constr_children_fit(child, cl, claimed))
                return false;
        }
        return true;
    }

    void bind_constr_children(CellInfo *cell, Loc cellLoc)
    {
        for (auto child : cell->constr_children) {
            Loc cl = constr_child_loc(child, cellLoc);
            BelId tb = ctx->getBelByLocation(cl);
            if (tb == BelId())
                continue;
            // The child may ALREADY be bound -- place_initial's ripup path
            // re-enters this loop for the cell it evicted, and if that cell is
            // a cluster root its children are still sitting where they were put
            // the first time.  Binding unconditionally then aborts the whole
            // run inside bindBel:
            //   Assertion failure: tileStatus[bel.tile].boundcells[bel.index]
            //                      == nullptr (xilinx/arch.h:847)
            if (child->bel == tb)
                continue; // already exactly where it belongs
            if (child->bel != BelId())
                ctx->unbindBel(child->bel);
            // Someone else holds the slot: leave the child unbound rather than
            // abort.  The orphan sweep after initial placement will site it,
            // and the tile-level validity check still refuses an illegal split.
            if (!ctx->checkBelAvail(tb))
                continue;
            ctx->bindBel(tb, child, STRENGTH_STRONG);
            child->attrs[ctx->id("BEL")] = ctx->getBelName(tb).str(ctx);
            bind_constr_children(child, cl);
        }
    }

    // Does this cell belong to a cluster whose members must share one tile?
    // Checked from the cell in either direction: a child of such a cluster, or
    // a root whose children are tile-pinned.
    bool is_in_tile_local_cluster(CellInfo *cell) const
    {
        CellInfo *root = cell;
        while (root->constr_parent != nullptr)
            root = root->constr_parent;
        if (root->constr_children.empty())
            return false;
        return cluster_is_tile_local(root);
    }

    // Is every descendant pinned to the root's own tile?  A MUXF7/F8 tree is
    // (all children carry constr_x/constr_y of 0, only z differs) and it is
    // ILLEGAL to split -- a MUXF7 can only be fed by the two LUTs of its slice.
    // A CARRY4 chain is not: it walks up the rows, so it can be relaxed and
    // reassembled by the annealer.  The distinction decides whether falling
    // back to placing the root alone is acceptable or produces a design the
    // validity checker will (rightly) refuse.
    bool cluster_is_tile_local(const CellInfo *cell) const
    {
        for (auto child : cell->constr_children) {
            if ((child->constr_x != child->UNCONSTR && child->constr_x != 0) ||
                (child->constr_y != child->UNCONSTR && child->constr_y != 0))
                return false;
            if (!cluster_is_tile_local(child))
                return false;
        }
        return true;
    }

    void place_initial(CellInfo *cell)
    {
        // Already seated by its cluster root -- nothing to choose.
        if (cell->bel != BelId())
            return;
        bool all_placed = false;
        int iters = 25;
        while (!all_placed) {
            BelId best_bel = BelId();
            uint64_t best_score = std::numeric_limits<uint64_t>::max(),
                     best_ripup_score = std::numeric_limits<uint64_t>::max();
            CellInfo *ripup_target = nullptr;
            BelId ripup_bel = BelId();
            if (cell->bel != BelId()) {
                ctx->unbindBel(cell->bel);
            }
            IdString targetType = cell->type;

            // Try to seat the WHOLE cluster first; fall back to the root alone
            // if nothing on the device can take it.  Strict is right for a
            // slice-local cluster (a MUXF7/F8 tree, whose children must share
            // the slice), but a CARRY4 chain is 232 cells deep on ethmin and
            // spans many rows, so demanding every descendant fit at every
            // candidate rejected the entire device and killed placement with
            //   ERROR: failed to place cell '...CARRY4' of type 'CARRY4'
            // before a single cell was placed.  Relaxing is what the placer did
            // unconditionally before, so the fallback is no worse than the old
            // behaviour -- and it says when it happens rather than silently
            // scattering the cluster.
            bool strict_cluster = true;
            auto proc_bel = [&](BelId bel) {
                if (ctx->getBelType(bel) == targetType && ctx->isValidBelForCell(cell, bel)) {
                    // Honour the cell's OWN absolute-z constraint.  DRAM
                    // elements are pinned to an exact slot -- pack_dram sets
                    //   constr_abs_z = true;
                    //   constr_z = (z << 4) | (o5 ? BEL_5LUT : BEL_6LUT)
                    // -- and the binder respects that for CHILDREN, imposing
                    // the root's half bits (constr_z | (rootLoc.z & ~0x3F)).
                    // Nothing applied it to the ROOT, which was chosen on bel
                    // TYPE alone: the RV32 register file's DPR0_0 roots landed
                    // on C5LUT and B5LUT when their own constraint named a
                    // different slot, so every sibling's position was computed
                    // from a root in the wrong place and nine could not be
                    // seated at all.  Only the low 6 bits are the cell's
                    // business; bit 6 up is the slice half, left free.
                    if (cell->constr_abs_z &&
                        (ctx->getBelLocation(bel).z & 0x3F) != (cell->constr_z & 0x3F))
                        return;
                    if (strict_cluster && !cell->constr_children.empty()) {
                        std::unordered_set<BelId> claimed;
                        if (!constr_children_fit(cell, ctx->getBelLocation(bel), claimed))
                            return;
                    }
                    if (ctx->checkBelAvail(bel)) {
                        uint64_t score = ctx->rng64();
                        if (score <= best_score) {
                            best_score = score;
                            best_bel = bel;
                        }
                    } else {
                        uint64_t score = ctx->rng64();
                        CellInfo *bound_cell = ctx->getBoundBelCell(bel);
                        if (score <= best_ripup_score && bound_cell->belStrength < STRENGTH_STRONG) {
                            best_ripup_score = score;
                            ripup_target = bound_cell;
                            ripup_bel = bel;
                        }
                    }
                }
            };

            auto scan_bels = [&]() {
                if (cell->region != nullptr && cell->region->constr_bels) {
                    for (auto bel : cell->region->bels) {
                        proc_bel(bel);
                    }
                } else {
                    for (auto bel : ctx->getBels()) {
                        proc_bel(bel);
                    }
                }
            };
            scan_bels();
            if (best_bel == BelId() && ripup_bel == BelId() && strict_cluster && !cell->constr_children.empty() &&
                !cluster_is_tile_local(cell)) {
                strict_cluster = false;
                scan_bels();
                if (best_bel != BelId() || ripup_bel != BelId())
                    log_warning("no site fits the whole cluster rooted at '%s'; placing the root alone and "
                                "leaving its children to the annealer\n",
                                ctx->nameOf(cell));
            }

            if (best_bel == BelId()) {
                if (iters == 0 || ripup_bel == BelId()) {
                    // SAY WHY.  "failed to place cell X" alone gives nothing to
                    // act on: the interesting question is always whether the
                    // cell itself had no legal bel, or whether its CLUSTER
                    // could not be seated as a unit -- and if the latter, which
                    // child blocked it.
                    if (!cell->constr_children.empty()) {
                        int n_desc = 0;
                        std::function<void(CellInfo *)> count = [&](CellInfo *c) {
                            for (auto ch : c->constr_children) {
                                ++n_desc;
                                count(ch);
                            }
                        };
                        count(cell);
                        log_info("  cluster root with %d descendant(s), tile-local=%s, strict=%s\n", n_desc,
                                 cluster_is_tile_local(cell) ? "yes" : "no", strict_cluster ? "yes" : "no");
                        // Explain the first candidate we would have liked.
                        int shown = 0;
                        auto explain = [&](BelId bel) {
                            if (shown >= 3 || ctx->getBelType(bel) != targetType)
                                return;
                            if (!ctx->isValidBelForCell(cell, bel))
                                return;
                            Loc rl = ctx->getBelLocation(bel);
                            for (auto child : cell->constr_children) {
                                Loc cl = constr_child_loc(child, rl);
                                BelId tb = ctx->getBelByLocation(cl);
                                const char *why = nullptr;
                                if (tb == BelId()) {
                                    log_info("    at %s (z=%d): child '%s' -> no bel at (%d,%d,z=%d) "
                                             "[constr x=%d y=%d z=%d abs=%d]\n",
                                             ctx->getBelName(bel).c_str(ctx), rl.z, ctx->nameOf(child), cl.x, cl.y,
                                             cl.z, child->constr_x, child->constr_y, child->constr_z,
                                             child->constr_abs_z ? 1 : 0);
                                    ++shown;
                                    return;
                                }
                                else if (ctx->getBelType(tb) != child->type)
                                    why = "bel there is the wrong type";
                                else if (!ctx->checkBelAvail(tb)) {
                                    CellInfo *occ = ctx->getBoundBelCell(tb);
                                    log_info("    at %s: child '%s' wants %s, held by '%s'%s\n",
                                             ctx->getBelName(bel).c_str(ctx), ctx->nameOf(child),
                                             ctx->getBelName(tb).c_str(ctx),
                                             occ ? ctx->nameOf(occ) : "?",
                                             (occ == child) ? "  <-- THE CHILD ITSELF, already placed" : "");
                                    ++shown;
                                    return;
                                }
                                else if (!ctx->isValidBelForCell(child, tb))
                                    why = "isValidBelForCell refused the child";
                                if (why != nullptr) {
                                    log_info("    at %s: child '%s' (%s) -> %s\n",
                                             ctx->getBelName(bel).c_str(ctx), ctx->nameOf(child),
                                             child->type.c_str(ctx), why);
                                    ++shown;
                                    return;
                                }
                            }
                        };
                        if (cell->region != nullptr && cell->region->constr_bels)
                            for (auto bel : cell->region->bels)
                                explain(bel);
                        else
                            for (auto bel : ctx->getBels())
                                explain(bel);
                    }
                    log_error("failed to place cell '%s' of type '%s'\n", cell->name.c_str(ctx),
                              cell->type.c_str(ctx));
                }
                --iters;
                ctx->unbindBel(ripup_target->bel);
                best_bel = ripup_bel;
            } else {
                all_placed = true;
            }
            ctx->bindBel(best_bel, cell, STRENGTH_WEAK);

            // Back annotate location
            cell->attrs[ctx->id("BEL")] = ctx->getBelName(cell->bel).str(ctx);
            // ...and bring the cluster with it.  Bound STRENGTH_STRONG so a
            // later ripup cannot pick a child off individually and strand it.
            if (strict_cluster && !cell->constr_children.empty())
                bind_constr_children(cell, ctx->getBelLocation(best_bel));
            cell = ripup_target;
        }
    }

    // Attempt a SA position swap, return true on success or false on failure
    bool try_swap_position(CellInfo *cell, BelId newBel)
    {
        static const double epsilon = 1e-20;
        moveChange.reset(this);
        if (!require_legal && is_constrained(cell))
            return false;
        // A TILE-LOCAL cluster must never be broken up, not even during
        // legalisation (which is why this is not gated on !require_legal like
        // the test above).  Such a cluster -- a MUXF7/F8 tree, a LUT+FF pair --
        // is only legal with every member in one slice, and moving one member
        // on its own is what splits it.  The tile-level validity check cannot
        // catch that reliably: it is CACHED PER TILE and a move only dirties
        // the tiles it touches, so breaking the invariant in the PARENT's tile
        // goes unnoticed until the final full sweep, by which point the move is
        // long since committed and the run dies with
        //   ERROR: post-placement validity check failed for Bel ... (no cell)
        // Refusing the move is the only place the rule can be enforced cheaply.
        if (is_in_tile_local_cluster(cell))
            return false;
        BelId oldBel = cell->bel;
        CellInfo *other_cell = ctx->getBoundBelCell(newBel);
        if (other_cell != nullptr && is_in_tile_local_cluster(other_cell))
            return false;
        if (!require_legal && other_cell != nullptr &&
            (is_constrained(other_cell) || other_cell->belStrength > STRENGTH_WEAK)) {
            return false;
        }
        int old_dist = get_constraints_distance(ctx, cell);
        int new_dist;
        if (other_cell != nullptr)
            old_dist += get_constraints_distance(ctx, other_cell);
        double delta = 0;

        int net_delta_score = 0;
        if (cfg.netShareWeight > 0)
            net_delta_score += update_nets_by_tile(cell, ctx->getBelLocation(cell->bel), ctx->getBelLocation(newBel));

        ctx->unbindBel(oldBel);
        if (other_cell != nullptr) {
            ctx->unbindBel(newBel);
        }

        ctx->bindBel(newBel, cell, STRENGTH_WEAK);

        if (other_cell != nullptr) {
            ctx->bindBel(oldBel, other_cell, STRENGTH_WEAK);
            if (cfg.netShareWeight > 0)
                net_delta_score +=
                        update_nets_by_tile(other_cell, ctx->getBelLocation(newBel), ctx->getBelLocation(oldBel));
        }

        add_move_cell(moveChange, cell, oldBel);

        if (other_cell != nullptr) {
            add_move_cell(moveChange, other_cell, newBel);
        }

        if (!ctx->isBelLocationValid(newBel) || ((other_cell != nullptr && !ctx->isBelLocationValid(oldBel)))) {
            ctx->unbindBel(newBel);
            if (other_cell != nullptr)
                ctx->unbindBel(oldBel);
            goto swap_fail;
        }

        // Recalculate metrics for all nets touched by the peturbation
        compute_cost_changes(moveChange);

        new_dist = get_constraints_distance(ctx, cell);
        if (other_cell != nullptr)
            new_dist += get_constraints_distance(ctx, other_cell);
        delta = lambda * (moveChange.timing_delta / std::max<double>(last_timing_cost, epsilon)) +
                (1 - lambda) * (double(moveChange.wirelen_delta) / std::max<double>(last_wirelen_cost, epsilon));
        delta += (cfg.constraintWeight / temp) * (new_dist - old_dist) / last_wirelen_cost;
        if (cfg.netShareWeight > 0)
            delta += -cfg.netShareWeight * (net_delta_score / std::max<double>(total_net_share, epsilon));
        n_move++;
        // SA acceptance criterea
        if (delta < 0 || (temp > 1e-8 && (ctx->rng() / float(0x3fffffff)) <= std::exp(-delta / temp))) {
            n_accept++;
        } else {
            if (other_cell != nullptr)
                ctx->unbindBel(oldBel);
            ctx->unbindBel(newBel);
            goto swap_fail;
        }
        commit_cost_changes(moveChange);
#if 0
        log_info("swap %s -> %s\n", cell->name.c_str(ctx), ctx->getBelName(newBel).c_str(ctx));
        if (other_cell != nullptr)
            log_info("swap %s -> %s\n", other_cell->name.c_str(ctx), ctx->getBelName(oldBel).c_str(ctx));
#endif
        return true;
    swap_fail:
        ctx->bindBel(oldBel, cell, STRENGTH_WEAK);
        if (other_cell != nullptr) {
            ctx->bindBel(newBel, other_cell, STRENGTH_WEAK);
            if (cfg.netShareWeight > 0)
                update_nets_by_tile(other_cell, ctx->getBelLocation(oldBel), ctx->getBelLocation(newBel));
        }
        if (cfg.netShareWeight > 0)
            update_nets_by_tile(cell, ctx->getBelLocation(newBel), ctx->getBelLocation(oldBel));
        return false;
    }

    inline bool is_constrained(CellInfo *cell)
    {
        return cell->constr_parent != nullptr || !cell->constr_children.empty();
    }

    // Swap the Bel of a cell with another, return the original location
    BelId swap_cell_bels(CellInfo *cell, BelId newBel)
    {
        BelId oldBel = cell->bel;
#if 0
        log_info("%s old: %s new: %s\n", cell->name.c_str(ctx), ctx->getBelName(cell->bel).c_str(ctx), ctx->getBelName(newBel).c_str(ctx));
#endif
        CellInfo *bound = ctx->getBoundBelCell(newBel);
        if (bound != nullptr)
            ctx->unbindBel(newBel);
        ctx->unbindBel(oldBel);
        ctx->bindBel(newBel, cell, is_constrained(cell) ? STRENGTH_STRONG : STRENGTH_WEAK);
        if (bound != nullptr) {
            ctx->bindBel(oldBel, bound, is_constrained(bound) ? STRENGTH_STRONG : STRENGTH_WEAK);
            if (cfg.netShareWeight > 0)
                update_nets_by_tile(bound, ctx->getBelLocation(newBel), ctx->getBelLocation(oldBel));
        }
        if (cfg.netShareWeight > 0)
            update_nets_by_tile(cell, ctx->getBelLocation(oldBel), ctx->getBelLocation(newBel));
        return oldBel;
    }

    // Discover the relative positions of all cells in a chain
    void discover_chain(Loc baseLoc, CellInfo *cell, std::vector<std::pair<CellInfo *, Loc>> &cell_rel)
    {
        Loc cellLoc = ctx->getBelLocation(cell->bel);
        Loc rel{cellLoc.x - baseLoc.x, cellLoc.y - baseLoc.y, cellLoc.z};
        cell_rel.emplace_back(std::make_pair(cell, rel));
        for (auto child : cell->constr_children)
            discover_chain(baseLoc, child, cell_rel);
    }

    // Attempt to swap a chain with a non-chain
    bool try_swap_chain(CellInfo *cell, BelId newBase)
    {
        std::vector<std::pair<CellInfo *, Loc>> cell_rel;
        std::unordered_set<IdString> cells;
        std::vector<std::pair<CellInfo *, BelId>> moves_made;
        std::vector<std::pair<CellInfo *, BelId>> dest_bels;
        double delta = 0;
        int orig_share_cost = total_net_share;
        moveChange.reset(this);
#if 0
        if (ctx->debug)
            log_info("finding cells for chain swap %s\n", cell->name.c_str(ctx));
#endif
        Loc baseLoc = ctx->getBelLocation(cell->bel);
        discover_chain(baseLoc, cell, cell_rel);
        Loc newBaseLoc = ctx->getBelLocation(newBase);
        NPNR_ASSERT(newBaseLoc.z == baseLoc.z);
        for (const auto &cr : cell_rel)
            cells.insert(cr.first->name);

        for (const auto &cr : cell_rel) {
            Loc targetLoc = {newBaseLoc.x + cr.second.x, newBaseLoc.y + cr.second.y, cr.second.z};
            BelId targetBel = ctx->getBelByLocation(targetLoc);
            if (targetBel == BelId())
                return false;
            // Compare against the type of the MEMBER being moved, not the
            // chain root's.  Both are the same for a homogeneous chain (a
            // carry chain of CARRY4s), which is why this went unnoticed, but a
            // xc7 MUXF7/MUXF8 cluster is heterogeneous: a SELMUX2_1 root with
            // SLICE_LUTX children.  Testing every member against the root's
            // type rejected the swap the moment a LUT member was considered,
            // so mux clusters could never move as a unit and ended up split
            // across slices -- which the router cannot close, because a MUXF7
            // must be fed by the two LUTs of its own slice.
            if (ctx->getBelType(targetBel) != cr.first->type)
                return false;
            CellInfo *bound = ctx->getBoundBelCell(targetBel);
            // We don't consider swapping chains with other chains, at least for the time being - unless it is
            // part of this chain
            if (bound != nullptr && !cells.count(bound->name) &&
                (bound->belStrength >= STRENGTH_STRONG || is_constrained(bound)))
                return false;
            dest_bels.emplace_back(std::make_pair(cr.first, targetBel));
        }
#if 0
        if (ctx->debug)
            log_info("trying chain swap %s\n", cell->name.c_str(ctx));
#endif
        // <cell, oldBel>
        for (const auto &db : dest_bels) {
            BelId oldBel = swap_cell_bels(db.first, db.second);
            moves_made.emplace_back(std::make_pair(db.first, oldBel));
            CellInfo *bound = ctx->getBoundBelCell(oldBel);
            add_move_cell(moveChange, db.first, oldBel);
            if (bound != nullptr)
                add_move_cell(moveChange, bound, db.second);
        }
        for (const auto &mm : moves_made) {
            if (!ctx->isBelLocationValid(mm.first->bel) || !check_cell_bel_region(mm.first, mm.first->bel) ||
                bel_excluded(mm.first, mm.first->bel))
                goto swap_fail;
            if (!ctx->isBelLocationValid(mm.second))
                goto swap_fail;
            CellInfo *bound = ctx->getBoundBelCell(mm.second);
            if (bound && (!check_cell_bel_region(bound, bound->bel) || bel_excluded(bound, bound->bel)))
                goto swap_fail;
        }
        compute_cost_changes(moveChange);
        delta = lambda * (moveChange.timing_delta / last_timing_cost) +
                (1 - lambda) * (double(moveChange.wirelen_delta) / last_wirelen_cost);
        if (cfg.netShareWeight > 0) {
            delta +=
                    cfg.netShareWeight * (orig_share_cost - total_net_share) / std::max<double>(total_net_share, 1e-20);
        }
        n_move++;
        // SA acceptance criterea
        if (delta < 0 || (temp > 1e-9 && (ctx->rng() / float(0x3fffffff)) <= std::exp(-delta / temp))) {
            n_accept++;
#if 0
            if (ctx->debug)
                log_info("accepted chain swap %s\n", cell->name.c_str(ctx));
#endif
        } else {
            goto swap_fail;
        }
        commit_cost_changes(moveChange);
        return true;
    swap_fail:
        for (const auto &entry : boost::adaptors::reverse(moves_made))
            swap_cell_bels(entry.first, entry.second);
        return false;
    }

    // Find a random Bel of the correct type for a cell, within the specified
    // diameter
    // Exclusion bbox (NEXTPNR_EXCLUDE_STAMPED_BBOX=1): unstamped cells stay
    // out of the bounding box of pre-placed (BEL-attr) fabric cells -- i.e.
    // the frozen macro's region, whose routing is LOCKED so densely that
    // arcs into it are unroutable for foreign logic.
    int excl_x0 = -1, excl_y0 = -1, excl_x1 = -1, excl_y1 = -1;
    void setup_exclusion_bbox()
    {
        if (getenv("NEXTPNR_EXCLUDE_STAMPED_BBOX") == nullptr)
            return;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->bel == BelId() || !ci->attrs.count(ctx->id("BEL")))
                continue;
            std::string t = ci->type.str(ctx);
            if (t.substr(0, 6) != "SLICE_" && t.substr(0, 4) != "RAMD" && t != "CARRY4")
                continue;
            Loc l = ctx->getBelLocation(ci->bel);
            if (excl_x0 < 0) {
                excl_x0 = excl_x1 = l.x;
                excl_y0 = excl_y1 = l.y;
            } else {
                excl_x0 = std::min(excl_x0, l.x);
                excl_x1 = std::max(excl_x1, l.x);
                excl_y0 = std::min(excl_y0, l.y);
                excl_y1 = std::max(excl_y1, l.y);
            }
        }
        if (excl_x0 >= 0)
            log_info("Excluding unstamped cells from stamped bbox (%d,%d)-(%d,%d)\n", excl_x0, excl_y0, excl_x1,
                     excl_y1);
    }
    bool bel_excluded(const CellInfo *cell, BelId bel) const
    {
        if (excl_x0 < 0 || bel == BelId())
            return false;
        if (cell->attrs.count(ctx->id("BEL")))
            return false; // stamped cells belong there
        Loc l = ctx->getBelLocation(bel);
        return l.x >= excl_x0 && l.x <= excl_x1 && l.y >= excl_y0 && l.y <= excl_y1;
    }

    BelId random_bel_for_cell(CellInfo *cell, int force_z = -1)
    {
        IdString targetType = cell->type;
        Loc curr_loc = ctx->getBelLocation(cell->bel);
        int count = 0;

        int dx = diameter, dy = diameter;
        if (cell->region != nullptr && cell->region->constr_bels) {
            dx = std::min(cfg.hpwl_scale_x * diameter,
                          (region_bounds[cell->region->name].x1 - region_bounds[cell->region->name].x0) + 1);
            dy = std::min(cfg.hpwl_scale_y * diameter,
                          (region_bounds[cell->region->name].y1 - region_bounds[cell->region->name].y0) + 1);
            // Clamp location to within bounds
            curr_loc.x = std::max(region_bounds[cell->region->name].x0, curr_loc.x);
            curr_loc.x = std::min(region_bounds[cell->region->name].x1, curr_loc.x);
            curr_loc.y = std::max(region_bounds[cell->region->name].y0, curr_loc.y);
            curr_loc.y = std::min(region_bounds[cell->region->name].y1, curr_loc.y);
        }

        while (true) {
            int nx = ctx->rng(2 * dx + 1) + std::max(curr_loc.x - dx, 0);
            int ny = ctx->rng(2 * dy + 1) + std::max(curr_loc.y - dy, 0);
            int beltype_idx, beltype_cnt;
            std::tie(beltype_idx, beltype_cnt) = bel_types.at(targetType);
            if (beltype_cnt < cfg.minBelsForGridPick)
                nx = ny = 0;
            if (nx >= int(fast_bels.at(beltype_idx).size()))
                continue;
            if (ny >= int(fast_bels.at(beltype_idx).at(nx).size()))
                continue;
            const auto &fb = fast_bels.at(beltype_idx).at(nx).at(ny);
            if (fb.size() == 0)
                continue;
            BelId bel = fb.at(ctx->rng(int(fb.size())));
            if (force_z != -1) {
                Loc loc = ctx->getBelLocation(bel);
                if (loc.z != force_z)
                    continue;
            }
            if (!check_cell_bel_region(cell, bel))
                continue;
            if (bel_excluded(cell, bel))
                continue;
            if (locked_bels.find(bel) != locked_bels.end())
                continue;
            count++;
            return bel;
        }
    }

    // Return true if a net is to be entirely ignored
    inline bool ignore_net(NetInfo *net)
    {
        return net->driver.cell == nullptr || net->driver.cell->bel == BelId() ||
               ctx->getBelGlobalBuf(net->driver.cell->bel);
    }

    // Get the bounding box for a net
    inline BoundingBox get_net_bounds(NetInfo *net)
    {
        BoundingBox bb;
        NPNR_ASSERT(net->driver.cell != nullptr);
        Loc dloc = ctx->getBelLocation(net->driver.cell->bel);
        bb.x0 = dloc.x;
        bb.x1 = dloc.x;
        bb.y0 = dloc.y;
        bb.y1 = dloc.y;
        bb.nx0 = 1;
        bb.nx1 = 1;
        bb.ny0 = 1;
        bb.ny1 = 1;
        for (auto user : net->users) {
            if (user.cell->bel == BelId())
                continue;
            Loc uloc = ctx->getBelLocation(user.cell->bel);
            if (bb.x0 == uloc.x)
                ++bb.nx0;
            else if (uloc.x < bb.x0) {
                bb.x0 = uloc.x;
                bb.nx0 = 1;
            }
            if (bb.x1 == uloc.x)
                ++bb.nx1;
            else if (uloc.x > bb.x1) {
                bb.x1 = uloc.x;
                bb.nx1 = 1;
            }
            if (bb.y0 == uloc.y)
                ++bb.ny0;
            else if (uloc.y < bb.y0) {
                bb.y0 = uloc.y;
                bb.ny0 = 1;
            }
            if (bb.y1 == uloc.y)
                ++bb.ny1;
            else if (uloc.y > bb.y1) {
                bb.y1 = uloc.y;
                bb.ny1 = 1;
            }
        }

        return bb;
    }

    // Get the timing cost for an arc of a net
    inline double get_timing_cost(NetInfo *net, size_t user)
    {
        int cc;
        if (net->driver.cell == nullptr)
            return 0;
        if (ctx->getPortTimingClass(net->driver.cell, net->driver.port, cc) == TMG_IGNORE)
            return 0;
        if (cfg.budgetBased) {
            double delay = ctx->getDelayNS(ctx->predictDelay(net, net->users.at(user)));
            return std::min(10.0, std::exp(delay - ctx->getDelayNS(net->users.at(user).budget) / 10));
        } else {
            auto crit = net_crit.find(net->name);
            if (crit == net_crit.end() || crit->second.criticality.empty())
                return 0;
            double delay = ctx->getDelayNS(ctx->predictDelay(net, net->users.at(user)));
            return delay * std::pow(crit->second.criticality.at(user), crit_exp);
        }
    }

    // Set up the cost maps
    void setup_costs()
    {
        for (auto net : sorted(ctx->nets)) {
            NetInfo *ni = net.second;
            if (ignore_net(ni))
                continue;
            net_bounds[ni->udata] = get_net_bounds(ni);
            if (cfg.timing_driven && int(ni->users.size()) < cfg.timingFanoutThresh)
                for (size_t i = 0; i < ni->users.size(); i++)
                    net_arc_tcost[ni->udata][i] = get_timing_cost(ni, i);
        }
    }

    // Get the total wiring cost for the design
    wirelen_t total_wirelen_cost()
    {
        wirelen_t cost = 0;
        for (const auto &net : net_bounds)
            cost += net.hpwl(cfg);
        return cost;
    }

    // Get the total timing cost for the design
    double total_timing_cost()
    {
        double cost = 0;
        for (const auto &net : net_arc_tcost) {
            for (auto arc_cost : net) {
                cost += arc_cost;
            }
        }
        return cost;
    }

    // Cost-change-related data for a move
    struct MoveChangeData
    {

        enum BoundChangeType
        {
            NO_CHANGE,
            CELL_MOVED_INWARDS,
            CELL_MOVED_OUTWARDS,
            FULL_RECOMPUTE
        };

        std::vector<decltype(NetInfo::udata)> bounds_changed_nets_x, bounds_changed_nets_y;
        std::vector<std::pair<decltype(NetInfo::udata), size_t>> changed_arcs;

        std::vector<BoundChangeType> already_bounds_changed_x, already_bounds_changed_y;
        std::vector<std::vector<bool>> already_changed_arcs;

        std::vector<BoundingBox> new_net_bounds;
        std::vector<std::pair<std::pair<decltype(NetInfo::udata), size_t>, double>> new_arc_costs;

        wirelen_t wirelen_delta = 0;
        double timing_delta = 0;

        void init(SAPlacer *p)
        {
            already_bounds_changed_x.resize(p->ctx->nets.size());
            already_bounds_changed_y.resize(p->ctx->nets.size());
            already_changed_arcs.resize(p->ctx->nets.size());
            for (auto &net : p->ctx->nets) {
                already_changed_arcs.at(net.second->udata).resize(net.second->users.size());
            }
            new_net_bounds = p->net_bounds;
        }

        void reset(SAPlacer *p)
        {
            for (auto bc : bounds_changed_nets_x) {
                new_net_bounds[bc] = p->net_bounds[bc];
                already_bounds_changed_x[bc] = NO_CHANGE;
            }
            for (auto bc : bounds_changed_nets_y) {
                new_net_bounds[bc] = p->net_bounds[bc];
                already_bounds_changed_y[bc] = NO_CHANGE;
            }
            for (const auto &tc : changed_arcs)
                already_changed_arcs[tc.first][tc.second] = false;
            bounds_changed_nets_x.clear();
            bounds_changed_nets_y.clear();
            changed_arcs.clear();
            new_arc_costs.clear();
            wirelen_delta = 0;
            timing_delta = 0;
        }

    } moveChange;

    void add_move_cell(MoveChangeData &mc, CellInfo *cell, BelId old_bel)
    {
        Loc curr_loc = ctx->getBelLocation(cell->bel);
        Loc old_loc = ctx->getBelLocation(old_bel);
        // Check net bounds
        for (const auto &port : cell->ports) {
            NetInfo *pn = port.second.net;
            if (pn == nullptr)
                continue;
            if (ignore_net(pn))
                continue;
            BoundingBox &curr_bounds = mc.new_net_bounds[pn->udata];
            // Incremental bounding box updates
            // Note that everything other than full updates are applied immediately rather than being queued,
            // so further updates to the same net in the same move are dealt with correctly.
            // If a full update is already queued, this can be considered a no-op
            if (mc.already_bounds_changed_x[pn->udata] != MoveChangeData::FULL_RECOMPUTE) {
                // Bounds x0
                if (curr_loc.x < curr_bounds.x0) {
                    // Further out than current bounds x0
                    curr_bounds.x0 = curr_loc.x;
                    curr_bounds.nx0 = 1;
                    if (mc.already_bounds_changed_x[pn->udata] == MoveChangeData::NO_CHANGE) {
                        // Checking already_bounds_changed_x ensures that each net is only added once
                        // to bounds_changed_nets, lest we add its HPWL change multiple times skewing the
                        // overall cost change
                        mc.already_bounds_changed_x[pn->udata] = MoveChangeData::CELL_MOVED_OUTWARDS;
                        mc.bounds_changed_nets_x.push_back(pn->udata);
                    }
                } else if (curr_loc.x == curr_bounds.x0 && old_loc.x > curr_bounds.x0) {
                    curr_bounds.nx0++;
                    if (mc.already_bounds_changed_x[pn->udata] == MoveChangeData::NO_CHANGE) {
                        mc.already_bounds_changed_x[pn->udata] = MoveChangeData::CELL_MOVED_OUTWARDS;
                        mc.bounds_changed_nets_x.push_back(pn->udata);
                    }
                } else if (old_loc.x == curr_bounds.x0 && curr_loc.x > curr_bounds.x0) {
                    if (mc.already_bounds_changed_x[pn->udata] == MoveChangeData::NO_CHANGE)
                        mc.bounds_changed_nets_x.push_back(pn->udata);
                    if (curr_bounds.nx0 == 1) {
                        mc.already_bounds_changed_x[pn->udata] = MoveChangeData::FULL_RECOMPUTE;
                    } else {
                        curr_bounds.nx0--;
                        if (mc.already_bounds_changed_x[pn->udata] == MoveChangeData::NO_CHANGE)
                            mc.already_bounds_changed_x[pn->udata] = MoveChangeData::CELL_MOVED_INWARDS;
                    }
                }

                // Bounds x1
                if (curr_loc.x > curr_bounds.x1) {
                    // Further out than current bounds x1
                    curr_bounds.x1 = curr_loc.x;
                    curr_bounds.nx1 = 1;
                    if (mc.already_bounds_changed_x[pn->udata] == MoveChangeData::NO_CHANGE) {
                        // Checking already_bounds_changed_x ensures that each net is only added once
                        // to bounds_changed_nets, lest we add its HPWL change multiple times skewing the
                        // overall cost change
                        mc.already_bounds_changed_x[pn->udata] = MoveChangeData::CELL_MOVED_OUTWARDS;
                        mc.bounds_changed_nets_x.push_back(pn->udata);
                    }
                } else if (curr_loc.x == curr_bounds.x1 && old_loc.x < curr_bounds.x1) {
                    curr_bounds.nx1++;
                    if (mc.already_bounds_changed_x[pn->udata] == MoveChangeData::NO_CHANGE) {
                        mc.already_bounds_changed_x[pn->udata] = MoveChangeData::CELL_MOVED_OUTWARDS;
                        mc.bounds_changed_nets_x.push_back(pn->udata);
                    }
                } else if (old_loc.x == curr_bounds.x1 && curr_loc.x < curr_bounds.x1) {
                    if (mc.already_bounds_changed_x[pn->udata] == MoveChangeData::NO_CHANGE)
                        mc.bounds_changed_nets_x.push_back(pn->udata);
                    if (curr_bounds.nx1 == 1) {
                        mc.already_bounds_changed_x[pn->udata] = MoveChangeData::FULL_RECOMPUTE;
                    } else {
                        curr_bounds.nx1--;
                        if (mc.already_bounds_changed_x[pn->udata] == MoveChangeData::NO_CHANGE)
                            mc.already_bounds_changed_x[pn->udata] = MoveChangeData::CELL_MOVED_INWARDS;
                    }
                }
            }
            if (mc.already_bounds_changed_y[pn->udata] != MoveChangeData::FULL_RECOMPUTE) {
                // Bounds y0
                if (curr_loc.y < curr_bounds.y0) {
                    // Further out than current bounds y0
                    curr_bounds.y0 = curr_loc.y;
                    curr_bounds.ny0 = 1;
                    if (mc.already_bounds_changed_y[pn->udata] == MoveChangeData::NO_CHANGE) {
                        mc.already_bounds_changed_y[pn->udata] = MoveChangeData::CELL_MOVED_OUTWARDS;
                        mc.bounds_changed_nets_y.push_back(pn->udata);
                    }
                } else if (curr_loc.y == curr_bounds.y0 && old_loc.y > curr_bounds.y0) {
                    curr_bounds.ny0++;
                    if (mc.already_bounds_changed_y[pn->udata] == MoveChangeData::NO_CHANGE) {
                        mc.already_bounds_changed_y[pn->udata] = MoveChangeData::CELL_MOVED_OUTWARDS;
                        mc.bounds_changed_nets_y.push_back(pn->udata);
                    }
                } else if (old_loc.y == curr_bounds.y0 && curr_loc.y > curr_bounds.y0) {
                    if (mc.already_bounds_changed_y[pn->udata] == MoveChangeData::NO_CHANGE)
                        mc.bounds_changed_nets_y.push_back(pn->udata);
                    if (curr_bounds.ny0 == 1) {
                        mc.already_bounds_changed_y[pn->udata] = MoveChangeData::FULL_RECOMPUTE;
                    } else {
                        curr_bounds.ny0--;
                        if (mc.already_bounds_changed_y[pn->udata] == MoveChangeData::NO_CHANGE)
                            mc.already_bounds_changed_y[pn->udata] = MoveChangeData::CELL_MOVED_INWARDS;
                    }
                }

                // Bounds y1
                if (curr_loc.y > curr_bounds.y1) {
                    // Further out than current bounds y1
                    curr_bounds.y1 = curr_loc.y;
                    curr_bounds.ny1 = 1;
                    if (mc.already_bounds_changed_y[pn->udata] == MoveChangeData::NO_CHANGE) {
                        mc.already_bounds_changed_y[pn->udata] = MoveChangeData::CELL_MOVED_OUTWARDS;
                        mc.bounds_changed_nets_y.push_back(pn->udata);
                    }
                } else if (curr_loc.y == curr_bounds.y1 && old_loc.y < curr_bounds.y1) {
                    curr_bounds.ny1++;
                    if (mc.already_bounds_changed_y[pn->udata] == MoveChangeData::NO_CHANGE) {
                        mc.already_bounds_changed_y[pn->udata] = MoveChangeData::CELL_MOVED_OUTWARDS;
                        mc.bounds_changed_nets_y.push_back(pn->udata);
                    }
                } else if (old_loc.y == curr_bounds.y1 && curr_loc.y < curr_bounds.y1) {
                    if (mc.already_bounds_changed_y[pn->udata] == MoveChangeData::NO_CHANGE)
                        mc.bounds_changed_nets_y.push_back(pn->udata);
                    if (curr_bounds.ny1 == 1) {
                        mc.already_bounds_changed_y[pn->udata] = MoveChangeData::FULL_RECOMPUTE;
                    } else {
                        curr_bounds.ny1--;
                        if (mc.already_bounds_changed_y[pn->udata] == MoveChangeData::NO_CHANGE)
                            mc.already_bounds_changed_y[pn->udata] = MoveChangeData::CELL_MOVED_INWARDS;
                    }
                }
            }

            if (cfg.timing_driven && int(pn->users.size()) < cfg.timingFanoutThresh) {
                // Output ports - all arcs change timing
                if (port.second.type == PORT_OUT) {
                    int cc;
                    TimingPortClass cls = ctx->getPortTimingClass(cell, port.first, cc);
                    if (cls != TMG_IGNORE)
                        for (size_t i = 0; i < pn->users.size(); i++)
                            if (!mc.already_changed_arcs[pn->udata][i]) {
                                mc.changed_arcs.emplace_back(std::make_pair(pn->udata, i));
                                mc.already_changed_arcs[pn->udata][i] = true;
                            }
                } else if (port.second.type == PORT_IN) {
                    auto usr = fast_port_to_user.at(&port.second);
                    if (!mc.already_changed_arcs[pn->udata][usr]) {
                        mc.changed_arcs.emplace_back(std::make_pair(pn->udata, usr));
                        mc.already_changed_arcs[pn->udata][usr] = true;
                    }
                }
            }
        }
    }

    void compute_cost_changes(MoveChangeData &md)
    {
        for (const auto &bc : md.bounds_changed_nets_x) {
            if (md.already_bounds_changed_x[bc] == MoveChangeData::FULL_RECOMPUTE)
                md.new_net_bounds[bc] = get_net_bounds(net_by_udata[bc]);
        }
        for (const auto &bc : md.bounds_changed_nets_y) {
            if (md.already_bounds_changed_x[bc] != MoveChangeData::FULL_RECOMPUTE &&
                md.already_bounds_changed_y[bc] == MoveChangeData::FULL_RECOMPUTE)
                md.new_net_bounds[bc] = get_net_bounds(net_by_udata[bc]);
        }

        for (const auto &bc : md.bounds_changed_nets_x)
            md.wirelen_delta += md.new_net_bounds[bc].hpwl(cfg) - net_bounds[bc].hpwl(cfg);
        for (const auto &bc : md.bounds_changed_nets_y)
            if (md.already_bounds_changed_x[bc] == MoveChangeData::NO_CHANGE)
                md.wirelen_delta += md.new_net_bounds[bc].hpwl(cfg) - net_bounds[bc].hpwl(cfg);

        if (cfg.timing_driven) {
            for (const auto &tc : md.changed_arcs) {
                double old_cost = net_arc_tcost.at(tc.first).at(tc.second);
                double new_cost = get_timing_cost(net_by_udata.at(tc.first), tc.second);
                md.new_arc_costs.emplace_back(std::make_pair(tc, new_cost));
                md.timing_delta += (new_cost - old_cost);
                md.already_changed_arcs[tc.first][tc.second] = false;
            }
        }
    }

    void commit_cost_changes(MoveChangeData &md)
    {
        for (const auto &bc : md.bounds_changed_nets_x)
            net_bounds[bc] = md.new_net_bounds[bc];
        for (const auto &bc : md.bounds_changed_nets_y)
            net_bounds[bc] = md.new_net_bounds[bc];
        for (const auto &tc : md.new_arc_costs)
            net_arc_tcost[tc.first.first].at(tc.first.second) = tc.second;
        curr_wirelen_cost += md.wirelen_delta;
        curr_timing_cost += md.timing_delta;
    }
    // Build the cell port -> user index
    void build_port_index()
    {
        for (auto net : sorted(ctx->nets)) {
            NetInfo *ni = net.second;
            for (size_t i = 0; i < ni->users.size(); i++) {
                auto &usr = ni->users.at(i);
                fast_port_to_user[&(usr.cell->ports.at(usr.port))] = i;
            }
        }
    }

    // Simple routeability driven placement
    const int large_cell_thresh = 50;
    int total_net_share = 0;
    std::vector<std::vector<std::unordered_map<IdString, int>>> nets_by_tile;
    void setup_nets_by_tile()
    {
        total_net_share = 0;
        nets_by_tile.resize(max_x + 1, std::vector<std::unordered_map<IdString, int>>(max_y + 1));
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (int(ci->ports.size()) > large_cell_thresh)
                continue;
            Loc loc = ctx->getBelLocation(ci->bel);
            auto &nbt = nets_by_tile.at(loc.x).at(loc.y);
            for (const auto &port : ci->ports) {
                if (port.second.net == nullptr)
                    continue;
                if (port.second.net->driver.cell == nullptr || ctx->getBelGlobalBuf(port.second.net->driver.cell->bel))
                    continue;
                int &s = nbt[port.second.net->name];
                if (s > 0)
                    ++total_net_share;
                ++s;
            }
        }
    }

    int update_nets_by_tile(CellInfo *ci, Loc old_loc, Loc new_loc)
    {
        if (int(ci->ports.size()) > large_cell_thresh)
            return 0;
        int loss = 0, gain = 0;
        auto &nbt_old = nets_by_tile.at(old_loc.x).at(old_loc.y);
        auto &nbt_new = nets_by_tile.at(new_loc.x).at(new_loc.y);

        for (const auto &port : ci->ports) {
            if (port.second.net == nullptr)
                continue;
            if (port.second.net->driver.cell == nullptr || ctx->getBelGlobalBuf(port.second.net->driver.cell->bel))
                continue;
            int &o = nbt_old[port.second.net->name];
            --o;
            NPNR_ASSERT(o >= 0);
            if (o > 0)
                ++loss;
            int &n = nbt_new[port.second.net->name];
            if (n > 0)
                ++gain;
            ++n;
        }
        int delta = gain - loss;
        total_net_share += delta;
        return delta;
    }

    // Get the combined wirelen/timing metric
    inline double curr_metric()
    {
        return lambda * curr_timing_cost + (1 - lambda) * curr_wirelen_cost - cfg.netShareWeight * total_net_share;
    }

    // Map nets to their bounding box (so we can skip recompute for moves that do not exceed the bounds
    std::vector<BoundingBox> net_bounds;
    // Map net arcs to their timing cost (criticality * delay ns)
    std::vector<std::vector<double>> net_arc_tcost;

    // Fast lookup for cell port to net user index
    std::unordered_map<const PortInfo *, size_t> fast_port_to_user;

    // Wirelength and timing cost at last and current iteration
    wirelen_t last_wirelen_cost, curr_wirelen_cost;
    double last_timing_cost, curr_timing_cost;

    // Criticality data from timing analysis
    NetCriticalityMap net_crit;

    Context *ctx;
    float temp = 10;
    float crit_exp = 8;
    float lambda = 0.5;
    bool improved = false;
    int n_move, n_accept;
    int diameter = 35, max_x = 1, max_y = 1;
    std::unordered_map<IdString, std::tuple<int, int>> bel_types;
    std::unordered_map<IdString, BoundingBox> region_bounds;
    std::vector<std::vector<std::vector<std::vector<BelId>>>> fast_bels;
    std::unordered_set<BelId> locked_bels;
    std::vector<NetInfo *> net_by_udata;
    std::vector<decltype(NetInfo::udata)> old_udata;
    bool require_legal = true;
    const int legalise_dia = 4;
    Placer1Cfg cfg;
};

Placer1Cfg::Placer1Cfg(Context *ctx)
{
    constraintWeight = ctx->setting<float>("placer1/constraintWeight", 10);
    netShareWeight = ctx->setting<float>("placer1/netShareWeight", 0);
    minBelsForGridPick = ctx->setting<int>("placer1/minBelsForGridPick", 64);
    budgetBased = ctx->setting<bool>("placer1/budgetBased", false);
    startTemp = ctx->setting<float>("placer1/startTemp", 1);
    timingFanoutThresh = std::numeric_limits<int>::max();
    timing_driven = ctx->setting<bool>("timing_driven");
    slack_redist_iter = ctx->setting<int>("slack_redist_iter");
    hpwl_scale_x = 1;
    hpwl_scale_y = 1;
}

bool placer1(Context *ctx, Placer1Cfg cfg)
{
    try {
        SAPlacer placer(ctx, cfg);
        placer.place();
        log_info("Checksum: 0x%08x\n", ctx->checksum());
#ifndef NDEBUG
        ctx->lock();
        ctx->check();
        ctx->unlock();
#endif
        return true;
    } catch (log_execution_error_exception) {
#ifndef NDEBUG
        ctx->check();
#endif
        return false;
    }
}

bool placer1_refine(Context *ctx, Placer1Cfg cfg)
{
    try {
        SAPlacer placer(ctx, cfg);
        placer.place(true);
        log_info("Checksum: 0x%08x\n", ctx->checksum());
#ifndef NDEBUG
        ctx->lock();
        ctx->check();
        ctx->unlock();
#endif
        return true;
    } catch (log_execution_error_exception) {
#ifndef NDEBUG
        ctx->check();
#endif
        return false;
    }
}

NEXTPNR_NAMESPACE_END
