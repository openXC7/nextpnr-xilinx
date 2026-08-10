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

#include "pack.h"
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
#include "pins.h"

NEXTPNR_NAMESPACE_BEGIN

// Process the contents of packed_cells and new_cells
void XilinxPacker::flush_cells()
{
    for (auto pcell : packed_cells) {
        for (auto &port : ctx->cells[pcell]->ports) {
            disconnect_port(ctx, ctx->cells[pcell].get(), port.first);
        }
        ctx->cells.erase(pcell);
    }
    for (auto &ncell : new_cells) {
        NPNR_ASSERT(!ctx->cells.count(ncell->name));
        ctx->cells[ncell->name] = std::move(ncell);
    }
    packed_cells.clear();
    new_cells.clear();
}

void XilinxPacker::xform_cell(const std::unordered_map<IdString, XFormRule> &rules, CellInfo *ci)
{
    auto &rule = rules.at(ci->type);
    ci->attrs[ctx->id("X_ORIG_TYPE")] = ci->type.str(ctx);
    ci->type = rule.new_type;
    std::vector<IdString> orig_port_names;
    for (auto &port : ci->ports)
        orig_port_names.push_back(port.first);

    for (auto pname : orig_port_names) {
        if (rule.port_multixform.count(pname)) {
            auto old_port = ci->ports.at(pname);
            disconnect_port(ctx, ci, pname);
            ci->ports.erase(pname);
            for (auto new_name : rule.port_multixform.at(pname)) {
                ci->ports[new_name].name = new_name;
                ci->ports[new_name].type = old_port.type;
                connect_port(ctx, old_port.net, ci, new_name);
                ci->attrs[ctx->id("X_ORIG_PORT_" + new_name.str(ctx))] = pname.str(ctx);
            }
        } else {
            IdString new_name;
            if (rule.port_xform.count(pname)) {
                new_name = rule.port_xform.at(pname);
            } else {
                std::string stripped_name;
                for (auto c : pname.str(ctx))
                    if (c != '[' && c != ']')
                        stripped_name += c;
                new_name = ctx->id(stripped_name);
            }
            if (new_name != pname) {
                rename_port(ctx, ci, pname, new_name);
            }
            ci->attrs[ctx->id("X_ORIG_PORT_" + new_name.str(ctx))] = pname.str(ctx);
        }
    }

    std::vector<IdString> xform_params;
    for (auto &param : ci->params)
        if (rule.param_xform.count(param.first))
            xform_params.push_back(param.first);
    for (auto param : xform_params)
        ci->params[rule.param_xform.at(param)] = ci->params[param];

    for (auto &attr : rule.set_attrs)
        ci->attrs[attr.first] = attr.second;

    for (auto &param : rule.set_params)
        ci->params[param.first] = param.second;
}

void XilinxPacker::generic_xform(const std::unordered_map<IdString, XFormRule> &rules, bool print_summary)
{
    std::map<std::string, int> cell_count;
    std::map<std::string, int> new_types;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (rules.count(ci->type)) {
            cell_count[ci->type.str(ctx)]++;
            xform_cell(rules, ci);
            new_types[ci->type.str(ctx)]++;
        }
    }
    if (print_summary) {
        for (auto &nt : new_types) {
            log_info("    Created %d %s cells from:\n", nt.second, nt.first.c_str());
            for (auto &cc : cell_count) {
                if (rules.at(ctx->id(cc.first)).new_type != ctx->id(nt.first))
                    continue;
                log_info("        %6dx %s\n", cc.second, cc.first.c_str());
            }
        }
    }
}

std::unique_ptr<CellInfo> XilinxPacker::feed_through_lut(NetInfo *net, const std::vector<PortRef> &feed_users)
{
    // The feedthrough net name MUST be unique.  ctx->nets[netname] = ... below
    // silently OVERWRITES an existing entry (unlike create_internal_net, which
    // asserts), so a repeated name destroys the earlier net while the cells
    // wired to it keep their pointers.  On this design 398 of 569 feedthroughs
    // collapsed onto a single name, "$legal$1", leaving a LUT driving the very
    // net it was feeding through; timing analysis then reported the impossible
    // "negative fanin count for $legal$1$LUT$2.O6" with nothing to point at.
    // Bump the counter until both the net and the cell names are free.
    std::string base = net->name.str(ctx);
    // A feedthrough on a net still driven by a design GND/VCC CELL means
    // constant packing never claimed it -- the symptom that hid the
    // use-after-free in pack_constants.  PSEUDO_GND/PSEUDO_VCC are excluded on
    // purpose: feeding those through is deliberate (a constant CARRY4 DI must
    // come from a local LUT, see pack_carry_xc7), and warning on it would bury
    // the real case in 28 false alarms.
    if (net->driver.cell != nullptr &&
        (net->driver.cell->type == ctx->id("GND") || net->driver.cell->type == ctx->id("VCC")))
        log_warning("feeding through net '%s', still driven by a %s cell with %d users -- "
                    "constant packing did not claim it\n",
                    base.c_str(), net->driver.cell->type.c_str(ctx), int(net->users.size()));
    std::string nn, cn;
    do {
        nn = base + "$legal$" + std::to_string(++autoidx);
        cn = base + "$LUT$" + std::to_string(++autoidx);
    } while (ctx->nets.count(ctx->id(nn)) || ctx->cells.count(ctx->id(cn)));
    std::unique_ptr<NetInfo> feedthru_net{new NetInfo};
    feedthru_net->name = ctx->id(nn);
    std::unique_ptr<CellInfo> lut = create_lut(ctx, cn, {net}, feedthru_net.get(), Property(2));

    for (auto &usr : feed_users) {
        disconnect_port(ctx, usr.cell, usr.port);
        connect_port(ctx, feedthru_net.get(), usr.cell, usr.port);
    }

    IdString netname = feedthru_net->name;
    ctx->nets[netname] = std::move(feedthru_net);
    return lut;
}

std::unique_ptr<CellInfo> XilinxPacker::feed_through_muxf(NetInfo *net, IdString type,
                                                          const std::vector<PortRef> &feed_users)
{
    // Same uniqueness requirement as feed_through_lut above.
    std::unique_ptr<NetInfo> feedthru_net{new NetInfo};
    {
        std::string base = net->name.str(ctx), nn;
        do {
            nn = base + "$legal$" + std::to_string(++autoidx);
        } while (ctx->nets.count(ctx->id(nn)));
        feedthru_net->name = ctx->id(nn);
    }
    std::unique_ptr<CellInfo> mux =
            create_cell(ctx, type, ctx->id(net->name.str(ctx) + "$MUX$" + std::to_string(++autoidx)));
    connect_port(ctx, net, mux.get(), ctx->id("I0"));
    connect_port(ctx, feedthru_net.get(), mux.get(), ctx->id("O"));
    connect_port(ctx, ctx->nets[ctx->id("$PACKER_GND_NET")].get(), mux.get(), ctx->id("S"));

    for (auto &usr : feed_users) {
        disconnect_port(ctx, usr.cell, usr.port);
        connect_port(ctx, feedthru_net.get(), usr.cell, usr.port);
    }

    IdString netname = feedthru_net->name;
    ctx->nets[netname] = std::move(feedthru_net);
    return mux;
}

IdString XilinxPacker::int_name(IdString base, const std::string &postfix, bool is_hierarchy)
{
    return ctx->id(base.str(ctx) + (is_hierarchy ? "$subcell$" : "$intcell$") + postfix);
}

NetInfo *XilinxPacker::create_internal_net(IdString base, const std::string &postfix, bool is_hierarchy)
{
    std::unique_ptr<NetInfo> net{new NetInfo};
    IdString name = ctx->id(base.str(ctx) + (is_hierarchy ? "$subnet$" : "$intnet$") + postfix);
    net->name = name;
    NPNR_ASSERT(!ctx->nets.count(name));
    ctx->nets[name] = std::move(net);
    return ctx->nets.at(name).get();
}

void XilinxPacker::pack_luts()
{
    log_info("Packing LUTs..\n");

    std::unordered_map<IdString, XFormRule> lut_rules;
    for (int k = 1; k <= 6; k++) {
        IdString lut = ctx->id("LUT" + std::to_string(k));
        lut_rules[lut].new_type = id_SLICE_LUTX;
        for (int i = 0; i < k; i++)
            lut_rules[lut].port_xform[ctx->id("I" + std::to_string(i))] = ctx->id("A" + std::to_string(i + 1));
        lut_rules[lut].port_xform[ctx->id("O")] = ctx->id("O6");
    }
    lut_rules[ctx->id("LUT6_2")] = lut_rules[ctx->id("LUT6")];
    generic_xform(lut_rules, true);
}

void XilinxPacker::pack_ffs()
{
    log_info("Packing flipflops..\n");

    std::unordered_map<IdString, XFormRule> ff_rules;
    ff_rules[ctx->id("FDCE")].new_type = id_SLICE_FFX;
    ff_rules[ctx->id("FDCE")].port_xform[ctx->id("C")] = ctx->xc7 ? id_CK : id_CLK;
    ff_rules[ctx->id("FDCE")].port_xform[ctx->id("CLR")] = id_SR;
    // ff_rules[ctx->id("FDCE")].param_xform[ctx->id("IS_CLR_INVERTED")] = ctx->id("IS_SR_INVERTED");

    ff_rules[ctx->id("FDPE")].new_type = id_SLICE_FFX;
    ff_rules[ctx->id("FDPE")].port_xform[ctx->id("C")] = ctx->xc7 ? id_CK : id_CLK;
    ff_rules[ctx->id("FDPE")].port_xform[ctx->id("PRE")] = id_SR;
    // ff_rules[ctx->id("FDPE")].param_xform[ctx->id("IS_PRE_INVERTED")] = ctx->id("IS_SR_INVERTED");

    ff_rules[ctx->id("FDRE")].new_type = id_SLICE_FFX;
    ff_rules[ctx->id("FDRE")].port_xform[ctx->id("C")] = ctx->xc7 ? id_CK : id_CLK;
    ff_rules[ctx->id("FDRE")].port_xform[ctx->id("R")] = id_SR;
    ff_rules[ctx->id("FDRE")].set_attrs.emplace_back(ctx->id("X_FFSYNC"), "1");
    // ff_rules[ctx->id("FDRE")].param_xform[ctx->id("IS_R_INVERTED")] = ctx->id("IS_SR_INVERTED");

    ff_rules[ctx->id("FDSE")].new_type = id_SLICE_FFX;
    ff_rules[ctx->id("FDSE")].port_xform[ctx->id("C")] = ctx->xc7 ? id_CK : id_CLK;
    ff_rules[ctx->id("FDSE")].port_xform[ctx->id("S")] = id_SR;
    ff_rules[ctx->id("FDSE")].set_attrs.emplace_back(ctx->id("X_FFSYNC"), "1");
    // ff_rules[ctx->id("FDSE")].param_xform[ctx->id("IS_S_INVERTED")] = ctx->id("IS_SR_INVERTED");

    ff_rules[ctx->id("FDCE_1")] = ff_rules[ctx->id("FDCE")];
    ff_rules[ctx->id("FDCE_1")].set_params.emplace_back(ctx->id("IS_CLK_INVERTED"), 1);

    ff_rules[ctx->id("FDPE_1")] = ff_rules[ctx->id("FDPE")];
    ff_rules[ctx->id("FDPE_1")].set_params.emplace_back(ctx->id("IS_CLK_INVERTED"), 1);

    ff_rules[ctx->id("FDRE_1")] = ff_rules[ctx->id("FDRE")];
    ff_rules[ctx->id("FDRE_1")].set_params.emplace_back(ctx->id("IS_CLK_INVERTED"), 1);

    ff_rules[ctx->id("FDSE_1")] = ff_rules[ctx->id("FDSE")];
    ff_rules[ctx->id("FDSE_1")].set_params.emplace_back(ctx->id("IS_CLK_INVERTED"), 1);

    ff_rules[ctx->id("LDCE")].new_type = id_SLICE_FFX;
    ff_rules[ctx->id("LDCE")].port_xform[ctx->id("G")] = ctx->xc7 ? id_CK : id_CLK;
    ff_rules[ctx->id("LDCE")].port_xform[ctx->id("GE")] = id_CE;
    ff_rules[ctx->id("LDCE")].port_xform[ctx->id("CLR")] = id_SR;
    ff_rules[ctx->id("LDCE")].param_xform[ctx->id("IS_G_INVERTED")] = ctx->id("IS_CLK_INVERTED");
    ff_rules[ctx->id("LDCE")].set_attrs.emplace_back(ctx->id("X_FF_AS_LATCH"), "1");

    ff_rules[ctx->id("LDPE")].new_type = id_SLICE_FFX;
    ff_rules[ctx->id("LDPE")].port_xform[ctx->id("G")] = ctx->xc7 ? id_CK : id_CLK;
    ff_rules[ctx->id("LDPE")].port_xform[ctx->id("GE")] = id_CE;
    ff_rules[ctx->id("LDPE")].port_xform[ctx->id("PRE")] = id_SR;
    ff_rules[ctx->id("LDPE")].param_xform[ctx->id("IS_G_INVERTED")] = ctx->id("IS_CLK_INVERTED");
    ff_rules[ctx->id("LDPE")].set_attrs.emplace_back(ctx->id("X_FF_AS_LATCH"), "1");

    generic_xform(ff_rules, true);

    // Const-tied FF control pins (SR=GND meaning "no reset", CE=VCC meaning "always
    // enabled") are realised by SLICE-local config muxes (SRUSEDMUX/CEUSEDMUX), not by
    // fabric routing. The FASM backend already derives those bits from the pin being on
    // $PACKER_GND_NET / $PACKER_VCC_NET (or absent) -- see is_srused/is_ceused in
    // fasm.cc -- so disconnecting these pins is bitstream-neutral. It does, however, keep
    // them off the global const nets, which the router would otherwise have to fan out as
    // one enormous net (the dominant open-flow routing bottleneck: ~141 FF resets alone).
    {
        IdString gnd = ctx->id("$PACKER_GND_NET");
        IdString vcc = ctx->id("$PACKER_VCC_NET");
        int n_sr = 0, n_ce = 0;
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type != id_SLICE_FFX)
                continue;
            NetInfo *sr = get_net_or_empty(ci, id_SR);
            if (sr != nullptr && sr->name == gnd) {
                disconnect_port(ctx, ci, id_SR);
                ++n_sr;
            }
            NetInfo *ce = get_net_or_empty(ci, id_CE);
            if (ce != nullptr && ce->name == vcc) {
                disconnect_port(ctx, ci, id_CE);
                ++n_ce;
            }
        }
        log_info("    local-const FF control: disconnected %d SR(=GND) + %d CE(=VCC) pins "
                 "from global nets\n", n_sr, n_ce);

        // Unused RAMB36 data-cascade inputs are tied to a constant by the
        // netlist but are don't-care when the cascade is off, and Vivado
        // leaves them unrouted.  With the BRAM cascade pips blacklisted
        // (they were being abused as address-fanout shortcuts) the tie is
        // also UNROUTABLE -- and one unroutable const arc aborts router1's
        // whole final pass.  Disconnect like the FF controls above.
        int n_casc = 0;
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type != ctx->id("RAMB36E1_RAMB36E1"))
                continue;
            for (auto p : {"CASCADEINA", "CASCADEINB"}) {
                NetInfo *nn = get_net_or_empty(ci, ctx->id(p));
                if (nn != nullptr && (nn->name == gnd || nn->name == vcc)) {
                    disconnect_port(ctx, ci, ctx->id(p));
                    ++n_casc;
                }
            }
        }
        if (n_casc > 0)
            log_info("    local-const BRAM cascade: disconnected %d CASCADEIN pins\n", n_casc);

        // GT dedicated refclk inputs (GTNORTHREFCLK*, GTSOUTHREFCLK*,
        // GTGREFCLK*) are tied to constants by the netlist but have NO
        // general-fabric route (dedicated clock spines, absent from the
        // open db); refclk selection is GT config, and Vivado leaves the
        // unused inputs unrouted.  An unroutable const arc here was the
        // arc that silently aborted router1's whole final const pass on
        // every ethsoc build (thousands of const sinks left floating).
        int n_gt = 0;
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type != ctx->id("GTXE2_COMMON") && ci->type != ctx->id("GTXE2_CHANNEL"))
                continue;
            std::vector<IdString> victims;
            for (auto &port : ci->ports) {
                const std::string &pn = port.first.str(ctx);
                if (pn.find("GTNORTHREFCLK") != 0 && pn.find("GTSOUTHREFCLK") != 0 &&
                    pn.find("GTGREFCLK") != 0)
                    continue;
                NetInfo *nn = port.second.net;
                if (nn != nullptr && (nn->name == gnd || nn->name == vcc))
                    victims.push_back(port.first);
            }
            for (auto p : victims) {
                disconnect_port(ctx, ci, p);
                ++n_gt;
            }
        }
        if (n_gt > 0)
            log_info("    local-const GT refclk: disconnected %d dedicated-input pins\n", n_gt);
    }
}

void XilinxPacker::pack_lutffs()
{
    int pairs = 0;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->constr_parent != nullptr || !ci->constr_children.empty())
            continue;
        if (ci->type != id_SLICE_FFX)
            continue;
        // Don't force a LUT+FF cluster onto cells whose placement is already
        // pinned by an absolute BEL attribute (e.g. an imported Vivado
        // placement): the LUT and the FF it drives may legitimately sit in
        // different slices, and a relative-Z constraint would contradict the
        // pin and break legalisation.
        if (ci->attrs.count(ctx->id("BEL")))
            continue;
        NetInfo *d = get_net_or_empty(ci, id_D);
        if (d->driver.cell == nullptr || d->driver.cell->type != id_SLICE_LUTX || d->driver.port != id_O6)
            continue;
        CellInfo *lut = d->driver.cell;
        if (lut->constr_parent != nullptr || !lut->constr_children.empty())
            continue;
        if (lut->attrs.count(ctx->id("BEL")))
            continue;
        lut->constr_children.push_back(ci);
        ci->constr_parent = lut;
        ci->constr_x = 0;
        ci->constr_y = 0;
        ci->constr_z = (BEL_FF - BEL_6LUT);
        ++pairs;
    }
    log_info("Constrained %d LUTFF pairs.\n", pairs);
}

bool XilinxPacker::is_constrained(const CellInfo *cell)
{
    return cell->constr_x != cell->UNCONSTR || cell->constr_y != cell->UNCONSTR || cell->constr_z != cell->UNCONSTR;
}

void XilinxPacker::legalise_muxf_tree(CellInfo *curr, std::vector<CellInfo *> &mux_roots)
{
    if (curr->type.str(ctx).substr(0, 3) == "LUT")
        return;
    for (IdString p : {ctx->id("I0"), ctx->id("I1")}) {
        NetInfo *pn = get_net_or_empty(curr, p);
        if (pn == nullptr || pn->driver.cell == nullptr)
            continue;
        if (curr->type == ctx->id("MUXF7")) {
            if (curr->attrs.count(ctx->id("BEL")) && pn->driver.cell->attrs.count(ctx->id("BEL")) &&
                pn->driver.cell->type.str(ctx).substr(0, 3) == "LUT")
                continue;   // pinned pair: stamped placement is authoritative
            if (pn->driver.cell->type.str(ctx).substr(0, 3) != "LUT" || is_constrained(pn->driver.cell)) {
                PortRef pr;
                pr.cell = curr;
                pr.port = p;
                auto i_feed = feed_through_lut(pn, {pr});
                new_cells.push_back(std::move(i_feed));
                continue;
            }
        } else {
            IdString next_type;
            if (curr->type == ctx->id("MUXF9"))
                next_type = ctx->id("MUXF8");
            else if (curr->type == ctx->id("MUXF8"))
                next_type = ctx->id("MUXF7");
            else
                NPNR_ASSERT_FALSE("bad mux type");
            // BEL-pinned trees (R0 flow: Vivado placement stamped as BEL
            // attrs): a pinned F7 feeding a pinned F8 in the same slice is
            // hardware-direct even when the F7 output ALSO fans out to
            // fabric (F7MUX_OUT drives the F8 leg and the slice output mux
            // simultaneously) -- never feed it through, the synthetic $MUX$
            // child could not bind anyway (the real pinned mux owns the
            // BEL).
            bool pinned_pair = curr->attrs.count(ctx->id("BEL")) &&
                               pn->driver.cell->attrs.count(ctx->id("BEL"));
            if (!pinned_pair &&
                (pn->driver.cell->type != next_type || is_constrained(pn->driver.cell) ||
                 bool_or_default(pn->driver.cell->attrs, ctx->id("MUX_TREE_ROOT")))) {
                PortRef pr;
                pr.cell = curr;
                pr.port = p;
                auto i_feed = feed_through_muxf(pn, next_type, {pr});
                new_cells.push_back(std::move(i_feed));
                continue;
            }
        }
        legalise_muxf_tree(pn->driver.cell, mux_roots);
    }
}

void XilinxPacker::constrain_muxf_tree(CellInfo *curr, CellInfo *base, int zoffset)
{

    if (curr->type == id_SLICE_LUTX && (curr->constr_abs_z || curr->constr_parent != nullptr))
        return;

    int base_z = 0;
    if (base->type == ctx->id("MUXF7"))
        base_z = BEL_F7MUX;
    else if (base->type == ctx->id("MUXF8"))
        base_z = BEL_F8MUX;
    else if (base->type == ctx->id("MUXF9"))
        base_z = BEL_F9MUX;
    else if (base->constr_abs_z)
        base_z = base->constr_z;
    else
        NPNR_ASSERT_FALSE("unexpected mux base type");
    int curr_z = zoffset * 16;
    int input_spacing = 0;
    if (curr->type == ctx->id("MUXF7")) {
        curr_z += BEL_F7MUX;
        input_spacing = 1;
    } else if (curr->type == ctx->id("MUXF8")) {
        curr_z += BEL_F8MUX;
        input_spacing = 2;
    } else if (curr->type == ctx->id("MUXF9")) {
        curr_z += BEL_F9MUX;
        input_spacing = 4;
    } else
        curr_z += BEL_6LUT;
    if (curr != base && !curr->attrs.count(ctx->id("BEL"))) {
        curr->constr_x = 0;
        curr->constr_y = 0;
        curr->constr_z = curr_z - base_z;
        curr->constr_abs_z = false;
        curr->constr_parent = base;
        base->constr_children.push_back(curr);
    }
    if (curr->type == ctx->id("MUXF7") || curr->type == ctx->id("MUXF8") || curr->type == ctx->id("MUXF9")) {
        NetInfo *i0 = get_net_or_empty(curr, ctx->id("I0")), *i1 = get_net_or_empty(curr, ctx->id("I1"));
        if (i0 != nullptr && i0->driver.cell != nullptr)
            constrain_muxf_tree(i0->driver.cell, base, zoffset + input_spacing);
        if (i1 != nullptr && i1->driver.cell != nullptr)
            constrain_muxf_tree(i1->driver.cell, base, zoffset);
    }
}

void XilinxPacker::pack_muxfs()
{
    log_info("Packing MUX[789]s..\n");
    std::vector<CellInfo *> mux_roots;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        ci->attrs.erase(ctx->id("MUX_TREE_ROOT"));
        if (ci->type == ctx->id("MUXF9")) {
            if (ctx->xc7)
                log_error("MUXF9 is not supported on xc7!\n");
            mux_roots.push_back(ci);
        } else if (ci->type == ctx->id("MUXF8")) {
            NetInfo *o = get_net_or_empty(ci, ctx->id("O"));
            if (o == nullptr || o->users.size() != 1 || o->users.at(0).cell->type != ctx->id("MUXF9") ||
                is_constrained(o->users.at(0).cell) || o->users.at(0).port == ctx->id("S"))
                mux_roots.push_back(ci);
        } else if (ci->type == ctx->id("MUXF7")) {
            NetInfo *o = get_net_or_empty(ci, ctx->id("O"));
            if (o == nullptr || o->users.size() != 1 || o->users.at(0).cell->type != ctx->id("MUXF8") ||
                is_constrained(o->users.at(0).cell) || o->users.at(0).port == ctx->id("S"))
                mux_roots.push_back(ci);
        }
    }
    for (auto root : mux_roots)
        root->attrs[ctx->id("MUX_TREE_ROOT")] = 1;
    for (auto root : mux_roots)
        legalise_muxf_tree(root, mux_roots);
    for (auto root : mux_roots)
        constrain_muxf_tree(root, root, 0);
}

void XilinxPacker::finalise_muxfs()
{
    std::unordered_map<IdString, XFormRule> muxf_rules;
    muxf_rules[ctx->id("MUXF9")].new_type = id_F9MUX;
    muxf_rules[ctx->id("MUXF9")].port_xform[ctx->id("I0")] = ctx->id("0");
    muxf_rules[ctx->id("MUXF9")].port_xform[ctx->id("I1")] = ctx->id("1");
    muxf_rules[ctx->id("MUXF9")].port_xform[ctx->id("S")] = ctx->id("S0");
    muxf_rules[ctx->id("MUXF9")].port_xform[ctx->id("O")] = ctx->id("OUT");
    muxf_rules[ctx->id("MUXF8")].new_type = ctx->xc7 ? ctx->id("SELMUX2_1") : id_F8MUX;
    muxf_rules[ctx->id("MUXF8")].port_xform = muxf_rules[ctx->id("MUXF9")].port_xform;
    muxf_rules[ctx->id("MUXF7")].new_type = ctx->xc7 ? ctx->id("SELMUX2_1") : id_F7MUX;
    muxf_rules[ctx->id("MUXF7")].port_xform = muxf_rules[ctx->id("MUXF9")].port_xform;
    generic_xform(muxf_rules, true);
}

void XilinxPacker::pack_srls()
{
    std::unordered_map<IdString, XFormRule> srl_rules;
    srl_rules[ctx->id("SRL16E")].new_type = id_SLICE_LUTX;
    srl_rules[ctx->id("SRL16E")].port_xform[ctx->id("CLK")] = id_CLK;
    srl_rules[ctx->id("SRL16E")].port_xform[ctx->id("CE")] = id_WE;
    srl_rules[ctx->id("SRL16E")].port_xform[ctx->id("D")] = id_DI2;
    srl_rules[ctx->id("SRL16E")].port_xform[ctx->id("Q")] = id_O6;
    srl_rules[ctx->id("SRL16E")].set_attrs.emplace_back(ctx->id("X_LUT_AS_SRL"), "1");

    srl_rules[ctx->id("SRLC32E")].new_type = id_SLICE_LUTX;
    srl_rules[ctx->id("SRLC32E")].port_xform[ctx->id("CLK")] = id_CLK;
    srl_rules[ctx->id("SRLC32E")].port_xform[ctx->id("CE")] = id_WE;
    srl_rules[ctx->id("SRLC32E")].port_xform[ctx->id("D")] = id_DI1;
    srl_rules[ctx->id("SRLC32E")].port_xform[ctx->id("Q")] = id_O6;
    // Cascade shiftout: SRLC32E.Q31 is the dedicated MC31 output that feeds the
    // next SRL's DI mux (?DI1MUX <- ?MC31).  Map it to the MC31 port so the
    // cascade routes through the dedicated path (kept as .Q31 by patch_netlist)
    // rather than being dropped ("FIXME: Q31 support").
    srl_rules[ctx->id("SRLC32E")].port_xform[ctx->id("Q31")] = id_MC31;
    srl_rules[ctx->id("SRLC32E")].set_attrs.emplace_back(ctx->id("X_LUT_AS_SRL"), "1");
    // FIXME: Q31 support
    generic_xform(srl_rules, true);
    // Fixup SRL inputs
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type != id_SLICE_LUTX)
            continue;
        std::string orig_type = str_or_default(ci->attrs, ctx->id("X_ORIG_TYPE"));
        if (orig_type == "SRL16E") {
            for (int i = 3; i >= 0; i--) {
                rename_port(ctx, ci, ctx->id("A" + std::to_string(i)), ctx->id("A" + std::to_string(i + 2)));
            }
            // A 5LUT-position SRL16E (imported Vivado placement) takes its
            // data in via DI1 (the AI pin) and outputs on O5; the default
            // mapping above assumes the 6LUT position (DI2/O6).  A6 also
            // doesn't exist on the 5LUT bel.
            bool is5 = false;
            auto bel = ci->attrs.find(ctx->id("BEL"));
            if (bel != ci->attrs.end() &&
                bel->second.as_string().find("5LUT") != std::string::npos)
                is5 = true;
            if (is5) {
                rename_port(ctx, ci, id_DI2, id_DI1);
                rename_port(ctx, ci, id_O6, id_O5);
            }
            for (auto tp : is5 ? std::vector<IdString>{id_A1} : std::vector<IdString>{id_A1, id_A6}) {
                ci->ports[tp].name = tp;
                ci->ports[tp].type = PORT_IN;
                connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, tp);
            }
        } else if (orig_type == "SRLC32E") {
            for (int i = 4; i >= 0; i--) {
                rename_port(ctx, ci, ctx->id("A" + std::to_string(i)), ctx->id("A" + std::to_string(i + 2)));
            }
            for (auto tp : {id_A1}) {
                ci->ports[tp].name = tp;
                ci->ports[tp].type = PORT_IN;
                connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, tp);
            }
        }
    }
}

void XilinxPacker::pack_constants()
{
    log_info("Packing constants..\n");
    if (tied_pins.empty())
        get_tied_pins(ctx, tied_pins);
    if (invertible_pins.empty())
        get_invertible_pins(ctx, invertible_pins);
    if (!ctx->cells.count(ctx->id("$PACKER_GND_DRV"))) {
        // The IMPORTED design may already carry nets with these names: a
        // netlist replayed out of a Vivado DCP has GLOBAL_LOGIC0/1, which our
        // importer emits as $PACKER_GND_NET/$PACKER_VCC_NET precisely so the
        // constant handling below recognises them.
        //
        // "ctx->nets[name] = std::move(...)" OVERWRITES an existing map entry
        // rather than asserting, so creating the packer's own constants on top
        // of them FREES the imported NetInfo while every user cell still holds
        // a pointer to it.  The freed object keeps a plausible driver and user
        // list but its name reads back as a stale index -- on this design the
        // ground net (2869 users) came back named "F9MUX", a cell-type
        // constid.  Nothing then matched `name == "$PACKER_GND_NET"`, so the
        // carry and mux packers treated ground as ordinary signal and fed it
        // through legalisation LUTs until two cells claimed one net.
        //
        // Move any pre-existing net aside instead.  Its driver is a GND/VCC
        // cell, so the loop further down reties its users to the packer's own
        // constants in the normal way and erases it.
        for (const char *cn : {"$PACKER_GND_NET", "$PACKER_VCC_NET"}) {
            IdString id = ctx->id(cn);
            if (ctx->nets.count(id)) {
                log_info("    adopting imported constant net %s (%d user(s))\n", cn,
                         int(ctx->nets.at(id)->users.size()));
                rename_net(id, ctx->id(std::string(cn) + "$imported"));
            }
        }
        std::unique_ptr<CellInfo> gnd_cell{new CellInfo};
        gnd_cell->name = ctx->id("$PACKER_GND_DRV");
        gnd_cell->type = id_PSEUDO_GND;
        gnd_cell->ports[id_Y].name = id_Y;
        gnd_cell->ports[id_Y].type = PORT_OUT;
        std::unique_ptr<NetInfo> gnd_net = std::unique_ptr<NetInfo>(new NetInfo);
        gnd_net->name = ctx->id("$PACKER_GND_NET");
        gnd_net->driver.cell = gnd_cell.get();
        gnd_net->driver.port = id_Y;
        gnd_cell->ports.at(id_Y).net = gnd_net.get();

        std::unique_ptr<CellInfo> vcc_cell{new CellInfo};
        vcc_cell->name = ctx->id("$PACKER_VCC_DRV");
        vcc_cell->type = id_PSEUDO_VCC;
        vcc_cell->ports[id_Y].name = id_Y;
        vcc_cell->ports[id_Y].type = PORT_OUT;
        std::unique_ptr<NetInfo> vcc_net = std::unique_ptr<NetInfo>(new NetInfo);
        vcc_net->name = ctx->id("$PACKER_VCC_NET");
        vcc_net->driver.cell = vcc_cell.get();
        vcc_net->driver.port = id_Y;
        vcc_cell->ports.at(id_Y).net = vcc_net.get();

        ctx->cells[gnd_cell->name] = std::move(gnd_cell);
        ctx->nets[gnd_net->name] = std::move(gnd_net);
        ctx->cells[vcc_cell->name] = std::move(vcc_cell);
        ctx->nets[vcc_net->name] = std::move(vcc_net);
    }
    NetInfo *gnd = ctx->nets[ctx->id("$PACKER_GND_NET")].get(), *vcc = ctx->nets[ctx->id("$PACKER_VCC_NET")].get();

    std::vector<IdString> dead_nets;

    std::vector<std::tuple<CellInfo *, IdString, bool>> const_ports;

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (!tied_pins.count(ci->type))
            continue;
        auto &tp = tied_pins.at(ci->type);
        for (auto port : tp) {
            if (cell.second->ports.count(port.first) && cell.second->ports.at(port.first).net != nullptr &&
                cell.second->ports.at(port.first).net->driver.cell != nullptr)
                continue;
            const_ports.emplace_back(ci, port.first, port.second);
        }
    }

    for (auto net : sorted(ctx->nets)) {
        NetInfo *ni = net.second;
        if (ni->driver.cell != nullptr && ni->driver.cell->type == ctx->id("GND")) {
            IdString drv_cell = ni->driver.cell->name;
            for (auto &usr : ni->users) {
                const_ports.emplace_back(usr.cell, usr.port, false);
                usr.cell->ports.at(usr.port).net = nullptr;
            }
            dead_nets.push_back(net.first);
            ctx->cells.erase(drv_cell);
        } else if (ni->driver.cell != nullptr && ni->driver.cell->type == ctx->id("VCC")) {
            IdString drv_cell = ni->driver.cell->name;
            for (auto &usr : ni->users) {
                const_ports.emplace_back(usr.cell, usr.port, true);
                usr.cell->ports.at(usr.port).net = nullptr;
            }
            dead_nets.push_back(net.first);
            ctx->cells.erase(drv_cell);
        }
    }

    for (auto port : const_ports) {
        CellInfo *ci;
        IdString pname;
        bool cval;
        std::tie(ci, pname, cval) = port;

        if (!ci->ports.count(pname)) {
            ci->ports[pname].name = pname;
            ci->ports[pname].type = PORT_IN;
        }
        if (ci->ports.at(pname).net != nullptr) {
            // Case where a port with a default tie value is previously connected to an undriven net
            NPNR_ASSERT(ci->ports.at(pname).net->driver.cell == nullptr);
            disconnect_port(ctx, ci, pname);
        }

        if (!cval && invertible_pins.count(ci->type) && invertible_pins.at(ci->type).count(pname)) {
            // Invertible pins connected to zero are optimised to a connection to Vcc (which is easier to route)
            // and an inversion
            ci->params[ctx->id("IS_" + pname.str(ctx) + "_INVERTED")] = Property(1);
            cval = true;
        }

        connect_port(ctx, cval ? vcc : gnd, ci, pname);
    }

    for (auto dn : dead_nets) {
        ctx->nets.erase(dn);
    }
}

void XilinxPacker::rename_net(IdString old, IdString newname)
{
    std::unique_ptr<NetInfo> ni;
    std::swap(ni, ctx->nets[old]);
    ctx->nets.erase(old);
    ni->name = newname;
    ctx->nets[newname] = std::move(ni);
}

void XilinxPacker::tie_port(CellInfo *ci, const std::string &port, bool value, bool inv)
{
    IdString p = ctx->id(port);
    if (!ci->ports.count(p)) {
        ci->ports[p].name = p;
        ci->ports[p].type = PORT_IN;
    }
    if (value || inv)
        connect_port(ctx, ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get(), ci, p);
    else
        connect_port(ctx, ctx->nets.at(ctx->id("$PACKER_GND_NET")).get(), ci, p);
    if (!value && inv)
        ci->params[ctx->id("IS_" + port + "_INVERTED")] = Property(1);
}

void XilinxPacker::disconnect_constant_port(CellInfo *ci, IdString port)
{
    auto net = get_net_or_empty(ci, port);
    if (net == nullptr) return;
    if (net->name == ctx->id("$PACKER_GND_NET") || net->name == ctx->id("$PACKER_VCC_NET")) {
        disconnect_port(ctx, ci, port);
    }
}

void USPacker::pack_bram()
{
    log_info("Packing BRAM..\n");

    // Rules for normal TDP BRAM
    std::unordered_map<IdString, XFormRule> bram_rules;
    bram_rules[ctx->id("RAMB18E2")].new_type = id_RAMB18E2_RAMB18E2;
    bram_rules[ctx->id("RAMB18E2")].port_multixform[ctx->id(std::string("WEA[0]"))] = {ctx->id("WEA0"),
                                                                                       ctx->id("WEA1")};
    bram_rules[ctx->id("RAMB18E2")].port_multixform[ctx->id(std::string("WEA[1]"))] = {ctx->id("WEA2"),
                                                                                       ctx->id("WEA3")};
    bram_rules[ctx->id("RAMB36E2")].new_type = id_RAMB36E2_RAMB36E2;

    // Some ports have upper/lower bel pins in 36-bit mode
    std::vector<std::pair<IdString, std::vector<std::string>>> ul_pins;
    get_bram36_ul_pins(ctx, ul_pins);
    for (auto &ul : ul_pins) {
        for (auto &bp : ul.second)
            bram_rules[ctx->id("RAMB36E2")].port_multixform[ul.first].push_back(ctx->id(bp));
    }
    bram_rules[ctx->id("RAMB36E2")].port_multixform[ctx->id("ECCPIPECE")] = {ctx->id("ECCPIPECEL")};

    // Special rules for SDP rules, relating to WE connectivity
    std::unordered_map<IdString, XFormRule> sdp_bram_rules = bram_rules;
    for (int i = 0; i < 2; i++) {
        // Connects to two WEBWE bel pins
        sdp_bram_rules[ctx->id("RAMB18E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .push_back(ctx->id("WEBWE" + std::to_string(i * 2)));
        sdp_bram_rules[ctx->id("RAMB18E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .push_back(ctx->id("WEBWE" + std::to_string(i * 2 + 1)));
        // Not used in SDP mode
        sdp_bram_rules[ctx->id("RAMB18E2")]
                .port_multixform[ctx->id(std::string("WEA[" + std::to_string(i) + "]"))] = {};
    }
    for (int i = 0; i < 2; i++) {
        // Connects to two WEA bel pins
        sdp_bram_rules[ctx->id("RAMB18E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i + 2) + "]"))]
                .push_back(ctx->id("WEA" + std::to_string(i * 2)));
        sdp_bram_rules[ctx->id("RAMB18E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i + 2) + "]"))]
                .push_back(ctx->id("WEA" + std::to_string(i * 2 + 1)));
    }

    for (int i = 0; i < 4; i++) {
        sdp_bram_rules[ctx->id("RAMB36E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .clear();
        sdp_bram_rules[ctx->id("RAMB36E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i + 4) + "]"))]
                .clear();
        // Connects to two WEBWE bel pins
        sdp_bram_rules[ctx->id("RAMB36E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .push_back(ctx->id("WEBWEL" + std::to_string(i)));
        sdp_bram_rules[ctx->id("RAMB36E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .push_back(ctx->id("WEBWEU" + std::to_string(i)));
        sdp_bram_rules[ctx->id("RAMB36E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i + 4) + "]"))]
                .push_back(ctx->id("WEAL" + std::to_string(i)));
        sdp_bram_rules[ctx->id("RAMB36E2")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i + 4) + "]"))]
                .push_back(ctx->id("WEAU" + std::to_string(i)));
        // Not used in SDP mode
        sdp_bram_rules[ctx->id("RAMB36E2")]
                .port_multixform[ctx->id(std::string("WEA[" + std::to_string(i) + "]"))] = {};
    }

    // 72-bit BRAMs: drop upper bits of WEB in TDP mode
    for (int i = 4; i < 8; i++)
        bram_rules[ctx->id("RAMB36E2")].port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))] = {};

    // Process SDP BRAM first
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if ((ci->type == ctx->id("RAMB18E2") &&
             int_or_default(ci->params, ctx->id(std::string("WRITE_WIDTH_B")), 0) == 36) ||
            (ci->type == ctx->id("RAMB36E2") &&
             int_or_default(ci->params, ctx->id(std::string("WRITE_WIDTH_B")), 0) == 72))
            xform_cell(sdp_bram_rules, ci);
    }

    // Rewrite byte enables according to data width
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("RAMB18E2") || ci->type == ctx->id("RAMB36E2")) {
            for (char port : {'A', 'B'}) {
                int write_width = int_or_default(ci->params, ctx->id(std::string("WRITE_WIDTH_") + port), 18);
                int we_width;
                if (ci->type == ctx->id("RAMB36E2"))
                    we_width = 4;
                else
                    we_width = (port == 'B') ? 4 : 2;
                if (write_width >= (9 * we_width))
                    continue;
                int used_we_width = std::max(write_width / 9, 1);
                for (int i = used_we_width; i < we_width; i++) {
                    NetInfo *low_we = get_net_or_empty(ci, ctx->id(std::string(port == 'B' ? "WEBWE[" : "WEA[") +
                                                                   std::to_string(i % used_we_width) + "]"));
                    IdString curr_we = ctx->id(std::string(port == 'B' ? "WEBWE[" : "WEA[") + std::to_string(i) + "]");
                    if (!ci->ports.count(curr_we)) {
                        ci->ports[curr_we].type = PORT_IN;
                        ci->ports[curr_we].name = curr_we;
                    }
                    disconnect_port(ctx, ci, curr_we);
                    connect_port(ctx, low_we, ci, curr_we);
                }
            }
        }
    }

    generic_xform(bram_rules, false);

    // These pins have no logical mapping, so must be tied after transformation
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == id_RAMB18E2_RAMB18E2) {
            for (int i = 2; i < 4; i++) {
                IdString port = ctx->id("WEA" + std::to_string(i));
                if (!ci->ports.count(port)) {
                    ci->ports[port].name = port;
                    ci->ports[port].type = PORT_IN;
                    connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, port);
                }
            }
        }
    }
}

void XC7Packer::pack_bram()
{
    log_info("Packing BRAM..\n");

    // Rules for normal TDP BRAM
    std::unordered_map<IdString, XFormRule> bram_rules;
    bram_rules[ctx->id("RAMB18E1")].new_type = id_RAMB18E1_RAMB18E1;
    bram_rules[ctx->id("RAMB18E1")].port_multixform[ctx->id(std::string("WEA[0]"))] = {ctx->id("WEA0"),
                                                                                       ctx->id("WEA1")};
    bram_rules[ctx->id("RAMB18E1")].port_multixform[ctx->id(std::string("WEA[1]"))] = {ctx->id("WEA2"),
                                                                                       ctx->id("WEA3")};
    bram_rules[ctx->id("RAMB36E1")].new_type = id_RAMB36E1_RAMB36E1;

    // Some ports have upper/lower bel pins in 36-bit mode
    std::vector<std::pair<IdString, std::vector<std::string>>> ul_pins;
    get_bram36_ul_pins(ctx, ul_pins);
    for (auto &ul : ul_pins) {
        for (auto &bp : ul.second)
            bram_rules[ctx->id("RAMB36E1")].port_multixform[ul.first].push_back(ctx->id(bp));
    }
    bram_rules[ctx->id("RAMB36E1")].port_multixform[ctx->id("ADDRARDADDR[15]")].push_back(ctx->id("ADDRARDADDRL15"));
    bram_rules[ctx->id("RAMB36E1")].port_multixform[ctx->id("ADDRBWRADDR[15]")].push_back(ctx->id("ADDRBWRADDRL15"));

    // Special rules for SDP rules, relating to WE connectivity
    std::unordered_map<IdString, XFormRule> sdp_bram_rules = bram_rules;
    for (int i = 0; i < 4; i++) {
        // Connects to two WEBWE bel pins
        sdp_bram_rules[ctx->id("RAMB18E1")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .push_back(ctx->id("WEBWE" + std::to_string(i * 2)));
        sdp_bram_rules[ctx->id("RAMB18E1")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .push_back(ctx->id("WEBWE" + std::to_string(i * 2 + 1)));
        // Not used in SDP mode
        sdp_bram_rules[ctx->id("RAMB18E1")]
                .port_multixform[ctx->id(std::string("WEA[" + std::to_string(i) + "]"))] = {};
    }

    for (int i = 0; i < 8; i++) {
        sdp_bram_rules[ctx->id("RAMB36E1")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .clear();
        // Connects to two WEBWE bel pins
        sdp_bram_rules[ctx->id("RAMB36E1")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .push_back(ctx->id("WEBWEL" + std::to_string(i)));
        sdp_bram_rules[ctx->id("RAMB36E1")]
                .port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))]
                .push_back(ctx->id("WEBWEU" + std::to_string(i)));
        // Not used in SDP mode
        sdp_bram_rules[ctx->id("RAMB36E1")]
                .port_multixform[ctx->id(std::string("WEA[" + std::to_string(i) + "]"))] = {};
    }

    // 72-bit BRAMs: drop upper bits of WEB in TDP mode
    for (int i = 4; i < 8; i++)
        bram_rules[ctx->id("RAMB36E1")].port_multixform[ctx->id(std::string("WEBWE[" + std::to_string(i) + "]"))] = {};

    // Process SDP BRAM first
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if ((ci->type == ctx->id("RAMB18E1") &&
             int_or_default(ci->params, ctx->id(std::string("WRITE_WIDTH_B")), 0) == 36) ||
            (ci->type == ctx->id("RAMB36E1") &&
             int_or_default(ci->params, ctx->id(std::string("WRITE_WIDTH_B")), 0) == 72))
            xform_cell(sdp_bram_rules, ci);
    }

    // Rewrite byte enables according to data width
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("RAMB18E1") || ci->type == ctx->id("RAMB36E1")) {
            for (char port : {'A', 'B'}) {
                int write_width = int_or_default(ci->params, ctx->id(std::string("WRITE_WIDTH_") + port), 18);
                int we_width;
                if (ci->type == ctx->id("RAMB36E1"))
                    we_width = 4;
                else
                    we_width = (port == 'B') ? 4 : 2;
                if (write_width >= (9 * we_width))
                    continue;
                int used_we_width = std::max(write_width / 9, 1);
                for (int i = used_we_width; i < we_width; i++) {
                    NetInfo *low_we = get_net_or_empty(ci, ctx->id(std::string(port == 'B' ? "WEBWE[" : "WEA[") +
                                                                   std::to_string(i % used_we_width) + "]"));
                    IdString curr_we = ctx->id(std::string(port == 'B' ? "WEBWE[" : "WEA[") + std::to_string(i) + "]");
                    if (!ci->ports.count(curr_we)) {
                        ci->ports[curr_we].type = PORT_IN;
                        ci->ports[curr_we].name = curr_we;
                    }
                    disconnect_port(ctx, ci, curr_we);
                    connect_port(ctx, low_we, ci, curr_we);
                }
            }
        }
    }

    generic_xform(bram_rules, false);

    // These pins have no logical mapping, so must be tied after transformation
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == id_RAMB18E1_RAMB18E1) {
            int wwa = int_or_default(ci->params, ctx->id("WRITE_WIDTH_A"), 0);
            for (int i = ((wwa == 0) ? 0 : 2); i < 4; i++) {
                IdString port = ctx->id("WEA" + std::to_string(i));
                if (!ci->ports.count(port)) {
                    ci->ports[port].name = port;
                    ci->ports[port].type = PORT_IN;
                    connect_port(ctx, ctx->nets[ctx->id("$PACKER_GND_NET")].get(), ci, port);
                }
            }
            int wwb = int_or_default(ci->params, ctx->id("WRITE_WIDTH_B"), 0);
            if (wwb != 36) {
                for (int i = 4; i < 8; i++) {
                    IdString port = ctx->id("WEBWE" + std::to_string(i));
                    if (!ci->ports.count(port)) {
                        ci->ports[port].name = port;
                        ci->ports[port].type = PORT_IN;
                        connect_port(ctx, ctx->nets[ctx->id("$PACKER_GND_NET")].get(), ci, port);
                    }
                }
            }
            for (auto p : {ctx->id("ADDRATIEHIGH0"), ctx->id("ADDRATIEHIGH1"), ctx->id("ADDRBTIEHIGH0"),
                           ctx->id("ADDRBTIEHIGH1")}) {
                if (!ci->ports.count(p)) {
                    ci->ports[p].name = p;
                    ci->ports[p].type = PORT_IN;
                } else {
                    disconnect_port(ctx, ci, p);
                }
                connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, p);
            }
        } else if (ci->type == id_RAMB36E1_RAMB36E1) {
            for (auto p : {ctx->id("ADDRARDADDRL15"), ctx->id("ADDRBWRADDRL15")}) {
                if (!ci->ports.count(p)) {
                    ci->ports[p].name = p;
                    ci->ports[p].type = PORT_IN;
                } else {
                    disconnect_port(ctx, ci, p);
                }
                connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, p);
            }
            if (int_or_default(ci->params, ctx->id("WRITE_WIDTH_A"), 0) == 1) {
                disconnect_port(ctx, ci, ctx->id("DIADI1"));
                connect_port(ctx, get_net_or_empty(ci, ctx->id("DIADI0")), ci, ctx->id("DIADI1"));
                ci->attrs[ctx->id("X_ORIG_PORT_DIADI1")] = std::string("DIADI[0]");
                disconnect_port(ctx, ci, ctx->id("DIPADIP0"));
                disconnect_port(ctx, ci, ctx->id("DIPADIP1"));
            }
            if (int_or_default(ci->params, ctx->id("WRITE_WIDTH_B"), 0) == 1) {
                disconnect_port(ctx, ci, ctx->id("DIBDI1"));
                connect_port(ctx, get_net_or_empty(ci, ctx->id("DIBDI0")), ci, ctx->id("DIBDI1"));
                ci->attrs[ctx->id("X_ORIG_PORT_DIBDI1")] = std::string("DIBDI[0]");
                disconnect_port(ctx, ci, ctx->id("DIPBDIP0"));
                disconnect_port(ctx, ci, ctx->id("DIPBDIP1"));
            }
            if (int_or_default(ci->params, ctx->id("WRITE_WIDTH_B"), 0) != 72) {
                for (std::string s : {"L", "U"}) {
                    for (int i = 4; i < 8; i++) {
                        IdString port = ctx->id("WEBWE" + s + std::to_string(i));
                        if (!ci->ports.count(port)) {
                            ci->ports[port].name = port;
                            ci->ports[port].type = PORT_IN;
                            connect_port(ctx, ctx->nets[ctx->id("$PACKER_GND_NET")].get(), ci, port);
                        }
                    }
                }
            } else {
                // Tie WEA low
                for (std::string s : {"L", "U"}) {
                    for (int i = 0; i < 4; i++) {
                        IdString port = ctx->id("WEA" + s + std::to_string(i));
                        if (!ci->ports.count(port)) {
                            ci->ports[port].name = port;
                            ci->ports[port].type = PORT_IN;
                            connect_port(ctx, ctx->nets[ctx->id("$PACKER_GND_NET")].get(), ci, port);
                        }
                    }
                }
            }
        }
    }
}

void USPacker::pack_uram()
{
    std::unordered_map<IdString, XFormRule> uram_rules;
    uram_rules[id_URAM288].new_type = id_BEL_URAM288;
    uram_rules[ctx->id("URAM288_BASE")] = uram_rules[id_URAM288];
    generic_xform(uram_rules, true);
}

void XilinxPacker::pack_inverters()
{
    // FIXME: fold where possible
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("INV")) {
            ci->params[ctx->id("INIT")] = Property(1, 2);
            rename_port(ctx, ci, ctx->id("I"), ctx->id("I0"));
            ci->type = ctx->id("LUT1");
        }
    }
}

bool Arch::pack()
{
    if (xc7) {
        XC7Packer packer;
        packer.ctx = getCtx();
        packer.pack_constants();
        packer.pack_inverters();
        packer.pack_io();
        // packer.prepare_iologic();
        packer.prepare_clocking();
        packer.pack_constants();
        packer.pack_iologic();
        packer.pack_idelayctrl();
        packer.pack_cfg();
        packer.pack_plls();
        packer.pack_gt();
        packer.pack_gbs();
        packer.pack_muxfs();
        packer.pack_carries();
        packer.pack_srls();
        packer.pack_luts();
        packer.pack_dram();
        packer.pack_bram();
        packer.pack_dsps();
        packer.pack_ffs();
        packer.finalise_muxfs();
        packer.pack_lutffs();
    } else {
        USPacker packer;
        packer.ctx = getCtx();
        packer.pack_constants();
        packer.pack_inverters();
        packer.pack_io();
        packer.prepare_iologic();
        packer.prepare_clocking();
        packer.pack_constants();
        packer.pack_iologic();
        packer.pack_idelayctrl();
        packer.pack_clocking();
        packer.pack_muxfs();
        packer.pack_carries();
        packer.pack_luts();
        packer.pack_dram();
        packer.pack_bram();
        packer.pack_uram();
        packer.pack_dsps();
        packer.pack_ffs();
        packer.finalise_muxfs();
        packer.pack_lutffs();
    }

    // Confine FRESH (unstamped) fabric logic to a compact region hugging the
    // frozen macro.  Without this the SA placer scatters the sparse fresh cells
    // (arp_ctrl + reset/clock glue) across the WHOLE die -- e.g. a 6-bit reset
    // counter split X8..X210 / Y132..Y327 -- so cpu_clk combinational paths span
    // the chip, fmax fails, the reset counter never settles and the design is
    // stuck in reset (phy_reset held -> board dark).  NEXTPNR_FRESH_REGION_MARGIN
    // = N expands the stamped-cell bbox by N tiles and pins every unstamped
    // SLICE/CARRY cell inside it (IO/clock/GT stay free -- they must reach pads).
    if (const char *mg = getenv("NEXTPNR_FRESH_REGION_MARGIN")) {
        int margin = atoi(mg);
        int x0 = 1 << 30, y0 = 1 << 30, x1 = -1, y1 = -1;
        for (auto &cell : cells) {
            CellInfo *ci = cell.second.get();
            auto it = ci->attrs.find(id("BEL"));
            if (it == ci->attrs.end())
                continue; // fresh cell: not part of the stamped-macro bbox
            std::string t = ci->type.str(this);
            if (t.substr(0, 6) != "SLICE_" && t != "CARRY4")
                continue;
            BelId b = getBelByName(id(it->second.as_string()));
            if (b == BelId())
                continue;
            Loc l = getBelLocation(b);
            x0 = std::min(x0, l.x); x1 = std::max(x1, l.x);
            y0 = std::min(y0, l.y); y1 = std::max(y1, l.y);
        }
        if (x1 >= 0) {
            x0 = std::max(0, x0 - margin);           y0 = std::max(0, y0 - margin);
            x1 = std::min(getGridDimX() - 1, x1 + margin);
            y1 = std::min(getGridDimY() - 1, y1 + margin);
            IdString rname = id("fresh_region");
            createRectangularRegion(rname, x0, y0, x1, y1);
            int n = 0;
            for (auto &cell : cells) {
                CellInfo *ci = cell.second.get();
                if (ci->attrs.count(id("BEL")))
                    continue; // stamped/frozen: leave where it is
                std::string t = ci->type.str(this);
                if (t.substr(0, 6) != "SLICE_" && t != "CARRY4")
                    continue; // only fabric logic; IO/clock/GT reach pads freely
                ci->region = region.at(rname).get();
                ++n;
            }
            log_info("NEXTPNR_FRESH_REGION_MARGIN=%d: fresh region (%d,%d)-(%d,%d), "
                     "constrained %d fresh cells\n", margin, x0, y0, x1, y1, n);
        }
    }

    assignArchInfo();
    attrs[id("step")] = std::string("pack");
    archInfoToAttributes();
    return true;
}

void Arch::assignCellInfo(CellInfo *cell)
{
    if (cell->type == id_SLICE_LUTX) {
        // input_count = number of connected input PORTS (NOT distinct nets):
        // physical pin usage decides bel legality -- a LUT6 with I5 connected
        // drives A6 and needs a 6LUT bel even if I5 duplicates another net.
        // (The imported-6LUT+5LUT shared-input case that once motivated a
        // distinct-net count is now handled by the fully-frozen-tile fast
        // path in xc7_logic_tile_valid, so no dedup is needed here.)
        cell->lutInfo.input_count = 0;
        for (IdString a : {id_A1, id_A2, id_A3, id_A4, id_A5, id_A6}) {
            NetInfo *pn = get_net_or_empty(cell, a);
            if (pn != nullptr)
                cell->lutInfo.input_sigs[cell->lutInfo.input_count++] = pn;
        }
        cell->lutInfo.output_count = 0;
        for (IdString o : {id_O6, id_O5}) {
            NetInfo *pn = get_net_or_empty(cell, o);
            if (pn != nullptr)
                cell->lutInfo.output_sigs[cell->lutInfo.output_count++] = pn;
        }
        for (int i = cell->lutInfo.output_count; i < 2; i++)
            cell->lutInfo.output_sigs[i] = nullptr;
        cell->lutInfo.di1_net = get_net_or_empty(cell, id_DI1);
        cell->lutInfo.di2_net = get_net_or_empty(cell, id_DI2);
        cell->lutInfo.wclk = get_net_or_empty(cell, id_CLK);
        cell->lutInfo.memory_group = 0; // fixme
        cell->lutInfo.is_srl = cell->attrs.count(id("X_LUT_AS_SRL"));
        cell->lutInfo.is_memory = cell->attrs.count(id("X_LUT_AS_DRAM"));
        cell->lutInfo.only_drives_carry = false;
        // Connectivity is sufficient: a LUT whose only user is the carry
        // drives nothing else, whether or not it is chain-constrained
        // (imported Vivado placements pin feeder LUTs absolutely instead
        // of via constr_parent).
        if (xc7) {
            if (cell->lutInfo.output_count > 0 &&
                cell->lutInfo.output_sigs[0] != nullptr && cell->lutInfo.output_sigs[0]->users.size() == 1 &&
                cell->lutInfo.output_sigs[0]->users.at(0).cell->type == id_CARRY4)
                cell->lutInfo.only_drives_carry = true;
        } else {
            if (cell->lutInfo.output_count > 0 &&
                cell->lutInfo.output_sigs[0] != nullptr && cell->lutInfo.output_sigs[0]->users.size() == 1 &&
                cell->lutInfo.output_sigs[0]->users.at(0).cell->type == id_CARRY8)
                cell->lutInfo.only_drives_carry = true;
        }

        const IdString addr_msb_sigs[] = {id_WA7, id_WA8, id_WA9};
        for (int i = 0; i < 3; i++)
            cell->lutInfo.address_msb[i] = get_net_or_empty(cell, addr_msb_sigs[i]);

    } else if (cell->type == id_SLICE_FFX) {
        cell->ffInfo.d = get_net_or_empty(cell, id_D);
        cell->ffInfo.clk = get_net_or_empty(cell, xc7 ? id_CK : id_CLK);
        cell->ffInfo.ce = get_net_or_empty(cell, id_CE);
        cell->ffInfo.sr = get_net_or_empty(cell, id_SR);
        cell->ffInfo.is_clkinv = bool_or_default(cell->params, id("IS_CLK_INVERTED"), false);
        cell->ffInfo.is_srinv = bool_or_default(cell->params, id("IS_R_INVERTED"), false) ||
                                bool_or_default(cell->params, id("IS_S_INVERTED"), false) ||
                                bool_or_default(cell->params, id("IS_CLR_INVERTED"), false) ||
                                bool_or_default(cell->params, id("IS_PRE_INVERTED"), false);
        cell->ffInfo.is_latch = cell->attrs.count(id("X_FF_AS_LATCH"));
        cell->ffInfo.ffsync = cell->attrs.count(id("X_FFSYNC"));
    } else if (cell->type == id_F7MUX || cell->type == id_F8MUX || cell->type == id_F9MUX ||
               cell->type == id("SELMUX2_1")) {
        cell->muxInfo.sel = get_net_or_empty(cell, id_S0);
        cell->muxInfo.out = get_net_or_empty(cell, id_OUT);
    } else if (cell->type == id_CARRY8) {
        for (int i = 0; i < 8; i++) {
            cell->carryInfo.out_sigs[i] = get_net_or_empty(cell, id("O" + std::to_string(i)));
            cell->carryInfo.cout_sigs[i] = get_net_or_empty(cell, id("CO" + std::to_string(i)));
            cell->carryInfo.x_sigs[i] = get_net_or_empty(cell, id(std::string(1, 'A' + i) + "X"));
        }
    } else if (cell->type == id_CARRY4) {
        for (int i = 0; i < 4; i++) {
            cell->carryInfo.out_sigs[i] = get_net_or_empty(cell, id("O" + std::to_string(i)));
            cell->carryInfo.cout_sigs[i] = get_net_or_empty(cell, id("CO" + std::to_string(i)));
            cell->carryInfo.x_sigs[i] = nullptr;
        }
        cell->carryInfo.x_sigs[0] = get_net_or_empty(cell, id("CYINIT"));
    }
}

void Arch::assignArchInfo()
{
    for (auto cell : sorted(cells)) {
        assignCellInfo(cell.second);
    }
}

NEXTPNR_NAMESPACE_END
