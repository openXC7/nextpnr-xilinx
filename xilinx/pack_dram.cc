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

CellInfo *XilinxPacker::create_dram_lut(const std::string &name, CellInfo *base, const DRAMControlSet &ctrlset,
                                        std::vector<NetInfo *> address, NetInfo *di, NetInfo *dout, int z)
{
    std::unique_ptr<CellInfo> dram_lut = create_cell(ctx, ctx->id("RAMD64E"), ctx->id(name));
    for (int i = 0; i < int(address.size()); i++)
        connect_port(ctx, address[i], dram_lut.get(), ctx->id("RADR" + std::to_string(i)));
    connect_port(ctx, di, dram_lut.get(), ctx->id("I"));
    connect_port(ctx, dout, dram_lut.get(), ctx->id("O"));
    connect_port(ctx, ctrlset.wclk, dram_lut.get(), ctx->id("CLK"));
    connect_port(ctx, ctrlset.we, dram_lut.get(), ctx->id("WE"));
    for (int i = 0; i < int(ctrlset.wa.size()); i++)
        connect_port(ctx, ctrlset.wa[i], dram_lut.get(), ctx->id("WADR" + std::to_string(i)));
    dram_lut->params[ctx->id("IS_WCLK_INVERTED")] = ctrlset.wclk_inv ? 1 : 0;

    xform_cell(dram_rules, dram_lut.get());

    dram_lut->constr_abs_z = true;
    dram_lut->constr_z = (z << 4) | BEL_6LUT;
    if (base != nullptr) {
        dram_lut->constr_parent = base;
        dram_lut->constr_x = 0;
        dram_lut->constr_y = 0;
        base->constr_children.push_back(dram_lut.get());
    }
    // log_info("%s: z = %d (%d) -> %s\n", name.c_str(), z, dram_lut->constr_z, dram_lut->constr_parent ?
    // dram_lut->constr_parent->name.c_str(ctx) : "*");
    CellInfo *dl = dram_lut.get();
    new_cells.push_back(std::move(dram_lut));
    return dl;
}

CellInfo *XilinxPacker::create_dram32_lut(const std::string &name, CellInfo *base, const DRAMControlSet &ctrlset,
                                          std::vector<NetInfo *> address, NetInfo *di, NetInfo *dout, bool o5, int z)
{
    std::unique_ptr<CellInfo> dram_lut = create_cell(ctx, ctx->id("RAMD32"), ctx->id(name));
    for (int i = 0; i < int(address.size()); i++)
        connect_port(ctx, address[i], dram_lut.get(), ctx->id("RADR" + std::to_string(i)));
    connect_port(ctx, di, dram_lut.get(), ctx->id("I"));
    connect_port(ctx, dout, dram_lut.get(), ctx->id("O"));
    connect_port(ctx, ctrlset.wclk, dram_lut.get(), ctx->id("CLK"));
    connect_port(ctx, ctrlset.we, dram_lut.get(), ctx->id("WE"));
    for (int i = 0; i < int(ctrlset.wa.size()); i++)
        connect_port(ctx, ctrlset.wa[i], dram_lut.get(), ctx->id("WADR" + std::to_string(i)));
    dram_lut->params[ctx->id("IS_WCLK_INVERTED")] = ctrlset.wclk_inv ? 1 : 0;

    xform_cell(o5 ? dram32_5_rules : dram32_6_rules, dram_lut.get());

    dram_lut->constr_abs_z = true;
    dram_lut->constr_z = (z << 4) | (o5 ? BEL_5LUT : BEL_6LUT);
    if (base != nullptr) {
        dram_lut->constr_parent = base;
        dram_lut->constr_x = 0;
        dram_lut->constr_y = 0;
        base->constr_children.push_back(dram_lut.get());
    }
    // log_info("%s: z = %d (%d) -> %s\n", name.c_str(), z, dram_lut->constr_z, dram_lut->constr_parent ?
    // dram_lut->constr_parent->name.c_str(ctx) : "*");
    CellInfo *dl = dram_lut.get();
    new_cells.push_back(std::move(dram_lut));
    return dl;
}

void XilinxPacker::create_muxf_tree(CellInfo *base, const std::string &name_base, const std::vector<NetInfo *> &data,
                                    const std::vector<NetInfo *> &select, NetInfo *out, int zoffset)
{
    int levels = 0;
    if (data.size() <= 2)
        levels = 1;
    else if (data.size() <= 4)
        levels = 2;
    else if (data.size() <= 8)
        levels = 3;
    else
        NPNR_ASSERT_FALSE("muxf tree too large");
    NPNR_ASSERT(int(select.size()) == levels);
    std::vector<std::vector<NetInfo *>> int_data;
    CellInfo *mux_root = nullptr;
    int_data.push_back(data);
    for (int i = 0; i < levels; i++) {
        IdString mux_type;
        if (i == 0)
            mux_type = ctx->id("MUXF7");
        else if (i == 1)
            mux_type = ctx->id("MUXF8");
        else if (i == 2)
            mux_type = ctx->id("MUXF9");
        else
            NPNR_ASSERT_FALSE("unknown muxf type");
        int_data.emplace_back();
        auto &last = int_data.at(int_data.size() - 2);
        for (int j = 0; j < int(last.size()) / 2; j++) {
            NetInfo *output =
                    (i == (levels - 1))
                            ? out
                            : create_internal_net(base->name,
                                                  name_base + "_muxq_" + std::to_string(i) + "_" + std::to_string(j),
                                                  false);
            int_data.back().push_back(output);
            auto mux = create_cell(
                    ctx, mux_type,
                    int_name(base->name, name_base + "_muxf_" + std::to_string(i) + "_" + std::to_string(j), false));
            connect_port(ctx, last.at(j * 2), mux.get(), ctx->id("I0"));
            connect_port(ctx, last.at(j * 2 + 1), mux.get(), ctx->id("I1"));
            connect_port(ctx, select.at(i), mux.get(), ctx->id("S"));
            connect_port(ctx, output, mux.get(), ctx->id("O"));
            if (i == (levels - 1))
                mux_root = mux.get();
            new_cells.push_back(std::move(mux));
        }
    }
    constrain_muxf_tree(mux_root, base, zoffset);
}

void XilinxPacker::pack_dram()
{

    log_info("Packing DRAM..\n");

    std::unordered_map<DRAMControlSet, std::vector<CellInfo *>, DRAMControlSetHash> dram_groups;
    std::unordered_map<IdString, DRAMType> dram_types;

    dram_types[ctx->id("RAM32X1S")]   = {5, 1, 0};
    dram_types[ctx->id("RAM32X2S")]   = {5, 2, 0};
    dram_types[ctx->id("RAM32X1S_1")] = {5, 1, 0};
    dram_types[ctx->id("RAM32X1D")]   = {5, 1, 1};
    dram_types[ctx->id("RAM64X1S")]   = {6, 1, 0};
    dram_types[ctx->id("RAM64X1S_1")] = {6, 1, 0};
    dram_types[ctx->id("RAM64X1D")]   = {6, 1, 1};
    dram_types[ctx->id("RAM128X1S")]  = {7, 1, 0};
    dram_types[ctx->id("RAM128X1D")]  = {7, 1, 1};
    dram_types[ctx->id("RAM256X1S")]  = {8, 1, 0};
    dram_types[ctx->id("RAM256X1D")]  = {8, 1, 1};
    dram_types[ctx->id("RAM512X1S")]  = {9, 1, 0};
    dram_types[ctx->id("RAM512X1D")]  = {9, 1, 1};

    // Transform from RAMD64E UNISIM to SLICE_LUTX bel
    dram_rules[ctx->id("RAMD64E")].new_type = id_SLICE_LUTX;
    dram_rules[ctx->id("RAMD64E")].set_attrs.emplace_back(ctx->id("X_LUT_AS_DRAM"), Property(1));
    dram_rules[ctx->id("RAMD64E")].set_attrs.emplace_back(ctx->id("X_IS_SLICEM"), Property(1));
    for (int i = 0; i < 6; i++)
        dram_rules[ctx->id("RAMD64E")].port_xform[ctx->id("RADR" + std::to_string(i))] =
                ctx->id("A" + std::to_string(i + 1));
    for (int i = 0; i < 8; i++)
        dram_rules[ctx->id("RAMD64E")].port_xform[ctx->id("WADR" + std::to_string(i))] =
                ctx->id("WA" + std::to_string(i + 1));
    dram_rules[ctx->id("RAMD64E")].port_xform[ctx->id("I")] = id_DI1;
    dram_rules[ctx->id("RAMD64E")].port_xform[ctx->id("O")] = id_O6;

    // Rules for upper and lower RAMD32E
    dram32_6_rules[ctx->id("RAMD32")].new_type = id_SLICE_LUTX;
    dram32_6_rules[ctx->id("RAMD32")].set_attrs.emplace_back(ctx->id("X_LUT_AS_DRAM"), Property(1));
    for (int i = 0; i < 5; i++)
        dram32_6_rules[ctx->id("RAMD32")].port_xform[ctx->id("RADR" + std::to_string(i))] =
                ctx->id("A" + std::to_string(i + 1));
    for (int i = 0; i < 5; i++)
        dram32_6_rules[ctx->id("RAMD32")].port_xform[ctx->id("WADR" + std::to_string(i))] =
                ctx->id("WA" + std::to_string(i + 1));
    dram32_6_rules[ctx->id("RAMD32")].port_xform[ctx->id("I")] = id_DI1;
    dram32_6_rules[ctx->id("RAMD32")].port_xform[ctx->id("O")] = id_O6;

    dram32_5_rules = dram32_6_rules;
    dram32_5_rules[ctx->id("RAMD32")].port_xform[ctx->id("I")] = id_DI1;
    dram32_5_rules[ctx->id("RAMD32")].port_xform[ctx->id("O")] = id_O5;

    // Optimise DRAM with tied-low inputs, to more efficiently routeable tied-high inputs
    int inverted_ports = 0;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        auto dt_iter = dram_types.find(ci->type);
        if (dt_iter == dram_types.end())
            continue;
        auto &dt = dt_iter->second;
        for (int i = 0; i < std::min(dt.abits, 6); i++) {
            IdString aport = ctx->id(dt.abits <= (7 - (dt.rports > 0)) ? ("A" + std::to_string(i)) : ("A[" + std::to_string(i) + "]"));
            if (!ci->ports.count(aport))
                continue;
            NetInfo *anet = get_net_or_empty(ci, aport);
            if (anet == nullptr || anet->name != ctx->id("$PACKER_GND_NET"))
                continue;
            IdString raport;
            if (dt.rports >= 1) {
                NPNR_ASSERT(dt.rports == 1); // FIXME
                raport = ctx->id(dt.abits <= 6 ? ("DPRA" + std::to_string(i)) : ("DPRA[" + std::to_string(i) + "]"));
                NetInfo *ranet = get_net_or_empty(ci, raport);
                if (ranet == nullptr || ranet->name != ctx->id("$PACKER_GND_NET"))
                    continue;
            }
            disconnect_port(ctx, ci, aport);
            if (raport != IdString())
                disconnect_port(ctx, ci, raport);
            connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, aport);
            if (raport != IdString())
                connect_port(ctx, ctx->nets[ctx->id("$PACKER_VCC_NET")].get(), ci, raport);
            ++inverted_ports;
            if (ci->params.count(ctx->id("INIT"))) {
                Property &init = ci->params[ctx->id("INIT")];
                for (int j = 0; j < int(init.str.size()); j++) {
                    if (j & (1 << i))
                        init.str[j] = init.str[j & ~(1 << i)];
                }
                init.update_intval();
            }
        }
    }
    log_info("   Transformed %d tied-low DRAM address inputs to be tied-high\n", inverted_ports);

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        auto dt_iter = dram_types.find(ci->type);
        if (dt_iter == dram_types.end())
            continue;
        auto &dt = dt_iter->second;
        DRAMControlSet dcs;
        for (int i = 0; i < dt.abits; i++)
            dcs.wa.push_back(get_net_or_empty(
                    ci, ctx->id(dt.abits <= (7 - (dt.rports > 0)) ? ("A" + std::to_string(i)) : ("A[" + std::to_string(i) + "]"))));
        if (dt.abits == 5) {
            dcs.wa.push_back(ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
        }
        dcs.wclk = get_net_or_empty(ci, ctx->id("WCLK"));
        dcs.we = get_net_or_empty(ci, ctx->id("WE"));
        dcs.wclk_inv = bool_or_default(ci->params, ctx->id("IS_WCLK_INVERTED"));
        if(ci->type == ctx->id("RAM32X1S_1") || ci->type == ctx->id("RAM64X1S_1")){
            dcs.wclk_inv = true;
        }
        dcs.memtype = ci->type;
        dram_groups[dcs].push_back(ci);
    }

    int height = ctx->xc7 ? 4 : 8;
    // Grouped DRAM
    for (auto &group : dram_groups) {
        auto &cs = group.first;
        if (cs.memtype == ctx->id("RAM64X1D")) {
            int z = height - 1;
            CellInfo *base = nullptr;
            for (auto cell : group.second) {
                NPNR_ASSERT(cell->type == ctx->id("RAM64X1D")); // FIXME

                int z_size = 0;
                if (get_net_or_empty(cell, ctx->id("SPO")) != nullptr)
                    z_size++;
                if (get_net_or_empty(cell, ctx->id("DPO")) != nullptr)
                    z_size++;

                if (z == (height - 1) || (z - z_size + 1) < 0) {
                    z = (height - 1);
                    // Topmost cell is the write address input
                    std::vector<NetInfo *> address(cs.wa.begin(), cs.wa.begin() + std::min<size_t>(cs.wa.size(), 6));
                    base = create_dram_lut(cell->name.str(ctx) + "/ADDR", nullptr, cs, address, nullptr, nullptr, z);
                    z--;
                }

                NetInfo *dpo = get_net_or_empty(cell, ctx->id("DPO"));
                NetInfo *spo = get_net_or_empty(cell, ctx->id("SPO"));
                disconnect_port(ctx, cell, ctx->id("DPO"));
                disconnect_port(ctx, cell, ctx->id("SPO"));

                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                if (spo != nullptr) {
                    if (z == (height - 2)) {
                        // Can fold DPO into address buffer
                        connect_port(ctx, spo, base, ctx->id("O6"));
                        connect_port(ctx, di, base, ctx->id("DI1"));
                        if (cell->params.count(ctx->id("INIT")))
                            base->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                    } else {
                        std::vector<NetInfo *> address(cs.wa.begin(),
                                                       cs.wa.begin() + std::min<size_t>(cs.wa.size(), 6));
                        CellInfo *dpr = create_dram_lut(cell->name.str(ctx) + "/SP", base, cs, address, di, spo, z);
                        if (cell->params.count(ctx->id("INIT")))
                            dpr->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                        z--;
                    }
                }

                if (dpo != nullptr) {
                    std::vector<NetInfo *> address;
                    for (int i = 0; i < 6; i++)
                        address.push_back(get_net_or_empty(cell, ctx->id("DPRA" + std::to_string(i))));
                    CellInfo *dpr = create_dram_lut(cell->name.str(ctx) + "/DP", base, cs, address, di, dpo, z);
                    if (cell->params.count(ctx->id("INIT")))
                        dpr->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                    z--;
                }

                packed_cells.insert(cell->name);
            }
        } else if (cs.memtype == ctx->id("RAM32X1D")) {
            int z = (height - 1);
            CellInfo *base = nullptr;
            for (auto cell : group.second) {
                NPNR_ASSERT(cell->type == ctx->id("RAM32X1D"));

                int z_size = 0;
                if (get_net_or_empty(cell, ctx->id("SPO")) != nullptr)
                    z_size++;
                if (get_net_or_empty(cell, ctx->id("DPO")) != nullptr)
                    z_size++;

                auto init_property=get_or_default(cell->params, ctx->id("INIT"), Property(0));
                // Only the high-order LUT5 is used, str is in reverse order, 
				// first fill the back with zeros to make up 32 bits, 
				// then insert 32 zeros in the front, which is equivalent to shifting left 32 bits
                init_property.str.append(32 - init_property.str.size(), '0');
                init_property.str.insert(0, 32, '0');
                init_property.update_intval();

                bool spo_is_base = false;
                if (z == (height - 1) || (z - z_size + 1) < 0) {
                    z = (height - 1);
                    // Topmost cell is the write address input
                    std::vector<NetInfo *> address(cs.wa.begin(), cs.wa.begin() + std::min<size_t>(cs.wa.size(), 5));
                    address.push_back(ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
                    base = create_dram_lut(cell->name.str(ctx) + "/ADDR", nullptr, cs, address, nullptr, nullptr, z);
                    z--;
                    spo_is_base = true;
                }

                NetInfo *dpo = get_net_or_empty(cell, ctx->id("DPO"));
                NetInfo *spo = get_net_or_empty(cell, ctx->id("SPO"));
                disconnect_port(ctx, cell, ctx->id("DPO"));
                disconnect_port(ctx, cell, ctx->id("SPO"));

                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                if (spo != nullptr) {
                    if (spo_is_base) {
                        // Can fold DPO into address buffer
                        connect_port(ctx, spo, base, ctx->id("O6"));
                        connect_port(ctx, di, base, ctx->id("DI1"));
                        if (cell->params.count(ctx->id("INIT")))
                            base->params[ctx->id("INIT")] = init_property;
                    } else {
                        std::vector<NetInfo *> address(cs.wa.begin(),
                                                       cs.wa.begin() + std::min<size_t>(cs.wa.size(), 5));
                        address.push_back(ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
                        CellInfo *dpr = create_dram_lut(cell->name.str(ctx) + "/SP", base, cs, address, di, spo, z);
                        if (cell->params.count(ctx->id("INIT")))
                            dpr->params[ctx->id("INIT")] = init_property;
                        z--;
                    }
                }

                if (dpo != nullptr) {
                    std::vector<NetInfo *> address;
                    for (int i = 0; i < 5; i++)
                        address.push_back(get_net_or_empty(cell, ctx->id("DPRA" + std::to_string(i))));
                    address.push_back(ctx->nets[ctx->id("$PACKER_VCC_NET")].get());                   
                    CellInfo *dpr = create_dram_lut(cell->name.str(ctx) + "/DP", base, cs, address, di, dpo, z);
                    if (cell->params.count(ctx->id("INIT")))
                        dpr->params[ctx->id("INIT")] = init_property;
                    z--;
                }

                packed_cells.insert(cell->name);
            } 
        } else if (cs.memtype == ctx->id("RAM32X1S")) {
            int z = (height - 1);
            CellInfo *base = nullptr;
            for (auto cell : group.second) {
                NPNR_ASSERT(cell->type == ctx->id("RAM32X1S"));

                auto init_property=get_or_default(cell->params, ctx->id("INIT"), Property(0));
                // Only the high-order LUT5 is used, str is in reverse order, 
				// first fill the back with zeros to make up 32 bits, 
				// then insert 32 zeros in the front, which is equivalent to shifting left 32 bits
                init_property.str.append(32-init_property.str.size(), '0');
                init_property.str.insert(0, 32, '0');
                init_property.update_intval();

                
                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                NetInfo *dout = get_net_or_empty(cell, ctx->id("O"));

                disconnect_port(ctx, cell, ctx->id("O"));

                if (z <0)
                    z = (height - 1);
                if (z == (height - 1)) {
                    // base
                    // Topmost cell is the write address input
                    std::vector<NetInfo *> address(cs.wa.begin(), cs.wa.begin() + std::min<size_t>(cs.wa.size(), 5));
                    address.push_back(ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
                    base = create_dram_lut(cell->name.str(ctx), nullptr, cs, address, di, dout, z);
                    if (cell->params.count(ctx->id("INIT")))
                        base->params[ctx->id("INIT")] = init_property;
                    z--;
                } else {
                    std::vector<NetInfo *> address(cs.wa.begin(),
                                                       cs.wa.begin() + std::min<size_t>(cs.wa.size(), 5));
                    address.push_back(ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
                    CellInfo *ram_lut = create_dram_lut(cell->name.str(ctx), base, cs, address, di, dout, z);
                    if (cell->params.count(ctx->id("INIT")))
                        ram_lut->params[ctx->id("INIT")] = init_property;
                    z--;
                }
                packed_cells.insert(cell->name);
            }
        } 
        else if (cs.memtype == ctx->id("RAM32X2S")) {
            int z = (height - 1);
            CellInfo *base = nullptr;
            for (auto cell : group.second) {
                NPNR_ASSERT(cell->type == ctx->id("RAM32X2S"));
                // Handles up to two RAM32X2S
                if (z < 1) {
                    z = (height - 1);
                    base = nullptr;
                } 
                // Read address lines (single-ended read and write a set of address lines)
                std::vector<NetInfo *> address;
                for (int i = 0; i < 5; i++)
                    address.push_back(get_net_or_empty(cell, ctx->id("A" + std::to_string(i))));
                // Read high-order LUT5, and set the sixth read address line to high level
                address.push_back(ctx->nets[ctx->id("$PACKER_VCC_NET")].get());

                for (int i = 0; i < 2; i++) {
                    NetInfo *di = get_net_or_empty(cell, ctx->id("D" + std::to_string(i)));
                    NetInfo *dout = get_net_or_empty(cell, ctx->id("O" + std::to_string(i)));
                    disconnect_port(ctx, cell, ctx->id("O" + std::to_string(i)));

                    CellInfo *ram_lut = create_dram_lut(
                        cell->name.str(ctx) + "/RAM32X1S" + std::to_string(i) + "/SP",
                        base, cs, address, di, dout, z);

                    IdString init_param = ctx->id("INIT_0" + std::to_string(i));
                    if (cell->params.count(init_param)) {
                        auto init_property = cell->params.at(init_param);
                        init_property.str.append(32 - init_property.str.size(), '0');
                        init_property.str.insert(0, 32, '0');
                        init_property.update_intval();
                        ram_lut->params[ctx->id("INIT")] = init_property;
                    }
                    if (base == nullptr) base = ram_lut;
                    z--;                    
                }
                packed_cells.insert(cell->name);               
            }
        } else if (cs.memtype == ctx->id("RAM64X1S")) {
             int z = (height - 1);
             CellInfo *base = nullptr;
             for (auto cell : group.second) {
                NPNR_ASSERT(cell->type == ctx->id("RAM64X1S"));
                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                NetInfo *dout = get_net_or_empty(cell, ctx->id("O"));

                disconnect_port(ctx, cell, ctx->id("O"));

                if (z <0)
                    z = (height - 1);
                if (z == (height - 1)) {
                    base = create_dram_lut(cell->name.str(ctx), nullptr, cs, cs.wa, di, dout, z);
                    if (cell->params.count(ctx->id("INIT")))
                        base->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                    z--;                    
             } else {
                    CellInfo *ram_lut = create_dram_lut(cell->name.str(ctx), base, cs, cs.wa, di, dout, z);
                    if (cell->params.count(ctx->id("INIT")))
                        ram_lut->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                    z--;                    
                }
                packed_cells.insert(cell->name);
             }
        } else if (cs.memtype == ctx->id("RAM32X1S_1")) {
            int z = (height - 1);
            CellInfo *base = nullptr;
            for (auto cell : group.second) {
                NPNR_ASSERT(cell->type == ctx->id("RAM32X1S_1"));

                auto init_property=get_or_default(cell->params, ctx->id("INIT"), Property(0));
                // Only the high-order LUT5 is used, str is in reverse order, 
                // first fill the back with zeros to make up 32 bits, 
                // then insert 32 zeros in the front, which is equivalent to shifting left 32 bits
                init_property.str.append(32-init_property.str.size(), '0');
                init_property.str.insert(0, 32, '0');
                init_property.update_intval();

                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                NetInfo *dout = get_net_or_empty(cell, ctx->id("O"));

                disconnect_port(ctx, cell, ctx->id("O"));

                if (z <0)
                    z = (height - 1);
                if (z == (height - 1)) {
                    std::vector<NetInfo *> address(cs.wa.begin(), cs.wa.begin() + std::min<size_t>(cs.wa.size(), 5));
                    address.push_back(ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
                    base = create_dram_lut(cell->name.str(ctx), nullptr, cs, address, di, dout, z);
                    if (cell->params.count(ctx->id("INIT")))
                        base->params[ctx->id("INIT")] = init_property;
                    base->attrs[ctx->id("CLK_STATUS")] = Property("CLKINV");
                    z--;
                } else {
                    std::vector<NetInfo *> address(cs.wa.begin(),
                                                       cs.wa.begin() + std::min<size_t>(cs.wa.size(), 5));
                    address.push_back(ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
                    CellInfo *ram_lut = create_dram_lut(cell->name.str(ctx), base, cs, address, di, dout, z);
                    if (cell->params.count(ctx->id("INIT")))
                        ram_lut->params[ctx->id("INIT")] = init_property;
                    ram_lut->attrs[ctx->id("CLK_STATUS")] = Property("CLKINV");
                    z--;
                }
                packed_cells.insert(cell->name);

             }
        } else if (cs.memtype == ctx->id("RAM64X1S_1")) {
            int z = (height - 1);
            CellInfo *base = nullptr;
            for (auto cell : group.second) {
                NPNR_ASSERT(cell->type == ctx->id("RAM64X1S_1"));
                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                NetInfo *dout = get_net_or_empty(cell, ctx->id("O"));

                disconnect_port(ctx, cell, ctx->id("O"));

                if (z <0)
                    z = (height - 1);
                if (z == (height - 1)) {
                    base = create_dram_lut(cell->name.str(ctx), nullptr, cs, cs.wa, di, dout, z);
                    if (cell->params.count(ctx->id("INIT")))
                        base->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                    base->attrs[ctx->id("CLK_STATUS")] = Property("CLKINV");
                    z--;                    
                } else {
                        CellInfo *ram_lut = create_dram_lut(cell->name.str(ctx), base, cs, cs.wa, di, dout, z);
                        if (cell->params.count(ctx->id("INIT")))
                            ram_lut->params[ctx->id("INIT")] = cell->params[ctx->id("INIT")];
                        ram_lut->attrs[ctx->id("CLK_STATUS")] = Property("CLKINV");
                        z--;                    
                    }
                packed_cells.insert(cell->name);
            }            
        } else if (cs.memtype == ctx->id("RAM128X1S")) {
            for (auto cell : group.second) {
                int z = (height - 1);
                CellInfo *base = nullptr;
                // Only the high-order LUT5 is used, str is in reverse order,
                // first fill the back with zeros to make up 32 bits, and then insert 32 zeros in the front,
                // which is equivalent to shifting a vector 32 bits left to store the output of two 64-bit LUTs
                std::vector<NetInfo *> dout_interm;     

                NPNR_ASSERT(cell->type == ctx->id("RAM128X1S"));
                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                NetInfo *dout = get_net_or_empty(cell, ctx->id("O"));
                disconnect_port(ctx, cell, ctx->id("O"));

                // Split the address into lower 6 bits (for 64-bit LUT) and upper 1 bit (for MUXF7 selection)
                std::vector<NetInfo *> addressw_low(cs.wa.begin(), cs.wa.begin() + 6);
                std::vector<NetInfo *> addressw_high(cs.wa.begin() + 6, cs.wa.end());   
                
                // Create the first 64-bit LUT (lower 64 bits)
                NetInfo *dout_low = create_internal_net(cell->name, "O_LOW", false);
                CellInfo *dram_low = create_dram_lut(cell->name.str(ctx) + "/LOW", base, cs, addressw_low, di, dout_low, z);
                z--;
                dout_interm.push_back(dout_low);

                if (base == nullptr) base = dram_low;
                
                // Create a second 64-bit LUT (upper 64 bits)
                NetInfo *dout_high = create_internal_net(cell->name, "O_HIGH", false);
                CellInfo *dram_high = create_dram_lut(cell->name.str(ctx) + "/HIGH", base, cs, 
                                                    addressw_low, di, dout_high, z);
                z--;
                dout_interm.push_back(dout_high);

                // If the original RAM128X1S has INIT parameters,
                // divide it into two halves, each with 64 bits,
                // and set them to two 64-bit LUTs respectively.
                if (cell->params.count(ctx->id("INIT"))) {
                    Property init = cell->params.at(ctx->id("INIT"));
                    dram_low->params[ctx->id("INIT")] = init.extract(0, 64);
                    dram_high->params[ctx->id("INIT")] = init.extract(64, 64);
                }

                // Use the create_muxf_tree function to create MUXF7 and select the output of two 64-bit LUTs
                create_muxf_tree(dram_low, "O", dout_interm, addressw_high, dout, 2);
                packed_cells.insert(cell->name);         
            }
        } else if (cs.memtype == ctx->id("RAM256X1S")) {
            for (auto cell : group.second) {
                int z = (height - 1);
                std::vector<NetInfo *> dout_interm;
                NPNR_ASSERT(cell->type == ctx->id("RAM256X1S"));

                // get input and output ports
                NetInfo *di = get_net_or_empty(cell, ctx->id("D"));
                NetInfo *dout = get_net_or_empty(cell, ctx->id("O"));
                // Disconnect the original output since we will be using the new output network
                disconnect_port(ctx, cell, ctx->id("O"));

                // Split address lines: lower 6 bits for DRAM LUT, upper 2 bits for MUX selection
                std::vector<NetInfo *> addressw_low(cs.wa.begin(), cs.wa.begin() + 6);
                std::vector<NetInfo *> addressw_high(cs.wa.begin() + 6, cs.wa.end());   

                NetInfo *dout_d = create_internal_net(cell->name, "O_RAMS64E_D", false);
                CellInfo *lut_d = create_dram_lut(cell->name.str(ctx) + "/RAMS64E_D", 
                                                nullptr, cs, addressw_low, di, dout_d, z);
                z--;
                dout_interm.push_back(dout_d);

                NetInfo *dout_c = create_internal_net(cell->name, "O_RAMS64E_C", false);
                CellInfo *lut_c = create_dram_lut(cell->name.str(ctx) + "/RAMS64E_C", 
                                                lut_d, cs, addressw_low, di, dout_c, z);
                z--;
                dout_interm.push_back(dout_c);

                NetInfo *dout_b = create_internal_net(cell->name, "O_RAMS64E_B", false);
                CellInfo *lut_b = create_dram_lut(cell->name.str(ctx) + "/RAMS64E_B", 
                                                lut_d, cs, addressw_low, di, dout_b, z);
                z--;
                dout_interm.push_back(dout_b);

                NetInfo *dout_a = create_internal_net(cell->name, "O_RAMS64E_A", false);
                CellInfo *lut_a = create_dram_lut(cell->name.str(ctx) + "/RAMS64E_A", 
                                                lut_d, cs, addressw_low, di, dout_a, z);
                dout_interm.push_back(dout_a);

                if (cell->params.count(ctx->id("INIT"))) {
                    Property init = cell->params.at(ctx->id("INIT"));
                    lut_a->params[ctx->id("INIT")] = init.extract(192, 64);
                    lut_b->params[ctx->id("INIT")] = init.extract(128, 64);
                    lut_c->params[ctx->id("INIT")] = init.extract(64, 64);
                    lut_d->params[ctx->id("INIT")] = init.extract(0, 64);
                }
                // tree to select the correct DRAM LUT output
                create_muxf_tree(lut_d, "O", dout_interm, addressw_high, dout, 0);
                packed_cells.insert(cell->name);  

            } 
        } else if (cs.memtype == ctx->id("RAM128X1D")) {
            
            for (auto cell : group.second) {
                CellInfo *base = nullptr;
                int z = (height - 1);
                NPNR_ASSERT(cell->type == ctx->id("RAM128X1D"));

                auto    init = get_or_default(cell->params, ctx->id("INIT"), Property(0, 128));
                NetInfo *dpo = get_net_or_empty(cell, ctx->id("DPO"));
                NetInfo *spo = get_net_or_empty(cell, ctx->id("SPO"));
                
                // disconnect the original output
                disconnect_port(ctx, cell, ctx->id("DPO"));
                disconnect_port(ctx, cell, ctx->id("SPO"));

                // prepare address lines
                std::vector<NetInfo *> addressw_64(cs.wa.begin(), cs.wa.begin() + std::min<size_t>(cs.wa.size(), 6));
                std::vector<NetInfo *> addressw_high(cs.wa.begin() + std::min<size_t>(cs.wa.size(), 6), cs.wa.end());

                std::vector<NetInfo *> spo_pre, dpo_pre;

                NetInfo *spo_low = create_internal_net(cell->name, "SPO_LOW", false);
                CellInfo *sp_low = create_dram_lut(cell->name.str(ctx) + "/SP.LOW", base, cs,
                                                addressw_64, get_net_or_empty(cell, ctx->id("D")), spo_low, z);
                if (base == nullptr)
                    base = sp_low;
                spo_pre.push_back(spo_low);
                sp_low->params[ctx->id("INIT")] = init.extract(0, 64);
                z--;

                NetInfo *spo_high = create_internal_net(cell->name, "SPO_HIGH", false);
                CellInfo *sp_high = create_dram_lut(cell->name.str(ctx) + "/SP.HIGH", base, cs,
                                                    addressw_64, get_net_or_empty(cell, ctx->id("D")), spo_high, z);
                spo_pre.push_back(spo_high);
                sp_high->params[ctx->id("INIT")] = init.extract(64, 64);
                z--;

                std::vector<NetInfo *> addressr_64, addressr_high;
                for (int i = 0; i < 7; i++) {
                    (i >= 6 ? addressr_high : addressr_64)
                            .push_back(get_net_or_empty(cell, ctx->id("DPRA[" + std::to_string(i) + "]")));
                }

                NetInfo *dpo_low = create_internal_net(cell->name, "DPO_LOW", false);
                CellInfo *dp_low = create_dram_lut(cell->name.str(ctx) + "/DP.LOW", base, cs,
                                                addressr_64, get_net_or_empty(cell, ctx->id("D")), dpo_low, z);
                dpo_pre.push_back(dpo_low);
                dp_low->params[ctx->id("INIT")] = init.extract(0, 64);
                z--;

                NetInfo *dpo_high = create_internal_net(cell->name, "DPO_HIGH", false);
                CellInfo *dp_high = create_dram_lut(cell->name.str(ctx) + "/DP.HIGH", base, cs,
                                                    addressr_64, get_net_or_empty(cell, ctx->id("D")), dpo_high, z);
                dpo_pre.push_back(dpo_high);
                dp_high->params[ctx->id("INIT")] = init.extract(64, 64);
                z--;

                // create MUXF7 tree
                create_muxf_tree(base, "SPO", spo_pre, addressw_high, spo, 2);
                create_muxf_tree(base, "DPO", dpo_pre, addressr_high, dpo, 0);

                packed_cells.insert(cell->name);
            }
        } else if (cs.memtype == ctx->id("RAMS32")
                || cs.memtype == ctx->id("RAMD32")
                || cs.memtype == ctx->id("RAMS64E")
                || cs.memtype == ctx->id("RAMD64E")) {
            log_error("Cannot pack unsupported primitive: %s\n", cs.memtype.c_str(ctx));
        }
    }
    // Raw distributed-RAM leaf primitives (RAMD64E / RAMD32 / RAMS32),
    // e.g. from an imported Vivado netlist where the Unisim Transform
    // already decomposed RAM32M/RAM64M.  Reuse the same xform rules the
    // macro path applies via create_dram_lut/create_dram32_lut.  The
    // 5LUT-vs-6LUT variant for 32-bit cells is taken from the absolute
    // BEL attribute when present (imported placements pin them anyway).
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (packed_cells.count(ci->name))
            continue;
        if (ci->type == ctx->id("RAMD64E")) {
            xform_cell(dram_rules, ci);
        } else if (ci->type == ctx->id("RAMD32") || ci->type == ctx->id("RAMS32")) {
            bool is5 = false;
            auto bel = ci->attrs.find(ctx->id("BEL"));
            if (bel != ci->attrs.end() &&
                bel->second.as_string().find("5LUT") != std::string::npos)
                is5 = true;
            if (ci->type == ctx->id("RAMS32")) {
                // single-port: the slice write address is the read
                // address -- mirror the ADR nets onto WADR before the
                // xform so the WA pins are modelled like the macro path
                for (int i = 0; i < 5; i++) {
                    IdString adr = ctx->id("ADR" + std::to_string(i));
                    IdString wadr = ctx->id("WADR" + std::to_string(i));
                    NetInfo *a = get_net_or_empty(ci, adr);
                    rename_port(ctx, ci, adr, ctx->id("RADR" + std::to_string(i)));
                    if (a != nullptr) {
                        ci->ports[wadr].name = wadr;
                        ci->ports[wadr].type = PORT_IN;
                        connect_port(ctx, a, ci, wadr);
                    }
                }
                ci->type = ctx->id("RAMD32");
            }
            xform_cell(is5 ? dram32_5_rules : dram32_6_rules, ci);
        }
    }

    // Whole-SLICE DRAM
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("RAM64M") || ci->type == ctx->id("RAM32M")) {
            bool is_64 = (cell.second->type == ctx->id("RAM64M"));
            int abits = is_64 ? 6 : 5;
            int dbits = is_64 ? 1 : 2;
            DRAMControlSet dcs;
            for (int i = 0; i < abits; i++)
                dcs.wa.push_back(get_net_or_empty(ci, ctx->id("ADDRD[" + std::to_string(i) + "]")));
            dcs.wclk = get_net_or_empty(ci, ctx->id("WCLK"));
            dcs.we = get_net_or_empty(ci, ctx->id("WE"));
            dcs.wclk_inv = bool_or_default(ci->params, ctx->id("IS_WCLK_INVERTED"));
            CellInfo *base = nullptr;
            int zoffset = ctx->xc7 ? 0 : 4;
            // Imported placement: a BEL attr on the RAM32M/RAM64M parent
            // ("SITE/A6LUT") was silently DROPPED when the macro split into
            // DPR subcells -- HeAP then floated the whole DRAM anywhere (an
            // SVS-stamped RAM32M landed on a slice holding BEL-pinned FFs of
            // a DIFFERENT clock; every input arc of that slice unroutable).
            // Pin each subcell absolutely at the stamped site (leaf from its
            // slot letter + 5/6LUT), and skip the relative-constraint chain
            // for pinned parents (pinned cells must not be chain-constrained,
            // cf. the carry import).
            std::string dram_site;
            if (ci->attrs.count(ctx->id("BEL"))) {
                std::string b = ci->attrs.at(ctx->id("BEL")).as_string();
                auto sl = b.find('/');
                dram_site = (sl == std::string::npos) ? b : b.substr(0, sl);
            }
            bool pinned = !dram_site.empty();
            for (int i = 0; i < 4; i++) {
                std::vector<NetInfo *> address;
                for (int j = 0; j < abits; j++) {
                    address.push_back(get_net_or_empty(ci, ctx->id(stringf("ADDR%c[%d]", 'A' + i, j))));
                }
                if (is_64) {
                    NetInfo *di = get_net_or_empty(ci, ctx->id(stringf("DI%c", 'A' + i)));
                    NetInfo *dout = get_net_or_empty(ci, ctx->id(stringf("DO%c", 'A' + i)));
                    disconnect_port(ctx, ci, ctx->id(stringf("DI%c", 'A' + i)));
                    disconnect_port(ctx, ci, ctx->id(stringf("DO%c", 'A' + i)));
                    CellInfo *dram = create_dram_lut(stringf("%s/DPR%d", ctx->nameOf(ci), i),
                                                     pinned ? nullptr : base, dcs, address, di,
                                                     dout, zoffset + i);
                    if (pinned)
                        dram->attrs[ctx->id("BEL")] =
                                stringf("%s/%c6LUT", dram_site.c_str(), 'A' + i);
                    if (base == nullptr)
                        base = dram;
                    if (ci->params.count(ctx->id(stringf("INIT_%c", 'A' + i))))
                        dram->params[ctx->id("INIT")] = ci->params[ctx->id(stringf("INIT_%c", 'A' + i))];
                } else {
                    for (int j = 0; j < dbits; j++) {
                        NetInfo *di = get_net_or_empty(ci, ctx->id(stringf("DI%c[%d]", 'A' + i, j)));
                        NetInfo *dout = get_net_or_empty(ci, ctx->id(stringf("DO%c[%d]", 'A' + i, j)));
                        disconnect_port(ctx, ci, ctx->id(stringf("DI%c[%d]", 'A' + i, j)));
                        disconnect_port(ctx, ci, ctx->id(stringf("DO%c[%d]", 'A' + i, j)));
                        CellInfo *dram = create_dram32_lut(stringf("%s/DPR%d_%d", ctx->nameOf(ci), i, j),
                                                           pinned ? nullptr : base, dcs,
                                                           address, di, dout, (j == 0), zoffset + i);
                        if (pinned)
                            dram->attrs[ctx->id("BEL")] =
                                    stringf("%s/%c%cLUT", dram_site.c_str(), 'A' + i, (j == 0) ? '5' : '6');
                        if (base == nullptr)
                            base = dram;
                        if (ci->params.count(ctx->id(stringf("INIT_%c", 'A' + i)))) {
                            auto orig_init =
                                    ci->params.at(ctx->id(stringf("INIT_%c", 'A' + i))).extract(0, 64).as_bits();
                            std::string init;
                            for (int k = 31; k >= 0; k--) {
                                init.push_back(orig_init.at(k * 2 + j) ? Property::State::S1 : Property::State::S0);
                            }
                            dram->params[ctx->id("INIT")] = Property::from_string(init);
                        }
                    }
                }
            }
            packed_cells.insert(ci->name);
        }
    }

    flush_cells();
}

NEXTPNR_NAMESPACE_END
