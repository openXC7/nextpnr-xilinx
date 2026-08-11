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
#include <boost/algorithm/string.hpp>
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

CellInfo *XC7Packer::insert_ibuf(IdString name, IdString type, NetInfo *i, NetInfo *o)
{
    auto inbuf = create_cell(ctx, type, name);
    connect_port(ctx, i, inbuf.get(), ctx->id("I"));
    connect_port(ctx, o, inbuf.get(), ctx->id("O"));
    CellInfo *inbuf_ptr = inbuf.get();
    new_cells.push_back(std::move(inbuf));
    return inbuf_ptr;
}

CellInfo *XC7Packer::insert_diffibuf(IdString name, IdString type, const std::array<NetInfo *, 2> &i, NetInfo *o)
{
    auto inbuf = create_cell(ctx, type, name);
    connect_port(ctx, i[0], inbuf.get(), ctx->id("I"));
    connect_port(ctx, i[1], inbuf.get(), ctx->id("IB"));
    connect_port(ctx, o, inbuf.get(), ctx->id("O"));
    CellInfo *inbuf_ptr = inbuf.get();
    new_cells.push_back(std::move(inbuf));
    return inbuf_ptr;
}

std::string XC7Packer::get_tilename_by_sitename(Context *ctx, std::string site)
{
    if (ctx->site_by_name.count(site)) {
        int tile, siteid;
        std::tie(tile, siteid) = ctx->site_by_name.at(site);
        return ctx->chip_info->tile_insts[tile].name.get();
    }
    return std::string();
}

void XC7Packer::decompose_iob(CellInfo *xil_iob, bool is_hr, const std::string &iostandard)
{
    bool is_se_ibuf = xil_iob->type == ctx->id("IBUF") || xil_iob->type == ctx->id("IBUF_IBUFDISABLE") ||
                      xil_iob->type == ctx->id("IBUF_INTERMDISABLE");
    bool is_se_iobuf = xil_iob->type == ctx->id("IOBUF") || xil_iob->type == ctx->id("IOBUF_DCIEN") ||
                       xil_iob->type == ctx->id("IOBUF_INTERMDISABLE");
    bool is_se_obuf = xil_iob->type == ctx->id("OBUF") || xil_iob->type == ctx->id("OBUFT");

    auto pad_site = [&](NetInfo *n) {
        for (auto user : n->users)
            if (user.cell->type == ctx->id("PAD"))
                return ctx->getBelSite(ctx->getBelByName(ctx->id(user.cell->attrs[ctx->id("BEL")].as_string())));
        NPNR_ASSERT_FALSE(("can't find PAD for net " + n->name.str(ctx)).c_str());
    };

    // Name a pad the way the user wrote it, so a placement complaint can be traced
    // back to a line of XDC rather than to a site number.
    auto pad_desc = [&](NetInfo *n, const std::string &site) {
        std::string d;
        for (auto user : n->users) {
            if (user.cell->type != ctx->id("PAD"))
                continue;
            d = "'" + user.cell->name.str(ctx) + "'";
            for (auto attr : {ctx->id("LOC"), ctx->id("PACKAGE_PIN")})
                if (user.cell->attrs.count(attr)) {
                    d += " (package pin " + user.cell->attrs.at(attr).as_string() + ")";
                    break;
                }
            break;
        }
        if (d.empty())
            d = "<unknown pad>";
        return d + " at site " + site;
    };

    // A differential pair is the master (P) and slave (N) site of ONE IOB tile, and only
    // the master site carries the M bels.  Constraining P to an N pin, or splitting the
    // pair across two tiles, therefore fails later as a BEL lookup -- "No Bel named
    // 'IOB_X1Y147/IOB33M/OUTBUF' located for this chip" -- which says nothing about the
    // pins the user actually wrote.  Check it up front and say what is wrong. (fixes #9)
    auto check_diff_pair = [&](NetInfo *p_net, NetInfo *n_net, const std::string &site_p,
                               const std::string &site_n, bool is18, const char *m_bel, const char *s_bel) {
        auto has_bel = [&](const std::string &name) { return ctx->getBelByName(ctx->id(name)) != BelId(); };
        bool p_is_master = has_bel(site_p + m_bel);
        bool n_is_slave = has_bel(site_n + s_bel);
        bool same_tile = get_tilename_by_sitename(ctx, site_p) == get_tilename_by_sitename(ctx, site_n);
        if (p_is_master && n_is_slave && same_tile)
            return;
        const char *why = !p_is_master ? "its P side is constrained to the N (slave) pin of a pair"
                          : !n_is_slave ? "its N side is constrained to a P (master) pin"
                                        : "the two pins are not the two halves of one differential pair";
        log_error("%s '%s' cannot be placed: %s.\n"
                  "       P side: %s\n"
                  "       N side: %s\n"
                  "       Constrain the P port to an IO_L<n>P_... pin and the N port to the matching\n"
                  "       IO_L<n>N_... pin of the same pair (see package_pins.csv for the part).\n",
                  xil_iob->type.c_str(ctx), xil_iob->name.c_str(ctx), why,
                  pad_desc(p_net, site_p).c_str(), pad_desc(n_net, site_n).c_str());
        (void)is18;
    };

    /*
     * IO primitives in Xilinx are complex "macros" that usually expand to more than one BEL
     * To avoid various nasty bugs (such as auto-transformation by Vivado of dedicated INV primitives to LUT1s), we
     * have to maintain this hierarchy so it can be re-built during DCP conversion in RapidWright
     */
    std::unordered_map<IdString, PortInfo> orig_ports = xil_iob->ports;
    std::vector<CellInfo *> subcells;

    if (is_se_ibuf || is_se_iobuf) {
        log_info("Generating input buffer for '%s'\n", xil_iob->name.c_str(ctx));
        NetInfo *pad_net = get_net_or_empty(xil_iob, is_se_iobuf ? ctx->id("IO") : ctx->id("I"));
        NPNR_ASSERT(pad_net != nullptr);
        std::string site = pad_site(pad_net);
        if (!is_se_iobuf)
            disconnect_port(ctx, xil_iob, ctx->id("I"));

        NetInfo *top_out = get_net_or_empty(xil_iob, ctx->id("O"));
        disconnect_port(ctx, xil_iob, ctx->id("O"));

        IdString ibuf_type = ctx->id("IBUF");
        if (xil_iob->type == ctx->id("IBUF_IBUFDISABLE") || xil_iob->type == ctx->id("IOBUF_DCIEN"))
            ibuf_type = ctx->id("IBUF_IBUFDISABLE");
        if (xil_iob->type == ctx->id("IBUF_INTERMDISABLE") || xil_iob->type == ctx->id("IOBUF_INTERMDISABLE"))
            ibuf_type = ctx->id("IBUF_INTERMDISABLE");

        CellInfo *inbuf = insert_ibuf(int_name(xil_iob->name, "IBUF", is_se_iobuf), ibuf_type, pad_net, top_out);
        std::string tile = get_tilename_by_sitename(ctx, site);
        if ((boost::starts_with(tile, "RIOB18_") || boost::starts_with(tile, "LIOB18_")))
            inbuf->attrs[ctx->id("BEL")] = site + "/IOB18/INBUF_DCIEN";
        else
            inbuf->attrs[ctx->id("BEL")] = site + "/IOB33/INBUF_EN";
        replace_port(xil_iob, ctx->id("IBUFDISABLE"), inbuf, ctx->id("IBUFDISABLE"));
        replace_port(xil_iob, ctx->id("INTERMDISABLE"), inbuf, ctx->id("INTERMDISABLE"));

        if (is_se_iobuf)
            subcells.push_back(inbuf);
    }

    if (is_se_obuf || is_se_iobuf) {
        log_info("Generating output buffer for '%s'\n", xil_iob->name.c_str(ctx));
        NetInfo *pad_net = get_net_or_empty(xil_iob, is_se_iobuf ? ctx->id("IO") : ctx->id("O"));
        NPNR_ASSERT(pad_net != nullptr);
        std::string site = pad_site(pad_net);
        disconnect_port(ctx, xil_iob, is_se_iobuf ? ctx->id("IO") : ctx->id("O"));
        bool has_dci = xil_iob->type == ctx->id("IOBUF_DCIEN");
        CellInfo *obuf = insert_obuf(
                int_name(xil_iob->name, (is_se_iobuf || xil_iob->type == ctx->id("OBUFT")) ? "OBUFT" : "OBUF",
                         !is_se_obuf),
                is_se_iobuf ? (has_dci ? ctx->id("OBUFT_DCIEN") : ctx->id("OBUFT")) : xil_iob->type,
                get_net_or_empty(xil_iob, ctx->id("I")), pad_net, get_net_or_empty(xil_iob, ctx->id("T")));
        std::string tile = get_tilename_by_sitename(ctx, site);
        if ((boost::starts_with(tile, "RIOB18_") || boost::starts_with(tile, "LIOB18_")))
            obuf->attrs[ctx->id("BEL")] = site + "/IOB18/OUTBUF_DCIEN";
        else
            obuf->attrs[ctx->id("BEL")] = site + "/IOB33/OUTBUF";
        replace_port(xil_iob, ctx->id("DCITERMDISABLE"), obuf, ctx->id("DCITERMDISABLE"));
        if (is_se_iobuf)
            subcells.push_back(obuf);
    }

    bool is_diff_ibuf = xil_iob->type == ctx->id("IBUFDS") || xil_iob->type == ctx->id("IBUFGDS") ||
                        xil_iob->type == ctx->id("IBUFDS_INTERMDISABLE");
    bool is_diff_iobuf = xil_iob->type == ctx->id("IOBUFDS") || xil_iob->type == ctx->id("IOBUFDS_DCIEN");
    bool is_diff_out_iobuf = xil_iob->type == ctx->id("IOBUFDS_DIFF_OUT") ||
                             xil_iob->type == ctx->id("IOBUFDS_DIFF_OUT_DCIEN") ||
                             xil_iob->type == ctx->id("IOBUFDS_DIFF_OUT_INTERMDISABLE");
    bool is_diff_obuf = xil_iob->type == ctx->id("OBUFDS") || xil_iob->type == ctx->id("OBUFTDS");

    if (is_diff_ibuf || is_diff_iobuf) {
        NetInfo *pad_p_net =
                get_net_or_empty(xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IO") : ctx->id("I"));
        NPNR_ASSERT(pad_p_net != nullptr);
        std::string site_p = pad_site(pad_p_net);
        NetInfo *pad_n_net =
                get_net_or_empty(xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IOB") : ctx->id("IB"));
        NPNR_ASSERT(pad_n_net != nullptr);
        std::string site_n = pad_site(pad_n_net);
        std::string tile_p = get_tilename_by_sitename(ctx, site_p);
        bool is_riob18 = (boost::starts_with(tile_p, "RIOB18_") || boost::starts_with(tile_p, "LIOB18_"));

        if (!is_diff_iobuf && !is_diff_out_iobuf) {
            disconnect_port(ctx, xil_iob, ctx->id("I"));
            disconnect_port(ctx, xil_iob, ctx->id("IB"));
        }

        NetInfo *top_out = get_net_or_empty(xil_iob, ctx->id("O"));
        disconnect_port(ctx, xil_iob, ctx->id("O"));

        IdString ibuf_type = ctx->id("IBUFDS");
        CellInfo *inbuf = insert_diffibuf(int_name(xil_iob->name, "IBUF", is_se_iobuf), ibuf_type,
                                          {pad_p_net, pad_n_net}, top_out);
        if (is_riob18) {
            inbuf->attrs[ctx->id("BEL")] = site_p + "/IOB18M/INBUF_DCIEN";
            inbuf->attrs[ctx->id("X_IOB_SITE_TYPE")] = std::string("IOB18M");
        } else {
            inbuf->attrs[ctx->id("BEL")] = site_p + "/IOB33M/INBUF_EN";
            inbuf->attrs[ctx->id("X_IOB_SITE_TYPE")] = std::string("IOB33M");
        }

        if (is_diff_iobuf)
            subcells.push_back(inbuf);
    }

    if (is_diff_obuf || is_diff_out_iobuf || is_diff_iobuf) {
        // FIXME: true diff outputs
        NetInfo *pad_p_net =
                get_net_or_empty(xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IO") : ctx->id("O"));
        NPNR_ASSERT(pad_p_net != nullptr);
        std::string site_p = pad_site(pad_p_net);
        NetInfo *pad_n_net =
                get_net_or_empty(xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IOB") : ctx->id("OB"));
        NPNR_ASSERT(pad_n_net != nullptr);
        std::string site_n = pad_site(pad_n_net);
        std::string tile_p = get_tilename_by_sitename(ctx, site_p);
        bool is_riob18 = (boost::starts_with(tile_p, "RIOB18_") || boost::starts_with(tile_p, "LIOB18_"));

        disconnect_port(ctx, xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IO") : ctx->id("O"));
        disconnect_port(ctx, xil_iob, (is_diff_iobuf || is_diff_out_iobuf) ? ctx->id("IOB") : ctx->id("OB"));

        NetInfo *inv_i = create_internal_net(xil_iob->name, is_diff_obuf ? "I_B" : "OBUFTDS$subnet$I_B");
        CellInfo *inv = insert_outinv(int_name(xil_iob->name, is_diff_obuf ? "INV" : "OBUFTDS$subcell$INV"),
                                      get_net_or_empty(xil_iob, ctx->id("I")), inv_i);
        if (is_riob18) {
            inv->attrs[ctx->id("BEL")] = site_n + "/IOB18S/O_ININV";
            inv->attrs[ctx->id("X_IOB_SITE_TYPE")] = std::string("IOB18S");
        } else {
            inv->attrs[ctx->id("BEL")] = site_n + "/IOB33S/O_ININV";
            inv->attrs[ctx->id("X_IOB_SITE_TYPE")] = std::string("IOB33S");
        }

        check_diff_pair(pad_p_net, pad_n_net, site_p, site_n, is_riob18,
                        is_riob18 ? "/IOB18M/OUTBUF_DCIEN" : "/IOB33M/OUTBUF",
                        is_riob18 ? "/IOB18S/OUTBUF_DCIEN" : "/IOB33S/OUTBUF");

        bool has_dci = xil_iob->type == ctx->id("IOBUFDS_DCIEN") || xil_iob->type == ctx->id("IOBUFDSE3");

        CellInfo *obuf_p = insert_obuf(int_name(xil_iob->name, is_diff_obuf ? "P" : "OBUFTDS$subcell$P"),
                                       (is_diff_iobuf || is_diff_out_iobuf || (xil_iob->type == ctx->id("OBUFTDS")))
                                               ? (has_dci ? ctx->id("OBUFT_DCIEN") : ctx->id("OBUFT"))
                                               : ctx->id("OBUF"),
                                       get_net_or_empty(xil_iob, ctx->id("I")), pad_p_net,
                                       get_net_or_empty(xil_iob, ctx->id("T")));

        if (is_riob18) {
            obuf_p->attrs[ctx->id("BEL")] = site_p + "/IOB18M/OUTBUF_DCIEN";
            obuf_p->attrs[ctx->id("X_IOB_SITE_TYPE")] = std::string("IOB18M");
        } else {
            obuf_p->attrs[ctx->id("BEL")] = site_p + "/IOB33M/OUTBUF";
            obuf_p->attrs[ctx->id("X_IOB_SITE_TYPE")] = std::string("IOB33M");
        }
        subcells.push_back(obuf_p);
        connect_port(ctx, get_net_or_empty(xil_iob, ctx->id("DCITERMDISABLE")), obuf_p, ctx->id("DCITERMDISABLE"));

        CellInfo *obuf_n = insert_obuf(int_name(xil_iob->name, is_diff_obuf ? "N" : "OBUFTDS$subcell$N"),
                                       (is_diff_iobuf || is_diff_out_iobuf || (xil_iob->type == ctx->id("OBUFTDS")))
                                               ? (has_dci ? ctx->id("OBUFT_DCIEN") : ctx->id("OBUFT"))
                                               : ctx->id("OBUF"),
                                       inv_i, pad_n_net, get_net_or_empty(xil_iob, ctx->id("T")));

        if (is_riob18) {
            obuf_n->attrs[ctx->id("BEL")] = site_n + "/IOB18S/OUTBUF_DCIEN";
            obuf_n->attrs[ctx->id("X_IOB_SITE_TYPE")] = std::string("IOB18S");
        } else {
            obuf_n->attrs[ctx->id("BEL")] = site_n + "/IOB33S/OUTBUF";
            obuf_n->attrs[ctx->id("X_IOB_SITE_TYPE")] = std::string("IOB33S");
        }
        connect_port(ctx, get_net_or_empty(xil_iob, ctx->id("DCITERMDISABLE")), obuf_n, ctx->id("DCITERMDISABLE"));

        disconnect_port(ctx, xil_iob, ctx->id("DCITERMDISABLE"));

        subcells.push_back(inv);
        subcells.push_back(obuf_p);
        subcells.push_back(obuf_n);
    }

    if (!subcells.empty()) {
        for (auto sc : subcells) {
            sc->attrs[ctx->id("X_ORIG_MACRO_PRIM")] = xil_iob->type.str(ctx);
            for (auto &p : sc->ports) {
                std::string macro_ports;
                for (auto &orig : orig_ports) {
                    if ((orig.second.net != nullptr) && (orig.second.net == p.second.net)) {
                        macro_ports += orig.first.str(ctx);
                        macro_ports += ',';
                        macro_ports += (orig.second.type == PORT_INOUT) ? "inout"
                                       : (orig.second.type == PORT_OUT) ? "out"
                                                                        : "in";
                        macro_ports += ";";
                    }
                }
                if (!macro_ports.empty()) {
                    macro_ports.erase(macro_ports.size() - 1);
                    sc->attrs[ctx->id("X_MACRO_PORTS_" + p.first.str(ctx))] = macro_ports;
                }
            }
        }
    }
}

void XC7Packer::pack_io()
{
    // make sure the supporting data structure of
    // get_tilename_by_sitename()
    // is initialized before we use it below
    ctx->setup_byname();

    log_info("Inserting IO buffers..\n");

    get_top_level_pins(ctx, toplevel_ports);
    // Insert PAD cells on top level IO, and IO buffers where one doesn't exist already
    std::vector<std::pair<CellInfo *, PortRef>> pad_and_buf;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("$nextpnr_ibuf") || ci->type == ctx->id("$nextpnr_iobuf") ||
            ci->type == ctx->id("$nextpnr_obuf"))
            pad_and_buf.push_back(insert_pad_and_buf(ci));
    }
    flush_cells();
    std::unordered_set<BelId> used_io_bels;
    int unconstr_io_count = 0;
    for (auto &iob : pad_and_buf) {
        CellInfo *pad = iob.first;
        // Process location constraints
        if (pad->attrs.count(ctx->id("PACKAGE_PIN"))) {
            pad->attrs[ctx->id("LOC")] = pad->attrs.at(ctx->id("PACKAGE_PIN"));
        }
        if (pad->attrs.count(ctx->id("LOC"))) {
            std::string loc = pad->attrs.at(ctx->id("LOC")).to_string();
            std::string site = ctx->getPackagePinSite(loc);
            if (site.empty())
                log_error("Unable to constrain IO '%s', device does not have a pin named '%s'\n", pad->name.c_str(ctx),
                          loc.c_str());
            log_info("    Constraining '%s' to site '%s'\n", pad->name.c_str(ctx), site.c_str());
            std::string tile = get_tilename_by_sitename(ctx, site);
            log_info("    Tile '%s'\n", tile.c_str());
            if (boost::starts_with(tile, "GTP_COMMON") || boost::starts_with(tile, "GTP_CHANNEL") ||
                boost::starts_with(tile, "GTX_COMMON") || boost::starts_with(tile, "GTX_CHANNEL")) {
                auto pad_bel = std::string(site + "/PAD");
                pad->attrs[id_BEL] = pad_bel;
            } else {
                if ((boost::starts_with(tile, "RIOB18_") || boost::starts_with(tile, "LIOB18_")))
                    pad->attrs[id_BEL] = std::string(site + "/IOB18/PAD");
                else
                    pad->attrs[id_BEL] = std::string(site + "/IOB33/PAD");
            }
            if (boost::starts_with(tile, "MONITOR_"))
                log_error("Cannot place regular IO on monitor/XADC site\n");
        }
        if (pad->attrs.count(ctx->id("BEL"))) {
            used_io_bels.insert(ctx->getBelByName(ctx->id(pad->attrs.at(ctx->id("BEL")).as_string())));
        } else {
            ++unconstr_io_count;
        }
    }
    std::queue<BelId> available_io_bels;
    IdString pad_id = ctx->xc7 ? ctx->id("PAD") : ctx->id("IOB_PAD");
    for (auto bel : ctx->getBels()) {
        if (int(available_io_bels.size()) >= unconstr_io_count)
            break;
        if (ctx->locInfo(bel).bel_data[bel.index].site_variant != 0)
            continue;
        if (ctx->getBelType(bel) != pad_id)
            continue;
        if (ctx->getBelPackagePin(bel) == ".")
            continue;
        if (used_io_bels.count(bel))
            continue;
        available_io_bels.push(bel);
    }
    int avail_count = int(available_io_bels.size());
    // Constrain unconstrained IO
    for (auto &iob : pad_and_buf) {
        CellInfo *pad = iob.first;
        if (!pad->attrs.count(ctx->id("BEL"))) {
            if (available_io_bels.empty()) {
                log_error("IO placer ran out of available IOs (%d available IO, %d unconstrained pins)\n", avail_count,
                          unconstr_io_count);
            }
            pad->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(available_io_bels.front()));
            available_io_bels.pop();
        }
    }
    // Decompose macro IO primitives to smaller primitives that map logically to the actual IO Bels
    for (auto &iob : pad_and_buf) {
        auto pad_cell = iob.first;
        auto buf_cell = iob.second.cell;

        if (packed_cells.count(buf_cell->name))
            continue;

        if (buf_cell->type == ctx->id("IBUFDS_GTE2")) {
            constrain_ibufds_gt_site(buf_cell, pad_cell->attrs[id_BEL].as_string());
            auto net = buf_cell->ports[ctx->id("O")].net;
            if (net == nullptr)
                log_error("IBUFDS_GTE2 instance %s output port is not connected\n", buf_cell->name.c_str(ctx));
            // Scan users of IBUFDS_GTE2.O.  Accepted silicon-legal patterns:
            //   - GTXE2_COMMON / GTPE2_COMMON (QPLL clustering)
            //   - GTXE2_CHANNEL direct (CPLL mode)
            //   - BUFG / BUFH / BUFHCE / BUFR (MGT REFCLK routed straight
            //     into a fabric clock buffer — used by the VC707 telegraph
            //     SGMIICLK path: IBUFDS_GTE2 -> BUFG -> 125 MHz fabric clock)
            CellInfo *gt_common = nullptr;
            bool has_gtxe2_channel_direct = false;
            bool has_bufg_direct = false;
            for (auto &usr : net->users) {
                if (usr.cell->type == id_GTPE2_COMMON || usr.cell->type == id_GTXE2_COMMON)
                    gt_common = usr.cell;
                else if (usr.cell->type == id_GTXE2_CHANNEL)
                    has_gtxe2_channel_direct = true;
                else if (usr.cell->type == ctx->id("BUFG")
                      || usr.cell->type == ctx->id("BUFH")
                      || usr.cell->type == ctx->id("BUFHCE")
                      || usr.cell->type == ctx->id("BUFR"))
                    has_bufg_direct = true;
            }
            if (gt_common) {
                constrain_gt(pad_cell, gt_common);
                continue;
            }
            // GTX CPLL mode: GTXE2_CHANNEL already directly connected (second pad pass).
            if (has_gtxe2_channel_direct)
                continue;
            // Direct fabric clock buffer — no GT* clustering needed.
            if (has_bufg_direct)
                continue;
            log_error("IBUFDS_GTE2 instance %s output port must be connected to "
                      "a GTPE2_COMMON, GTXE2_COMMON, GTXE2_CHANNEL, or BUFG/BUFH/BUFR\n",
                buf_cell->name.c_str(ctx));
        }

        // This OBUF is integrated into the GTP/GTX channel pad and does not need placing
        if (buf_cell->type == ctx->id("OBUF")) {
            auto net = buf_cell->ports[ctx->id("I")].net;
            if (net != nullptr) {
                auto driver_cell = net->driver.cell;
                if (driver_cell != nullptr && (driver_cell->type == id_GTPE2_CHANNEL || driver_cell->type == id_GTXE2_CHANNEL)) {
                    packed_cells.insert(buf_cell->name);
                    constrain_gt(pad_cell, driver_cell);
                    continue;
                }
            }
        }
        // This IBUF is integrated into the GTP/GTX channel pad and does not need placing
        if (buf_cell->type == ctx->id("IBUF")) {
            auto net = buf_cell->ports[ctx->id("O")].net;
            if (net != nullptr && net->users.size() == 1) {
                auto user_cell = net->users[0].cell;
                if (user_cell->type == id_GTPE2_CHANNEL || user_cell->type == id_GTXE2_CHANNEL) {
                    packed_cells.insert(buf_cell->name);
                    constrain_gt(pad_cell, user_cell);
                    continue;
                }
            }
        }

        decompose_iob(buf_cell, true, str_or_default(iob.first->attrs, ctx->id("IOSTANDARD"), ""));
        packed_cells.insert(buf_cell->name);
    }
    flush_cells();

    std::unordered_map<IdString, XFormRule> hriobuf_rules, hpiobuf_rules;
    hriobuf_rules[ctx->id("OBUF")].new_type = ctx->id("IOB33_OUTBUF");
    hriobuf_rules[ctx->id("OBUF")].port_xform[ctx->id("I")] = ctx->id("IN");
    hriobuf_rules[ctx->id("OBUF")].port_xform[ctx->id("O")] = ctx->id("OUT");
    hriobuf_rules[ctx->id("OBUF")].port_xform[ctx->id("T")] = ctx->id("TRI");
    hriobuf_rules[ctx->id("OBUFT")] = hriobuf_rules[ctx->id("OBUF")];

    hriobuf_rules[ctx->id("IBUF")].new_type = ctx->id("IOB33_INBUF_EN");
    hriobuf_rules[ctx->id("IBUF")].port_xform[ctx->id("I")] = ctx->id("PAD");
    hriobuf_rules[ctx->id("IBUF")].port_xform[ctx->id("O")] = ctx->id("OUT");
    hriobuf_rules[ctx->id("IBUF_INTERMDISABLE")] = hriobuf_rules[ctx->id("IBUF")];
    hriobuf_rules[ctx->id("IBUF_IBUFDISABLE")] = hriobuf_rules[ctx->id("IBUF")];
    hriobuf_rules[ctx->id("IBUFDS_INTERMDISABLE_INT")] = hriobuf_rules[ctx->id("IBUF")];
    hriobuf_rules[ctx->id("IBUFDS_INTERMDISABLE_INT")].port_xform[ctx->id("IB")] = ctx->id("DIFFI_IN");
    hriobuf_rules[ctx->id("IBUFDS")] = hriobuf_rules[ctx->id("IBUF")];
    hriobuf_rules[ctx->id("IBUFDS")].port_xform[ctx->id("IB")] = ctx->id("DIFFI_IN");
    // IBUFGDS: legacy clock-capable spelling of IBUFDS, same primitive on 7-series (#74)
    hriobuf_rules[ctx->id("IBUFGDS")] = hriobuf_rules[ctx->id("IBUFDS")];

    hpiobuf_rules[ctx->id("OBUF")].new_type = ctx->id("IOB18_OUTBUF_DCIEN");
    hpiobuf_rules[ctx->id("OBUF")].port_xform[ctx->id("I")] = ctx->id("IN");
    hpiobuf_rules[ctx->id("OBUF")].port_xform[ctx->id("O")] = ctx->id("OUT");
    hpiobuf_rules[ctx->id("OBUF")].port_xform[ctx->id("T")] = ctx->id("TRI");
    hpiobuf_rules[ctx->id("OBUFT")] = hpiobuf_rules[ctx->id("OBUF")];

    hpiobuf_rules[ctx->id("IBUF")].new_type = ctx->id("IOB18_INBUF_DCIEN");
    hpiobuf_rules[ctx->id("IBUF")].port_xform[ctx->id("I")] = ctx->id("PAD");
    hpiobuf_rules[ctx->id("IBUF")].port_xform[ctx->id("O")] = ctx->id("OUT");
    hpiobuf_rules[ctx->id("IBUF_INTERMDISABLE")] = hpiobuf_rules[ctx->id("IBUF")];
    hpiobuf_rules[ctx->id("IBUF_IBUFDISABLE")] = hpiobuf_rules[ctx->id("IBUF")];
    hriobuf_rules[ctx->id("IBUFDS_INTERMDISABLE_INT")] = hriobuf_rules[ctx->id("IBUF")];
    hpiobuf_rules[ctx->id("IBUFDS_INTERMDISABLE_INT")].port_xform[ctx->id("IB")] = ctx->id("DIFFI_IN");
    hpiobuf_rules[ctx->id("IBUFDS")] = hpiobuf_rules[ctx->id("IBUF")];
    hpiobuf_rules[ctx->id("IBUFDS")].port_xform[ctx->id("IB")] = ctx->id("DIFFI_IN");
    hpiobuf_rules[ctx->id("IBUFGDS")] = hpiobuf_rules[ctx->id("IBUFDS")];

    // Special xform for OBUFx and IBUFx.
    std::unordered_map<IdString, XFormRule> rules;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        // GTP bufs don't need transforming, they are hardwired
        if (boost::starts_with(ci->type.str(ctx), "GTP")) continue;
        if (boost::starts_with(ci->type.str(ctx), "GTX")) continue;
        if (boost::starts_with(ci->type.str(ctx), "IBUFDS_GTE2")) continue;
        if (!ci->attrs.count(ctx->id("BEL")))
            continue;
        if (ci->type == id_PAD)
            continue;
        std::string belname = ci->attrs[id_BEL].c_str();
        size_t pos = belname.find("/");
        if (belname.substr(pos+1, 5) == "IOB18")
            rules = hpiobuf_rules;
        else if (belname.substr(pos+1, 5) == "IOB33")
            rules = hriobuf_rules;
        else
            // Cell has a BEL attribute that isn't an IOB — most likely a
            // SLICE/CLB/etc. cell that was pinned via JSON.  Skip it; the
            // SLICE-bound BEL is enforced later by the placer.
            continue;
        if (rules.count(ci->type)) {
            xform_cell(rules, ci);
        }
    }

    std::unordered_map<IdString, XFormRule> hrio_rules;
    hrio_rules[ctx->id("PAD")].new_type = ctx->id("PAD");

    hrio_rules[ctx->id("INV")].new_type = ctx->id("INVERTER");
    hrio_rules[ctx->id("INV")].port_xform[ctx->id("I")] = ctx->id("IN");
    hrio_rules[ctx->id("INV")].port_xform[ctx->id("O")] = ctx->id("OUT");

    hrio_rules[ctx->id("PS7")].new_type = ctx->id("PS7_PS7");
    hrio_rules[ctx->id("PCIE_2_1")].new_type = ctx->id("PCIE_2_1_PCIE_2_1");

    generic_xform(hrio_rules, true);

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        std::string type = ci->type.str(ctx);
        if (!boost::starts_with(type, "IOB33") && !boost::starts_with(type, "IOB18"))
            continue;
        if (!ci->attrs.count(ctx->id("X_IOB_SITE_TYPE")))
            continue;
        type.replace(0, 5, ci->attrs.at(ctx->id("X_IOB_SITE_TYPE")).as_string());
        ci->type = ctx->id(type);
    }

    // check all PAD cells for IOSTANDARD/DRIVE
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        std::string type = ci->type.str(ctx);
        if (type != "PAD") continue;
        check_valid_pad(ci, type);
    }
}

void XC7Packer::check_valid_pad(CellInfo *ci, std::string type)
{
    // GT pads don't need IOSTANDARD constraints
    auto bel = ci->attrs[id_BEL].as_string();
    if (boost::starts_with(bel, "IPAD") || boost::starts_with(bel, "OPAD")) return;

    auto iostandard_attr = ci->attrs.find(id_IOSTANDARD);
    if (iostandard_attr == ci->attrs.end())
        log_error("port %s of type %s has no IOSTANDARD property", ci->name.c_str(ctx), type.c_str());

    auto iostandard = iostandard_attr->second.as_string();
    if (!boost::starts_with(iostandard, "LVTTL") &&
        !boost::starts_with(iostandard, "LVCMOS")) return;

    auto drive_attr = ci->attrs.find(ctx->id("DRIVE"));
    // no drive strength attribute: use default
    if (drive_attr == ci->attrs.end()) return;
    int64_t drive;
    if (drive_attr->second.is_string)
        drive = std::stoi(drive_attr->second.as_string());
    else
        drive = drive_attr->second.as_int64();

    bool is_iob33 = boost::starts_with(type, "IOB33");
    if (is_iob33) {
        if (drive == 4 || drive == 8 || drive == 12) return;
        if (iostandard != "LVCMOS12" && drive == 16) return;
        if ((iostandard == "LVCMOS18" || iostandard == "LVTTL") && drive == 24) return;
    } else { // IOB18
        if (drive == 2 || drive == 4 || drive == 6 || drive == 8)     return;
        if (iostandard != "LVCMOS12" && (drive == 12 || drive == 16)) return;
    }

    std::string drive_attr_str;
    if (drive_attr->second.is_string)
        drive_attr_str = drive_attr->second.as_string();
    else
        drive_attr_str = std::to_string(drive);
    log_error("unsupported DRIVE strength property %s for port %s",
        drive_attr_str.c_str(), ci->name.c_str(ctx));
}

std::string XC7Packer::get_ologic_site(const std::string &io_bel)
{
    BelId ibc_bel;
    if (boost::contains(io_bel, "IOB18"))
        ibc_bel = ctx->getBelByName(ctx->id(io_bel.substr(0, io_bel.find('/')) + "/IOB18/OUTBUF_DCIEN"));
    else
        ibc_bel = ctx->getBelByName(ctx->id(io_bel.substr(0, io_bel.find('/')) + "/IOB33/OUTBUF"));
    std::queue<WireId> visit;
    visit.push(ctx->getBelPinWire(ibc_bel, ctx->id("IN")));

    while (!visit.empty()) {
        WireId cursor = visit.front();
        visit.pop();
        for (auto bp : ctx->getWireBelPins(cursor)) {
            std::string site = ctx->getBelSite(bp.bel);
            if (boost::starts_with(site, "OLOGIC"))
                return site;
        }
        for (auto pip : ctx->getPipsUphill(cursor))
            visit.push(ctx->getPipSrcWire(pip));
    }
    NPNR_ASSERT_FALSE("failed to find OLOGIC");
}

std::string XC7Packer::get_ilogic_site(const std::string &io_bel)
{
    BelId ibc_bel;
    if (boost::contains(io_bel, "IOB18"))
        ibc_bel = ctx->getBelByName(ctx->id(io_bel.substr(0, io_bel.find('/')) + "/IOB18/INBUF_DCIEN"));
    else
        ibc_bel = ctx->getBelByName(ctx->id(io_bel.substr(0, io_bel.find('/')) + "/IOB33/INBUF_EN"));
    std::queue<WireId> visit;
    visit.push(ctx->getBelPinWire(ibc_bel, ctx->id("OUT")));

    while (!visit.empty()) {
        WireId cursor = visit.front();
        visit.pop();
        for (auto bp : ctx->getWireBelPins(cursor)) {
            std::string site = ctx->getBelSite(bp.bel);
            if (boost::starts_with(site, "ILOGIC"))
                return site;
        }
        for (auto pip : ctx->getPipsDownhill(cursor))
            visit.push(ctx->getPipDstWire(pip));
    }
    NPNR_ASSERT_FALSE("failed to find ILOGIC");
}

std::string XC7Packer::get_idelay_site(const std::string &io_bel)
{
    BelId ibc_bel;
    if (boost::contains(io_bel, "IOB18"))
        ibc_bel = ctx->getBelByName(ctx->id(io_bel.substr(0, io_bel.find('/')) + "/IOB18/INBUF_DCIEN"));
    else
      ibc_bel = ctx->getBelByName(ctx->id(io_bel.substr(0, io_bel.find('/')) + "/IOB33/INBUF_EN"));
    std::queue<WireId> visit;
    visit.push(ctx->getBelPinWire(ibc_bel, ctx->id("OUT")));

    while (!visit.empty()) {
        WireId cursor = visit.front();
        visit.pop();
        for (auto bp : ctx->getWireBelPins(cursor)) {
            std::string site = ctx->getBelSite(bp.bel);
            if (boost::starts_with(site, "IDELAY"))
                return site;
        }
        for (auto pip : ctx->getPipsDownhill(cursor))
            visit.push(ctx->getPipDstWire(pip));
    }
    NPNR_ASSERT_FALSE("failed to find IDELAY");
}

std::string XC7Packer::get_odelay_site(const std::string &io_bel)
{
    BelId obc_bel;
    if (boost::contains(io_bel, "IOB18"))
        obc_bel = ctx->getBelByName(ctx->id(io_bel.substr(0, io_bel.find('/')) + "/IOB18/OUTBUF_DCIEN"));
    else
        log_error("BEL %s is located on a high range bank. High range banks do not have ODELAY", io_bel.c_str());

    std::queue<WireId> visit;
    visit.push(ctx->getBelPinWire(obc_bel, ctx->id("IN")));

    while (!visit.empty()) {
        WireId cursor = visit.front();
        visit.pop();
        for (auto bp : ctx->getWireBelPins(cursor)) {
            std::string site = ctx->getBelSite(bp.bel);
            if (boost::starts_with(site, "ODELAY"))
                return site;
        }
        for (auto pip : ctx->getPipsUphill(cursor))
            visit.push(ctx->getPipSrcWire(pip));
    }
    NPNR_ASSERT_FALSE("failed to find ODELAY");
}

std::string XC7Packer::get_ioctrl_site(const std::string &io_bel)
{
    std::vector<std::string> parts;
    boost::split(parts, io_bel, boost::is_any_of("/"));
    auto loc         = parts[0];
    auto iobank      = parts[1];
    auto pad_bel_str = loc + "/" + iobank + "/PAD";
    auto msg         = "could not get bel for: '" + pad_bel_str + "'";

    BelId pad_bel = ctx->getBelByName(ctx->id(pad_bel_str));
    NPNR_ASSERT_MSG(0 <= pad_bel.tile && 0 <= pad_bel.index, msg.c_str());

    int hclk_tile = ctx->getHclkForIob(pad_bel);
    auto &td = ctx->chip_info->tile_insts[hclk_tile];
    for (int i = 0; i < td.num_sites; i++) {
        auto &sd = td.site_insts[i];
        std::string sn = sd.name.get();
        if (boost::starts_with(sn, "IDELAYCTRL"))
            return sn;
    }
    NPNR_ASSERT_FALSE("failed to find IOCTRL");
}

void XC7Packer::fold_inverter(CellInfo *cell, std::string port)
{
    IdString p = ctx->id(port);
    NetInfo *net = get_net_or_empty(cell, p);
    if (net == nullptr)
        return;
    CellInfo *drv = net->driver.cell;
    if (drv == nullptr)
        return;
    if (drv->type == ctx->id("LUT1") && int_or_default(drv->params, ctx->id("INIT"), 0) == 1) {
        disconnect_port(ctx, cell, p);
        NetInfo *preinv = get_net_or_empty(drv, ctx->id("I0"));
        connect_port(ctx, preinv, cell, p);
        cell->params[ctx->id("IS_" + port + "_INVERTED")] = 1;
        if (net->users.empty())
            packed_cells.insert(drv->name);
    } else if (drv->type == ctx->id("INV")) {
        disconnect_port(ctx, cell, p);
        NetInfo *preinv = get_net_or_empty(drv, ctx->id("I"));
        connect_port(ctx, preinv, cell, p);
        cell->params[ctx->id("IS_" + port + "_INVERTED")] = 1;
        if (net->users.empty())
            packed_cells.insert(drv->name);
    }
}

void XC7Packer::pack_iologic()
{
    std::unordered_map<IdString, BelId> iodelay_to_io;
    std::unordered_map<IdString, XFormRule> iologic_rules;

    // SERDES
    iologic_rules[ctx->id("ISERDESE2")].new_type = ctx->id("ISERDESE2_ISERDESE2");
    iologic_rules[ctx->id("OSERDESE2")].new_type = ctx->id("OSERDESE2_OSERDESE2");

    // DELAY
    iologic_rules[ctx->id("IDELAYE2")].new_type = ctx->id("IDELAYE2_IDELAYE2");
    iologic_rules[ctx->id("ODELAYE2")].new_type = ctx->id("ODELAYE2_ODELAYE2");

    // Handles pseudo-diff output buffers without finding multiple sinks
    auto find_p_outbuf = [&](NetInfo *net) {
        CellInfo *outbuf = nullptr;
        for (auto &usr : net->users) {
            IdString type = usr.cell->type;
            if (type == ctx->id("IOB33_OUTBUF") || type == ctx->id("IOB33M_OUTBUF")
                || type == ctx->id("IOB18_OUTBUF_DCIEN") || type == ctx->id("IOB18M_OUTBUF_DCIEN")) {
                if (outbuf != nullptr)
                    return (CellInfo *)nullptr; // drives multiple outputs
                outbuf = usr.cell;
            } else if (type == ctx->id("ODELAYE2")) {
                auto dataout = usr.cell->ports.find(ctx->id("DATAOUT"));
                if (dataout != usr.cell->ports.end()) {
                        for (auto &user : dataout->second.net->users) {
                            IdString dataout_type = user.cell->type;
                            if (dataout_type == ctx->id("IOB18_OUTBUF_DCIEN") ||
                                dataout_type == ctx->id("IOB18M_OUTBUF_DCIEN")) {
                                if (outbuf != nullptr)
                                    return (CellInfo *)nullptr; // drives multiple outputs
                                outbuf = user.cell;
                            }
                        }
                } else {
                    if (outbuf != nullptr)
                        return (CellInfo *)nullptr; // drives multiple outputs
                }
            }
        }
        return outbuf;
    };

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("IDELAYE2")) {
            NetInfo *d = get_net_or_empty(ci, ctx->id("IDATAIN"));
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected IDATAIN input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            BelId io_bel;
            if (   boost::contains(drv->type.str(ctx), "INBUF_EN")
                || boost::contains(drv->type.str(ctx), "INBUF_DCIEN"))
                io_bel = ctx->getBelByName(ctx->id(drv->attrs.at(ctx->id("BEL")).as_string()));
            else
                log_error("%s '%s' has IDATAIN input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                          ctx->nameOf(ci), drv->type.c_str(ctx));
            std::string iol_site = get_idelay_site(ctx->getBelName(io_bel).str(ctx));
            ci->attrs[ctx->id("BEL")] = iol_site + "/IDELAYE2";
            ci->attrs[ctx->id("X_IO_BEL")] = ctx->getBelName(io_bel).str(ctx);
            iodelay_to_io[ci->name] = io_bel;
        } else if (ci->type == ctx->id("ODELAYE2")) {
            NetInfo *clkin = get_net_or_empty(ci, ctx->id("CLKIN"));
            if (clkin != nullptr && clkin->name == ctx->id("$PACKER_GND_NET")) disconnect_port(ctx, ci, ctx->id("CLKIN"));

            NetInfo *dataout = get_net_or_empty(ci, ctx->id("DATAOUT"));
            if (dataout == nullptr || dataout->users.empty())
                log_error("%s '%s' has disconnected DATAOUT input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            BelId io_bel;
            auto no_users = dataout->users.size();
            for (auto userport : dataout->users) {
                CellInfo *user = userport.cell;
                auto user_type = user->type.str(ctx);
                // OBUFDS has the negative pin connected to an inverter
                if (no_users == 2 && user_type == "INVERTER") continue;
                if (   boost::contains(user_type, "OUTBUF_EN")
                    || boost::contains(user_type, "OUTBUF_DCIEN"))
                    io_bel = ctx->getBelByName(ctx->id(user->attrs.at(ctx->id("BEL")).as_string()));
                else
                    // TODO: support SIGNAL_PATTERN = CLOCK
                    log_error("%s '%s' has DATAOUT connected to unsupported cell type %s\n",
                              ci->type.c_str(ctx), ctx->nameOf(ci), user_type.c_str());
            }
            std::string iol_site = get_odelay_site(ctx->getBelName(io_bel).str(ctx));
            ci->attrs[ctx->id("BEL")] = iol_site + "/ODELAYE2";
            ci->attrs[ctx->id("X_IO_BEL")] = ctx->getBelName(io_bel).str(ctx);
            iodelay_to_io[ci->name] = io_bel;
        }
    }

    std::unordered_set<BelId> used_oserdes_bels;
    std::unordered_set<CellInfo *> unconstrained_oserdes;

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("ODDR")) {
            NetInfo *q = get_net_or_empty(ci, ctx->id("Q"));
            if (q == nullptr || q->users.empty())
                log_error("%s '%s' has disconnected Q output\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            BelId io_bel;
            CellInfo *ob = find_p_outbuf(q);
            if (ob != nullptr)
                io_bel = ctx->getBelByName(ctx->id(ob->attrs.at(ctx->id("BEL")).as_string()));
            else
                log_error("%s '%s' has illegal fanout on Q output\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            auto io_bel_str = ctx->getBelName(io_bel).str(ctx);
            std::string ol_site = get_ologic_site(io_bel_str);

            PortRef dest_port = *q->users.begin();
            auto is_tristate = dest_port.port == ctx->id("TRI");

            std::unordered_map<IdString, XFormRule> oddr_rules;
            if (boost::contains(io_bel_str, "IOB18"))
                oddr_rules[ctx->id("ODDR")].new_type = is_tristate ? ctx->id("OLOGICE2_TFF") : ctx->id("OLOGICE2_OUTFF");
            else
                oddr_rules[ctx->id("ODDR")].new_type = is_tristate ? ctx->id("OLOGICE3_TFF") : ctx->id("OLOGICE3_OUTFF");
            oddr_rules[ctx->id("ODDR")].port_xform[ctx->id("C")] = ctx->id("CK");
            NetInfo *s_net = get_net_or_empty(ci, ctx->id("S"));
            NetInfo *r_net = get_net_or_empty(ci, ctx->id("R"));
            if (s_net != nullptr && s_net->name == ctx->id("$PACKER_GND_NET"))
                disconnect_port(ctx, ci, ctx->id("S"));
            else 
                oddr_rules[ctx->id("ODDR")].port_xform[ctx->id("S")] = ctx->id("SR");
            if (r_net != nullptr && r_net->name == ctx->id("$PACKER_GND_NET"))
                disconnect_port(ctx, ci, ctx->id("R"));
            else 
                oddr_rules[ctx->id("ODDR")].port_xform[ctx->id("R")] = ctx->id("SR");
            xform_cell(oddr_rules, ci);

            ci->attrs[ctx->id("BEL")] = ol_site + (is_tristate ? "/TFF" : "/OUTFF");
        } else if (ci->type == id_OSERDESE2) {
            // according to ug953 they should be left unconnected or connected to ground
            // when not in slave mode, which is the same, since there are no wire routes to GND
            NetInfo *shiftin1 = get_net_or_empty(ci, ctx->id("SHIFTIN1"));
            if (shiftin1 != nullptr && shiftin1->name == ctx->id("$PACKER_GND_NET")) { disconnect_port(ctx, ci, ctx->id("SHIFTIN1")); shiftin1 = nullptr; }
            NetInfo *shiftin2 = get_net_or_empty(ci, ctx->id("SHIFTIN2"));
            if (shiftin2 != nullptr && shiftin2->name == ctx->id("$PACKER_GND_NET")) { disconnect_port(ctx, ci, ctx->id("SHIFTIN2")); shiftin2 = nullptr; }
            // If this is tied to GND it's just unused. This does not have a route to GND anyway.
            NetInfo *tbytein = get_net_or_empty(ci, ctx->id("TBYTEIN"));
            if (tbytein != nullptr && tbytein->name == ctx->id("$PACKER_GND_NET")) disconnect_port(ctx, ci, ctx->id("TBYTEIN"));
            
            std::string serdes_mode = str_or_default(ci->params, ctx->id("SERDES_MODE"), "MASTER");
            if (serdes_mode == "MASTER") {
                NetInfo *q = get_net_or_empty(ci, ctx->id("OQ"));
                NetInfo *ofb = get_net_or_empty(ci, ctx->id("OFB"));
                bool q_disconnected = q == nullptr || q->users.empty();
                bool ofb_disconnected = ofb == nullptr || ofb->users.empty();
                if (q_disconnected && ofb_disconnected) {
                    log_error("%s '%s' has disconnected OQ/OFB output ports\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                }
                BelId io_bel;
                CellInfo *ob = !q_disconnected ? find_p_outbuf(q) : find_p_outbuf(ofb);
                if (ob != nullptr) {
                    io_bel = ctx->getBelByName(ctx->id(ob->attrs.at(id_BEL).as_string()));
                    std::string ol_site = get_ologic_site(ctx->getBelName(io_bel).str(ctx));
                    auto bel_name = ol_site + "/OSERDESE2";
                    ci->attrs[id_BEL] = bel_name;
                    used_oserdes_bels.insert(ctx->getBelByName(ctx->id(bel_name)));

                    if (shiftin1 != nullptr || shiftin2 != nullptr) {
                        // slave location is one below master, so place the slave there
                        std::vector<std::string> parts;
                        boost::split(parts, bel_name, boost::is_any_of("Y"));
                        auto y_coord_slave = std::stoi(parts.back()) - 1;
                        auto slave_bel_name = parts.front() + "Y" + std::to_string(y_coord_slave) + "/OSERDESE2";

                        CellInfo *slave_cell;
                        if (shiftin1 != nullptr) slave_cell = shiftin1->driver.cell;
                        else slave_cell = shiftin2->driver.cell;
                        slave_cell->attrs[id_BEL] = slave_bel_name;
                        used_oserdes_bels.insert(ctx->getBelByName(ctx->id(slave_bel_name)));
                    }
                } else if (ofb->users.size() == 1 && ofb->users.at(0).cell->type == ctx->id("ISERDESE2")) {
                    unconstrained_oserdes.insert(ci);
                } else {
                    log_error("%s '%s' has illegal fanout on OQ or OFB output. This means that it is not connected to a corresponding OLOGIC BEL.\n", 
                              ci->type.c_str(ctx), ctx->nameOf(ci));
                }
                auto check_shiftin = [&] (NetInfo *shiftin) {
                    if (shiftin != nullptr) {
                        auto driver = shiftin->driver.cell;
                        auto driver_mode = str_or_default(driver->params, ctx->id("SERDES_MODE"), "MASTER");
                        if (driver_mode != "SLAVE")
                            log_error("%s '%s' can only be connected to the SHIFTOUT port of another slave OSERDESE2, but is connected to this master: '%s'\n",
                                      ci->type.c_str(ctx), ctx->nameOf(ci), driver->name.c_str(ctx));
                        if (!boost::starts_with(shiftin->driver.port.str(ctx), "SHIFTOUT")) {
                            log_error("%s '%s' can only be connected to the SHIFTOUT port of another OSERDESE2, but is connected to port: '%s'\n",
                                      ci->type.c_str(ctx), ctx->nameOf(ci), shiftin->driver.port.c_str(ctx));
                        }
                    }
                };
                check_shiftin(shiftin1);
                check_shiftin(shiftin2);
            } else {
                if (serdes_mode != "SLAVE")
                    log_error("%s '%s' has invalid SERDES_MODE '%s'. Valid modes are: MASTER, SLAVE.'\n",
                              ci->type.c_str(ctx), ctx->nameOf(ci), serdes_mode.c_str());

                NetInfo *shiftout1 = get_net_or_empty(ci, ctx->id("SHIFTOUT1"));
                NetInfo *shiftout2 = get_net_or_empty(ci, ctx->id("SHIFTOUT2"));
                auto check_shiftout = [&] (NetInfo *shiftout) {
                    if (shiftout == nullptr)
                        log_warning("%s '%s' is in slave mode, but has unconnected SHIFTOUT1 or SHIFTOUT2 port\n",
                                    ci->type.c_str(ctx), ctx->nameOf(ci));
                    else if (shiftout->users.size() == 1) {
                            auto user = shiftout->users.at(0);
                            auto user_mode = str_or_default(user.cell->params, ctx->id("SERDES_MODE"), "MASTER");
                            if (user_mode != "MASTER")
                                log_error("%s '%s' can only be connected to the SHIFTIN port of another master OSERDESE2, but is connected to this slave: '%s'\n",
                                          ci->type.c_str(ctx), ctx->nameOf(ci), user.cell->name.c_str(ctx));
                            if (!boost::starts_with(user.port.str(ctx), "SHIFTIN"))
                                log_error("%s '%s' can only be connected to the SHIFTIN port of another OSERDESE2, but is connected to port: '%s'\n",
                                          ci->type.c_str(ctx), ctx->nameOf(ci), user.port.c_str(ctx));
                    } else {
                        log_error("%s '%s' has multiple fanout on a SHIFTOUT port, which is not allowed.\n",
                                  ci->type.c_str(ctx), ctx->nameOf(ci));
                    }
                };
                check_shiftout(shiftout1);
                check_shiftout(shiftout2);
            }
        } else if (ci->type == ctx->id("IDDR")) {
            fold_inverter(ci, "C");

            BelId io_bel;
            NetInfo *d = get_net_or_empty(ci, ctx->id("D"));
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            if (   boost::contains(drv->type.str(ctx), "INBUF_EN")
                || boost::contains(drv->type.str(ctx), "INBUF_DCIEN"))
                io_bel = ctx->getBelByName(ctx->id(drv->attrs.at(ctx->id("BEL")).as_string()));
            else if (boost::contains(drv->type.str(ctx), "IDELAYE2") && d->driver.port == ctx->id("DATAOUT"))
                io_bel = iodelay_to_io.at(drv->name);
            else
                log_error("%s '%s' has D input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                            ctx->nameOf(ci), drv->type.c_str(ctx));

            std::string iol_site = get_ilogic_site(ctx->getBelName(io_bel).str(ctx));
            ci->attrs[ctx->id("BEL")] = iol_site + "/IFF";

            std::unordered_map<IdString, XFormRule> iddr_rules;
            iddr_rules[ctx->id("IDDR")].new_type = ctx->id("ILOGICE3_IFF");
            iddr_rules[ctx->id("IDDR")].port_multixform[ctx->id("C")] = { ctx->id("CK"), ctx->id("CKB") };
            iddr_rules[ctx->id("IDDR")].port_xform[ctx->id("S")] = ctx->id("SR");
            iddr_rules[ctx->id("IDDR")].port_xform[ctx->id("R")] = ctx->id("SR");

            NetInfo *s_net = get_net_or_empty(ci, ctx->id("S"));
            NetInfo *r_net = get_net_or_empty(ci, ctx->id("R"));
            if (s_net != nullptr && s_net->name == ctx->id("$PACKER_GND_NET"))
                disconnect_port(ctx, ci, ctx->id("S"));
            else 
                iddr_rules[ctx->id("IDDR")].port_xform[ctx->id("S")] = ctx->id("SR");
            if (r_net != nullptr && r_net->name == ctx->id("$PACKER_GND_NET"))
                disconnect_port(ctx, ci, ctx->id("R"));
            else 
                iddr_rules[ctx->id("IDDR")].port_xform[ctx->id("R")] = ctx->id("SR");
            xform_cell(iddr_rules, ci);

        } else if (ci->type == id_ISERDESE2) {
            fold_inverter(ci, "CLKB");
            fold_inverter(ci, "OCLKB");

            bool ofb_used = str_or_default(ci->params, id_OFB_USED, "FALSE") == "TRUE";
            std::string iobdelay = str_or_default(ci->params, id_IOBDELAY, "NONE");

            if (ofb_used) {
                BelId bel;
                NetInfo *d = get_net_or_empty(ci, id_OFB);
                if (d == nullptr || d->driver.cell == nullptr)
                    log_error("%s '%s' has disconnected OFB input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                CellInfo *drv = d->driver.cell;
                if (boost::contains(drv->type.str(ctx), "OSERDESE2") && d->driver.port == id_OFB) {
                    // We place this later, when we place the OSERDESE2, see below
                } else
                    log_error("%s '%s' has OFB input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                            ctx->nameOf(ci), drv->type.c_str(ctx));
            } else {
                BelId io_bel;
                if (iobdelay == "IFD") {
                    // disconnect constant D/SHIFTIN1/2 ports, if DDLY is used, as vivado does
                    disconnect_constant_port(ci, id_D);
                    disconnect_constant_port(ci, ctx->id("SHIFTIN1"));
                    disconnect_constant_port(ci, ctx->id("SHIFTIN2"));

                    NetInfo *d = get_net_or_empty(ci, id_DDLY);
                    if (d == nullptr || d->driver.cell == nullptr)
                        log_error("%s '%s' has disconnected DDLY input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                    CellInfo *drv = d->driver.cell;
                    if (boost::contains(drv->type.str(ctx), "IDELAYE2") && d->driver.port == id_DATAOUT)
                        io_bel = iodelay_to_io.at(drv->name);
                    else
                        log_error("%s '%s' has DDLY input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                                ctx->nameOf(ci), drv->type.c_str(ctx));
                } else if (iobdelay == "NONE") {
                    NetInfo *d = get_net_or_empty(ci, id_D);
                    if (d == nullptr || d->driver.cell == nullptr)
                        log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                    CellInfo *drv = d->driver.cell;
                    if (   boost::contains(drv->type.str(ctx), "INBUF_EN")
                        || boost::contains(drv->type.str(ctx), "INBUF_DCIEN"))
                        io_bel = ctx->getBelByName(ctx->id(drv->attrs.at(id_BEL).as_string()));
                    else
                        log_error("%s '%s' has D input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                                ctx->nameOf(ci), drv->type.c_str(ctx));
                } else {
                    log_error("%s '%s' has unsupported IOBDELAY value '%s'\n", ci->type.c_str(ctx), ctx->nameOf(ci),
                            iobdelay.c_str());
                }

                std::string iol_site = get_ilogic_site(ctx->getBelName(io_bel).str(ctx));
                ci->attrs[id_BEL] = iol_site + "/ISERDESE2";
            }
        }

        // disconnect some ports that vivado also disconnects when wired to constant
        disconnect_constant_port(ci, id_CLKDIVP);
    }

    // place OSERDESE2 which are not connected to an output, but to another ISERDESE2 via OFB
    std::queue<BelId> available_oserdes_bels;
    for (auto bel : ctx->getBels()) {
        if (ctx->getBelType(bel) != id_OSERDESE2_OSERDESE2)
            continue;
        if (used_oserdes_bels.count(bel))
            continue;
        available_oserdes_bels.push(bel);
    }

    int avail_count = int(available_oserdes_bels.size());
    int unconstr_oserdes = int(unconstrained_oserdes.size());
    for (auto ci : unconstrained_oserdes) {
        if (available_oserdes_bels.empty()) {
            log_error("IO placer ran out of available OSERDESE2 (%d available, %d unconstrained)\n", avail_count,
                unconstr_oserdes);
        }
        auto oserdes_bel_name = std::string(ctx->nameOfBel(available_oserdes_bels.front()));
        ci->attrs[id_BEL] = oserdes_bel_name;

        NetInfo *d = get_net_or_empty(ci, id_OFB);
        NPNR_ASSERT_MSG(d != nullptr, "Only OSERDESE2 with connected OFB should be unconstrained at this point");
        NPNR_ASSERT_MSG(d->users.size() == 1, "OSERDESE2 OFB can only be connected to one cell");
        auto iserdes = d->users.at(0).cell;
        std::string iserdes_bel_name = oserdes_bel_name;
        boost::replace_all(iserdes_bel_name, "OLOGIC", "ILOGIC");
        boost::replace_all(iserdes_bel_name, "OSERDES", "ISERDES");
        NPNR_ASSERT_MSG(iserdes->attrs.count(id_BEL)  == 0, "ISERDESE2 which is connected to OFB of OSERDESE2 already placed");
        iserdes->attrs[id_BEL] = iserdes_bel_name;
        available_oserdes_bels.pop();
    }

    flush_cells();
    generic_xform(iologic_rules, false);
    flush_cells();
}

void XC7Packer::pack_idelayctrl()
{
    auto get_iodelay_group_name = [&](CellInfo * ci) {
        std::string group_name = "";
        auto iodelay_group = ci->attrs.find(ctx->id("IODELAY_GROUP"));
        if (iodelay_group != ci->attrs.end()) {
            group_name = (*iodelay_group).second.as_string();
        }
        return group_name;
    };

    std::unordered_map<std::string, CellInfo *> idelayctrl_map;
    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;
        if (ci->type == ctx->id("IDELAYCTRL")) {
            std::string group_name = get_iodelay_group_name(ci);
            if (0 < idelayctrl_map.count(group_name)) {
                if (group_name == "")
                    log_error("Found more than one IDELAYCTRL cell for the default IODELAY_GROUP!\n");
                else
                    log_error("Found more than one IDELAYCTRL cell for the IODELAY_GROUP with name %s!\n", group_name.c_str());
            }
            idelayctrl_map[group_name] = ci;
        }
    }
    if (idelayctrl_map.empty())
        return;
    for (auto group : idelayctrl_map)
    {
        auto group_name = group.first;
        auto idelayctrl = group.second;
        std::set<std::string> ioctrl_sites;
        for (auto cell : sorted(ctx->cells)) {
            CellInfo *ci = cell.second;
            if (ci->type == ctx->id("IDELAYE2_IDELAYE2") || ci->type == ctx->id("ODELAYE2_ODELAYE2")) {
                auto grp_name = get_iodelay_group_name(ci);
                if (!ci->attrs.count(ctx->id("BEL")) || grp_name != group_name)
                    continue;
                ioctrl_sites.insert(get_ioctrl_site(ci->attrs.at(ctx->id("X_IO_BEL")).as_string()));
            }
        }
        if (ioctrl_sites.empty())
            log_error("Found IDELAYCTRL but no I/ODELAYs in group %s\n", group_name.c_str());
        NetInfo *rdy = get_net_or_empty(idelayctrl, ctx->id("RDY"));
        disconnect_port(ctx, idelayctrl, ctx->id("RDY"));
        std::vector<NetInfo *> dup_rdys;
        int i = 0;
        for (auto site : ioctrl_sites) {
            auto dup_idc = create_cell(ctx, ctx->id("IDELAYCTRL"),
                                    int_name(idelayctrl->name, "CTRL_DUP_" + std::to_string(i), false));
            connect_port(ctx, get_net_or_empty(idelayctrl, ctx->id("REFCLK")), dup_idc.get(), ctx->id("REFCLK"));
            connect_port(ctx, get_net_or_empty(idelayctrl, ctx->id("RST")), dup_idc.get(), ctx->id("RST"));
            if (rdy != nullptr) {
                NetInfo *dup_rdy =
                        (ioctrl_sites.size() == 1)
                                ? rdy
                                : create_internal_net(idelayctrl->name, "CTRL_DUP_" + std::to_string(i) + "_RDY", false);
                connect_port(ctx, dup_rdy, dup_idc.get(), ctx->id("RDY"));
                dup_rdys.push_back(dup_rdy);
            }
            dup_idc->attrs[ctx->id("BEL")] = site + "/IDELAYCTRL";
            new_cells.push_back(std::move(dup_idc));
            ++i;
        }
        disconnect_port(ctx, idelayctrl, ctx->id("REFCLK"));
        disconnect_port(ctx, idelayctrl, ctx->id("RST"));

        if (rdy != nullptr) {
            // AND together all the RDY signals
            std::vector<NetInfo *> int_anded_rdy;
            int_anded_rdy.push_back(dup_rdys.front());
            for (size_t j = 1; j < dup_rdys.size(); j++) {
                NetInfo *anded_net =
                        (j == (dup_rdys.size() - 1))
                                ? rdy
                                : create_internal_net(idelayctrl->name, "ANDED_RDY_" + std::to_string(j), false);
                auto lut = create_lut(ctx, idelayctrl->name.str(ctx) + "/RDY_AND_LUT_" + std::to_string(j),
                                    {int_anded_rdy.at(j - 1), dup_rdys.at(j)}, anded_net, Property(8));
                int_anded_rdy.push_back(anded_net);
                new_cells.push_back(std::move(lut));
            }
        }

        packed_cells.insert(idelayctrl->name);
    }

    flush_cells();

    ioctrl_rules[ctx->id("IDELAYCTRL")].new_type = ctx->id("IDELAYCTRL_IDELAYCTRL");

    generic_xform(ioctrl_rules);
}

void XC7Packer::pack_cfg()
{
    log_info("Packing cfg...\n");
    std::unordered_map<IdString, XFormRule> cfg_rules;
    cfg_rules[id_BSCANE2].new_type      = id_BSCAN;
    cfg_rules[id_DCIRESET].new_type     = id_DCIRESET_DCIRESET;
    cfg_rules[id_DNA_PORT].new_type     = id_DNA_PORT_DNA_PORT;
    cfg_rules[id_EFUSE_USR].new_type    = id_EFUSE_USR_EFUSE_USR;
    cfg_rules[id_ICAPE2].new_type       = id_ICAP_ICAP;
    cfg_rules[id_FRAME_ECCE2].new_type  = id_FRAME_ECC_FRAME_ECC;
    cfg_rules[id_STARTUPE2].new_type    = id_STARTUP_STARTUP;
    cfg_rules[id_USR_ACCESSE2].new_type = id_USR_ACCESS_USR_ACCESS;
    generic_xform(cfg_rules);

    for (auto cell : sorted(ctx->cells)) {
        CellInfo *ci = cell.second;

        if (ci->type == id_BSCAN) {
            int chain = int_or_default(ci->params, id_JTAG_CHAIN, 1);
            if (chain < 1 || 4 < chain)
                log_error("Instance '%s': Invalid JTAG_CHAIN number of '%d\n'. Allowed values are: 1-4.", ci->name.c_str(ctx), chain);
            auto bel = "BSCAN_X0Y" + std::to_string(chain - 1) + "/BSCAN";
            ci->attrs[id_BEL] = bel;
            log_info("    Constraining '%s' to site '%s'\n", ci->name.c_str(ctx), bel.c_str());
        } else if (ci->type == id_STARTUP_STARTUP || ci->type == id_DCIRESET_DCIRESET ||
                   ci->type == id_DNA_PORT_DNA_PORT || ci->type == id_EFUSE_USR_EFUSE_USR ||
                   ci->type == id_ICAP_ICAP || ci->type == id_FRAME_ECC_FRAME_ECC ||
                   ci->type == id_USR_ACCESS_USR_ACCESS) {
            // These configuration primitives live in a single dedicated site each. The
            // placer has no way to discover that site on its own, so without an explicit
            // preplacement it aborts with "Unable to find legal placement for cell". BSCAN
            // above is the exception only because JTAG_CHAIN already names its site.
            // Affects any design that instantiates e.g. STARTUPE2 to source a clock from
            // CFGMCLK, or ICAPE2 / DNA_PORT for configuration access.
            preplace_unique(ci);
        }
    }
}

NEXTPNR_NAMESPACE_END
