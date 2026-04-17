/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2020  David Shah <dave@ds0.me>
 *  Copyright (C) 2023  Hans Baier <hansfbaier@gmail.com>
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
#include "pack.h"

NEXTPNR_NAMESPACE_BEGIN

std::string XC7Packer::get_gt_site(const std::string &io_bel)
{
    auto pad_site = io_bel.substr(0, io_bel.find('/'));

    int tileid, siteid;
    std::tie(tileid, siteid) = ctx->site_by_name.at(pad_site);
    auto tile = &ctx->chip_info->tile_insts[tileid];
    for (int s = 0; s < tile->num_sites; s++) {
        auto site = &tile->site_insts[s];
        auto site_name = std::string(site->name.get());
        if (boost::starts_with(site_name, "GTP") || boost::starts_with(site_name, "GTX"))
            return site_name;
    }

    auto msg = std::string("failed to find GTP/GTX site for ") + io_bel;
    NPNR_ASSERT_FALSE(msg.c_str());
}

void XC7Packer::constrain_ibufds_gt_site(CellInfo *buf_cell, const std::string &io_bel)
{
    auto pad_site = io_bel.substr(0, io_bel.find('/'));

    int tileid, siteid;
    std::tie(tileid, siteid) = ctx->site_by_name.at(pad_site);
    auto tile = &ctx->chip_info->tile_insts[tileid];

    int32_t min_buf_y = 0x7FFFFFFF;
    int32_t max_buf_y = 0;
    int32_t min_pad_y = 0x7FFFFFFF;
    int32_t max_pad_y = 0;
    int32_t pad_y = -1;

    for (int s = 0; s < tile->num_sites; s++) {
        auto site = &tile->site_insts[s];
        auto site_name = std::string(site->name.get());
        if (boost::starts_with(site_name, "IPAD_")) {
            auto sy = site->site_y;
            if (sy < min_pad_y) min_pad_y = sy;
            if (max_pad_y < sy) max_pad_y = sy;
            if (site_name == pad_site) pad_y = sy;
        }
        if (boost::starts_with(site_name, "IBUFDS_GTE2_")) {
            auto sy = site->site_y;
            if (sy < min_buf_y) min_buf_y = sy;
            if (max_buf_y < sy) max_buf_y = sy;
        }
    }

    if (pad_y < 0) {
        log_error("failed to find IBUFDS_GTE2 site for %s\n", io_bel.c_str());
    }

    NPNR_ASSERT(min_pad_y < max_pad_y);
    NPNR_ASSERT(min_buf_y < max_buf_y);

    auto rel_buf_y = (pad_y - min_pad_y) >> 1;
    auto buf_y = min_buf_y + rel_buf_y;

    int32_t num_pads = max_pad_y - min_pad_y + 1;
    NPNR_ASSERT_MSG(num_pads == 4, "A GTP_COMMON/GTX_COMMON tile only should have four input pads");
    auto buf_bel = std::string("IBUFDS_GTE2_X0Y" + std::to_string(buf_y)) + "/IBUFDS_GTE2";

    if (buf_cell->attrs.find(id_BEL) != buf_cell->attrs.end()) {
        auto existing_buf_bel = buf_cell->attrs[id_BEL].as_string();
        if (existing_buf_bel != buf_bel)
            log_error("Location of IBUFDS_GTE2 %s on %s conflicts with previous placement on %s\n",
                buf_cell->name.c_str(ctx), buf_bel.c_str(), existing_buf_bel.c_str());
        return;
    }

    buf_cell->attrs[id_BEL] = buf_bel;
    buf_cell->attrs[ctx->id("_REL_BUF_Y")] = Property(rel_buf_y);
    log_info("    Constraining '%s' to site '%s'\n", buf_cell->name.c_str(ctx), buf_bel.c_str());
    log_info("    Tile '%s'\n", tile->name.get());
}

void XC7Packer::constrain_gt(CellInfo *pad_cell, CellInfo *gt_cell)
{
    if (pad_cell->attrs.find(id_BEL) != pad_cell->attrs.end()) {
        auto pad_bel = pad_cell->attrs[id_BEL].as_string();
        auto gt_site = get_gt_site(pad_bel);
        auto gt_bel = gt_site + "/" + gt_cell->type.str(ctx);
        if (gt_cell->attrs.find(id_BEL) != gt_cell->attrs.end()) {
            auto existing_gt_bel = gt_cell->attrs[id_BEL];
            if (existing_gt_bel != gt_bel)
                log_error("Location of pad %s on %s conflicts with previous placement of %s on %s\n",
                    pad_cell->name.c_str(ctx), pad_bel.c_str(), gt_cell->name.c_str(ctx), gt_site.c_str());
            return;
        }
        gt_cell->attrs[id_BEL] = gt_bel;
        log_info("    Constraining '%s' to site '%s'\n", gt_cell->name.c_str(ctx), gt_site.c_str());
        std::string tile = get_tilename_by_sitename(ctx, gt_site);
        log_info("    Tile '%s'\n", tile.c_str());

    } else log_error("Pad cell %s has not been placed\n", pad_cell->name.c_str(ctx));
}

void XC7Packer::pack_gt()
{
    log_info("Packing Gigabit Transceivers..\n");

    std::vector<CellInfo *> all_plls;

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();

        if (ci->type == id_GTPE2_COMMON || ci->type == id_GTXE2_COMMON) {
            all_plls.push_back(ci);
            const IdString refclk0_used_attr = ctx->id("_GTREFCLK0_USED"),
                           refclk1_used_attr = ctx->id("_GTREFCLK1_USED");
            bool refclk0_used = false, refclk1_used = false;
            bool is_gtp = ci->type == id_GTPE2_COMMON;
            std::string gt_type = is_gtp ? "GTP" : "GTX";

            fold_inverter(ci, "DRPCLK");
            if (is_gtp) {
                fold_inverter(ci, "PLL0LOCKDETCLK");
                fold_inverter(ci, "PLL1LOCKDETCLK");
                // the other inverter BELs are not yet supported by prjxray
            } else { // GTX
                fold_inverter(ci, "QPLLLOCKDETCLK");

                /*  the other inverter BELs are not yet supported by prjxray
                fold_inverter(ci, "GTGREFCLK");
                fold_inverter(ci, "QPLLCLKSPARE0");
                fold_inverter(ci, "QPLLCLKSPARE1");
                fold_inverter(ci, "QPLLPMASCANCLK0");
                fold_inverter(ci, "QPLLPMASCANCLK1");
                */
            }

            for (auto &port : ci->ports) {
                auto port_name = port.first.str(ctx);
                auto port_net  = port.second.net;
                bool used = port_net != nullptr &&
                            port_net->name != ctx->id("$PACKER_VCC_NET") &&
                            port_net->name != ctx->id("$PACKER_GND_NET");
                bool internal_refclk = false;

                if (port_name == "DRPCLK") {
                    ci->setParam(ctx->id("_DRPCLK_USED"), Property(used));
                } else if (boost::starts_with(port_name, "GTREFCLK")) {
                    CellInfo *driver = port_net->driver.cell;
                    if (driver == nullptr) log_error("Port %s connected to net %s has no driver!", port_name.c_str(), port_net->name.c_str(ctx));
                    if (driver->type != id_IBUFDS_GTE2) {
                        log_warning("Driver %s of net %s connected to a %sE2_COMMON PLL is not an IBUFDS_GTE2 block, but %s\n",
                            driver->name.c_str(ctx), port_net->name.c_str(ctx), gt_type.c_str(), driver->type.c_str(ctx));

                        // Do we really need this here?
                        // Would that work in other cases too?
                        if (driver->type != id_BUFGCTRL)
                            log_error("GTP_COMMON GTREFCLK connected to unsupported cell type %s\n", driver->type.c_str(ctx));

                        // vivado internally always connects to GTGREFCLK0, even if GTGREFCLK1 is connected in the verilog
                        auto gtg_port = is_gtp ? id_GTGREFCLK0 : id_GTGREFCLK;
                        log_warning("Internal REFCLK is used for instance '%s', which is not recommended. Connecting refclock to port %s instead.\n",
                            ci->name.c_str(ctx), gtg_port.c_str(ctx));
                        rename_port(ctx, ci, port.first, gtg_port);
                        internal_refclk = true;
                        ci->setParam(ctx->id("_GTGREFCLK_USED"), Property(1, 1));
                    } else { // driver is IBUFDS_GTE2
                        log_info("Driver %s of net %s is a IBUFDS_GTE2 block\n",
                            driver->name.c_str(ctx), port_net->name.c_str(ctx));
                        if (used) {
                            auto rel_buf_y = driver->attrs[ctx->id("_REL_BUF_Y")].as_int64();
                            // replicate Vivado's behavior here:
                            // since GTREFCLK0 is hardwired to the lower IBUFDS_GTE2
                            // and GTREFCLK1 is hardwired to the upper buffer
                            // the used flags activate those inputs
                            // the don't need to be routed, so we just disconnect the port here
                            disconnect_port(ctx, ci, port.first);
                            if (rel_buf_y == 1) {
                                refclk1_used = true;
                                ci->setParam(refclk1_used_attr, Property(1, 1));
                            } else {
                                refclk0_used = true;
                                ci->setParam(refclk0_used_attr, Property(1, 1));
                            }
                            continue;
                        }
                    }

                    if (!internal_refclk) {
                        // if we could not determine refclk input by IBUFDS_GTE2 location
                        // then we just use whatever port is connected
                        if (boost::ends_with(port_name, "0")) {
                            refclk0_used = used;
                            ci->setParam(refclk0_used_attr, Property(used));
                        } else {
                            refclk1_used = used;
                            ci->setParam(refclk1_used_attr, Property(used));
                        }
                    }
                }
            }
            ci->setParam(ctx->id("_BOTH_GTREFCLK_USED"), Property(refclk0_used && refclk1_used));
        } else if (ci->type == id_GTPE2_CHANNEL) {
            fold_inverter(ci, "CLKRSVD0");
            fold_inverter(ci, "CLKRSVD1");
            fold_inverter(ci, "CPLLLOCKDETCLK");
            fold_inverter(ci, "DMONITORCLK");
            fold_inverter(ci, "DRPCLK");
            fold_inverter(ci, "GTGREFCLK");
            fold_inverter(ci, "PMASCANCLK0");
            fold_inverter(ci, "PMASCANCLK1");
            fold_inverter(ci, "PMASCANCLK2");
            fold_inverter(ci, "PMASCANCLK3");
            fold_inverter(ci, "QPLLLOCKDETCLK");
            fold_inverter(ci, "RXUSRCLK");
            fold_inverter(ci, "RXUSRCLK2");
            fold_inverter(ci, "SCANCLK");
            fold_inverter(ci, "SIGVALIDCLK");
            fold_inverter(ci, "TSTCLK0");
            fold_inverter(ci, "TSTCLK1");
            fold_inverter(ci, "TXPHDLYTSTCLK");
            fold_inverter(ci, "TXUSRCLK");
            fold_inverter(ci, "TXUSRCLK2");

            {
                // Collect bracket-named ports before the loop to avoid iterator
                // invalidation when rename_port erases+inserts into ci->ports.
                std::vector<std::pair<IdString,IdString>> to_rename;
                for (auto &port : ci->ports) {
                    auto port_name = port.first.str(ctx);
                    auto net = get_net_or_empty(ci, port.first);

                    // If one of the clock ports is tied, then Vivado just disconnects them
                    if (net != nullptr && (boost::starts_with(port_name, "PLL") && boost::ends_with(port_name, "CLK"))
                                    && !boost::contains(port_name, "CLKMONITOR")) {
                        if (net->name == ctx->id("$PACKER_GND_NET") || net->name == ctx->id("$PACKER_VCC_NET")) {
                            disconnect_port(ctx, ci, port.first);
                            continue;
                        }
                        auto driver = net->driver.cell;
                        if (driver->type != id_GTPE2_COMMON)
                            log_error("The clock port '%s' of the GTPE2_CHANNEL instance %s can only be driven "
                                        "by the clock ouputs of a GTPE2_COMMON instance, but not %s\n",
                                        port_name.c_str(), ci->name.c_str(ctx), driver->type.c_str(ctx));
                        auto drv_port = net->driver.port.str(ctx);
                        auto port_prefix = port_name.substr(0, 4);
                        auto port_suffix = port_name.substr(4);
                        if (!boost::starts_with(drv_port, port_prefix) || !boost::ends_with(drv_port, port_suffix))
                            log_error("The port %s of a GTPE2_CHANNEL instance can only be connected to the port %sOUT%s "
                                        "of a GTPE2_COMMON instance, but not to %s.\n", port_name.c_str(), port_prefix.c_str(), port_suffix.c_str(),
                                        drv_port.c_str());
                        // These ports are hardwired. Disconnect
                        disconnect_port(ctx, ci, port.first);
                    }

                    if (boost::contains(port_name, "[") && boost::contains(port_name, "]")) {
                        auto new_port_name = std::string(port_name);
                        boost::replace_all(new_port_name, "[", "");
                        boost::replace_all(new_port_name, "]", "");
                        to_rename.emplace_back(ctx->id(port_name), ctx->id(new_port_name));
                    }
                }
                for (auto &pr : to_rename)
                    rename_port(ctx, ci, pr.first, pr.second);
            }
        } else if (ci->type == id_GTXE2_CHANNEL) {
            fold_inverter(ci, "CLKRSVD0");
            fold_inverter(ci, "CLKRSVD1");
            fold_inverter(ci, "CPLLLOCKDETCLK");
            fold_inverter(ci, "DMONITORCLK");
            fold_inverter(ci, "DRPCLK");
            fold_inverter(ci, "GTGREFCLK");
            fold_inverter(ci, "PMASCANCLK0");
            fold_inverter(ci, "PMASCANCLK1");
            fold_inverter(ci, "PMASCANCLK2");
            fold_inverter(ci, "PMASCANCLK3");
            fold_inverter(ci, "QPLLLOCKDETCLK");
            fold_inverter(ci, "RXUSRCLK");
            fold_inverter(ci, "RXUSRCLK2");
            fold_inverter(ci, "SCANCLK");
            fold_inverter(ci, "SIGVALIDCLK");
            fold_inverter(ci, "TSTCLK0");
            fold_inverter(ci, "TSTCLK1");
            fold_inverter(ci, "TXPHDLYTSTCLK");
            fold_inverter(ci, "TXUSRCLK");
            fold_inverter(ci, "TXUSRCLK2");

            {
                std::vector<std::pair<IdString,IdString>> to_rename;
                for (auto &port : ci->ports) {
                    auto port_name = port.first.str(ctx);
                    auto net = get_net_or_empty(ci, port.first);

                    // If one of the clock ports is tied, then Vivado just disconnects them
                    if (net != nullptr && ((boost::starts_with(port_name, "PLL") && boost::ends_with(port_name, "CLK")) ||
                                           boost::contains(port_name, "REFCLK") ||
                                           boost::starts_with(port_name, "QPLL")) &&
                                           !boost::contains(port_name, "CLKMONITOR") &&
                                           !boost::contains(port_name, "CLKLOST")) {
                        if (net->name == ctx->id("$PACKER_GND_NET") || net->name == ctx->id("$PACKER_VCC_NET")) {
                            disconnect_port(ctx, ci, port.first);
                            continue;
                        }
                        auto driver = net->driver.cell;
                        // GTX CPLL mode is possible: GTXE2_CHANNEL.GTREFCLK* driven directly by IBUFDS_GTE2
                        // (no GTXE2_COMMON required in the design)
                        if (driver->type != id_GTXE2_COMMON && driver->type !=id_IBUFDS_GTE2)
                            log_error("The clock port '%s' of the GTXE2_CHANNEL instance %s can only be driven "
                                        "by the clock ouputs of a GTXE2_COMMON or IBUFDS_GTE2 instance, but not %s\n",
                                        port_name.c_str(), ci->name.c_str(ctx), driver->type.c_str(ctx));
                        auto drv_port = net->driver.port.str(ctx);
                        auto port_prefix = port_name.substr(0, 4);
                        auto port_suffix = port_name.substr(4);
                        if (drv_port != "O" && (!boost::starts_with(drv_port, port_prefix) || !boost::ends_with(drv_port, port_suffix)))
                            log_warning("The port %s of a GTXE2_CHANNEL instance can only be connected to the port %sOUT%s "
                                        "of a GTXE2_COMMON instance, or IBUFDS_GTE2, but not to %s.\n",
                                        port_name.c_str(), port_prefix.c_str(), port_suffix.c_str(), drv_port.c_str());
                        // GTX CPLL mode: GTREFCLk0/1 treatment similar to GTXE2_COMMON
                        bool used = net->name != ctx->id("$PACKER_VCC_NET") &&
                                    net->name != ctx->id("$PACKER_GND_NET");
                        if (used && port_name == "GTREFCLK0")
                            ci->setParam(ctx->id("_GTREFCLK0_USED"), Property(1, 1));
                        if (used && port_name == "GTREFCLK1")
                            ci->setParam(ctx->id("_GTREFCLK1_USED"), Property(1, 1));
                        // These ports are hardwired. Disconnect
                        disconnect_port(ctx, ci, port.first);
                    }

                    if (boost::contains(port_name, "[") && boost::contains(port_name, "]")) {
                        auto new_port_name = std::string(port_name);
                        boost::replace_all(new_port_name, "[", "");
                        boost::replace_all(new_port_name, "]", "");
                        to_rename.emplace_back(ctx->id(port_name), ctx->id(new_port_name));
                    }
                }
                for (auto &pr : to_rename)
                    rename_port(ctx, ci, pr.first, pr.second);
            }
        }    
    }
}

NEXTPNR_NAMESPACE_END
