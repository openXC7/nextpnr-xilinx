/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2018  Clifford Wolf <clifford@symbioticeda.com>
 *  Copyright (C) 2018-19  David Shah <david@symbioticeda.com>
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
#include <boost/range/adaptor/reversed.hpp>
#include <cmath>
#include <cstring>
#include <fstream>
#include <queue>
#include "log.h"
#include "nextpnr.h"
#include "placer1.h"
#include "placer_heap.h"
#include "router1.h"
#include "router2.h"
#include "timing.h"
#include "util.h"
#include "design_utils.h"

NEXTPNR_NAMESPACE_BEGIN

static std::pair<std::string, std::string> split_identifier_name(const std::string &name)
{
    size_t first_slash = name.find('/');
    NPNR_ASSERT(first_slash != std::string::npos);
    return std::make_pair(name.substr(0, first_slash), name.substr(first_slash + 1));
};

static std::pair<std::string, std::string> split_identifier_name_dot(const std::string &name)
{
    size_t first_dot = name.find('.');
    NPNR_ASSERT(first_dot != std::string::npos);
    return std::make_pair(name.substr(0, first_dot), name.substr(first_dot + 1));
};

// -----------------------------------------------------------------------

void IdString::initialize_arch(const BaseCtx *ctx)
{
#define X(t) initialize_add(ctx, #t, ID_##t);

#include "constids.inc"

#undef X
}

// -----------------------------------------------------------------------

static const ChipInfoPOD *get_chip_info(const RelPtr<ChipInfoPOD> *ptr) { return ptr->get(); }

Arch::Arch(ArchArgs args) : args(args)
{
    try {
        blob_file.open(args.chipdb);
        if (args.chipdb.empty() || !blob_file.is_open())
            log_error("Unable to read chipdb %s\n", args.chipdb.c_str());
        const char *blob = reinterpret_cast<const char *>(blob_file.data());
        chip_info = get_chip_info(reinterpret_cast<const RelPtr<ChipInfoPOD> *>(blob));
    } catch (...) {
        log_error("Unable to read chipdb %s\n", args.chipdb.c_str());
    }

    for (int i = 0; i < chip_info->extra_constids->bba_id_count; i++) {
        // log_info("%s %d\n", chip_info->extra_constids->bba_ids[i].get(), int(idstring_idx_to_str->size()));
        IdString::initialize_add(this, chip_info->extra_constids->bba_ids[i].get(),
                                 i + chip_info->extra_constids->known_id_count);
    }

    if (std::string(chip_info->name.get()).find("xc7") == 0)
        xc7 = true;
    else
        xc7 = false;

    tileStatus.resize(chip_info->num_tiles);
    for (int i = 0; i < chip_info->num_tiles; i++) {
        tileStatus[i].boundcells.resize(chip_info->tile_types[chip_info->tile_insts[i].type].num_bels);
        tileStatus[i].sitevariant.resize(chip_info->tile_insts[i].num_sites);
    }

    if (xc7)
        setup_pip_blacklist();
}

// -----------------------------------------------------------------------

std::string Arch::getChipName() const { return chip_info->name.get(); }

// -----------------------------------------------------------------------

IdString Arch::archArgsToId(ArchArgs args) const { return IdString(); }

// -----------------------------------------------------------------------

void Arch::setup_byname() const
{
    if (tile_by_name.empty()) {
        for (int i = 0; i < chip_info->num_tiles; i++) {
            tile_by_name[chip_info->tile_insts[i].name.get()] = i;
        }
    }

    if (site_by_name.empty()) {
        for (int i = 0; i < chip_info->num_tiles; i++) {
            auto &tile = chip_info->tile_insts[i];
            for (int j = 0; j < tile.num_sites; j++) {
                auto name = tile.site_insts[j].name.get();
                site_by_name[name] = std::make_pair(i, j);
            }
        }
    }
}

BelId Arch::getBelByName(IdString name) const
{
    BelId ret;

    setup_byname();

    // Accept BOTH spellings: "SITE/BEL" and the three-part "SITE/SITETYPE/BEL"
    // that a physical netlist uses for IO.
    //
    // ORDER MATTERS.  This chipdb names an IO bel WITH its site-type prefix --
    // "IOB18/PAD", "IOB18M/DCITERMDISABLE_GND" -- so for "IOB_X1Y276/IOB18/PAD"
    // the bel to look up is "IOB18/PAD", not "PAD".  Dropping the middle
    // component unconditionally therefore failed to resolve every IO pad, and
    // the empty BelId it returned was fed to getBelSite(), which indexes the
    // chipdb out of range and hands back a GARBAGE site name -- that name was
    // then concatenated into the buffer's own BEL attribute and only surfaced
    // in the placer as an unprintable "No Bel named '\325<\265/IOB33/OUTBUF'".
    // Try the full remainder first and only then the stripped form, so both
    // spellings resolve and neither shadows the other.
    auto split = split_identifier_name(name.str(this));
    std::vector<std::string> candidates{split.second};
    {
        size_t slash = split.second.find('/');
        if (slash != std::string::npos)
            candidates.push_back(split.second.substr(slash + 1));
    }
    for (const auto &cand : candidates) {
        IdString belname = id(cand);
        if (site_by_name.count(split.first)) {
            int tile, site;
            std::tie(tile, site) = site_by_name.at(split.first);
            auto &tile_info = chip_info->tile_types[chip_info->tile_insts[tile].type];
            for (int i = 0; i < tile_info.num_bels; i++) {
                if (tile_info.bel_data[i].site == site && tile_info.bel_data[i].name == belname.index) {
                    ret.tile = tile;
                    ret.index = i;
                    break;
                }
            }
        } else if (tile_by_name.count(split.first)) {
            int tile = tile_by_name.at(split.first);
            auto &tile_info = chip_info->tile_types[chip_info->tile_insts[tile].type];
            for (int i = 0; i < tile_info.num_bels; i++) {
                if (tile_info.bel_data[i].name == belname.index) {
                    ret.tile = tile;
                    ret.index = i;
                    break;
                }
            }
        }
        if (ret != BelId())
            break;
    }
    // An unknown site/tile returns an EMPTY BelId, which every caller already
    // handles -- placer1 answers with "No Bel named 'X' ... on cell 'Y'".
    // tile_by_name.at() instead threw std::out_of_range out of the placer,
    // reporting neither the name nor the cell, and the name it was given was
    // unprintable anyway.

    return ret;
}

BelRange Arch::getBelsByTile(int x, int y) const
{
    BelRange br;

    br.b.cursor_tile = y * chip_info->width + x;
    br.e.cursor_tile = y * chip_info->width + x;
    br.b.cursor_index = 0;
    br.e.cursor_index = chip_info->tile_types[chip_info->tile_insts[br.b.cursor_tile].type].num_bels;
    br.b.chip = chip_info;
    br.e.chip = chip_info;
    if (br.e.cursor_index == -1)
        ++br.e.cursor_index;
    else
        ++br.e;
    return br;
}

WireId Arch::getBelPinWire(BelId bel, IdString pin) const
{
    WireId ret;

    NPNR_ASSERT(bel != BelId());

    const bool debug_this = false;

    int num_bel_wires = locInfo(bel).bel_data[bel.index].num_bel_wires;
    const BelWirePOD *bel_wires = locInfo(bel).bel_data[bel.index].bel_wires.get();

    if (debug_this) log_info("looking for pin %s in bel %s\n", pin.c_str(this), getBelName(bel).c_str(this));
    for (int i = 0; i < num_bel_wires; i++) {
        const char *wire_name;
        if (debug_this) {
            WireId tmp;
            tmp.tile = bel.tile;
            tmp.index = bel_wires[i].wire_index;
            wire_name = getWireName(tmp).c_str(this);
            log_info("check wire %s\n", wire_name);
        }

        if (bel_wires[i].port == pin.index) {
            if (debug_this) log_info("got wire %s\n", wire_name);
            return canonicalWireId(chip_info, bel.tile, bel_wires[i].wire_index);
        }
    }

    return ret;
}

PortType Arch::getBelPinType(BelId bel, IdString pin) const
{
    NPNR_ASSERT(bel != BelId());

    int num_bel_wires = locInfo(bel).bel_data[bel.index].num_bel_wires;
    const BelWirePOD *bel_wires = locInfo(bel).bel_data[bel.index].bel_wires.get();

    for (int i = 0; i < num_bel_wires; i++)
        if (bel_wires[i].port == pin.index)
            return PortType(bel_wires[i].type);

    return PORT_INOUT;
}

// -----------------------------------------------------------------------

WireId Arch::getWireByName(IdString name) const
{
    if (wire_by_name_cache.count(name))
        return wire_by_name_cache.at(name);
    WireId ret;
    setup_byname();

    const std::string &s = name.str(this);
    if (s.substr(0, 9) == "SITEWIRE/") {
        auto sp2 = split_identifier_name(s.substr(9));
        int tile, site;
        std::tie(tile, site) = site_by_name.at(sp2.first);
        auto &tile_info = chip_info->tile_types[chip_info->tile_insts[tile].type];
        IdString wirename = id(sp2.second);
        for (int i = 0; i < tile_info.num_wires; i++) {
            if (tile_info.wire_data[i].site == site && tile_info.wire_data[i].name == wirename.index) {
                ret.tile = tile;
                ret.index = i;
                break;
            }
        }
    } else {
        auto sp = split_identifier_name(s);
        int tile = tile_by_name.at(sp.first);
        auto &tile_info = chip_info->tile_types[chip_info->tile_insts[tile].type];
        IdString wirename = id(sp.second);
        for (int i = 0; i < tile_info.num_wires; i++) {
            if (tile_info.wire_data[i].site == -1 && tile_info.wire_data[i].name == wirename.index) {
                ret.tile = tile;
                ret.index = i;
                break;
            }
        }
    }

    wire_by_name_cache[name] = ret;

    return ret;
}

IdString Arch::getWireType(WireId wire) const { return IdString(wireIntent(wire)); }
std::vector<std::pair<IdString, std::string>> Arch::getWireAttrs(WireId wire) const
{
    return {{id("INTENT"), IdString(wireIntent(wire)).str(this)}};
}

// -----------------------------------------------------------------------

PipId Arch::getPipByName(IdString name) const
{
    if (pip_by_name_cache.count(name))
        return pip_by_name_cache.at(name);
    PipId ret;
    setup_byname();

    const std::string &s = name.str(this);
    if (s.substr(0, 8) == "SITEPIP/") {
        auto sp2 = split_identifier_name(s.substr(8));
        int tile, site;
        std::tie(tile, site) = site_by_name.at(sp2.first);
        auto &tile_info = chip_info->tile_types[chip_info->tile_insts[tile].type];
        auto sp3 = split_identifier_name(sp2.second);
        IdString belname = id(sp3.first), pinname = id(sp3.second);
        for (int i = 0; i < tile_info.num_pips; i++) {
            if (tile_info.pip_data[i].site == site && tile_info.pip_data[i].bel == belname.index &&
                tile_info.pip_data[i].extra_data == pinname.index) {
                ret.tile = tile;
                ret.index = i;
                break;
            }
        }
    } else if (s.find("->") != std::string::npos) {
        // "SRCWIRE->DSTWIRE", which is the form getPipName() PRINTS for a
        // fabric pip.  Without this branch the printer and the parser are not
        // inverses: getPipName writes wire NAMES joined by "->", while the code
        // below expects "TILE/<src_index>.<dst_index>" with numeric indices, so
        // nextpnr could not read back routing it had written itself.  That made
        // the ROUTING net attribute (common/nextpnr.cc, bindPip on load) unusable
        // for xc7 -- it asserted in split_identifier_name_dot on the first pip.
        // applyFixedRoutes already resolves this same spelling by name; do the
        // same here so a routed JSON round-trips.
        size_t arrow = s.find("->");
        WireId src = getWireByName(id(s.substr(0, arrow)));
        WireId dst = getWireByName(id(s.substr(arrow + 2)));
        if (src != WireId() && dst != WireId()) {
            for (auto pip : getPipsDownhill(src)) {
                if (getPipDstWire(pip) == dst) {
                    ret = pip;
                    break;
                }
            }
        }
    } else {
        auto sp = split_identifier_name(s);
        int tile = tile_by_name.at(sp.first);
        auto &tile_info = chip_info->tile_types[chip_info->tile_insts[tile].type];

        auto spn = split_identifier_name_dot(sp.second);
        int fromwire = std::stoi(spn.first), towire = std::stoi(spn.second);

        for (int i = 0; i < tile_info.num_pips; i++) {
            if (tile_info.pip_data[i].site == -1 && tile_info.pip_data[i].src_index == fromwire &&
                tile_info.pip_data[i].dst_index == towire) {
                ret.tile = tile;
                ret.index = i;
                break;
            }
        }
    }

    pip_by_name_cache[name] = ret;

    return ret;
}

IdString Arch::getPipName(PipId pip) const
{
    NPNR_ASSERT(pip != PipId());
    auto loc_info  = locInfo(pip);
    auto pip_data  = loc_info.pip_data[pip.index];
    auto tile_inst = chip_info->tile_insts[pip.tile];
    auto site      = pip_data.site;
    auto bel       = pip_data.bel;

    if (site != -1 && pip_data.flags == PIP_SITE_INTERNAL && bel != -1) {
        return id(std::string("SITEPIP/") +
                    tile_inst.site_insts[site].name.get() +
                    std::string("/") + IdString(bel).str(this) + "/" +
                    IdString(loc_info.wire_data[pip_data.src_index].name).str(this));
    } else {
        return id(getWireName(getPipSrcWire(pip)).str(this) + "->" +
                  getWireName(getPipDstWire(pip)).str(this));
    }
}

void Arch::setup_pip_blacklist()
{
    for (int i = 0; i < chip_info->num_tiletypes; i++) {
        auto &td = chip_info->tile_types[i];
        std::string type = IdString(td.type).str(this);
        // Clock plumbing tiles with NO prjxray documentation at all (no
        // pip db, tilegrid bits:{}) -- any route through them is silently
        // unprogrammable and the clock dies in an unconfigured mux.
        // Proven on HW with the clk_wiz counter: nextpnr sent MMCM
        // CLKOUT0 -> BUFG via HCLK_CLB_CK_IN/CLK_FEED while Vivado's
        // golden uses the documented CLK_HROW CK_IN_R path.
        if (type == "HCLK_CLB" || boost::starts_with(type, "CLK_FEED") ||
            boost::starts_with(type, "HCLK_FEEDTHRU")) {
            for (int j = 0; j < td.num_pips; j++)
                blacklist_pips[td.type].insert(j);
            continue;
        }
        // LIOI (left HP-bank IOI): the pad's ONLY way to fabric is
        // through the ILOGIC (default config = transparent, matches
        // golden's zero bits), but two detours must be blocked:
        //  - IDELAY (unprogrammable IDELAY config; golden goes I0->ILOGIC_D)
        //  - I2GCLK (the clock-capable spine; golden only exits via the
        //    LOGIC_OUTS ppip for ordinary signals, and our boards have no
        //    left-bank clock-capable inputs)
        if (boost::starts_with(type, "LIOI") && !boost::starts_with(type, "LIOI3")) {
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string dest_name = IdString(td.wire_data[pd.dst_index].name).str(this);
                if ((boost::contains(dest_name, "IDELAY") && boost::contains(dest_name, "IDATAIN")) ||
                    boost::contains(dest_name, "I2GCLK"))
                    blacklist_pips[td.type].insert(j);
            }
        }
        // BUFGCTRL bel routethrus: the chipdb models I0/I1 -> O through the
        // global buffer as a pseudo-pip, and the router will happily pass a
        // clock through an UNUSED BUFG slot -- which emits no BUFGCTRL
        // config (IN_USE/CE/S), so the buffer never propagates on silicon,
        // and the unused-slot I-mux defaults collide with the route's IMUX
        // choice (fasm2frames FasmInconsistentBits).  A real BUFG cell
        // sources its O from the bel pin, never from this pseudo-pip.
        if (boost::starts_with(type, "CLK_BUFG_BOT_R") || boost::starts_with(type, "CLK_BUFG_TOP_R")) {
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string dest_name = IdString(td.wire_data[pd.dst_index].name).str(this);
                std::string src_name = IdString(td.wire_data[pd.src_index].name).str(this);
                if (boost::contains(src_name, "BUFGCTRL") &&
                    (boost::ends_with(src_name, "_I0") || boost::ends_with(src_name, "_I1")) &&
                    boost::contains(dest_name, "BUFGCTRL") && boost::ends_with(dest_name, "_O"))
                    blacklist_pips[td.type].insert(j);
            }
        }
        // BRAM address-cascade and FIFO-alias plumbing: the router uses the
        // inter-BRAM ADDR cascade muxes (CASCOUT/CASCINTOP, real bits) and
        // the FIFO36/FIFO18 address/data alias wires as shortcuts to fan
        // BRAM addresses/data between vertically adjacent tiles.  Vivado
        // never routes user nets this way (golden ethsoc has zero such
        // features) and the silicon behaviour of a cascade carrying a
        // foreign fanout is unproven -- top functional suspect for the
        // dark ethsoc R0 build.  Force golden-style INT fanout instead.
        if (boost::starts_with(type, "BRAM_L") || boost::starts_with(type, "BRAM_R")) {
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string dest_name = IdString(td.wire_data[pd.dst_index].name).str(this);
                std::string src_name = IdString(td.wire_data[pd.src_index].name).str(this);
                if (boost::contains(dest_name, "CASCOUT") || boost::contains(src_name, "CASCOUT") ||
                    boost::contains(dest_name, "CASCINTOP") || boost::contains(src_name, "CASCINTOP") ||
                    boost::contains(dest_name, "CASCINBOT") || boost::contains(src_name, "CASCINBOT"))
                    blacklist_pips[td.type].insert(j);
            }
        }
        // EXPERIMENT (NEXTPNR_PIP_BLACKLIST=<file>): forbid an explicit list
        // of pip types, one "TILETYPE.DST.SRC" per line -- the generic knob
        // for bisecting suspected-misencoded pips against hardware.
        if (const char *blf = getenv("NEXTPNR_PIP_BLACKLIST")) {
            static std::set<std::string> bl;
            static bool loaded = false;
            if (!loaded) {
                std::ifstream f(blf);
                std::string ln;
                while (std::getline(f, ln))
                    if (!ln.empty()) bl.insert(ln);
                loaded = true;
                log_info("pip blacklist: %d entries from %s\n", (int)bl.size(), blf);
            }
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string key = type + "." +
                    IdString(td.wire_data[pd.dst_index].name).str(this) + "." +
                    IdString(td.wire_data[pd.src_index].name).str(this);
                if (bl.count(key))
                    blacklist_pips[td.type].insert(j);
            }
        }
        // EXPERIMENT (NEXTPNR_NO_LONGLINES=1): avoid the LV/LH/LVB long
        // lines entirely.  They are non-directional pips whose reverse-
        // orientation segbits were never validated by a working Vivado
        // design (golden ibex/ethsoc drive long lines forward only) --
        // prime suspect for mis-encoded bits that break open-flow nets on
        // silicon while passing every DB-space audit.
        if (getenv("NEXTPNR_NO_LONGLINES") &&
            (boost::starts_with(type, "INT_L") || boost::starts_with(type, "INT_R"))) {
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string dn = IdString(td.wire_data[pd.dst_index].name).str(this);
                std::string sn = IdString(td.wire_data[pd.src_index].name).str(this);
                auto is_ll = [](const std::string &w) {
                    return boost::starts_with(w, "LV") || boost::starts_with(w, "LH") ||
                           boost::starts_with(w, "LVB");
                };
                if (is_ll(dn) || is_ll(sn))
                    blacklist_pips[td.type].insert(j);
            }
        }
        // PHASER/DQS plumbing: present in the chipdb graph but prjxray has
        // no bits for any of it -- the router used INT_DQS_IOTOPHASER as a
        // shortcut onto CMT CLK_IN wires and the clock died unprogrammed
        // (proven on HW with the clk_wiz counter).  No PHASER primitives are
        // supported anyway, so blacklist every pip touching those wires.
        for (int j = 0; j < td.num_pips; j++) {
            auto &pd = td.pip_data[j];
            std::string dest_name = IdString(td.wire_data[pd.dst_index].name).str(this);
            std::string src_name = IdString(td.wire_data[pd.src_index].name).str(this);
            if (boost::contains(dest_name, "PHASER") || boost::contains(src_name, "PHASER"))
                blacklist_pips[td.type].insert(j);
        }
        if (boost::starts_with(type, "HCLK_CMT")) {
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string dest_name = IdString(td.wire_data[pd.dst_index].name).str(this);
                std::string src_name  = IdString(td.wire_data[pd.src_index].name).str(this);
                if (boost::contains(dest_name, "FREQ_REF"))
                    blacklist_pips[td.type].insert(j);
                // prjxray has no segbits for the whole CK_IN<n> <- MUX_CLK_<m>
                // family (the 045-hclk-cmt-pips fuzzer solved CK_IN <- CCIO/
                // BUFHCLK/MUX_CLK_MMCM*/MUX_CLK_PLL* sources, but never the
                // numbered intermediate MUX_CLK_<m> hops), so any clock routed
                // through one is silently dropped by fasm2frames and the clock
                // dies there.  Seen on HW twice: a GT/SGMII refclk -> BUFG
                // transit through CK_IN0 <- MUX_CLK_8, and the ethsoc IP clocks
                // through CK_IN1 <- MUX_CLK_8 / CK_IN0 <- MUX_CLK_7 /
                // CK_IN10 <- MUX_CLK_5.  Blacklist the family so routing uses
                // the solved CMT pips instead (matching Vivado's paths).
                if (boost::contains(dest_name, "HCLK_CMT_CK_IN")) {
                    auto pos = src_name.find("HCLK_CMT_MUX_CLK_");
                    if (pos != std::string::npos) {
                        char next = src_name[pos + sizeof("HCLK_CMT_MUX_CLK_") - 1];
                        // Three combos solved 2026-06-11 by targeted specimens
                        // (see prjxray database FIXES.md): the GT MGT-spine
                        // taps used for quad-113 GT clocks.
                        bool solved =
                            (dest_name == "HCLK_CMT_CK_IN1" && src_name == "HCLK_CMT_MUX_CLK_8") ||
                            (dest_name == "HCLK_CMT_CK_IN0" && src_name == "HCLK_CMT_MUX_CLK_7") ||
                            (dest_name == "HCLK_CMT_CK_IN10" && src_name == "HCLK_CMT_MUX_CLK_5");
                        if (next >= '0' && next <= '9' && !solved)
                            blacklist_pips[td.type].insert(j);
                    }
                }
            }
        } else if (boost::starts_with(type, "CLK_HROW_TOP")) {
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string dest_name = IdString(td.wire_data[pd.dst_index].name).str(this);
                std::string src_name = IdString(td.wire_data[pd.src_index].name).str(this);

                // NOTE: the CK_BUFG_CASCIN->CK_BUFG_CASCO cascade pip is NOT
                // blacklisted.  Vivado's dedicated clock-input path for an
                // IBUFDS->BUFG net descends this cascade (CCIO -> CK_IN_R ->
                // CASCO -> CASCIN -> ... -> CK_MUXED -> BUFGCTRL.I0).
                // Blacklisting it left the BUFG input with only the fabric
                // CLK_BUFG_IMUX pip, which does not deliver a working clock
                // (dead BUFG, frozen design).  routeClock() binds this cascade
                // STRENGTH_LOCKED before the general router runs, so re-enabling
                // it cannot create a routing loop for ordinary nets.
                (void)src_name;
            }
        } else if (boost::starts_with(type, "HCLK_IOI")) {
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string dest_name = IdString(td.wire_data[pd.dst_index].name).str(this);
                std::string src_name = IdString(td.wire_data[pd.src_index].name).str(this);

                if (boost::contains(dest_name, "RCLK_BEFORE_DIV") &&
                    boost::contains(src_name, "IMUX"))
                    blacklist_pips[td.type].insert(j);
            }
        } else if (boost::contains(type, "IOI")) {
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string dest_name = IdString(td.wire_data[pd.dst_index].name).str(this);
                std::string src_name = IdString(td.wire_data[pd.src_index].name).str(this);

                if (boost::contains(dest_name, "CLKB") && boost::contains(src_name, "IMUX22"))
                    blacklist_pips[td.type].insert(j);
                if (boost::contains(dest_name, "OCLKB") && boost::contains(src_name, "IOI_OCLK_"))
                    blacklist_pips[td.type].insert(j);
                if (boost::contains(dest_name, "OCLKM") && boost::contains(src_name, "IMUX31"))
                    blacklist_pips[td.type].insert(j);
                if (boost::contains(dest_name, "_CLKDIV") && boost::contains(src_name, "IMUX8_"))
                    blacklist_pips[td.type].insert(j);
                if (boost::contains(type, "_SING") && dest_name == "IOI_ILOGIC0_CLK" && src_name == "IOI_LEAF_GCLK0")
                    blacklist_pips[td.type].insert(j);
            }
        } else if (boost::starts_with(type, "CMT_TOP_R")) {
            for (int j = 0; j < td.num_pips; j++) {
                auto &pd = td.pip_data[j];
                std::string dest_name = IdString(td.wire_data[pd.dst_index].name).str(this);
                std::string src_name = IdString(td.wire_data[pd.src_index].name).str(this);

                if (boost::contains(dest_name, "PLLOUT_CLK_FREQ_BB_REBUFOUT"))
                    blacklist_pips[td.type].insert(j);
                if (boost::contains(dest_name, "MMCM_CLK_FREQ_BB"))
                    blacklist_pips[td.type].insert(j);
            }
        }
    }
    // NEXTPNR_PIP_BLACKLIST_TILE=<file>: reserve individual pips in NAMED tile
    // instances (one "TILENAME.DST.SRC" per line), for pips whose config bit
    // collides with a live IOB config bit in the shared INT frame column.  Unlike
    // NEXTPNR_PIP_BLACKLIST (per tile TYPE) this forbids the pip in ONE tile only.
    if (const char *blf = getenv("NEXTPNR_PIP_BLACKLIST_TILE")) {
        if (tile_by_name.empty())
            for (int i = 0; i < chip_info->num_tiles; i++)
                tile_by_name[chip_info->tile_insts[i].name.get()] = i;
        std::ifstream f(blf);
        std::string ln;
        int n = 0;
        while (std::getline(f, ln)) {
            if (ln.empty() || ln[0] == '#')
                continue;
            auto d2 = ln.rfind('.');
            if (d2 == std::string::npos || d2 == 0)
                continue;
            auto d1 = ln.rfind('.', d2 - 1);
            if (d1 == std::string::npos)
                continue;
            std::string tname = ln.substr(0, d1);
            std::string dst = ln.substr(d1 + 1, d2 - d1 - 1);
            std::string src = ln.substr(d2 + 1);
            auto it = tile_by_name.find(tname);
            if (it == tile_by_name.end())
                continue;
            int ti = it->second;
            auto &td2 = chip_info->tile_types[chip_info->tile_insts[ti].type];
            for (int j = 0; j < td2.num_pips; j++) {
                auto &pd = td2.pip_data[j];
                if (IdString(td2.wire_data[pd.dst_index].name).str(this) == dst &&
                    IdString(td2.wire_data[pd.src_index].name).str(this) == src) {
                    blacklist_pip_instances[ti].insert(j);
                    ++n;
                }
            }
        }
        log_info("pip blacklist (per-tile-instance): %d pips from %s\n", n, blf);
    }
}

IdString Arch::getPipType(PipId pip) const { return id("PIP"); }

std::vector<std::pair<IdString, std::string>> Arch::getPipAttrs(PipId pip) const { return {}; }

// -----------------------------------------------------------------------

std::vector<IdString> Arch::getBelPins(BelId bel) const
{
    std::vector<IdString> ret;
    NPNR_ASSERT(bel != BelId());

    int num_bel_wires = locInfo(bel).bel_data[bel.index].num_bel_wires;
    const BelWirePOD *bel_wires = locInfo(bel).bel_data[bel.index].bel_wires.get();

    for (int i = 0; i < num_bel_wires; i++) {
        IdString id;
        id.index = bel_wires[i].port;
        ret.push_back(id);
    }

    return ret;
}

BelId Arch::getBelByLocation(Loc loc) const
{
    BelId bi;
    if (loc.x >= chip_info->width || loc.y >= chip_info->height)
        return BelId();
    bi.tile = (loc.y * chip_info->width + loc.x);
    auto &li = locInfo(bi);
    for (int i = 0; i < li.num_bels; i++) {
        if (li.bel_data[i].z == loc.z) {
            bi.index = i;
            return bi;
        }
    }
    return BelId();
}

std::vector<std::pair<IdString, std::string>> Arch::getBelAttrs(BelId bel) const { return {}; }

// -----------------------------------------------------------------------

delay_t Arch::estimateDelay(WireId src, WireId dst, bool debug) const
{
    if (src == dst)
        return 0;
    int src_x, src_y, dst_x, dst_y;
    int src_intent = wireIntent(src); // , dst_intent = wireIntent(dst);
    // if (src_intent == ID_PSEUDO_GND || dst_intent == ID_PSEUDO_VCC)
    //    return 500;
    int dst_tile = dst.tile == -1 ? chip_info->nodes[dst.index].tile_wires[0].tile : dst.tile;
    int src_tile = src.tile == -1 ? chip_info->nodes[src.index].tile_wires[0].tile : src.tile;

    if (sink_locs.count(dst)) {
        dst_x = sink_locs.at(dst).x;
        dst_y = sink_locs.at(dst).y;
        if (src_tile == dst_tile || (sink_locs.count(src) && (sink_locs.at(dst) == sink_locs.at(src)))) {
            return 1000;
        }
    } else if (dst.tile != -1 && chip_info->tile_insts[dst.tile].num_sites > 0) {
        auto &site = chip_info->tile_insts[dst.tile].site_insts[wireInfo(dst).site != -1 ? wireInfo(dst).site : 0];
        if (site.inter_x != -1) {
            dst_x = site.inter_x;
            dst_y = site.inter_y;
        } else {
            dst_x = dst.tile % chip_info->width;
            dst_y = dst.tile / chip_info->width;
        }
    } else {
        dst_x = dst_tile % chip_info->width;
        dst_y = dst_tile / chip_info->width;
    }

    if (src.tile == -1) {
        if (src_intent == ID_PSEUDO_GND || src_intent == ID_PSEUDO_VCC) {
            if (gnd_glbl == IdString()) {
                gnd_glbl = id("PSEUDO_GND_WIRE_GLBL");
                gnd_row = id("PSEUDO_GND_WIRE_ROW");
                vcc_glbl = id("PSEUDO_VCC_WIRE_GLBL");
                vcc_row = id("PSEUDO_VCC_WIRE_ROW");
            }
            if (debug)
                log_info("%s %d %d\n", IdString(wireInfo(src).name).c_str(this), wireInfo(src).name, gnd_glbl.index);
            if (wireInfo(src).name == gnd_glbl.index || wireInfo(src).name == vcc_glbl.index)
                return 15000;

            src_x = src_tile % chip_info->width;
            src_y = src_tile / chip_info->width;
            if (wireInfo(src).name == gnd_row.index || wireInfo(src).name == vcc_row.index)
                src_x = chip_info->width / 2;
        } else {
            auto &src_n = chip_info->nodes[src.index];
            src_x = -1;
            src_y = -1;
            for (int i = 0; i < std::min(200, src_n.num_tile_wires); i++) {
                // Approximate the nearest location to dest
                int ti = src_n.tile_wires[i].tile;
                auto &tw = chip_info->tile_types[chip_info->tile_insts[ti].type].wire_data[src_n.tile_wires[i].index];
                if (tw.num_downhill == 0 && src_intent != ID_NODE_PINFEED)
                    continue;
                int tix = ti % chip_info->width, tiy = ti / chip_info->width;
                if (src_x == -1 || std::abs(tix - dst_x) < std::abs(src_x - dst_x))
                    src_x = tix;
                if (src_y == -1 || std::abs(tiy - dst_y) < std::abs(src_y - dst_y))
                    src_y = tiy;
            }
            if (src_x == -1) {
                src_x = chip_info->nodes[src.index].tile_wires[0].tile % chip_info->width;
                src_y = chip_info->nodes[src.index].tile_wires[0].tile / chip_info->width;
            }
        }

    } else if (src.tile != -1 && chip_info->tile_insts[src.tile].num_sites > 0) {
        auto &site = chip_info->tile_insts[src.tile].site_insts[wireInfo(src).site != -1 ? wireInfo(src).site : 0];
        if (site.inter_x != -1) {
            src_x = site.inter_x;
            src_y = site.inter_y;
        } else {
            src_x = src.tile % chip_info->width;
            src_y = src.tile / chip_info->width;
        }
    } else {
        src_x = src_tile % chip_info->width;
        src_y = src_tile / chip_info->width;
    }
    if (debug)
        log_info("    src (%d, %d) dst (%d, %d)\n", src_x, src_y, dst_x, dst_y);
    /*
        delay_t base = 150 * std::min(std::abs(dst_x - src_x), 30) + 40 * std::max(std::abs(dst_x - src_x) - 30, 0)
                +  150 * std::min(std::abs(dst_y - src_y), 10) + 60 * std::max(std::abs(dst_y - src_y)  - 10, 0)
                + 500;
        auto &srci = wireInfo(src);*/
    /*
    if (srci.intent == ID_NODE_HLONG || srci.intent == ID_NODE_VLONG)
        base -= 180;
    if (srci.intent == ID_NODE_HQUAD || srci.intent == ID_NODE_VQUAD || srci.intent == ID_NODE_DOUBLE)
        base -= 120;
    */
    delay_t base = 30 * std::min(std::abs(dst_x - src_x), 18) + 10 * std::max(std::abs(dst_x - src_x) - 18, 0) +
                   60 * std::min(std::abs(dst_y - src_y), 6) + 20 * std::max(std::abs(dst_y - src_y) - 6, 0) + 300;

    if (xc7)
        base = (base * 3) / 2;

    if (sink_locs.count(dst))
        base += 1000;
    if (src_intent == ID_NODE_PINFEED && dst_x == src_x && dst_y == src_y)
        base -= 200;
    else if ((src_intent == ID_NODE_LOCAL || src_intent == ID_NODE_PINBOUNCE) && dst_x == src_x && dst_y == src_y)
        base -= 100;
    if (src_intent == ID_NODE_CLE_OUTPUT)
        base -= 80;

    return base;
}

ArcBounds Arch::getRouteBoundingBox(WireId src, WireId dst) const
{
    int dst_tile = dst.tile == -1 ? chip_info->nodes[dst.index].tile_wires[0].tile : dst.tile;
    int src_tile = src.tile == -1 ? chip_info->nodes[src.index].tile_wires[0].tile : src.tile;

    int x0, x1, y0, y1;
    x0 = src_tile % chip_info->width;
    x1 = x0;
    y0 = src_tile / chip_info->width;
    y1 = y0;
    auto expand = [&](int x, int y) {
        x0 = std::min(x0, x);
        x1 = std::max(x1, x);
        y0 = std::min(y0, y);
        y1 = std::max(y1, y);
    };

    expand(dst_tile % chip_info->width, dst_tile / chip_info->width);

    if (source_locs.count(src))
        expand(source_locs.at(src).x, source_locs.at(src).y);

    if (sink_locs.count(dst)) {
        expand(sink_locs.at(dst).x, sink_locs.at(dst).y);
    } else if (dst.tile != -1 && chip_info->tile_insts[dst.tile].num_sites > 0) {
        auto &site = chip_info->tile_insts[dst.tile].site_insts[wireInfo(dst).site != -1 ? wireInfo(dst).site : 0];
        if (site.inter_x != -1) {
            expand(site.inter_x, site.inter_y);
        }
    }

    if (src.tile != -1 && chip_info->tile_insts[src.tile].num_sites > 0) {
        auto &site = chip_info->tile_insts[dst.tile].site_insts[wireInfo(dst).site != -1 ? wireInfo(dst).site : 0];
        if (site.inter_x != -1) {
            expand(site.inter_x, site.inter_y);
        }
    }
    return {x0, y0, x1, y1};
}

delay_t Arch::getBoundingBoxCost(WireId src, WireId dst, int distance) const
{
    int src_intent = wireIntent(src);
    if (src.tile == -1 && (src_intent == ID_PSEUDO_GND || src_intent == ID_PSEUDO_VCC))
        return 0;
    if (distance < 5)
        return 0;
    return (distance - 5) * 0;
}

delay_t Arch::getWireRipupDelayPenalty(WireId wire) const
{
    if (wireIntent(wire) == ID_NODE_PINFEED)
        return (3 * getRipupDelayPenalty()) / 2;
    else
        return getRipupDelayPenalty();
}

delay_t Arch::predictDelay(const NetInfo *net_info, const PortRef &sink) const
{
    if (net_info->driver.cell == nullptr || net_info->driver.cell->bel == BelId() || sink.cell->bel == BelId())
        return 0;
    int src_x = net_info->driver.cell->bel.tile % chip_info->width,
        src_y = net_info->driver.cell->bel.tile / chip_info->width;

    int dst_x = sink.cell->bel.tile % chip_info->width, dst_y = sink.cell->bel.tile / chip_info->width;

    if (net_info->driver.cell->bel.tile == sink.cell->bel.tile) {
        Loc dl = getBelLocation(net_info->driver.cell->bel), sl = getBelLocation(sink.cell->bel);
        if ((dl.z >> 4) == (sl.z >> 4))
            return 0;
        else if ((dl.z & 0xF) == BEL_FF2)
            return 700; // penalize FF2 as it makes routing harder
        else
            return 150;
    } else {
        delay_t base = 30 * std::min(std::abs(dst_x - src_x), 18) + 10 * std::max(std::abs(dst_x - src_x) - 18, 0) +
                       60 * std::min(std::abs(dst_y - src_y), 6) + 20 * std::max(std::abs(dst_y - src_y) - 6, 0) + 300;

        if (xc7)
            base = (base * 3) / 2;
        return base;
    }
}

bool Arch::getBudgetOverride(const NetInfo *net_info, const PortRef &sink, delay_t &budget) const { return false; }

// -----------------------------------------------------------------------

bool Arch::place()
{
    std::string placer = str_or_default(settings, id("placer"), defaultPlacer);

    // PLACEMENT IS OPT-IN ON THIS ARCH.  Designs here arrive fully placed --
    // place_lef stamps a BEL on every cell and hands over --fixed-routes -- so
    // an unplaced cell is a DEFECT in whatever produced the netlist, not work
    // for a placer to absorb.  Silently placing the remainder is how a run
    // spent 51 minutes inside HeAP on ~23 cells nobody knew were unstamped.
    //
    // So unless --placer was given explicitly, refuse to place and NAME the
    // cells.  Pass "--placer sa" to opt in when placement really is wanted.
    if (!bool_or_default(settings, id("placer_explicit"), false)) {
        std::vector<CellInfo *> unplaced;
        for (auto &cell : cells) {
            CellInfo *ci = cell.second.get();
            // Pseudo constant drivers are bound by the constant router, never
            // by a placer, so they are legitimately without a bel here.
            if (ci->type == id_PSEUDO_GND || ci->type == id_PSEUDO_VCC)
                continue;
            // A cell CONSTRAINED to another is positioned by its parent, not by
            // a placer: the dist-RAM expansion of a stamped RAM256X1S is four
            // RAMS64E plus an F7/F7/F8 tree, all constr_abs_z children of a
            // base that inherited the macro's BEL.  Demanding a BEL attribute
            // on each of them would reject a design that is in fact fully
            // determined.
            if (ci->constr_parent != nullptr)
                continue;
            // place_lef's CE-buffer promotion invents $cebuf$N cells and stamps
            // a site for the BUFHCE and BUFR forms, but leaves the BUFG form
            // unplaced ON PURPOSE for a placer to site -- place_lef_core.ml has
            // no BUFGCTRL site allocator, so that branch is a bare
            // "ignore (add_buf bit \"BUFG\" sinks)" with no ftstamps entry.
            // These are produced BY the placement step, not evidence that the
            // netlist and the placement disagree, so they are exempt: the
            // placer below still sites them exactly as it did before placement
            // became opt-in.  ibex hits this (9 of them); ethmin does not, only
            // because it runs with TOPO_BUFG_MAX=0 and creates none.
            if (ci->name.str(this).rfind("$cebuf$", 0) == 0)
                continue;
            if (ci->bel == BelId() && !ci->attrs.count(id("BEL")))
                unplaced.push_back(ci);
        }
        if (!unplaced.empty()) {
            log_info("Unplaced cells (showing up to 20 of %d):\n", int(unplaced.size()));
            for (size_t i = 0; i < std::min<size_t>(20, unplaced.size()); i++)
                log_info("    %s (%s)\n", unplaced[i]->name.c_str(this), unplaced[i]->type.c_str(this));
            log_error("%d cell(s) have no BEL and no placement was requested.  This flow expects a "
                      "FULLY PLACED design (place_lef stamps every cell); an unplaced cell means the "
                      "netlist and the placement disagree.  Re-run with '--placer sa' if placing them "
                      "here is genuinely what you want.\n",
                      int(unplaced.size()));
        }
    }

    if (placer == "heap") {
        PlacerHeapCfg cfg(getCtx());
        cfg.criticalityExponent = 7;
        cfg.ioBufTypes.insert(id("IOB_IBUFCTRL"));
        cfg.ioBufTypes.insert(id("IOB_OUTBUF"));
        cfg.ioBufTypes.insert(id_PSEUDO_GND);
        cfg.ioBufTypes.insert(id_PSEUDO_VCC);
        cfg.alpha = 0.08;
        cfg.beta = 0.4;
        cfg.placeAllAtOnce = true;
        cfg.hpwl_scale_x = 1;
        cfg.hpwl_scale_y = 2;
        cfg.spread_scale_x = 2;
        cfg.spread_scale_y = 1;
        // Congestion-reduction knobs, env-adjustable.  The HeAP cut-spreader
        // treats a region as over-used (and spreads cells out of it) once its
        // occupancy exceeds beta*capacity.  LOWERING beta lowers the target
        // density, so cells spread further and leave more free bels/routing
        // tracks per region -- trading wirelength/timing for routability on
        // designs the default placement leaves unroutable (e.g. the dsr_0_ /
        // txu_iLast hard arcs).  spread_scale_{x,y} = how aggressively an
        // over-used region is grown each spreading pass.
        //   NEXTPNR_PLACER_BETA   (default 0.4; try 0.2-0.3 to de-congest)
        //   NEXTPNR_SPREAD_SCALE_X / _Y  (defaults 2 / 1)
        //   NEXTPNR_PLACER_ALPHA  (solver/spread blend, default 0.08)
        if (const char *e = getenv("NEXTPNR_PLACER_BETA"))
            cfg.beta = float(atof(e));
        if (const char *e = getenv("NEXTPNR_SPREAD_SCALE_X"))
            cfg.spread_scale_x = atoi(e);
        if (const char *e = getenv("NEXTPNR_SPREAD_SCALE_Y"))
            cfg.spread_scale_y = atoi(e);
        if (const char *e = getenv("NEXTPNR_PLACER_ALPHA"))
            cfg.alpha = float(atof(e));
        log_info("HeAP congestion knobs: beta=%.3f spread_scale=%d,%d alpha=%.3f\n",
                 cfg.beta, cfg.spread_scale_x, cfg.spread_scale_y, cfg.alpha);
        cfg.netShareWeight = 0.2;
        cfg.solverTolerance = 0.6e-6;
        cfg.cellGroups.emplace_back();
        cfg.cellGroups.back().insert(id_SLICE_LUTX);
        cfg.cellGroups.back().insert(id_SLICE_FFX);
        cfg.cellGroups.back().insert(id_CARRY8);
        if (!placer_heap(getCtx(), cfg))
            return false;
    } else if (placer == "sa") {
        if (!placer1(getCtx(), Placer1Cfg(getCtx())))
            return false;
    } else {
        log_error("US+ architecture does not support placer '%s'\n", placer.c_str());
    }
    fixupPlacement();
    getCtx()->attrs[getCtx()->id("step")] = std::string("place");
    archInfoToAttributes();
    return true;
}

void Arch::routeVcc()
{
    // Route BOTH constant pseudo-nets (Vcc and Gnd) through their real bridge
    // pips before the main router, so fasm.cc emits the const distribution and
    // silicon actually gets the constants.  (Originally Vcc-only + router1-only;
    // Gnd was left to defaults, which floated address-path const-0 inputs high
    // -> corrupt PC = 0x..fff0.)  Per-sink uphill BFS to the const backbone,
    // BOUNDED and terminating at ANY PSEUDO_VCC/GND-intent wire (the tile's
    // local row/global pseudo wire, a few hops above VCC_WIRE/GND_WIRE) instead
    // of traversing the whole pseudo network to the single bound source -> no
    // O(device) stall.  Real sink-side bridge pips get bound + emitted to FASM.
    const int iter_max = 50000;
    std::vector<std::pair<IdString, int>> cnets = {
        { id("$PACKER_VCC_NET"), ID_PSEUDO_VCC },
        { id("$PACKER_GND_NET"), ID_PSEUDO_GND },
    };
    // Record GND sinks the backbone fill can't reach, so a second pass can drive
    // them from a local LUT1(INIT=0) instead (see pack_carry_xc7.cc).  One line
    // per holdout: "<cell> <port>" (e.g. "mem_addr_reg_13__i_2 DI0").
    std::ofstream holdout_out;
    if (const char *hf = getenv("NEXTPNR_GND_HOLDOUT_FILE"))
        holdout_out.open(hf);
    for (auto &cn : cnets) {
        if (!nets.count(cn.first))
            continue;
        NetInfo *net = nets[cn.first].get();
        int pseudo_intent = cn.second;
        log_info("Routing %s connections...\n", cn.first.c_str(this));
        WireId src = getCtx()->getNetinfoSourceWire(net);
        if (src != WireId())
            bindWire(src, net, STRENGTH_STRONG);
        int unrouted = 0, max_iter_seen = 0;
        for (auto &usr : net->users) {
            std::queue<WireId> visit;
            std::unordered_map<WireId, PipId> backtrace;
            WireId dest = WireId();
            WireId sink = getCtx()->getNetinfoSinkWire(net, usr);
            if (sink == WireId())
                log_error("Pin '%s' of bel '%s' has no associated wire\n", usr.port.c_str(this), nameOfBel(usr.cell->bel));
            visit.push(sink);
            int iter = 0;
            while (!visit.empty() && iter < iter_max) {
                ++iter;
                WireId curr = visit.front();
                visit.pop();
                if (getBoundWireNet(curr) == net || wireIntent(curr) == pseudo_intent) {
                    dest = curr;
                    break;
                }
                // Don't route the const net THROUGH a wire owned by a signal net
                // (e.g. the frozen macro's locked routing) -- the old code only
                // vetted src wires, so a signal-owned dst wire slipped into the
                // path and tripped bindWire's wire-ownership assert.
                if (getBoundWireNet(curr) != nullptr)
                    continue;
                for (auto uh : getPipsUphill(curr)) {
                    if (!checkPipAvail(uh))
                        continue;
                    WireId s = getPipSrcWire(uh);
                    if (backtrace.count(s))
                        continue;
                    if (!checkWireAvail(s) && getBoundWireNet(s) != net)
                        continue;
                    backtrace[s] = uh;
                    visit.push(s);
                }
            }
            if (iter > max_iter_seen)
                max_iter_seen = iter;
            if (dest == WireId()) {
                ++unrouted;
                if (getenv("NEXTPNR_LOG_CONST_HOLDOUTS"))
                    log_info("    %s HOLDOUT: %s.%s (bel %s)\n", cn.first.c_str(this),
                             usr.cell->name.c_str(this), usr.port.c_str(this),
                             nameOfBel(usr.cell->bel));
                // GND holdouts only: a local LUT1(INIT=0) can replace these.
                if (holdout_out.is_open() && cn.second == ID_PSEUDO_GND)
                    holdout_out << usr.cell->name.c_str(this) << " "
                                << usr.port.c_str(this) << "\n";
                continue;
            }
            while (backtrace.count(dest)) {
                auto uh = backtrace[dest];
                dest = getPipDstWire(uh);
                if (getBoundWireNet(dest) == nullptr)
                    bindWire(dest, net, STRENGTH_STRONG);
                if (getBoundPipNet(uh) == nullptr)
                    bindPip(uh, net, STRENGTH_STRONG);
            }
        }
        log_info("    %s: %d/%d sinks bridged (%d left to main router; max BFS %d)\n",
                 cn.first.c_str(this), int(net->users.size()) - unrouted, int(net->users.size()),
                 unrouted, max_iter_seen);
    }
}

// BODGE: template a GT-clock -> BUFG route from the known-good Vivado path.
// The chipdb graph has no edge from the GT clock spine into the CLK_HROW
// CK_IN_R inputs, so no router can find this route; but the pips along
// Vivado's dedicated path all exist per-tile.  Bind them by name with
// STRENGTH_LOCKED (router2 then leaves the net alone) exactly as the golden
// ethsoc bitstream routes its three GT clocks on xc7vx485t quad 113:
//   CK_IN_R<k> (HROW of the GT region, Y26) -> CASCO<L> -> CASCIN/CASCO
//   cascade up Y78/Y130/Y182 -> CK_MUXED<L> -> BUFGCTRL<L/2>_I0 (tile Y204),
// with lane L in {2,6,12}.  The sink BUFGCTRL is moved to the lane's site.
// Hardcoded for xc7vx485t bottom-right; enable with NEXTPNR_GT_CLK_BODGE=1.
bool Arch::gtClockTemplateRoute(NetInfo *clk_net, PortRef &usr)
{
    if (!getenv("NEXTPNR_GT_CLK_BODGE"))
        return false;
    auto driver_type = clk_net->driver.cell ? clk_net->driver.cell->type : IdString();
    std::string dtype = driver_type.str(this);
    if (dtype.find("GTXE2") == std::string::npos && dtype.find("IBUFDS_GTE2") == std::string::npos)
        return false;
    if (usr.cell->type != id_BUFGCTRL)
        return false;

    static int next_slot = 0;
    static const int lanes[3] = {2, 6, 12};
    static const int ck_in[3] = {0, 1, 2};
    if (next_slot >= 3) {
        log_warning("GT clock bodge: out of template lanes for net %s\n", nameOf(clk_net));
        return false;
    }
    int L = lanes[next_slot], K = ck_in[next_slot];
    next_slot++;

    // Move the sink BUFGCTRL to the lane's dedicated site (BUFGCTRL_X0Y<L/2>)
    std::string target_site = "BUFGCTRL_X0Y" + std::to_string(L / 2);
    BelId target = getBelByName(id(target_site + "/BUFGCTRL"));
    if (target == BelId()) {
        log_warning("GT clock bodge: no bel %s/BUFGCTRL\n", target_site.c_str());
        return false;
    }
    if (usr.cell->bel != target) {
        CellInfo *evicted = getBoundBelCell(target);
        BelId old_bel = usr.cell->bel;
        if (old_bel != BelId())
            unbindBel(old_bel);
        if (evicted != nullptr) {
            unbindBel(target);
            if (old_bel != BelId())
                bindBel(old_bel, evicted, STRENGTH_STRONG);
        }
        bindBel(target, usr.cell, STRENGTH_STRONG);
        log_info("    GT clock bodge: moved %s to %s\n", nameOf(usr.cell), target_site.c_str());
    }

    // Bind the golden pip chain, resolving pips by tile + local wire names
    struct TPip { std::string tile, dst, src; };
    std::vector<TPip> pips;
    pips.push_back({"CLK_HROW_BOT_R_X192Y26",
                    "CLK_HROW_BOT_R_CK_BUFG_CASCO" + std::to_string(L),
                    "CLK_HROW_CK_IN_R" + std::to_string(K)});
    for (int y : {78, 130, 182})
        pips.push_back({"CLK_HROW_BOT_R_X192Y" + std::to_string(y),
                        "CLK_HROW_BOT_R_CK_BUFG_CASCO" + std::to_string(L),
                        "CLK_HROW_BOT_R_CK_BUFG_CASCIN" + std::to_string(L)});
    pips.push_back({"CLK_BUFG_BOT_R_X192Y204",
                    "CLK_BUFG_BUFGCTRL" + std::to_string(L / 2) + "_I0",
                    "CLK_BUFG_BOT_R_CK_MUXED" + std::to_string(L)});
    int bound = 0;
    setup_byname();
    for (auto &tp : pips) {
        auto tbn = tile_by_name.find(tp.tile);
        if (tbn == tile_by_name.end()) {
            log_warning("GT clock bodge: tile not found: %s\n", tp.tile.c_str());
            continue;
        }
        int tile = tbn->second;
        auto &td = chip_info->tile_types[chip_info->tile_insts[tile].type];
        PipId pip;
        for (int i = 0; i < td.num_pips; i++) {
            auto &pd = td.pip_data[i];
            if (pd.site != -1)
                continue;
            if (IdString(td.wire_data[pd.dst_index].name).str(this) == tp.dst &&
                IdString(td.wire_data[pd.src_index].name).str(this) == tp.src) {
                pip.tile = tile;
                pip.index = i;
                break;
            }
        }
        if (pip == PipId()) {
            log_warning("GT clock bodge: pip not found: %s/%s.%s\n", tp.tile.c_str(),
                        tp.dst.c_str(), tp.src.c_str());
            continue;
        }
        WireId src = getPipSrcWire(pip), dst = getPipDstWire(pip);
        if (getBoundWireNet(src) == nullptr)
            bindWire(src, clk_net, STRENGTH_LOCKED);
        if (getBoundWireNet(dst) == nullptr)
            bindWire(dst, clk_net, STRENGTH_LOCKED);
        bindPip(pip, clk_net, STRENGTH_LOCKED);
        bound++;
    }
    // Bind the (relocated) sink site wire so the general router sees the
    // arc as complete.
    WireId sink_wire = getCtx()->getNetinfoSinkWire(clk_net, usr);
    if (sink_wire != WireId() && getBoundWireNet(sink_wire) == nullptr)
        bindWire(sink_wire, clk_net, STRENGTH_LOCKED);
    log_info("    GT clock bodge: net %s lane %d via CK_IN_R%d (%d/%d pips bound)\n",
             nameOf(clk_net), L, K, bound, int(pips.size()));
    return bound > 0;
}

// Import a frozen routing region ("hard macro") from an external file and LOCK
// it, exactly as the GT clock bodge locks the clock spine (bindPip /
// STRENGTH_LOCKED, which router2 then leaves untouched).  This lets a
// hold-critical island (e.g. the 125 MHz eth PCS+MAC) carry a Vivado-generated,
// hold-clean place+route while nextpnr routes only the relaxed remainder and the
// async CDC boundary around it -- nextpnr itself does no hold analysis, so the
// frozen paths must come from a tool that does.
//
// File format: one pip per line
//     <net_name> <tile>/<src_wire_index>.<dst_wire_index>
// i.e. the owning net followed by a pip identified as tile + its src/dst wire
// indices -- the SAME canonical form getPipByName() parses, and covers BOTH
// fabric pips and site-internal pips (router2 routes site-internal arcs too, so
// a true freeze must lock them).  '#' begins a comment; blank lines ignored.
// The net name must match a net in the loaded design.  writeFixedRoutes() emits
// exactly this form.
void Arch::applyFixedRoutes(const std::string &filename)
{
    std::ifstream in(filename);
    if (!in)
        log_error("failed to open fixed-routes file '%s'\n", filename.c_str());
    setup_byname();
    log_info("Applying fixed routes from '%s'...\n", filename.c_str());

    auto trim = [](std::string s) -> std::string {
        size_t a = s.find_first_not_of(" \t\r\n");
        if (a == std::string::npos)
            return std::string();
        size_t b = s.find_last_not_of(" \t\r\n");
        return s.substr(a, b - a + 1);
    };

    int nbound = 0, nlines = 0, miss_net = 0, miss_tile = 0, miss_pip = 0, conflict = 0, malformed = 0,
        redundant = 0;
    std::unordered_set<NetInfo *> fixed_nets;

    // hierarchy-seam-normalised net lookup: Vivado-extracted names join levels
    // with '/', yosys flatten with '.'; leaf names may contain literal '/' from
    // Vivado's own flattening.  Normalise both sides to '.' and fall back to
    // this map when the exact alias lookup misses.
    auto norm_name = [](std::string s) {
        for (auto &c : s)
            if (c == '/')
                c = '.';
        return s;
    };
    std::unordered_map<std::string, NetInfo *> norm_nets;
    for (auto &n : nets)
        norm_nets[norm_name(n.first.str(this))] = n.second.get();
    for (auto &a : net_aliases) {
        auto ni = nets.find(a.second);
        if (ni != nets.end())
            norm_nets[norm_name(a.first.str(this))] = ni->second.get();
    }
    // Two distinct Vivado nets can flatten to the same nextpnr base name; the
    // importer keeps one bare and prefixes '^' onto the other's leaf component
    // (e.g. inst.^pcspma_status = the SGMII_SPEED driver net vs inst.pcspma_status
    // = the TIMER net).  The Vivado-extracted routes carry each net's own true
    // name; the one that collided to the caret form is the DRIVEN internal net
    // whose golden pips must bind to IT, not to the bare collision partner.
    // Register the caret-stripped alias with precedence for caret nets that have
    // a real driver, so the routes lock the driven net's own tree.
    for (auto &n : nets) {
        std::string nm = norm_name(n.first.str(this));
        if (nm.find('^') == std::string::npos)
            continue;
        std::string stripped;
        for (char c : nm)
            if (c != '^')
                stripped += c;
        if (n.second->driver.cell != nullptr)
            norm_nets[stripped] = n.second.get(); // driven caret net wins the bare name
        else
            norm_nets.emplace(stripped, n.second.get());
    }

    // -------- topology-based net resolution (overrides name matching) --------
    // Vivado bus-bit reordering, scalar/register collisions and port-shadow
    // renames make a route's net NAME disagree with nextpnr's flattened net name
    // (e.g. routes 'addrb_dly[9]' whose signal nextpnr calls a different addrb_dly
    // bit -> golden pips lock to the WRONG net -> that net's real branch is left
    // unlocked and router2 can't complete it).  A route net's ROOT wire -- the one
    // src that never appears as a dst among its own pips -- is its driver's output
    // wire, which uniquely identifies the nextpnr net (drivers are unique).  Map
    // driver-wire -> net, group the routes by name, and for every group whose root
    // matches a placed driver, bind to THAT net regardless of the (renamed) name.
    // Map each net's driver to the WIRES the extracted routes actually start
    // from.  The routes skip the intra-slice output-mux site pips (no node), so a
    // net's ROOT wire is the slice OUTPUT (xMUX / *MUX / LOGIC_OUTS in the CLB
    // tile), NOT the raw driver belpin.  So key the belpin AND its same-tile
    // downstream output wires (a couple of intra-slice hops, staying out of INT).
    std::unordered_map<WireId, NetInfo *> drv_net;
    for (auto &n : nets) {
        NetInfo *ni = n.second.get();
        if (ni->driver.cell == nullptr || ni->driver.cell->bel == BelId())
            continue;
        WireId bp = getBelPinWire(ni->driver.cell->bel, ni->driver.port);
        if (bp == WireId())
            continue;
        int home = bp.tile;
        std::function<void(WireId, int)> rec = [&](WireId w, int depth) {
            if (w == WireId())
                return;
            // A NODAL wire is already canonical and encodes tile == -1 (see
            // canonicalWireId / wireInfo).  Re-canonicalising one indexes
            // tile_insts[-1] and returns a garbage WireId, which segfaults in
            // getPipsDownhill.  This bites whenever the driver's own belpin wire
            // is nodal: `home` is then -1, so the "same tile" containment test
            // below (d.tile != home) MATCHES other nodal wires instead of
            // excluding them, and the walk recurses straight into shared INT
            // routing with an invalid wire.
            WireId cw = w.tile < 0 ? w : canonicalWireId(chip_info, w.tile, w.index);
            // A wire reachable from TWO different drivers is a shared OUTMUX
            // (xMUX) output that several cells could drive -- mapping it to one
            // driver is a guess that binds the wrong net, so mark it ambiguous
            // (nullptr) and let it fall back to name matching.
            auto it = drv_net.find(cw);
            if (it == drv_net.end())
                drv_net[cw] = ni;
            else if (it->second != ni)
                it->second = nullptr;
            // home < 0 means the driver belpin is itself nodal, so "stay in the
            // driver's tile" cannot be expressed -- every nodal wire would
            // compare equal to it.  Record the mapping and stop, which is what
            // the containment test was there to achieve.
            if (depth <= 0 || home < 0)
                return;
            for (auto pip : getPipsDownhill(cw)) { // pips attach to the canonical wire
                WireId d = getPipDstWire(pip);
                if (d.tile != home) // don't cross into shared INT routing
                    continue;
                rec(d, depth - 1);
            }
        };
        rec(bp, 3);
    }
    auto resolve_sd = [&](const std::string &ps, WireId &src, WireId &dst) -> bool {
        size_t arrow = ps.find("->");
        if (arrow != std::string::npos) {
            WireId s = getWireByName(id(trim(ps.substr(0, arrow))));
            WireId d = getWireByName(id(trim(ps.substr(arrow + 2))));
            if (s == WireId() || d == WireId())
                return false;
            src = canonicalWireId(chip_info, s.tile, s.index);
            dst = canonicalWireId(chip_info, d.tile, d.index);
            return true;
        }
        size_t slash = ps.find('/'), dot = ps.rfind('.');
        if (slash == std::string::npos || dot == std::string::npos || dot < slash)
            return false;
        auto tb = tile_by_name.find(ps.substr(0, slash));
        if (tb == tile_by_name.end())
            return false;
        int si, di;
        try {
            si = std::stoi(ps.substr(slash + 1, dot - slash - 1));
            di = std::stoi(ps.substr(dot + 1));
        } catch (...) {
            return false;
        }
        src = canonicalWireId(chip_info, tb->second, si);
        dst = canonicalWireId(chip_info, tb->second, di);
        return true;
    };
    std::unordered_map<std::string, std::vector<std::pair<WireId, WireId>>> byname;
    {
        std::string pl;
        while (std::getline(in, pl)) {
            auto h = pl.find('#');
            if (h != std::string::npos)
                pl = pl.substr(0, h);
            pl = trim(pl);
            if (pl.empty())
                continue;
            size_t sp2 = pl.find_first_of(" \t");
            if (sp2 == std::string::npos)
                continue;
            WireId s, d;
            if (resolve_sd(trim(pl.substr(sp2)), s, d))
                byname[pl.substr(0, sp2)].push_back({s, d});
        }
        in.clear();
        in.seekg(0);
    }
    std::unordered_map<std::string, NetInfo *> topo_net;
    int topo_hits = 0;
    for (auto &kv : byname) {
        std::unordered_set<WireId> dsts;
        for (auto &sd : kv.second)
            dsts.insert(sd.second);
        for (auto &sd : kv.second) {
            if (dsts.count(sd.first))
                continue; // has an upstream pip in this net -> not the root
            auto it = drv_net.find(sd.first);
            if (it != drv_net.end() && it->second != nullptr) {
                topo_net[kv.first] = it->second;
                topo_hits++;
                break;
            }
        }
    }
    log_info("    fixed-routes: %d/%zu route-nets resolved by driver topology\n", topo_hits, byname.size());

    // ---- slice output mux (xMUX) as a SITE pip ---------------------------
    // A Vivado node route crosses the slice's combinational output mux
    // ("CLBLM_R_X129Y5/CLBLM_L_D -> .../CLBLM_L_DMUX").  prjxray marks those 10
    // per CLB tile type is_pseudo=1 -- correctly, they are not interconnect --
    // so the chipdb carries no TILE pip and the wire-name scan above finds
    // nothing.  nextpnr models the same mux as a SITE pip feeding the site wire
    // "<lane>MUX", which is what makes fasm.cc emit <lane>OUTMUX.<sel>.
    //
    // Resolve through the NET'S DRIVER BEL rather than by parsing the tile wire
    // name: one CLB tile holds two slices and only the name prefix
    // (CLBLM_L_ vs CLBLM_M_, CLBLL_L_ vs CLBLL_LL_) distinguishes them, which is
    // fragile.  The driver is by definition in the site whose mux this is.
    int xmux_hits = 0, xmux_fail = 0;
    auto xmux_site_pip = [&](NetInfo *net, const std::string &srcname,
                             const std::string &dstname) -> PipId {
        size_t sl = dstname.rfind('/');
        std::string w = (sl == std::string::npos) ? dstname : dstname.substr(sl + 1);
        // short name must end "<A-D>MUX" (excludes CLBLM_IMUX*, GFAN etc.)
        if (w.size() < 4 || w.compare(w.size() - 3, 3, "MUX") != 0)
            return PipId();
        char lane = w[w.size() - 4];
        if (lane < 'A' || lane > 'D')
            return PipId();
        if (net == nullptr || net->driver.cell == nullptr)
            return PipId();
        BelId bel = net->driver.cell->bel;
        if (bel == BelId())
            return PipId();
        // the "<lane>MUX" wire belonging to the DRIVER's own site
        auto &l = locInfo(bel);
        auto &bd = l.bel_data[bel.index];
        IdString want = id(std::string(1, lane) + "MUX");
        WireId xmux;
        for (int i = 0; i < l.num_wires; i++) {
            auto &wd = l.wire_data[i];
            if (wd.site == bd.site && wd.name == want.index) {
                xmux.tile = bel.tile;
                xmux.index = i;
                break;
            }
        }
        if (xmux == WireId())
            return PipId();
        // Pick the uphill site pip carrying THIS net.  Prefer a source already
        // bound to it; otherwise fall back to the source the Vivado wire name
        // implies -- "<tile>_COUT" is the carry out (CY), anything else is the
        // LUT's direct output (O6).  Never guess beyond that: a wrong mux
        // selection silently mis-routes rather than failing loudly.
        // Site source wire names are LANE-QUALIFIED: the D6LUT's output is
        // "D6LUT_O6", not "O6", and the carry out feeding DMUX is "CARRY4_CO3"
        // (the prjxray feature spells the same selection DOUTMUX.CY).
        bool from_cout = srcname.size() >= 4 &&
                         srcname.compare(srcname.size() - 4, 4, "COUT") == 0;
        std::string implied = from_cout
                                      ? "CARRY4_CO" + std::to_string(lane - 'A')
                                      : std::string(1, lane) + "6LUT_O6";
        PipId by_name;
        for (auto p : getPipsUphill(xmux)) {
            WireId s = getPipSrcWire(p);
            if (getBoundWireNet(s) == net)
                return p;
            if (wireInfo(s).name == id(implied).index && by_name == PipId())
                by_name = p;
        }
        if (by_name != PipId()) {
            xmux_hits++;
            return by_name;
        }
        if (xmux_fail++ < 5) {
            std::string avail;
            for (auto p : getPipsUphill(xmux))
                avail += std::string(" ") + IdString(wireInfo(getPipSrcWire(p)).name).str(this);
            log_warning("fixed-routes: xMUX %c of %s: no source '%s' (have:%s)\n", lane,
                        nameOf(net), implied, avail.c_str());
        }
        return PipId();
    };

    std::string line;
    while (std::getline(in, line)) {
        auto hash = line.find('#');
        if (hash != std::string::npos)
            line = line.substr(0, hash);
        line = trim(line);
        if (line.empty())
            continue;
        nlines++;
        size_t sp = line.find_first_of(" \t");
        if (sp == std::string::npos) {
            if (malformed++ < 20)
                log_warning("fixed-routes: malformed line (no pip): '%s'\n", line.c_str());
            continue;
        }
        std::string netname = line.substr(0, sp);
        std::string pipspec = trim(line.substr(sp));

        NetInfo *net = nullptr;
        auto ov = topo_net.find(netname);
        if (ov != topo_net.end())
            net = ov->second; // driver-topology match wins over the (renamed) name
        if (net == nullptr)
            net = getNetByAlias(id(netname));
        if (net == nullptr) {
            auto nn = norm_nets.find(norm_name(netname));
            if (nn != norm_nets.end())
                net = nn->second;
        }
        if (net == nullptr) {
            // CONSTANT nets have no shared name: Vivado calls them GND/VCC (or
            // <const0>/<const1>), the packer calls them $PACKER_GND_NET /
            // $PACKER_VCC_NET.  Nor can driver topology find them -- a const net
            // is multi-source, one tie-off per region, with no driver cell.
            // Map them explicitly so an imported route can carry the tie-off
            // distribution instead of leaving it to routeVcc()'s fill pass.
            const char *pk = nullptr;
            if (netname == "GND" || netname == "<const0>" || netname == "\\<const0>")
                pk = "$PACKER_GND_NET";
            else if (netname == "VCC" || netname == "<const1>" || netname == "\\<const1>")
                pk = "$PACKER_VCC_NET";
            if (pk != nullptr)
                net = getNetByAlias(id(pk));
        }
        if (net == nullptr) {
            if (miss_net++ < 20)
                log_warning("fixed-routes: net not found: '%s'\n", netname.c_str());
            continue;
        }

        // Two accepted pip forms:
        //  (a) index form  <tile>/<src_idx>.<dst_idx>   (nextpnr-native; writeFixedRoutes emits this)
        //  (b) wire-name   <src_wire>-><dst_wire>       (from Vivado/RapidWright extraction; wire names
        //                                                are shared Xilinx nomenclature, resolved by getWireByName)
        PipId pip;
        size_t arrow = pipspec.find("->");
        if (arrow != std::string::npos) {
            std::string srcname = trim(pipspec.substr(0, arrow));
            std::string dstname = trim(pipspec.substr(arrow + 2));
            WireId src = getWireByName(id(srcname));
            WireId dst = getWireByName(id(dstname));
            if (src == WireId() || dst == WireId()) {
                if (miss_tile++ < 20)
                    log_warning("fixed-routes: wire not found: '%s' or '%s'\n", srcname.c_str(), dstname.c_str());
                continue;
            }
            // Vivado-extracted names may be ANY member wire of a node (Vivado's
            // node root differs from ours); pips attach only to the canonical
            // node-root WireId, so canonicalise before the pip scan.
            src = canonicalWireId(chip_info, src.tile, src.index);
            dst = canonicalWireId(chip_info, dst.tile, dst.index);
            for (auto p : getPipsDownhill(src))
                if (getPipDstWire(p) == dst) {
                    pip = p;
                    break;
                }
            if (pip == PipId()) // fall back to an uphill scan (e.g. site-pin entry pips)
                for (auto p : getPipsUphill(dst))
                    if (getPipSrcWire(p) == src) {
                        pip = p;
                        break;
                    }
            if (pip == PipId())
                pip = xmux_site_pip(net, srcname, dstname);
            if (pip == PipId()) {
                if (miss_pip++ < 20)
                    log_warning("fixed-routes: no pip %s->%s\n", srcname.c_str(), dstname.c_str());
                continue;
            }
        } else {
            size_t slash = pipspec.find('/');
            size_t dot = pipspec.rfind('.');
            if (slash == std::string::npos || dot == std::string::npos || dot < slash) {
                if (malformed++ < 20)
                    log_warning("fixed-routes: bad pip spec: '%s'\n", pipspec.c_str());
                continue;
            }
            std::string tilename = pipspec.substr(0, slash);
            int src_idx, dst_idx;
            try {
                src_idx = std::stoi(pipspec.substr(slash + 1, dot - slash - 1));
                dst_idx = std::stoi(pipspec.substr(dot + 1));
            } catch (...) {
                if (malformed++ < 20)
                    log_warning("fixed-routes: non-numeric wire index: '%s'\n", pipspec.c_str());
                continue;
            }
            auto tbn = tile_by_name.find(tilename);
            if (tbn == tile_by_name.end()) {
                if (miss_tile++ < 20)
                    log_warning("fixed-routes: tile not found: '%s'\n", tilename.c_str());
                continue;
            }
            int tile = tbn->second;
            auto &td = chip_info->tile_types[chip_info->tile_insts[tile].type];
            for (int i = 0; i < td.num_pips; i++) {
                if (td.pip_data[i].src_index == src_idx && td.pip_data[i].dst_index == dst_idx) {
                    pip.tile = tile;
                    pip.index = i;
                    break;
                }
            }
            if (pip == PipId()) {
                if (miss_pip++ < 20)
                    log_warning("fixed-routes: no pip %s/%d.%d\n", tilename.c_str(), src_idx, dst_idx);
                continue;
            }
        }
        // Already claimed (e.g. by the clock spine or a duplicate line)?  Skip
        // rather than trip the bindPip assert; warn only on a real conflict.
        NetInfo *pn = getBoundPipNet(pip);
        if (pn != nullptr) {
            if (pn != net && conflict++ < 20)
                log_warning("fixed-routes: pip %s already bound to '%s', wanted '%s'\n",
                            getPipName(pip).str(this).c_str(), nameOf(pn), nameOf(net));
            continue;
        }
        WireId src = getPipSrcWire(pip), dst = getPipDstWire(pip);
        NetInfo *sn = getBoundWireNet(src), *dn = getBoundWireNet(dst);
        // A wire already owned by a DIFFERENT net means the file mis-associates
        // pips (e.g. a lossy net-name mapping).  Skip with a warning rather than
        // trip bindPip's wire-ownership assert.
        if ((sn != nullptr && sn != net) || (dn != nullptr && dn != net)) {
            if (conflict++ < 20)
                log_warning("fixed-routes: pip %s wire owned by another net (%s); skipping\n",
                            getPipName(pip).str(this).c_str(), nameOf(sn != nullptr && sn != net ? sn : dn));
            continue;
        }
        // A wire has exactly ONE driver.  If dst already belongs to this net it
        // may already be DRIVEN -- routeClock() runs before us and routes the
        // dedicated clock backbone, so a whole-design import (which carries the
        // clock nets too, they are TYPE == SIGNAL in Vivado) can offer a second,
        // fabric-side source for a BUFGCTRL input the backbone already feeds.
        // The check above only rejects a different NET, so the second pip was
        // bound and the FASM selected two sources for one mux -- fasm2frames
        // then aborts with FasmInconsistentBits (measured: 3 BUFGCTRL I0 muxes,
        // each CK_MUXED* from routeClock plus IMUX* from the import).
        if (dn == net) {
            bool already_driven = false;
            for (auto up : getPipsUphill(dst))
                if (getBoundPipNet(up) != nullptr) {
                    already_driven = true;
                    break;
                }
            if (already_driven) {
                redundant++;
                continue;
            }
        }
        if (sn == nullptr)
            bindWire(src, net, STRENGTH_LOCKED);
        if (dn == nullptr)
            bindWire(dst, net, STRENGTH_LOCKED);
        bindPip(pip, net, STRENGTH_LOCKED);
        fixed_nets.insert(net);
        nbound++;
    }
    log_info("    fixed-routes: bound %d/%d pips (net-miss %d, tile-miss %d, pip-miss %d, conflict %d, malformed %d, "
             "redundant-driver %d)\n",
             nbound, nlines, miss_net, miss_tile, miss_pip, conflict, malformed, redundant);
    if (xmux_hits > 0 || xmux_fail > 0)
        log_info("    fixed-routes: %d xMUX pseudo-pips resolved to site pips (%d unresolved)\n", xmux_hits,
                 xmux_fail);

    // LUT-pin TEMPLATE: align every frozen LUT's input ports to the physical
    // input sitewire each net's locked (inter-slice) route actually delivers it
    // to.  A6<->D6 is 1:1, and although A1..A5 permute among D1..D5 via the input
    // crossbar, with FIXED routes each net is pinned to ONE Dx sitewire -- so the
    // port carrying it must sit on the matching belpin or its arc targets the
    // wrong sitewire and the router cannot complete the last mile through the
    // reserved macro.  This is the structural fixup that lets the router route
    // the frozen block natively (no hooking): recognise the LUT, take its pin
    // assignment from the routing template.  Swapping ports drags X_ORIG_PORT
    // (drives the FASM INIT permutation) so the LUT function is preserved.
    //
    // Greedy per-target swaps converge (desired belpin per net is a bijection).
    // Fractured 5LUT+6LUT share the A1..A5 sitewires AND the same input nets, so
    // aligning each cell independently to the shared per-sitewire net stays
    // consistent.  Only fixed (locked-route-fed) LUTs are touched: a fresh LUT's
    // inputs carry no fixed net, so dnet stays null and it is skipped.
    // REPLICATE MODE: when the netlist, the placement AND the routing all come
    // from the same external implementation, each LUT's INIT and its pin
    // assignment are ALREADY mutually consistent -- Vivado's word is
    // authoritative and there is nothing to reconcile.  The template exists for
    // the opposite case (our own placement fed by a foreign route), and it
    // re-permutes INIT along with the ports, which is the known-buggy path
    // noted above.  Skip it entirely; set NEXTPNR_FIXEDROUTES_PINSWAP=1 to force
    // the old behaviour for comparison.
    // Two INDEPENDENT switches: skipping the pin swap must NOT imply skipping
    // the router.  The intra-site last mile (xFFMUX / xOUTMUX / input crossbar)
    // exists only as SITE pips, which an external node-level route never
    // contains -- and those muxes are emitted from BOUND site wires, so if
    // router2 never runs they are never selected.  Measured: a
    // ROUTE_FIXED_ONLY fasm had 24 FFMUX features where a routed build has
    // 2235, i.e. essentially every flip-flop left with an unconfigured D mux --
    // dead on hardware.  So allow "keep Vivado's pin assignment" while still
    // letting the router complete the last mile.
    bool replicate = getenv("NEXTPNR_FIXEDROUTES_NO_PINSWAP") != nullptr ||
                     (getenv("NEXTPNR_ROUTE_FIXED_ONLY") != nullptr &&
                      getenv("NEXTPNR_FIXEDROUTES_PINSWAP") == nullptr);
    int pin_swaps = 0;
    for (auto &cellp : cells) {
        if (replicate)
            break;
        CellInfo *ci = cellp.second.get();
        if (ci->type != id("SLICE_LUTX") || ci->bel == BelId())
            continue;
        int zpos = getBelLocation(ci->bel).z & 0xF;
        if (zpos != BEL_6LUT && zpos != BEL_5LUT)
            continue;
        int npin = (zpos == BEL_6LUT) ? 6 : 5;
        // Snapshot net -> ORIGINAL logical port (I0..I5) before any swap.  The
        // FASM INIT permutation is driven entirely by X_ORIG_PORT_A*, and the
        // hand-rolled label shuffling below loses a label whenever the target
        // pin was empty (orig_tgt == "" so X_ORIG_PORT_A<src> is erased and not
        // restored) -- that input then reads as don't-care and the emitted truth
        // table changes popcount.  Measured against Vivado's own netlist: only
        // 82 of 2265 LUT INITs survived, vs 1792 with no swapping at all.
        // Rederiving every label from the FINAL port assignment makes the
        // labels correct by construction, whatever the swaps did.
        std::unordered_map<NetInfo *, std::string> net_orig;
        for (int k = 1; k <= 6; k++) {
            IdString pk = id("A" + std::to_string(k));
            IdString ak = id("X_ORIG_PORT_A" + std::to_string(k));
            if (!ci->attrs.count(ak) || !ci->ports.count(pk))
                continue;
            NetInfo *n = ci->ports.at(pk).net;
            if (n != nullptr)
                net_orig[n] = ci->attrs.at(ak).as_string();
        }
        // Align all inputs: nextpnr's router treats each belpin as fixed to its
        // tile input (A6-only leaves A1..A5 mismatched -> hooking collides ->
        // 516-overused livelock), so the swap IS needed for routing.  KNOWN BUG:
        // for ~15 frozen LUTs the A1..A5 swap+INIT-permutation writes a WRONG
        // function (popcount changes) -- the detected pin disagrees with golden's
        // INIT.  TODO: reconcile the direct-input detection with the golden INIT
        // (or emit golden's INIT for stamped cells) so these stop corrupting.
        for (int t = 1; t <= npin; t++) {
            IdString tgt = id("A" + std::to_string(t));
            WireId sw = getBelPinWire(ci->bel, tgt);
            if (sw == WireId())
                continue;
            // The locked route binds to the CANONICAL node-root wire, which for a
            // LUT input differs from the raw belpin site wire -- so canonicalise
            // before probing.  The belpin's uphill is an intra-slice crossbar: it
            // can be fed from ANY of the tile input wires (D1..D5), so a naive scan
            // returns whichever is bound first -- the WRONG net.  The net golden
            // routes to THIS pin arrives on the DIRECT tile input wire whose name
            // ends in the same pin id (belpin "D5" <- "..._D5"); match only that.
            WireId csw = canonicalWireId(chip_info, sw.tile, sw.index);
            std::string swn = nameOfWire(sw);
            std::string pin = swn.substr(swn.rfind('/') + 1); // e.g. "D5"
            std::string want = "_" + pin;
            NetInfo *dnet = getBoundWireNet(csw);
            if (dnet != nullptr && !fixed_nets.count(dnet))
                dnet = nullptr;
            if (dnet == nullptr) {
                for (auto pip : getPipsUphill(csw)) {
                    WireId s = getPipSrcWire(pip);
                    std::string sn = nameOfWire(s);
                    std::string stail = sn.substr(sn.rfind('/') + 1);
                    if (stail.size() < want.size() ||
                        stail.compare(stail.size() - want.size(), want.size(), want) != 0)
                        continue; // not the direct tile input for this belpin
                    NetInfo *bn = getBoundWireNet(canonicalWireId(chip_info, s.tile, s.index));
                    if (bn != nullptr && fixed_nets.count(bn)) {
                        dnet = bn;
                        break;
                    }
                }
            }
            if (dnet == nullptr)
                continue;
            NetInfo *tnet = (ci->ports.count(tgt) && ci->ports.at(tgt).net) ? ci->ports.at(tgt).net : nullptr;
            if (tnet == dnet)
                continue; // already on the right belpin
            // find the port currently carrying dnet
            IdString src_pin = IdString();
            for (int k = 1; k <= npin; k++) {
                IdString pin = id("A" + std::to_string(k));
                if (ci->ports.count(pin) && ci->ports.at(pin).net == dnet) {
                    src_pin = pin;
                    break;
                }
            }
            if (src_pin == IdString())
                continue; // this cell doesn't use dnet
            // swap src_pin <-> tgt (nets + X_ORIG_PORT attrs)
            IdString oa_src = id("X_ORIG_PORT_" + src_pin.str(this)),
                     oa_tgt = id("X_ORIG_PORT_" + tgt.str(this));
            std::string orig_src = ci->attrs.count(oa_src) ? ci->attrs.at(oa_src).as_string() : std::string();
            std::string orig_tgt = ci->attrs.count(oa_tgt) ? ci->attrs.at(oa_tgt).as_string() : std::string();
            disconnect_port(getCtx(), ci, src_pin);
            if (tnet != nullptr)
                disconnect_port(getCtx(), ci, tgt);
            if (!ci->ports.count(tgt)) {
                ci->ports[tgt].name = tgt;
                ci->ports[tgt].type = PORT_IN;
            }
            connect_port(getCtx(), dnet, ci, tgt);
            if (tnet != nullptr)
                connect_port(getCtx(), tnet, ci, src_pin);
            ci->attrs.erase(oa_src);
            ci->attrs.erase(oa_tgt);
            if (!orig_src.empty())
                ci->attrs[oa_tgt] = orig_src;
            if (!orig_tgt.empty())
                ci->attrs[oa_src] = orig_tgt;
            pin_swaps++;
        }
        // Rebuild X_ORIG_PORT_A* from where the nets ACTUALLY ended up, so the
        // INIT permutation in get_lut_init matches the final pin assignment.
        for (int k = 1; k <= 6; k++) {
            IdString pk = id("A" + std::to_string(k));
            IdString ak = id("X_ORIG_PORT_A" + std::to_string(k));
            NetInfo *n = ci->ports.count(pk) ? ci->ports.at(pk).net : nullptr;
            // Only REWRITE labels for pins that carry a net; never erase.
            // Erasing a netless pin's label costs 430 extra unrouted arcs
            // (516 -> 946 with everything else equal): X_ORIG_PORT drives the
            // FASM INIT permutation but is also read when deciding how a LUT's
            // inputs may be permuted, so removing it changes routing. A stale
            // label on a pin with no net cannot mis-permute INIT anyway --
            // get_lut_init only consults log_to_bit for nets that exist.
            if (n == nullptr)
                continue;
            auto it = net_orig.find(n);
            if (it != net_orig.end())
                ci->attrs[ak] = Property(it->second);
        }
    }
    if (replicate)
        log_info("    fixed-routes: LUT-pin template SKIPPED (replicate mode -- the imported "
                 "netlist's INIT and pin assignment are authoritative)\n");
    else
        log_info("    fixed-routes: %d LUT-pin template swaps\n", pin_swaps);

    // Complete the last mile of every locked arc.  With the hard-macro
    // contract (router2 now RESERVES locked wires to their own net rather
    // than marking them globally unavailable), router2 completes each macro
    // net's site-pin last-mile itself with a real maze route -- so the old
    // external BFS "hook" (which GUESSED wires and failed ~386 arcs on the
    // golden macro) is disabled by default.  Set NEXTPNR_FIXEDROUTES_HOOK=1
    // to re-enable the legacy hooking for comparison.
    if (getenv("NEXTPNR_FIXEDROUTES_HOOK") == nullptr) {
        log_info("    fixed-routes: last-mile left to router2 (reserved-net contract)\n");
        return;
    }
    int hooked = 0, hook_fail = 0;
    for (NetInfo *net : fixed_nets) {
        // source: walk DOWNHILL from the driver wire until we meet the tree
        WireId src_wire = getCtx()->getNetinfoSourceWire(net);
        auto hook = [&](WireId start, bool downhill) -> bool {
            if (start == WireId())
                return true;
            if (getBoundWireNet(start) == net)
                return true;
            std::queue<WireId> visit;
            std::unordered_map<WireId, PipId> backtrace;
            visit.push(start);
            int iter = 0;
            WireId found = WireId();
            while (!visit.empty() && iter++ < 5000) {
                WireId cur = visit.front();
                visit.pop();
                if (getBoundWireNet(cur) == net) {
                    found = cur;
                    break;
                }
                if (downhill) {
                    for (auto p : getPipsDownhill(cur)) {
                        if (!checkPipAvail(p))
                            continue;
                        WireId next = getPipDstWire(p);
                        NetInfo *bn = getBoundWireNet(next);
                        if (bn != nullptr && bn != net)
                            continue;
                        if (backtrace.count(next))
                            continue;
                        backtrace[next] = p;
                        visit.push(next);
                    }
                } else {
                    for (auto p : getPipsUphill(cur)) {
                        if (!checkPipAvail(p))
                            continue;
                        WireId next = getPipSrcWire(p);
                        NetInfo *bn = getBoundWireNet(next);
                        if (bn != nullptr && bn != net)
                            continue;
                        if (backtrace.count(next))
                            continue;
                        backtrace[next] = p;
                        visit.push(next);
                    }
                }
            }
            if (found == WireId())
                return false;
            // walk back from found to start; verify ownership first (a
            // conflicting wire means an unresolved physical-pin clash)
            WireId cur = found;
            while (cur != start) {
                PipId p = backtrace.at(cur);
                WireId prev = downhill ? getPipSrcWire(p) : getPipDstWire(p);
                NetInfo *pn = getBoundWireNet(prev);
                if (pn != nullptr && pn != net)
                    return false;
                cur = prev;
            }
            cur = found;
            while (cur != start) {
                PipId p = backtrace.at(cur);
                WireId prev = downhill ? getPipSrcWire(p) : getPipDstWire(p);
                // the pip's DST wire: never rebind a wire that already has a
                // driver pip in this net -- overwriting it orphans the
                // original subtree and can create a pip cycle (router2's
                // arc-reconstruction walk then spins forever)
                WireId dstw = downhill ? getPipDstWire(p) : getPipDstWire(p);
                bool dst_driven = false;
                {
                    NetInfo *dn = getBoundWireNet(dstw);
                    if (dn == net && net->wires.count(dstw) && net->wires.at(dstw).pip != PipId())
                        dst_driven = true;
                }
                if (getBoundWireNet(cur) == nullptr)
                    bindWire(cur, net, STRENGTH_LOCKED);
                if (getBoundWireNet(prev) == nullptr)
                    bindWire(prev, net, STRENGTH_LOCKED);
                if (!dst_driven && getBoundPipNet(p) == nullptr)
                    bindPip(p, net, STRENGTH_LOCKED);
                cur = prev;
            }
            return true;
        };
        if (!hook(src_wire, true)) {
            if (hook_fail++ < 10)
                log_warning("fixed-routes: could not hook source of '%s' onto locked tree\n", nameOf(net));
        } else
            hooked++;
        for (auto &usr : net->users) {
            WireId sink = getCtx()->getNetinfoSinkWire(net, usr);
            if (!hook(sink, false)) {
                if (hook_fail++ < 10) {
                    log_warning("fixed-routes: could not hook sink %s.%s of '%s' onto locked tree\n",
                                usr.cell->name.c_str(this), usr.port.c_str(this), nameOf(net));
                    log_warning("    sink wire %s; first-hop uphill:\n", getWireName(sink).c_str(this));
                    for (auto p1 : getPipsUphill(sink)) {
                        WireId w1 = getPipSrcWire(p1);
                        NetInfo *b1 = getBoundWireNet(w1);
                        log_warning("      %s avail=%d net=%s\n", getWireName(w1).c_str(this),
                                    int(checkPipAvail(p1)), b1 ? nameOf(b1) : "-");
                    }
                }
            } else
                hooked++;
        }
    }
    log_info("    fixed-routes: hooked %d arc endpoints onto locked trees (%d failures)\n", hooked, hook_fail);

    // Break pip cycles: node canonicalisation can fold two distinct Vivado
    // pips (e.g. a bidirectional pair) into a 2-wire loop, and router2's
    // setup walks pip chains with no cycle guard (spins forever).  Walk every
    // bound wire's driver chain; any wire revisited within one walk closes a
    // cycle -- unbind the pip that closes it (the orphaned fragment then
    // terminates at an unbound wire, which router2 handles).
    int cycles_broken = 0;
    for (NetInfo *net : fixed_nets) {
        std::vector<PipId> to_unbind;
        std::unordered_set<WireId> checked;
        for (auto &wv : net->wires) {
            WireId start = wv.first;
            if (checked.count(start))
                continue;
            std::unordered_set<WireId> path;
            WireId cursor = start;
            int steps = 0;
            while (steps++ < 100000) {
                if (path.count(cursor)) {
                    to_unbind.push_back(net->wires.at(cursor).pip);
                    cycles_broken++;
                    break;
                }
                path.insert(cursor);
                auto it = net->wires.find(cursor);
                if (it == net->wires.end() || it->second.pip == PipId())
                    break; // reached root or unbound wire: terminates
                cursor = getPipSrcWire(it->second.pip);
            }
            for (auto w : path)
                checked.insert(w);
        }
        for (auto p : to_unbind)
            if (getBoundPipNet(p) != nullptr)
                unbindPip(p);
    }
    if (cycles_broken > 0)
        log_warning("fixed-routes: broke %d pip cycles created by node canonicalisation\n", cycles_broken);
}

// Dump the current routing in the applyFixedRoutes() file format
// (<net> <tile>/<src_idx>.<dst_idx>).  Emits EVERY bound pip, fabric and
// site-internal alike, so that read-back fully freezes the design (router2
// routes site-internal arcs, so those must be locked too).  Round-tripping a
// full routed design through write->read must re-bind every pip and leave the
// router with nothing to do (self-test); a filtered subset captures a hard
// macro's frozen island.
void Arch::writeFixedRoutes(const std::string &filename) const
{
    std::ofstream out(filename);
    if (!out)
        log_error("failed to open fixed-routes output '%s'\n", filename.c_str());
    out << "# nextpnr-xilinx fixed-routes: <net> <tile>/<src_idx>.<dst_idx>\n";
    int npips = 0, nnets = 0;
    for (auto &np : nets) {
        NetInfo *ni = np.second.get();
        bool any = false;
        for (auto &w : ni->wires) {
            PipId pip = w.second.pip;
            if (pip == PipId())
                continue;
            auto &pd = locInfo(pip).pip_data[pip.index];
            out << ni->name.str(this) << ' ' << chip_info->tile_insts[pip.tile].name.get() << '/' << pd.src_index
                << '.' << pd.dst_index << '\n';
            npips++;
            any = true;
        }
        if (any)
            nnets++;
    }
    log_info("Wrote %d fixed-route pips across %d nets to '%s'\n", npips, nnets, filename.c_str());
}

void Arch::routeClock()
{
    log_info("Routing global clocks...\n");
    // Clock nets whose golden distribution is captured in the fixed-routes file
    // (the frozen hard-macro's 125MHz eth clock: txoutclk / bufg_userclk / ...)
    // must NOT be (re)built here -- routeClock picks its own, unvalidated GCLK
    // lanes (GCLKxx) that don't propagate on silicon.  Leave them for
    // applyFixedRoutes to bind EXACTLY as golden (validated GCLK16/17 spine).
    auto clknorm = [](const std::string &s) {
        std::string r;
        for (char c : s)
            if (c != '^')
                r += (c == '/' ? '.' : c);
        return r;
    };
    std::unordered_set<std::string> fixed_clocks;
    {
        std::string fn = str_or_default(settings, id("fixed-routes"), "");
        if (!fn.empty()) {
            std::ifstream fin(fn);
            std::string ln;
            while (std::getline(fin, ln)) {
                auto h = ln.find('#');
                if (h != std::string::npos)
                    ln = ln.substr(0, h);
                size_t b = ln.find_first_not_of(" \t\r\n");
                if (b == std::string::npos)
                    continue;
                size_t sp = ln.find_first_of(" \t", b);
                if (sp == std::string::npos)
                    continue;
                fixed_clocks.insert(clknorm(ln.substr(b, sp - b)));
            }
        }
    }
    // Special pass for faster routing of global clock psuedo-net
    for (auto net : sorted(nets)) {
        NetInfo *clk_net = net.second;
        auto clk_driver = clk_net->driver;
        if (clk_driver.cell == nullptr)
            continue;
        auto driver_type = clk_driver.cell->type;
        auto no_users = clk_net->users.size();
        auto clk_net_user = no_users == 1 ? clk_net->users.front().cell : nullptr;
        auto clk_net_user_type = clk_net_user == nullptr ? IdString() : clk_net_user->type;
        auto from_pll_or_mmcm =
            driver_type == id_PLLE2_ADV_PLLE2_ADV ||
            driver_type == id_MMCME2_ADV_MMCME2_ADV;
        auto to_pll_or_mmcm =
            clk_net_user_type == id_PLLE2_ADV_PLLE2_ADV ||
            clk_net_user_type == id_MMCME2_ADV_MMCME2_ADV;
        auto to_pll_mmcm_clkin1 = to_pll_or_mmcm && clk_net->users.front().port == id_CLKIN1;

        // A single-user net feeding a BUFGCTRL clock input (I0) — e.g. the
        // IBUFDS->BUFG input net — must reach the buffer through the dedicated
        // CCIO->HCLK_CMT->CLK_HROW->CK_MUXED backbone.  Left to the general
        // router it grabs the fabric pip CLK_BUFG_..._IMUX*_*->BUFGCTRL_I0,
        // which does NOT deliver a working clock (the BUFG output is dead and
        // the whole design freezes).  Route it here as a global so the
        // dedicated path is bound LOCKED before the general router runs.
        auto to_bufg_input = (no_users == 1 && clk_net_user_type == id_BUFGCTRL &&
                              clk_net->users.front().port == id_I0);

        // check if we have a global clock net, skip otherwise
        bool is_global = false;
        if ((driver_type == id_BUFGCTRL    || driver_type == id_BUFCE_BUFG_PS ||
             driver_type == id_BUFCE_BUFCE || driver_type == id_BUFGCE_DIV_BUFGCE_DIV) &&
            clk_driver.port == id_O)
            is_global = true;
        else if (no_users == 1 && from_pll_or_mmcm &&
                 (clk_net_user_type == id_BUFGCTRL || clk_net_user_type == id_BUFCE_BUFCE ||
                  clk_net_user_type == id_BUFGCE_DIV_BUFGCE_DIV))
            is_global = true;
        else if (to_pll_mmcm_clkin1)
            is_global = true;
        else if (to_bufg_input)
            is_global = true;
        if (!is_global)
            continue;

        if (fixed_clocks.count(clknorm(clk_net->name.str(this)))) {
            log_info("    clock '%s' left to fixed-routes (golden distribution)\n", clk_net->name.c_str(this));
            continue;
        }

        log_info("    routing clock '%s'\n", clk_net->name.c_str(this));
        WireId clk_src = getCtx()->getNetinfoSourceWire(clk_net);
        if (clk_src == WireId()) {
            log_warning("    clock '%s': driver has no source wire (imported macro?), skipping\n",
                        clk_net->name.c_str(this));
            continue;
        }
        if (getBoundWireNet(clk_src) == nullptr)
            bindWire(clk_src, clk_net, STRENGTH_LOCKED);

        for (auto &usr : clk_net->users) {
            std::queue<WireId> visit;
            std::unordered_map<WireId, PipId> backtrace;
            WireId dest = WireId();

            auto sink_wire = getCtx()->getNetinfoSinkWire(clk_net, usr);
            if (sink_wire == WireId()) {
                log_warning("        clock '%s' user %s.%s has no sink wire, skipping arc\n",
                            clk_net->name.c_str(this), usr.cell->name.c_str(this), usr.port.c_str(this));
                continue;
            }
            if (getBoundWireNet(sink_wire) == clk_net)
                continue; // already routed by fixed-routes
            if (getCtx()->debug) {
                auto sink_wire_name = "(uninitialized)";
                if (sink_wire != WireId())
                    sink_wire_name = nameOfWire(sink_wire);
                log_info("        routing arc to %s.%s (wire %s):\n", usr.cell->name.c_str(this), usr.port.c_str(this), sink_wire_name);
            }

            visit.push(sink_wire);
            while (!visit.empty()) {
                WireId curr = visit.front();
                visit.pop();
                if (getBoundWireNet(curr) == clk_net) {
                    dest = curr;
                    break;
                }
                for (auto uh : getPipsUphill(curr)) {
                    if (!checkPipAvail(uh)) {
                        if (getCtx()->debug) log_info("            skipping unavailable pip %s\n", getPipName(uh).c_str(this));
                        continue;
                    }
                    WireId src = getPipSrcWire(uh);
                    auto srcname = getWireName(src).str(this);
                    if (backtrace.count(src)) {
                        continue;
                    }
                    int intent = wireIntent(src);
                    if (intent == ID_NODE_DOUBLE || intent == ID_NODE_HLONG || intent == ID_NODE_HQUAD ||
                        intent == ID_NODE_VLONG || intent == ID_NODE_VQUAD || intent == ID_NODE_SINGLE ||
                        intent == ID_NODE_CLE_OUTPUT || intent == ID_NODE_OPTDELAY || intent == ID_BENTQUAD ||
                        intent == ID_DOUBLE || intent == ID_HLONG || intent == ID_HQUAD || intent == ID_OPTDELAY ||
                        intent == ID_SINGLE || intent == ID_VLONG || intent == ID_VLONG12 || intent == ID_VQUAD ||
                        intent == ID_PINBOUNCE)
                        {
                            continue;
                        }
                    auto avail     = checkWireAvail(src);
                    auto bound_net = getBoundWireNet(src);
                    if (!avail && bound_net != clk_net)
                        {
                            if (getCtx()->debug)
                                log_info("            skipping unavailable wire %s used by net %s\n", srcname.c_str(), bound_net->name.c_str(this));
                            continue;
                        }
                    backtrace[src] = uh;
                    visit.push(src);
                }
            }
            if (dest == WireId()) {
                log_info("            failed to find a route using dedicated resources.\n");
                if (to_pll_mmcm_clkin1 || to_bufg_input) {
                    // Due to some missing pips, currently special case more lenient solution
                    std::queue<WireId> empty;
                    std::swap(visit, empty);
                    backtrace.clear();
                    visit.push(sink_wire);
                    while (!visit.empty()) {
                        WireId curr = visit.front();
                        visit.pop();
                        if (getBoundWireNet(curr) == clk_net) {
                            dest = curr;
                            break;
                        }
                        for (auto uh : getPipsUphill(curr)) {
                            if (!checkPipAvail(uh))
                                continue;
                            WireId src = getPipSrcWire(uh);
                            if (backtrace.count(src))
                                continue;
                            if (!checkWireAvail(src) && getBoundWireNet(src) != clk_net)
                                continue;
                            backtrace[src] = uh;
                            visit.push(src);
                        }
                    }
                    if (dest == WireId()) {
                        if (gtClockTemplateRoute(clk_net, usr))
                            continue;
                        continue;
                    }
                } else {
                    continue;
                }
            }
            while (backtrace.count(dest)) {
                auto uh = backtrace[dest];
                dest = getPipDstWire(uh);
                if (getCtx()->debug)
                    log_info("            bind pip %s\n", nameOfPip(uh));
                bindWire(dest, clk_net, STRENGTH_LOCKED);
                bindPip(uh, clk_net, STRENGTH_LOCKED);
            }
        }
    }
#if 0
    for (auto net : sorted(nets)) {
        NetInfo *ni = net.second;
        for (auto &usr : ni->users) {
            if (usr.cell->type != id_BUFGCTRL || usr.port != id("I0"))
                continue;
            WireId dst = getCtx()->getNetinfoSinkWire(ni, usr);
            std::queue<WireId> visit;
            visit.push(dst);
            int i = 0;
            while(!visit.empty() && i < 5000) {
                WireId curr = visit.front();
                visit.pop();
                log("  %s\n", nameOfWire(curr));
                for (auto pip : getPipsUphill(curr)) {
                    auto &pd = locInfo(pip).pip_data[pip.index];
                    log_info("    p %s sr %s (t %d s %d sv %d)\n", nameOfPip(pip), nameOfWire(getPipSrcWire(pip)), pd.flags, pd.site, pd.site_variant);
                    if (!checkPipAvail(pip)) {
                        log("      p unavail\n");
                        continue;
                    }
                    WireId src = getPipSrcWire(pip);
                    if (!checkWireAvail(src)) {
                        log("      w unavail (%s)\n", nameOf(getBoundWireNet(src)));
                        continue;
                    }
                    log_info("     p %s s %s\n", nameOfPip(pip), nameOfWire(src));
                    visit.push(src);
                }
                ++i;
            }
        }
    }
#endif
}

void Arch::findSourceSinkLocations()
{
    // Use a backwards BFS to find the real location of sinks, on a best-effort basis
#if 1
    for (auto net : sorted(nets)) {
        NetInfo *ni = net.second;
        for (auto &usr : ni->users) {
            BelId bel = usr.cell->bel;
            if (bel == BelId() || isLogicTile(bel) || (xc7 && isBRAMTile(bel)))
                continue; // don't need to do this for logic bels, which are always next to their INT
            WireId sink = getCtx()->getNetinfoSinkWire(ni, usr);
            if (sink == WireId() || sink_locs.count(sink))
                continue;
            std::queue<WireId> visit;
            std::unordered_map<WireId, WireId> backtrace;
            int iter = 0;
            // as this is a best-effort optimisation to slightly improve routing,
            // don't spend too long with a nice low iteration limit
            const int iter_max = 500;
            visit.push(sink);
            while (!visit.empty() && iter < iter_max) {
                ++iter;
                WireId cursor = visit.front();
                visit.pop();
                if (wireInfo(cursor).site == -1) {
                    int intent = wireIntent(cursor);
                    if (intent != ID_NODE_PINFEED && intent != ID_PSEUDO_VCC && intent != ID_PSEUDO_GND &&
                        intent != ID_INTENT_DEFAULT && intent != ID_NODE_DEDICATED && intent != ID_NODE_OPTDELAY &&
                        intent != ID_PINFEED && intent != ID_INPUT) {
                        int tile = cursor.tile == -1 ? chip_info->nodes[cursor.index].tile_wires[0].tile : cursor.tile;
                        sink_locs[sink] = Loc(tile % chip_info->width, tile / chip_info->width, 0);
                        if (getCtx()->debug) {
                            log_info("%s <---- %s\n", nameOfWire(sink), nameOfWire(cursor));
                        }

                        while (backtrace.count(cursor)) {
                            cursor = backtrace.at(cursor);
                            if (!sink_locs.count(cursor)) {
                                sink_locs[cursor] = Loc(tile % chip_info->width, tile / chip_info->width, 0);
                            }
                        }

                        break;
                    }
                }
                for (auto pip : getPipsUphill(cursor)) {
                    WireId src = getPipSrcWire(pip);
                    if (!backtrace.count(src)) {
                        backtrace[src] = cursor;
                        visit.push(getPipSrcWire(pip));
                    }
                }
            }
        }

        auto &drv = ni->driver;
        if (drv.cell != nullptr) {
            BelId bel = drv.cell->bel;
            if (bel == BelId() || isLogicTile(bel))
                continue; // don't need to do this for logic bels, which are always next to their INT
            WireId source = getCtx()->getNetinfoSourceWire(ni);
            if (source == WireId() || source_locs.count(source))
                continue;
            std::queue<WireId> visit;
            std::unordered_map<WireId, WireId> backtrace;
            int iter = 0;
            // as this is a best-effort optimisation to slightly improve routing,
            // don't spend too long with a nice low iteration limit
            const int iter_max = 500;
            visit.push(source);
            while (!visit.empty() && iter < iter_max) {
                ++iter;
                WireId cursor = visit.front();
                visit.pop();
                if (wireInfo(cursor).site == -1) {
                    int intent = wireIntent(cursor);
                    if (intent != ID_NODE_PINFEED && intent != ID_PSEUDO_VCC && intent != ID_PSEUDO_GND &&
                        intent != ID_INTENT_DEFAULT && intent != ID_NODE_DEDICATED && intent != ID_NODE_OPTDELAY &&
                        intent != ID_NODE_OUTPUT && intent != ID_NODE_INT_INTERFACE) {
                        int tile = cursor.tile == -1 ? chip_info->nodes[cursor.index].tile_wires[0].tile : cursor.tile;
                        source_locs[source] = Loc(tile % chip_info->width, tile / chip_info->width, 0);
                        if (getCtx()->debug) {
                            log_info("%s ----> %s\n", nameOfWire(source), nameOfWire(cursor));
                        }

                        while (backtrace.count(cursor)) {
                            cursor = backtrace.at(cursor);
                            if (!source_locs.count(cursor)) {
                                source_locs[cursor] = Loc(tile % chip_info->width, tile / chip_info->width, 0);
                            }
                        }

                        break;
                    }
                }
                for (auto pip : getPipsDownhill(cursor)) {
                    WireId dst = getPipDstWire(pip);
                    if (!backtrace.count(dst)) {
                        backtrace[dst] = cursor;
                        visit.push(getPipDstWire(pip));
                    }
                }
            }
        }
    }
#endif
}

bool Arch::route()
{
    assign_budget(getCtx(), true);
    std::string router = str_or_default(settings, id("router"), defaultRouter);
    // Route dedicated clocks BEFORE the Vcc flood.  The packer ties BUFGCTRL
    // CE0/S0/S1/IGNORE to $PACKER_VCC_NET, and routeVcc()'s uphill BFS to reach
    // those control pins greedily claims the adjacent clock-backbone wires
    // (CLK_BUFG_REBUF, HCLK_INT_INTERFACE, CK_BUFG_CASC).  If Vcc runs first it
    // locks the real clock out of its dedicated CCIO->CMT->HROW->CK_MUXED path,
    // forcing the IBUFDS->BUFG input onto a fabric IMUX pip that does not
    // deliver a working clock (dead design).  Clocks first; Vcc routes around
    // the LOCKED clock wires afterwards.
    // Lock any frozen hard-macro routing BEFORE routeClock and the general
    // router: macro-internal clock trees (BUFG spine included) come fully
    // bound from the fixed-routes file; routeClock then only fills in the
    // user clocks around the locked wires, and router2 treats everything
    // locked as immovable exactly like the clock spine.
    // routeClock FIRST so it has the full routing graph for the dedicated
    // clock backbone (IBUFDS_GTE2->BUFG etc.) exactly like the vanilla R0
    // flow -- locking macro data pips beforehand starves routeClock's
    // dedicated-resource search and the GT refclk arc falls to (and fails
    // in) router2.  Safe now that fixed-routes carries DATA nets only
    // (SIGNAL-typed); the macro clock trees are (re)built by routeClock.
    // Skip routeClock when the imported routing IS the routing: it builds a
    // clock tree that competes with the one being imported, and the two
    // disagree on which BUFG input to select.
    if (getenv("NEXTPNR_ROUTE_FIXED_ONLY") == nullptr)
        routeClock();
    else
        log_info("    route-fixed-only: skipping routeClock (import supplies the clock nets)\n");
    {
        std::string fixed = str_or_default(settings, id("fixed-routes"), "");
        if (!fixed.empty())
            applyFixedRoutes(fixed);
    }
    // NEXTPNR_ROUTE_FIXED_ONLY=1: treat the IMPORTED routing AS the routing and
    // run no general router at all.  Replaying a complete external route (e.g.
    // a Vivado implementation via dcp2routes) leaves router2 nothing useful to
    // do -- every inter-site pip is already bound and LOCKED -- while it still
    // spends hours failing to "complete" arcs whose wires it may not touch.
    //
    // --route-clock-only is NOT a substitute: it returns before routeVcc(), and
    // the fixed-routes file carries SIGNAL nets only (dcp2routes filters on
    // Vivado's TYPE == SIGNAL), so every constant tie-off would be left
    // unrouted and the bitstream dead.  Run the same finishing passes the
    // normal path does -- routeVcc() as the fill, then fixupRouting() -- so the
    // FASM is complete.
    if (getenv("NEXTPNR_ROUTE_FIXED_ONLY") != nullptr) {
        findSourceSinkLocations();
        // routeVcc() is SKIPPED here, not worked around.  A complete imported
        // route already carries the constant nets; running our own pass over
        // them puts two selections on one mux (fasm2frames dies with "wanted to
        // clear bit ... but was set by", naming a BUFG input), and the previous
        // fix -- filtering those nets out of the import -- threw away 542 real
        // clock features to dodge the clash.  Skipping the pass keeps the
        // import whole, which is the point of a replay.
        // NEXTPNR_FIXED_ROUTE_VCC=1 restores it for an import that genuinely
        // omits the constant nets.
        if (getenv("NEXTPNR_FIXED_ROUTE_VCC") != nullptr)
            routeVcc();
        else
            log_info("    route-fixed-only: skipping routeVcc (import supplies the constant nets)\n");
        fixupRouting();
        getCtx()->settings[getCtx()->id("route")] = 1;
        archInfoToAttributes();
        log_info("    route-fixed-only: imported routing kept as-is; no general router run\n");
        return true;
    }
    // --route-clock-only: nextpnr routes just the dedicated clock backbone
    // (IBUFDS->BUFG input on CCIO->CMT->HROW->CK_MUXED, and BUFG->global) and
    // hands the placed-but-otherwise-unrouted design off to an external router
    // (RapidWright's classic 7-series Router) that fills in all general nets.
    if (settings.find(id("route-clock-only")) != settings.end()) {
        getCtx()->settings[getCtx()->id("route")] = 1;
        archInfoToAttributes();
        return true;
    }
    // Dedicated pad connections (GT serial pins, refclk) have no pips in the
    // chipdb, so the router cannot reach them.  Declare them pre-routed BEFORE
    // it runs -- doing it in fixupRouting() is too late, the arcs have already
    // been attempted and skipped.
    bindDedicatedPads();
    findSourceSinkLocations();

    bool result;
    if (router == "router1") {
        result = router1(getCtx(), Router1Cfg(getCtx()));
    } else if (router == "router2") {
        auto cfg = Router2Cfg(getCtx());
        cfg.bb_margin_x = 4;
        cfg.bb_margin_y = 4;
        cfg.backwards_max_iter = 200;
        cfg.perf_profile = true;
        router2(getCtx(), cfg);
        result = true;
    } else {
        log_error("Xilinx architecture does not support router '%s'\n", router.c_str());
    }
    // routeVcc as a FILL pass: run AFTER the main router so signal nets claim
    // their wires first, then bridge the constant (pwr/gnd) nets through whatever
    // real pips remain free (the BFS already gates on checkWireAvail/checkPipAvail).
    // Pre-router binding over-constrained routing and made router2 fail to route
    // address-path FF arcs (e.g. mem_addr O5->AFFMUX), so const-routed builds
    // exited 255; as a post-router fill it only consumes leftover resources.
    routeVcc();
    fixupRouting();
    // POST-ROUTE timing.  Only router1 runs timing_analysis after routing, so
    // with router2 the ONLY "Max frequency" lines in the log come from
    // placer1's post-placement call -- an estimate built from
    // estimateDelay(), before a single wire is chosen.  Every Fmax quoted for
    // a router2 build is therefore a placement estimate, not routed timing,
    // which is a poor objective to optimise against.
    //   NEXTPNR_POST_ROUTE_TIMING=1  re-run the analysis on the real routing
    //   NEXTPNR_CRIT_PATH_REPORT=1   also dump the critical path stage by stage
    if (getenv("NEXTPNR_POST_ROUTE_TIMING") != nullptr) {
        log_info("Post-route timing analysis:\n");
        timing_analysis(getCtx(), false /* slack_histogram */, true /* print_fmax */,
                        getenv("NEXTPNR_CRIT_PATH_REPORT") != nullptr /* print_path */,
                        false /* warn_on_failure */);
    }
    getCtx()->settings[getCtx()->id("route")] = 1;
    archInfoToAttributes();
    return result;
}

std::string Arch::getPackagePinSite(const std::string &pin) const
{
    if (pin_to_site.empty()) {
        for (int t = 0; t < chip_info->num_tiles; t++) {
            auto &tile = chip_info->tile_insts[t];
            for (int s = 0; s < tile.num_sites; s++) {
                auto &site = tile.site_insts[s];
                if (site.pin[0] != '\0' && site.pin[0] != '.')
                    pin_to_site[site.pin.get()] = site.name.get();
            }
        }
    }
    auto site_iter = pin_to_site.find(pin);
    return site_iter != pin_to_site.end() ? site_iter->second : "";
}

std::string Arch::getBelPackagePin(BelId bel) const
{
    int s = locInfo(bel).bel_data[bel.index].site;
    NPNR_ASSERT(s != -1);
    auto &tile = chip_info->tile_insts[bel.tile];
    auto &site = tile.site_insts[s];
    return site.pin.get();
}

// -----------------------------------------------------------------------

std::vector<GraphicElement> Arch::getDecalGraphics(DecalId decal) const
{
    std::vector<GraphicElement> ret;

    const float lut6_x0 = 0.9;
    const float lut6_x1 = 0.92;
    const float lut6_y0 = 0.1;
    const float lut6_y1 = 0.16;
    const float lut5_x0 = 0.905;
    const float lut5_x1 = 0.915;
    const float lut5_y0 = 0.11;
    const float lut5_y1 = 0.15;
    const float lut_spacing = 0.1;

    const float ff1_x0 = 0.96;
    const float ff1_x1 = 0.98;
    const float ff1_y0 = 0.1;
    const float ff1_y1 = 0.12;
    const float ff2_x0 = 0.96;
    const float ff2_x1 = 0.98;
    const float ff2_y0 = 0.16;
    const float ff2_y1 = 0.18;
    const float ff_spacing = 0.1;

    if (decal.type == DecalId::TYPE_BEL) {
        auto style = decal.active ? GraphicElement::STYLE_ACTIVE : GraphicElement::STYLE_INACTIVE;
        auto bel_data = chip_info->tile_types[decal.tile_type].bel_data[decal.index];
        if (bel_data.type == ID_SLICE_LUTX) {
            int z = bel_data.z >> 4;
            if ((bel_data.z & 0xF) == BEL_5LUT) {
                ret.emplace_back(GraphicElement::TYPE_BOX, style, lut5_x0, lut5_y0 + lut_spacing * z, lut5_x1,
                                 lut5_y1 + lut_spacing * z, 1);
            } else {
                ret.emplace_back(GraphicElement::TYPE_BOX, style, lut6_x0, lut6_y0 + lut_spacing * z, lut6_x1,
                                 lut6_y1 + lut_spacing * z, 1);
            }
        } else if (bel_data.type == ID_SLICE_FFX) {
            int z = bel_data.z >> 4;
            if ((bel_data.z & 0xF) == BEL_FF2) {
                ret.emplace_back(GraphicElement::TYPE_BOX, style, ff2_x0, ff2_y0 + ff_spacing * z, ff2_x1,
                                 ff2_y1 + lut_spacing * z, 1);
            } else {
                ret.emplace_back(GraphicElement::TYPE_BOX, style, ff1_x0, ff1_y0 + ff_spacing * z, ff1_x1,
                                 ff1_y1 + ff_spacing * z, 1);
            }
        }
    }

    const float swb_x0 = 0.2;
    const float swb_x1 = 0.7;
    const float swb_y0 = 0.2;
    const float swb_y1 = 0.7;
    const float wire_len = 0.03;
    const float wire_space = 0.001;
    const float wire_margin = 0.005;

    if (decal.type == DecalId::TYPE_WIRE) {
        WireId wire;
        wire.tile = decal.tile_type;
        wire.index = decal.index;
        auto style = decal.active ? GraphicElement::STYLE_ACTIVE : GraphicElement::STYLE_INACTIVE;

        int wire_tile = (wire.tile == -1) ? 0 : wire.tile;
        int wires_per_side = int(((swb_x1 - swb_x0) - 2 * wire_margin) / wire_space);

        for (auto w : getTileWireRange(wire)) {
            auto wire_data = locInfo(w).wire_data[w.index];
            if (wire_data.site != -1)
                continue;
            int wx = (w.tile % chip_info->width) - (wire_tile % chip_info->width),
                wy = (w.tile / chip_info->width) - (wire_tile / chip_info->width);
            int side = w.index / wires_per_side;
            int offset = w.index % wires_per_side;
            if (side == 1 || side == 3) {
                float y = wy + swb_y0 + wire_margin + offset * wire_space;
                ret.emplace_back(GraphicElement::TYPE_LINE, style, wx + ((side == 3) ? swb_x1 : (swb_x0 - wire_len)), y,
                                 wx + ((side == 3) ? (swb_x1 + wire_len) : swb_x0), y, 1);
            } else if (side == 0 || side == 2) {
                float x = wx + swb_x0 + wire_margin + offset * wire_space;
                ret.emplace_back(GraphicElement::TYPE_LINE, style, x, wy + ((side == 2) ? swb_y1 : (swb_y0 - wire_len)),
                                 x, wy + ((side == 2) ? (swb_y1 + wire_len) : swb_y0), 1);
            }
        }
    }

    return ret;
}

DecalXY Arch::getBelDecal(BelId bel) const
{
    DecalXY decalxy;
    decalxy.decal.type = DecalId::TYPE_BEL;
    decalxy.decal.index = bel.index;
    decalxy.decal.tile_type = chip_info->tile_insts[bel.tile].type;
    decalxy.decal.active = getBoundBelCell(bel) != nullptr;
    decalxy.x = bel.tile % chip_info->width;
    decalxy.y = bel.tile / chip_info->width;
    return decalxy;
}

DecalXY Arch::getWireDecal(WireId wire) const
{
    DecalXY decalxy;
    decalxy.decal.type = DecalId::TYPE_WIRE;
    decalxy.decal.index = wire.index;
    decalxy.decal.tile_type = wire.tile;
    decalxy.decal.active = getBoundWireNet(wire) != nullptr;
    int wire_tile = (wire.tile == -1) ? 0 : wire.tile;
    decalxy.x = wire_tile % chip_info->width;
    decalxy.y = wire_tile / chip_info->width;
    return decalxy;
}

DecalXY Arch::getPipDecal(PipId pip) const { return {}; };

DecalXY Arch::getGroupDecal(GroupId pip) const { return {}; };

// -----------------------------------------------------------------------

bool Arch::getCellDelay(const CellInfo *cell, IdString fromPort, IdString toPort, DelayInfo &delay) const
{
    int tt_id = -1, inst_id = -1;
    if (cell->bel != BelId()) {
        tt_id = locInfo(cell->bel).timing_index;
        inst_id = locInfo(cell->bel).bel_data[cell->bel.index].timing_inst;
    }

    if (cell->type == id_SLICE_LUTX) {
        if (xc7 && inst_id != -1) {
            int z = locInfo(cell->bel).bel_data[cell->bel.index].z;
            IdString tiletype = getBelTileType(cell->bel);
            bool is_lut5 = (z & 0xF) == BEL_5LUT;
            bool is_slicem = (tiletype == id_CLBLM_L || tiletype == ID_CLBLM_R) && (z < 64);
            IdString variant = is_slicem ? (is_lut5 ? id("LUT_OR_MEM5LRAM") : id("LUT_OR_MEM6LRAM"))
                                         : (is_lut5 ? id("LUT5") : id("LUT6"));

            if (fromPort == id_CLK)
                return false;
            return xc7_cell_timing_lookup(tt_id, inst_id, variant, (is_lut5 && fromPort == id_A6) ? id_A5 : fromPort,
                                          (is_lut5 && toPort == id_O6) ? id_O5 : toPort, delay);
        }

        if (fromPort == id_A1 || fromPort == id_A2 || fromPort == id_A3 || fromPort == id_A4 || fromPort == id_A5 ||
            fromPort == id_A6) {
            if (toPort == id_O5 || toPort == id_O6) {
                delay.delay = 200; // FIXME
                return true;
            }
        }
    } else if (cell->type == id_CARRY4) {
        if (xc7 && inst_id != -1) {
            return xc7_cell_timing_lookup(tt_id, inst_id, id("CARRY4"), fromPort, toPort, delay);
        }
    } else if (cell->type == id_F7MUX || cell->type == id_F8MUX || cell->type == id_F9MUX ||
               cell->type == id("SELMUX2_1")) {
        if (xc7 && inst_id != -1) {
            return xc7_cell_timing_lookup(tt_id, inst_id, cell->type, fromPort, toPort, delay);
        }
        delay.delay = 100;
        return true;
    } else if (cell->type == id_BUFGCTRL) {
        if (fromPort == id("I0") || fromPort == id("I1"))
            if (toPort == id("O")) {
                delay.delay = 200; // FIXME
                return true;
            }
    }
    return false;
}

TimingPortClass Arch::getPortTimingClass(const CellInfo *cell, IdString port, int &clockInfoCount) const
{
    if (cell->type == id_SLICE_LUTX) {
        if (get_net_or_empty(cell, id_O5) == nullptr && get_net_or_empty(cell, id_O6) == nullptr)
            return TMG_IGNORE;
        if (port == id_A1 || port == id_A2 || port == id_A3 || port == id_A4 || port == id_A5 || port == id_A6)
            return TMG_COMB_INPUT;
        else if (port == id_O5 || port == id_O6)
            return TMG_COMB_OUTPUT;
    } else if (cell->type == id_CARRY4 && cell->bel != BelId()) {
        return cell->ports.at(port).type == PORT_OUT ? TMG_COMB_OUTPUT : TMG_COMB_INPUT;
    } else if (cell->type == id_SLICE_FFX) {
        if (port == (xc7 ? id_CK : id_CLK))
            return TMG_CLOCK_INPUT;
        else if (port == id_Q) {
            clockInfoCount = 1;
            return TMG_REGISTER_OUTPUT;
        } else {
            clockInfoCount = 1;
            return TMG_REGISTER_INPUT;
        }
    } else if (cell->type == id_F7MUX || cell->type == id_F8MUX || cell->type == id_F9MUX ||
               cell->type == id("SELMUX2_1")) {
        if (port == id_OUT)
            return TMG_COMB_OUTPUT;
        else
            return TMG_COMB_INPUT;
    } else if (cell->type == id_IOB_IBUFCTRL) {
        if (port == id("O"))
            return TMG_STARTPOINT;
    } else if (cell->type == id_IOB_OUTBUF) {
        if (port == id("I"))
            return TMG_ENDPOINT;
    } else if (cell->type == id_BUFGCTRL) {
        if (port == id("I0") || port == id("I1"))
            return TMG_COMB_INPUT;
        if (port == id("O"))
            return TMG_COMB_OUTPUT;
    }
    return TMG_IGNORE;
}

TimingClockingInfo Arch::getPortClockingInfo(const CellInfo *cell, IdString port, int index) const
{
    TimingClockingInfo info;
    // FF (SLICE_FFX/FDRE-family) timing.  Was a flat 0.1ns stub for all three,
    // which under-set clk->Q (real slow-corner ~0.26ns) and made hold analysis
    // impossible.  Values below are the golden-Vivado xc7 -2 calibration
    // (arcs.tsv per-type slow corner: FDRE clk->Q ~0.26, setup ~0.05, hold
    // ~0.14ns).  DelayInfo is single-valued today (min==max) so these are the
    // slow/late corner; the min/early (hold) corner scales by ~0.45 once
    // DelayInfo carries a separate min (Phase-1 hold pass).
    info.setup = getDelayFromNS(0.05);
    info.hold = getDelayFromNS(0.143);    // hold requirement: use as-is (min==max)
    info.clockToQ = getDelayFromNS(0.259);
    info.clockToQ.min = getDelayFromNS(0.117).delay;  // fast-corner clk->Q for hold
    info.clock_port = xc7 ? id_CK : id_CLK;
    info.edge = RISING_EDGE;
    return info;
}

int Arch::getHclkForIob(BelId pad)
{
    std::string tiletype = getBelTileType(pad).str(this);
    int ioi = pad.tile;
    // Find the IOI for IOB
    if (boost::starts_with(tiletype, "LIOB"))
        ioi += 1;
    else if (boost::starts_with(tiletype, "RIOB") ||
             boost::starts_with(tiletype, "GTP_") ||
             boost::starts_with(tiletype, "GTX_")) {
        ioi -= 1;
    } else {
        std::string message = "unknown IOB side of tile type " + tiletype;
        NPNR_ASSERT_FALSE(message.c_str());
    }
    return getHclkForIoi(ioi);
}

int Arch::getHclkForIoi(int ioi)
{
    // Find a wire driven by the HCLK
    WireId ioclk0;
    auto &td = chip_info->tile_types[chip_info->tile_insts[ioi].type];
    for (int i = 0; i < td.num_wires; i++) {
        std::string name = IdString(td.wire_data[i].name).str(this);
        if (name == "IOI_IOCLK0" || name == "IOI_SING_IOCLK0") {
            ioclk0 = canonicalWireId(chip_info, ioi, i);
            break;
        }
    }
    NPNR_ASSERT(ioclk0 != WireId());
    for (auto uh : getPipsUphill(ioclk0))
        return uh.tile;
    NPNR_ASSERT_FALSE("failed to find HCLK pips");
}
namespace {
template <typename Tres, typename Tgetter, typename Tkey>
boost::optional<const Tres &> db_binary_search(const Tres *list, int count, Tgetter key_getter, Tkey key)
{
    if (count < 7) {
        for (int i = 0; i < count; i++) {
            if (key_getter(list[i]) == key) {
                return boost::optional<const Tres &>(list[i]);
            }
        }
    } else {
        int b = 0, e = count - 1;
        while (b <= e) {
            int i = (b + e) / 2;
            if (key_getter(list[i]) == key) {
                return boost::optional<const Tres &>(list[i]);
            }
            if (key_getter(list[i]) > key)
                e = i - 1;
            else
                b = i + 1;
        }
    }
    return {};
}
} // namespace

bool Arch::xc7_cell_timing_lookup(int tt_id, int inst_id, IdString variant, IdString from_port, IdString to_port,
                                  DelayInfo &delay) const
{
    if (tt_id == -1 || inst_id == -1)
        return false;
    const InstanceTimingPOD &inst = chip_info->timing_data->tile_cell_timings[tt_id].instances[inst_id];
    auto found_var = db_binary_search(
            inst.celltypes.get(), inst.num_celltypes, [](const CellTimingPOD &ct) { return ct.variant_name; },
            variant.index);
    if (!found_var)
        return false;

    const CellTimingPOD &ct = *found_var;
    auto found_delay = db_binary_search(
            ct.delays.get(), ct.num_delays,
            [](const CellPropDelayPOD &ct) { return std::make_pair(ct.to_port, ct.from_port); },
            std::make_pair(to_port.index, from_port.index));
    if (!found_delay)
        return false;
    delay.delay = found_delay->max_delay;
    return true;
}

// HeAP IS RETIRED ON THIS ARCH.  Everything here arrives PLACED -- place_lef
// stamps a BEL on every cell and hands nextpnr --fixed-routes -- so there is
// nothing for an analytic placer to solve, and measured it does not merely add
// no value: it spent 51 MINUTES on the ~23 unstamped cells of the ethmin core
// before being killed, because the constrained majority gives its solver
// nothing to spread against.
//
// It used to be the DEFAULT, so any invocation that forgot "--placer sa" got it
// silently -- which is exactly how that 51-minute run happened
// (build_hardmacro_ethmin.sh passes no --placer).  Defaulting to sa makes the
// flow's real placer the one you get when you say nothing.
//
// The implementation is still compiled and still selectable on the other
// arches; only this arch's default and menu change.  To put it back, add "heap"
// to availablePlacers below.
const std::string Arch::defaultPlacer = "sa";

const std::vector<std::string> Arch::availablePlacers = {"sa"};

const std::string Arch::defaultRouter = "router2";
const std::vector<std::string> Arch::availableRouters = {"router1", "router2"};

void Arch::load_bel_blacklist() const
{
    blacklist_bels_loaded = true;
    const char *f = getenv("NEXTPNR_BEL_BLACKLIST");
    if (f == nullptr)
        return;
    std::ifstream in(f);
    if (!in)
        return;
    std::string ln;
    int n = 0, miss = 0;
    while (std::getline(in, ln)) {
        while (!ln.empty() && (ln.back() == '\r' || ln.back() == ' '))
            ln.pop_back();
        if (ln.empty() || ln[0] == '#')
            continue;
        BelId b = getBelByName(id(ln));
        if (b == BelId()) {
            miss++;
            continue;
        }
        blacklist_bels.insert((int64_t(b.tile) << 32) | uint32_t(b.index));
        n++;
    }
    log_info("bel blacklist: reserved %d bel(s) from %s (%d unresolved)\n", n, f, miss);
}

NEXTPNR_NAMESPACE_END
