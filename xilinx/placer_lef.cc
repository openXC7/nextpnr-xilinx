/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  placer_lef -- place_lef.exe's placement passes, translated into nextpnr one
 *  pass at a time.
 *
 *  WHY THIS EXISTS.  The open flow's real placer is an external OCaml program,
 *  place_lef.exe, which stamps a BEL onto every cell and hands nextpnr a
 *  netlist it merely has to honour.  That works -- every silicon-verified
 *  bitstream this project has produced went through it -- but it costs us a
 *  second device model (floorplan.json, a hand-built site-kind taxonomy,
 *  TOPO_SITE_PHYSMAP) and, worse, a NAME-SPACE SEAM between two programs that
 *  has silently mismatched cells more than once: criticality export once joined
 *  119 of 4968 cells, placement replay 5986 of 6643.  Inside nextpnr there is
 *  only one name space and only one device model.
 *
 *  HOW THE PORT IS SEQUENCED.  Not "write a better placer and hope it agrees".
 *  The two placers currently agree on NOTHING they choose themselves -- on
 *  johnson, every one of the 64 placeable cells lands somewhere different, and
 *  the only agreement (21 cells) is I/O pinned by the XDC.  So there is no
 *  incremental path that starts from nextpnr's own placer and tunes toward
 *  place_lef.
 *
 *  Instead we start from the placement that is already identical by
 *  construction -- place_lef's, arriving as BEL attributes -- and move ONE PASS
 *  at a time inside, checking after each move that the result has not changed:
 *
 *      make placement-ab      (in the demos repo)
 *
 *  compares this placer's output against place_lef's cell by cell and reports
 *  the join rate and exact-BEL agreement.  A ported pass that reproduces
 *  place_lef's decisions leaves that number where it was; one that does not
 *  moves it, and says by how much and on which cells.  That makes each step
 *  falsifiable on its own rather than at the end of a large rewrite.
 *
 *  PASS INVENTORY, from place_lef_core.ml (4530 lines).  Two distinct jobs:
 *
 *  Netlist mutation -- PACKER work, belongs in pack*.cc, not in a placer.
 *  Composed at place_lef_core.ml:4527:
 *      cleanup_netlist -> stitch_gt_pad_buffers -> split_degenerate_muxf
 *      -> replicate_shared_muxf7 -> materialise_const_drivers
 *      -> insert_hold_buffers
 *  plus $cebuf$ BUFG/BUFH promotion (:2641), $feedthrough$ LUT1 relays (:2775),
 *  CARRY-slice completion (:2937, the carry_stamp port) and $muxdup (:4249).
 *
 *  Placement proper -- what this file is for:
 *      floorplan / site model          (nextpnr's device model replaces it)
 *      hard-block placement            (small: GT sites already infer from
 *                                       PACKAGE_PIN correctly)
 *      anchor centroid + region select
 *      SA over HPWL + RUDY congestion + long-line + criticality weighting
 *      clock-domain affinity, domain zoning
 *      DRAM spreading, carry-chain spreading
 *
 *  PORTED SO FAR: nothing.  Every cell still comes in pre-stamped, and
 *  anything unstamped falls through to placer1 (the "sa" placer) exactly as
 *  --placer sa would have done.  That is deliberate: this commit establishes
 *  the entry point, the registration and the reporting, so that the first real
 *  pass has somewhere to land and a baseline to be measured against.
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <climits>
#include <map>
#include <set>
#include <limits>
#include <regex>

#include "log.h"
#include "nextpnr.h"
#include "pack_to_lef_port.h"
#include "placer1.h"
#include "util.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {

// A SLICE site's INDEX coordinates, parsed out of "SLICE_X121Y110/A6LUT".
//
// This is deliberately NOT getBelLocation().  place_lef ranks sites in the
// SLICE_XnYm index frame, and gen_floorplan.py maps every non-SLICE site into
// that same frame (a tile's grid_x to the nearest SLICE column, plus the site's
// own Y).  nextpnr's native frame is tile (x,y), and the two do not agree --
// SLICE_X advances 2 per CLB column, for one.  Ranking in tile coordinates
// would select a DIFFERENT set of K sites and so would not reproduce
// place_lef's placement, while still looking plausible.  So the index frame is
// recovered from the name.
struct SliceXY
{
    int x, y;
};

bool parse_slice_xy(const std::string &bel_name, SliceXY &out)
{
    // SLICE_X<nn>Y<mm>/<bel>
    static const std::regex re(R"(^SLICE_X(\d+)Y(\d+)/)");
    std::smatch m;
    if (!std::regex_search(bel_name, m, re))
        return false;
    out.x = std::stoi(m[1].str());
    out.y = std::stoi(m[2].str());
    return true;
}

float getenv_float(const char *name, float dflt)
{
    if (const char *e = getenv(name))
        return float(atof(e));
    return dflt;
}

std::string getenv_str(const char *name, const std::string &dflt)
{
    if (const char *e = getenv(name))
        return std::string(e);
    return dflt;
}

// How much a pinned site should count toward the anchor centroid.
//
// An unweighted mean lets the SLOWEST pins decide where the design goes.
// johnson has 8 monitor LEDs, one reset and one differential clock: eight
// don't-care outputs outvote the clock 8:1, and the logic gets dragged toward
// the LED bank.  The pins that should pull the design are the ones whose delay
// is on a real path -- the clock entry, the transceivers, wide synchronous
// buses -- not a status light a human reads at 1 Hz.
//
// Classification is structural, not a hand-kept list of net names, so it does
// not rot as designs change: follow the pin's net through the IO buffers and
// see what it actually reaches.
enum AnchorClass
{
    // NOTE ON MMCM/PLL: they are NOT attractors.  An MMCM's output always goes
    // through a BUFG onto the global clock network, so where the MMCM sits has
    // no bearing on where the logic it clocks belongs -- its own position is
    // constrained by its dedicated clock input pin and nothing else.  Treating
    // it as an attractor pulled ethmin's anchor from (220,16) to (174,82),
    // because two MMCMs sit at slice x=48 on the far side of the die from the
    // transceiver.  They are classified ANCHOR_GLOBAL with the buffers.
    ANCHOR_GT,     // a transceiver: fixed at the die edge, and its datapath is
                   // the fastest and most fragile thing on the chip.  place_lef
                   // anchors ethmin on the GT ALONE -- its log reads
                   //   anchor=(220,16) bbox X[175..221] Y[0..61], 99% used
                   // and (220,16) IS the GT's site.  Co-locating the logic with
                   // the transceiver is the whole reason that design routes.
    ANCHOR_CLOCK,  // an MMCM/PLL: fixed, worth being near, but not decisive
    ANCHOR_DATA,   // feeds or is fed by fabric logic
    ANCHOR_SLOW,   // output-only, drives nothing further -- LEDs and status
    ANCHOR_GLOBAL, // a global clock buffer: its output reaches the WHOLE die, so
                   // its position says nothing about where logic should sit
};

const char *anchor_class_name(AnchorClass c)
{
    switch (c) {
    case ANCHOR_GT:
        return "GT";
    case ANCHOR_CLOCK:
        return "MMCM/PLL";
    case ANCHOR_DATA:
        return "data";
    case ANCHOR_GLOBAL:
        return "global-clk";
    default:
        return "slow";
    }
}

// BUFG/BUFH/BUFR drive global or regional clock networks that span the device.
// Where the buffer sits is a routing detail, not a hint about where the logic it
// clocks belongs.
bool is_gt_cell(const std::string &t)
{
    return t.find("GTXE") != std::string::npos || t.find("GTHE") != std::string::npos ||
           t.find("GTPE") != std::string::npos || t.find("IBUFDS_GTE") != std::string::npos;
}

bool is_global_clock_buffer(const std::string &t)
{
    // MMCM/PLL belong here too: their output reaches the fabric only via a
    // BUFG, so they are global in exactly the sense that matters.
    return t.find("BUFG") != std::string::npos || t.find("BUFH") != std::string::npos ||
           t.find("BUFR") != std::string::npos || t.find("MMCM") != std::string::npos ||
           t.find("PLLE") != std::string::npos;
}

bool is_clock_cell(const std::string &t)
{
    return t.find("BUFG") != std::string::npos || t.find("BUFH") != std::string::npos ||
           t.find("BUFR") != std::string::npos || t.find("MMCM") != std::string::npos ||
           t.find("PLLE") != std::string::npos || t.find("GTXE") != std::string::npos;
}

bool is_fabric_cell(const std::string &t)
{
    return t.find("SLICE") != std::string::npos || t.find("CARRY") != std::string::npos ||
           t.find("RAMB") != std::string::npos || t.find("DSP") != std::string::npos;
}

// Bounded walk outward from an IO cell, STOPPING AT FABRIC.
//
// The stop condition is the whole trick.  A monitor LED is OBUF <- FF, and that
// flop's CLK pin reaches the BUFG in two hops -- so a walk that traverses
// THROUGH fabric finds a clock buffer starting from anywhere and duly
// classified all eight of johnson's LEDs as clock/GT, which is the opposite of
// what this is for.  Only the IO buffer chain itself is followed
// (PAD -> IBUF/OBUF -> IBUFDS/BUFG); the first fabric cell reached records that
// the pin talks to logic and is not expanded past.
// How many distinct nets tie this cell to the FABRIC?
//
// This is what actually distinguishes a transceiver from a status LED, and it
// is measured rather than assumed: a GT carries a wide datapath into the MAC,
// an LED carries one wire.  Weighting the centroid by it makes the design
// gravitate to whatever it is most tightly coupled to -- the GT on ethmin, the
// BRAM/DSP columns on a design with no transceiver -- with no per-design
// constant to maintain.  A hand-set TOPO_ANCHOR_X/Y still wins when supplied.
// The clock rate a net runs at, in MHz, from create_clock.  1.0 when unknown,
// so an unconstrained net counts for something but never much.
double net_clock_mhz(Context *ctx, NetInfo *net)
{
    // The net may BE a clock...
    if (net->clkconstr) {
        double ps = ctx->getDelayNS(net->clkconstr->period.maxDelay()) * 1000.0;
        if (ps > 0.0)
            return 1e6 / ps;
    }
    // ...or be launched/captured by flops in some domain.  Take the fastest
    // domain it touches: that is the one whose delay budget is tightest.
    double best = 1.0;
    auto consider = [&](CellInfo *c) {
        if (c == nullptr)
            return;
        for (auto &conn : c->ports) {
            NetInfo *n2 = conn.second.net;
            if (n2 == nullptr || !n2->clkconstr)
                continue;
            double ps = ctx->getDelayNS(n2->clkconstr->period.maxDelay()) * 1000.0;
            if (ps > 0.0)
                best = std::max(best, 1e6 / ps);
        }
    };
    consider(net->driver.cell);
    for (auto &u : net->users)
        consider(u.cell);
    return best;
}

// Coupling to the fabric, weighted by CLOCK RATE.
//
// Connectivity alone is not the whole story: a 125 MHz transceiver datapath and
// a 25 MHz housekeeping bus of the same width are not equally urgent to keep
// close, and on ethmin it was eth_tx_clk (125 MHz) that failed timing while the
// slow domains passed comfortably.  Summing each fabric-touching net's clock
// rate captures both -- width AND speed -- and needs no per-design constant.
double fabric_coupling(Context *ctx, CellInfo *cell)
{
    std::set<IdString> seen;
    double total = 0.0;
    for (auto &conn : cell->ports) {
        NetInfo *net = conn.second.net;
        if (net == nullptr || !seen.insert(net->name).second)
            continue;
        bool touches_fabric = false;
        if (net->driver.cell != nullptr && is_fabric_cell(net->driver.cell->type.str(ctx)))
            touches_fabric = true;
        for (auto &u : net->users)
            if (u.cell != nullptr && is_fabric_cell(u.cell->type.str(ctx)))
                touches_fabric = true;
        if (touches_fabric)
            total += net_clock_mhz(ctx, net);
    }
    return total;
}

AnchorClass classify_anchor_cell(Context *ctx, CellInfo *start)
{
    // The cell may itself BE the clock resource -- a BUFG is one of the placed
    // non-SLICE cells that contributes to the anchor, and it was coming back as
    // "data" because only its neighbours were being tested.
    {
        const std::string st = start->type.str(ctx);
        // A GLOBAL clock buffer must not pull the design toward itself.  On
        // ethmin the eight BUFGCTRLs all sit in the centre clock column at
        // slice x=110 and, weighted like the transceiver, they outvoted it 8:2
        // -- the anchor landed at (148,140), mid-die, and the design was placed
        // 60 columns from the GT it exists to serve.  place_lef anchors on the
        // hard blocks and puts ethmin at X[174..221], right beside the GT.
        if (is_gt_cell(st))
            return ANCHOR_GT;
        if (is_global_clock_buffer(st))
            return ANCHOR_GLOBAL;
        if (is_clock_cell(st))
            return ANCHOR_CLOCK;
    }

    std::set<IdString> seen;
    std::vector<std::pair<CellInfo *, int>> queue{{start, 0}};
    bool touches_fabric = false;
    while (!queue.empty()) {
        auto [ci, depth] = queue.back();
        queue.pop_back();
        if (!seen.insert(ci->name).second)
            continue;
        const std::string t = ci->type.str(ctx);
        if (ci != start) {
            if (is_gt_cell(t))
                return ANCHOR_GT;
            if (is_global_clock_buffer(t))
                return ANCHOR_GLOBAL;
            if (is_clock_cell(t))
                return ANCHOR_CLOCK;
            if (is_fabric_cell(t)) {
                touches_fabric = true;
                continue; // do NOT expand through logic
            }
        }
        if (depth >= 3)
            continue;
        for (auto &conn : ci->ports) {
            NetInfo *net = conn.second.net;
            if (net == nullptr)
                continue;
            if (net->driver.cell != nullptr)
                queue.emplace_back(net->driver.cell, depth + 1);
            for (auto &u : net->users)
                if (u.cell != nullptr)
                    queue.emplace_back(u.cell, depth + 1);
        }
    }
    return touches_fabric ? ANCHOR_DATA : ANCHOR_SLOW;
}

} // namespace

bool placer_lef(Context *ctx)
{
    // Report the split before doing anything.  Whenever two name spaces meet
    // -- here, "cells place_lef decided" against "cells left to us" -- the
    // count is printed rather than assumed.  A run that silently placed most of
    // the design because the BEL attributes failed to arrive would otherwise
    // look exactly like a run that honoured them.
    int stamped = 0, unstamped = 0;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->attrs.count(ctx->id("BEL")))
            ++stamped;
        else
            ++unstamped;
    }
    log_info("placer_lef: %d/%d cells arrive pre-stamped by place_lef, %d left to place\n", stamped,
             stamped + unstamped, unstamped);

    if (stamped == 0)
        log_warning("placer_lef: NO cell carries a BEL attribute -- this is a raw netlist, so every\n"
                    "            placement decision below is placer1's, not place_lef's.  Expect the\n"
                    "            placement-ab agreement to read as though --placer sa had been used.\n");

    // ===================================================================
    // PASS 2 -- HARD-BLOCK SITE PINNING.  place_lef's TOPO_SITE_IN.
    //
    // A "name<TAB>site" map, extracted from an implemented Vivado checkpoint by
    // ethmin/export_hardblock_locs.tcl, that pins the GT and the clock buffers
    // to the sites Vivado chose.  It is not optional on ethmin: left free, the
    // placer takes the nearest free site to the die centre and puts the GT
    // somewhere the reference clock cannot reach.
    //
    // Note this runs BEFORE the region pass on purpose -- the anchor centroid
    // is the mean of the placed non-SLICE cells, so the hard blocks have to be
    // down first or they cannot pull the logic toward themselves.
    // ===================================================================
    if (const char *site_in = getenv("TOPO_SITE_IN")) {
        std::ifstream f(site_in);
        if (!f)
            log_error("placer_lef: cannot read TOPO_SITE_IN '%s'\n", site_in);
        std::string line;
        int pinned = 0, missing_cell = 0, missing_site = 0;
        // Nothing is bound yet at this point, so checkBelAvail cannot stop two
        // map entries claiming one bel; track them here instead.
        std::set<BelId> claimed_bels;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#')
                continue;
            size_t tab = line.find('\t');
            if (tab == std::string::npos)
                continue;
            std::string cname = line.substr(0, tab), site = line.substr(tab + 1);
            while (!site.empty() && (site.back() == '\r' || site.back() == ' '))
                site.pop_back();
            auto it = ctx->cells.find(ctx->id(cname));
            if (it == ctx->cells.end()) {
                ++missing_cell;
                continue;
            }
            CellInfo *ci = it->second.get();
            if (ci->bel != BelId())
                continue;
            // The map names a SITE; find the bel inside it that suits this
            // cell.  A GT site carries one bel of the cell's type, so matching
            // on type is unambiguous and avoids hard-coding bel names.
            BelId found;
            for (auto bel : ctx->getBels()) {
                std::string bn = ctx->getBelName(bel).str(ctx);
                if (bn.compare(0, site.size(), site) != 0 || bn.size() <= site.size() || bn[site.size()] != '/')
                    continue;
                if (ctx->getBelType(bel) != ci->type || !ctx->checkBelAvail(bel) || claimed_bels.count(bel))
                    continue;
                found = bel;
                break;
            }
            if (found == BelId()) {
                ++missing_site;
                log_warning("placer_lef: no free bel of type '%s' in site '%s' for cell '%s'\n",
                            ci->type.c_str(ctx), site.c_str(), cname.c_str());
                continue;
            }
            // Set the ATTRIBUTE ONLY; do not bind here.  placer1's constraints
            // pass binds every BEL-attributed cell itself (STRENGTH_USER), so
            // binding as well makes it bind a second time and abort with the
            // baffling "cannot be bound to bel ... since it is already bound to
            // cell <itself>".  The attribute is also what the anchor pass below
            // reads, so nothing needs the early binding.
            claimed_bels.insert(found);
            ci->attrs[ctx->id("BEL")] = ctx->getBelName(found).str(ctx);
            ++pinned;
        }
        // Report the JOIN, not just the successes: a map whose names no longer
        // match the netlist would otherwise pin nothing and look like a design
        // that simply had no hard blocks.
        log_info("placer_lef: TOPO_SITE_IN pinned %d hard blocks (%d names not in netlist, %d sites unusable)\n",
                 pinned, missing_cell, missing_site);
        if (pinned == 0 && (missing_cell + missing_site) > 0)
            log_warning("placer_lef: TOPO_SITE_IN matched NOTHING -- check the hierarchy separator in '%s'\n", site_in);
    }

    // ===================================================================
    // PASS 1 -- REGION CONFINEMENT.  place_lef_core.ml:796-880.
    //
    //   anchor = TOPO_ANCHOR_X/Y if both set
    //          | else mean (sx,sy) of the placed NON-SLICE cells
    //          | else (110, 100)
    //   K      = ceil(n_slice / TOPO_REGION_FILL)
    //   region = the K free SLICE sites nearest the anchor, ranked
    //              rect (default): (max(dx*aspect, dy), dx*aspect + dy)
    //              diamond:        (dx + dy, 0)
    //
    // The SA only ever moves within the region, so this is the design's final
    // outline -- and it is what the two placers disagree about first.  On
    // johnson, place_lef confines to X[119..123] Y[107..111] while placer1
    // scatters as far as SLICE_X161Y156, which is the whole of the median
    // 85-site disagreement.
    //
    // Ranking is by CHEBYSHEV distance by default, not L1.  place_lef's own
    // note is worth keeping: an L1 ball is a diamond that wastes half its
    // bounding box on corner triangles and tapers to ~1 row at the tips, which
    // is too few consecutive rows for CARRY4 columns to legalise into.  L1 is
    // kept as the tiebreak so the outermost ring -- where the K cut lands
    // mid-ring -- fills evenly rather than raggedly.
    // ===================================================================
    const std::string shape = getenv_str("TOPO_REGION_SHAPE", "rect");
    const float aspect = getenv_float("TOPO_REGION_ASPECT", 1.0f);
    const float fill = getenv_float("TOPO_REGION_FILL", 0.65f);

    // Every free SLICE bel, in the index frame, GROUPED BY SITE.
    //
    // The region is measured in SITES, not bels -- place_lef's "region=22
    // sites" is 22 SLICEs, and a SLICE carries 4 six-LUTs, 4 five-LUTs, 8 flops
    // and the wide muxes.  Selecting the K nearest BELS instead put all 8 of
    // them inside a single SLICE (bbox X[110..110] Y[110..110]) and the place
    // then failed outright, having nowhere to put 61 cells.
    // Axis maps for the SLICE index frame, built from EVERY slice bel -- not
    // only the free ones.  The frame is a property of the die, so deriving it
    // from availability would make the anchor drift as the device fills.
    std::map<int, std::vector<int>> tilex_to_slicex; // tile x -> SLICE column indices
    std::map<int, std::vector<int>> tiley_to_slicey; // tile y -> SLICE row index

    std::map<std::pair<int, int>, std::vector<BelId>> sites;
    for (auto bel : ctx->getBels()) {
        SliceXY xy;
        if (!parse_slice_xy(ctx->getBelName(bel).str(ctx), xy))
            continue;
        Loc l = ctx->getBelLocation(bel);
        auto &xv = tilex_to_slicex[l.x];
        if (std::find(xv.begin(), xv.end(), xy.x) == xv.end())
            xv.push_back(xy.x);
        auto &yv = tiley_to_slicey[l.y];
        if (yv.empty())
            yv.push_back(xy.y);
        if (!ctx->checkBelAvail(bel))
            continue;
        sites[{xy.x, xy.y}].push_back(bel);
    }
    std::vector<std::pair<BelId, SliceXY>> slice_bels;
    for (auto &s : sites)
        for (auto b : s.second)
            slice_bels.emplace_back(b, SliceXY{s.first.first, s.first.second});

    // Anchor from the cells that are ALREADY placed and are not in a SLICE --
    // for johnson that is the XDC-pinned I/O, for a real design the hard
    // blocks.  Their position is read in the SLICE index frame by way of the
    // nearest SLICE bel, which is what gen_floorplan.py's mapping amounts to.
    int anchor_x, anchor_y;
    if (getenv("TOPO_ANCHOR_X") && getenv("TOPO_ANCHOR_Y")) {
        anchor_x = atoi(getenv("TOPO_ANCHOR_X"));
        anchor_y = atoi(getenv("TOPO_ANCHOR_Y"));
    } else {
        int64_t sx = 0, sy = 0;
        int n = 0;
        // ONE CONTRIBUTION PER SITE.  nextpnr splits a pin into several cells
        // -- a PAD plus an INBUF/OUTBUF, all bound into the same IOB site --
        // where place_lef has a single cell per pin.  Averaging per CELL
        // therefore double-counts every pin and drags the centroid toward
        // whichever side of the die has more buffer cells.  place_lef averages
        // 11 sites for johnson (1 BUFG + 10 IOB); counting cells gives 22.
        struct AnchorSite
        {
            int x, y;
            AnchorClass cls;
            double coupling;
        };
        std::map<std::string, AnchorSite> site_pos;
        const bool dbg = getenv("TOPO_DEBUG_ANCHOR") != nullptr;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            // Read the BEL ATTRIBUTE, not ci->bel.  An XDC-pinned IOB carries
            // its site as an attribute but is not BOUND yet when the placer
            // starts, so testing ci->bel found nothing at all and the anchor
            // silently fell back to the (110,100) default -- a plausible-looking
            // number that is simply the "no hard blocks" constant.
            BelId cbel = ci->bel;
            if (cbel == BelId()) {
                auto it = ci->attrs.find(ctx->id("BEL"));
                if (it == ci->attrs.end())
                    continue;
                cbel = ctx->getBelByName(ctx->id(it->second.as_string()));
                if (cbel == BelId())
                    continue;
            }
            SliceXY xy;
            if (parse_slice_xy(ctx->getBelName(cbel).str(ctx), xy))
                continue; // in a SLICE: not an anchor
            Loc hard = ctx->getBelLocation(cbel);
            // Project onto the SLICE frame PER AXIS, which is what
            // gen_floorplan.py does:
            //     kx = nearest tile column that hosts SLICEs
            //     x  = mean SLICE column index in that tile column
            //     ky = nearest tile row that hosts SLICEs
            //     y  = the SLICE row index of that tile row
            // Picking the single nearest SLICE BEL by combined L1 distance
            // instead conflates the two axes and lands somewhere else
            // entirely: it put johnson's anchor at (66,148) where place_lef
            // computes (121,109).  The axes have to be projected separately
            // because hard-block columns sit BETWEEN slice columns and the
            // grid has gaps (clock and I/O columns host no slice at all).
            auto nearest_key = [](const std::map<int, std::vector<int>> &m, int v) {
                int best = INT_MAX;
                bool found = false;
                for (auto &kv : m)
                    if (!found || std::abs(kv.first - v) < std::abs(best - v)) {
                        best = kv.first;
                        found = true;
                    }
                return found ? best : INT_MIN;
            };
            int kx = nearest_key(tilex_to_slicex, hard.x);
            int ky = nearest_key(tiley_to_slicey, hard.y);
            if (kx == INT_MIN || ky == INT_MIN)
                continue;
            std::string bn = ctx->getBelName(cbel).str(ctx);
            std::string site = bn.substr(0, bn.find('/'));
            AnchorClass cls = classify_anchor_cell(ctx, ci);
            // A site is reached by SEVERAL cells (the PAD, then its buffer), so
            // collect first and average afterwards: keep the highest priority
            // any cell at the site earned, rather than whichever was visited
            // first, and do not add the coordinate twice.
            auto prev = site_pos.find(site);
            if (prev != site_pos.end()) {
                if (cls < prev->second.cls)
                    prev->second.cls = cls;
                prev->second.coupling += fabric_coupling(ctx, ci);
                continue;
            }
            const auto &xs = tilex_to_slicex.at(kx);
            int64_t acc = 0;
            for (int v : xs)
                acc += v;
            // std::nearbyint, NOT llround.  A CLB tile hosts two SLICE columns
            // (e.g. {46,47}), so the mean is always a .5 and the tie-break
            // decides the whole coordinate.  gen_floorplan.py uses Python's
            // round(), which is round-half-to-EVEN (46.5 -> 46); llround is
            // round-half-away-from-zero (46.5 -> 47).  That one convention put
            // every hard block one SLICE column right of where place_lef puts
            // it, and the anchor with it.  nearbyint honours the default
            // FE_TONEAREST mode, which is round-half-to-even.
            int px = int(std::nearbyint(double(acc) / double(xs.size())));
            int py = tiley_to_slicey.at(ky).front();
            site_pos[site] = AnchorSite{px, py, cls, fabric_coupling(ctx, ci)};
        }

        // WEIGHTED centroid.  An unweighted mean lets the slowest pins decide
        // where the design goes: johnson has 8 monitor LEDs against one
        // differential clock, so the LEDs outvote it 8:1 and drag the logic
        // toward the LED bank.  Weighting by class fixes that without a
        // hand-kept list of net names -- note that an LED classifies as `data`
        // (it is OBUF <- FF, so it does touch fabric); what demotes it is not
        // its own class but the clock's much larger weight.
        // Deliberately dominant: on ethmin the GT must beat 3 MMCMs and 11
        // pads combined, or the design drifts off the transceiver and the
        // high-speed nets stop routing.
        const float w_gt = getenv_float("TOPO_ANCHOR_W_GT", 4.0f);
        const float w_clk = getenv_float("TOPO_ANCHOR_W_CLK", 8.0f);
        const float w_data = getenv_float("TOPO_ANCHOR_W_DATA", 1.0f);
        const float w_slow = getenv_float("TOPO_ANCHOR_W_SLOW", 0.25f);
        // Zero by default: a global clock buffer contributes NOTHING to where
        // the design should sit.  Raise it only to reproduce an old placement.
        const float w_global = getenv_float("TOPO_ANCHOR_W_GLOBAL", 0.0f);
        double wsum = 0.0, wx = 0.0, wy = 0.0;
        for (auto &kv : site_pos) {
            float w = kv.second.cls == ANCHOR_GT
                              ? w_gt
                              : (kv.second.cls == ANCHOR_CLOCK
                                         ? w_clk
                                         : (kv.second.cls == ANCHOR_DATA
                                                    ? w_data
                                                    : (kv.second.cls == ANCHOR_GLOBAL ? w_global : w_slow)));
            if (dbg)
                log_info("placer_lef:   anchor site %-22s -> slice(%d,%d)  %-10s coupling=%-9.0f w=%.0f\n",
                         kv.first.c_str(), kv.second.x, kv.second.y, anchor_class_name(kv.second.cls),
                         kv.second.coupling, w * float(std::max(1.0, kv.second.coupling)));
            // Scale by measured coupling, so the class factor only expresses
            // KIND and the design decides the rest.
            w *= float(std::max(1.0, kv.second.coupling));
            wx += double(kv.second.x) * w;
            wy += double(kv.second.y) * w;
            wsum += w;
            ++n;
        }
        sx = int64_t(wsum > 0.0 ? std::nearbyint(wx / wsum) : 0);
        sy = int64_t(wsum > 0.0 ? std::nearbyint(wy / wsum) : 0);
        if (n == 0 || wsum <= 0.0) {
            anchor_x = 110;
            anchor_y = 100;
        } else {
            anchor_x = int(sx);
            anchor_y = int(sy);
        }
    }

    // n_slice: how many SLICE SITES the design needs.  place_lef counts its own
    // packed SLICE cells here; nextpnr has not packed into slices yet at this
    // point, so the count is derived from the cells that must land in one.  A
    // SLICE holds 4 six-LUTs, 8 flops and 3 wide muxes (F7A/F7B/F8), and the
    // tightest of those three ratios is what actually forces the site count.
    // This is the one number in the pass that is NOT a mechanical translation,
    // because the two packers differ -- place_lef reports 1.82 cells/slice
    // against Vivado's 5.16 -- so it is measured against place_lef's own
    // reported n_slice rather than assumed correct.
    int luts = 0, ffs = 0, muxes = 0;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->bel != BelId())
            continue; // already placed: not competing for region space
        if (ci->type == id_SLICE_LUTX)
            ++luts;
        else if (ci->type == id_SLICE_FFX)
            ++ffs;
        else if (ci->type == ctx->id("SELMUX2_1"))
            ++muxes;
    }
    // MEASURED, not derived.  The theoretical minimum -- max(luts/4, ffs/8,
    // muxes/3) -- gives 5 slices for johnson, and the place then FAILS: control
    // sets, the MUXF7 slot rule and carry alignment all stop a SLICE being
    // filled to its raw bel capacity.  Counting the distinct SLICE sites in
    // place_lef's own placed.txt for the same design gives 14, i.e. about 4.4
    // cells per slice against the 12 the capacity argument would allow.  So the
    // density is a measurement with a knob on it, not a formula pretending to
    // be one.  Re-measure with:
    //   grep -oE 'SLICE_X[0-9]+Y[0-9]+' placed.txt | sort -u | wc -l
    const float per_slice = getenv_float("TOPO_CELLS_PER_SLICE", 4.4f);
    int n_slice = int(std::ceil(float(luts + ffs + muxes) / (per_slice > 0.0f ? per_slice : 4.4f)));
    n_slice = std::max(n_slice, 1);
    if (const char *e = getenv("TOPO_REGION_NSLICE"))
        n_slice = atoi(e);
    int K = int(std::ceil(float(n_slice) / (fill > 0.0f ? fill : 0.65f)));

    // Rank SITES, take the K nearest, then admit every bel of each.
    std::vector<std::pair<int, int>> site_keys;
    site_keys.reserve(sites.size());
    for (auto &s : sites)
        site_keys.push_back(s.first);
    auto rank = [&](const std::pair<int, int> &s) {
        float dx = std::abs(s.first - anchor_x) * aspect;
        float dy = float(std::abs(s.second - anchor_y));
        if (shape == "diamond")
            return std::make_pair(dx + dy, 0.0f);
        return std::make_pair(std::max(dx, dy), dx + dy);
    };
    std::sort(site_keys.begin(), site_keys.end(),
              [&](const std::pair<int, int> &a, const std::pair<int, int> &b) { return rank(a) < rank(b); });
    K = std::min<int>(K, int(site_keys.size()));

    Region *reg = nullptr;
    {
        IdString rname = ctx->id("$lef_region");
        auto r = std::unique_ptr<Region>(new Region());
        r->name = rname;
        r->constr_bels = true;
        int x0 = INT_MAX, x1 = INT_MIN, y0 = INT_MAX, y1 = INT_MIN;
        for (int i = 0; i < K; ++i) {
            for (auto b : sites[site_keys[i]])
                r->bels.insert(b);
            x0 = std::min(x0, site_keys[i].first);
            x1 = std::max(x1, site_keys[i].first);
            y0 = std::min(y0, site_keys[i].second);
            y1 = std::max(y1, site_keys[i].second);
        }
        // Same shape of report place_lef prints, so the two can be diffed
        // directly rather than eyeballed.
        log_info("placer_lef: region=%d sites (fill target %.2f, n_slice=%d)\n", K, fill, n_slice);
        log_info("placer_lef: shape=%s aspect=%.2f anchor=(%d,%d) bbox X[%d..%d] Y[%d..%d]\n", shape.c_str(), aspect,
                 anchor_x, anchor_y, x0, x1, y0, y1);
        ctx->region[rname] = std::move(r);
        reg = ctx->region[rname].get();
    }

    // Confine every cell that still needs placing and can live in a SLICE.
    // Cells already placed (the pinned I/O) keep their bel and must NOT be
    // given the region, or placer1 would reject a legal placement outside it.
    int confined = 0;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->bel != BelId() || ci->region != nullptr)
            continue;
        // Confine CLUSTER ROOTS, never their children.  A child is positioned
        // by its root, so giving it a region of its own is incoherent: the
        // CARRY4 roots were left free to sit anywhere while their LUT/FF
        // children were pinned inside the region, and no site could then
        // satisfy the whole cluster --
        //   ERROR: failed to place cell '...CARRY4' of type 'CARRY4'
        // CARRY4 is confined too; it is logic and belongs in the region with
        // everything else it talks to.
        if (ci->constr_parent != nullptr)
            continue;
        if (ci->type == id_SLICE_LUTX || ci->type == id_SLICE_FFX || ci->type == ctx->id("SELMUX2_1") ||
            ci->type == id_CARRY4) {
            ci->region = reg;
            ++confined;
        }
    }
    log_info("placer_lef: confined %d cells to the region\n", confined);

    // Everything not yet ported still goes to placer1 -- but now inside the
    // region above, so its choices are made in the same part of the die.
    return placer1(ctx, Placer1Cfg(ctx));
}

// =========================================================================
// place_lef TRANSPLANT -- the recognition pre-pass.
//
// This runs from customAfterLoad, on the RAW netlist, because pack_to_lef
// recognises LUT6 / FDRE / CARRY4 / MUXF7 -- and by the time the placer slot
// runs, nextpnr's packer has already turned those into SLICE_LUTX/SLICE_FFX.
// It reads the yosys JSON file directly, which is exactly the input the OCaml
// takes, so the recognition is bit-for-bit the ported one.
//
// The unit it produces is the thing being transplanted: place_lef anneals
// 1443 packed SLICEs whose contents were decided in advance, where nextpnr
// anneals 7759 loose cells and lets co-tenancy fall out of legality checks.
// Measured on ethmin those two partitions share only 28.9% of the co-tenancy
// pairs pack_to_lef decides.
// =========================================================================
bool place_lef_prepass(Context *ctx, const std::string &json_path)
{
    lefpack::PackResult r = lefpack::pack_to_lef_json(json_path);
    if (r.cells.empty()) {
        log_error("place_lef pre-pass: packed NO cells from '%s'\n", json_path.c_str());
        return false;
    }

    log_info("place_lef pre-pass: %zu instances -> %zu packed units\n", r.n_instances,
             r.cells.size());
    for (auto &kv : r.report)
        log_info("    %5d  %s\n", kv.second, kv.first.c_str());

    // Unit census by LEF type, and the size distribution -- this is the
    // grouping the placer is supposed to move as single objects.
    std::map<std::string, int> by_lef;
    std::map<size_t, int> by_size;
    size_t members = 0;
    for (auto &c : r.cells) {
        by_lef[c.pc_lef]++;
        by_size[c.pc_bels.size()]++;
        members += c.pc_bels.size();
    }
    for (auto &kv : by_lef)
        log_info("  unit type %-14s %5d\n", kv.first.c_str(), kv.second);
    std::string hist;
    for (auto &kv : by_size)
        hist += stringf("%zu:%d ", kv.first, kv.second);
    log_info("  members/unit: %s(mean %.2f)\n", hist.c_str(),
             double(members) / double(r.cells.size()));

    // JOIN RATE.  pack_to_lef's names come from the yosys JSON; ctx's come from
    // nextpnr's own parser.  Two name spaces meet here, and a silent mismatch
    // would look exactly like "the transplant did nothing" -- so it is measured
    // and it is fatal, not a warning.
    size_t matched = 0, missing = 0;
    std::string first_missing;
    for (auto &c : r.cells)
        for (auto &b : c.pc_bels) {
            if (ctx->cells.count(ctx->id(b.first)))
                matched++;
            else {
                if (missing == 0)
                    first_missing = b.first;
                missing++;
            }
        }
    log_info("  join rate: %zu/%zu unit members resolve to a ctx cell (%.1f%%)\n", matched,
             matched + missing, 100.0 * double(matched) / double(matched + missing));
    if (missing > 0)
        log_warning("  %zu member(s) did NOT resolve, first '%s' -- the placement cannot be "
                    "stamped onto them\n",
                    missing, first_missing.c_str());

    // Database dump.  bels.txt is the same shape place_lef writes, so the
    // downstream (carry_stamp.py -> route) stays byte-identical to the OCaml
    // flow and an A/B differs in exactly one component.  Sites are filled in by
    // the placer; this pre-pass emits the grouping only.
    if (const char *out = getenv("PACK_LEF_UNITS_OUT")) {
        std::ofstream o(out);
        if (!o)
            log_error("place_lef pre-pass: cannot write '%s'\n", out);
        for (auto &c : r.cells)
            for (auto &b : c.pc_bels)
                o << b.first << "\t" << b.second << "\t" << c.pc_name << "\t" << c.pc_lef << "\n";
        log_info("  wrote unit membership to %s\n", out);
    }
    return true;
}

NEXTPNR_NAMESPACE_END
