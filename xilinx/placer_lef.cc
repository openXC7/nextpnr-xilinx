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
#include "ocaml_random.h"
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
// A placement site, in place_lef's terms, built from nextpnr's chipdb instead
// of gen_floorplan.py's floorplan.json.  This is the interface adjustment that
// makes the transplant small: load_floorplan, the kind taxonomy and
// TOPO_SITE_PHYSMAP all stop being things that have to be ported.
struct LefSite
{
    std::string name;
    int sx = 0, sy = 0;
    bool sm = false;   // SLICEM: can host distributed RAM / SRL
    bool used = false;
    std::string kind;  // place_lef's floorplan kind: SLICE, BRAM, DSP, IO, ...
    BelId rep;         // representative bel (the A6LUT for a SLICE)
};

// Site NAME -> place_lef floorplan kind.  Mirrors kind_of_lef's codomain; the
// names come from prjxray, the same source gen_floorplan.py reads, so the two
// taxonomies agree by construction.
static std::string site_kind_of_name(const std::string &s)
{
    auto pre = [&](const char *p) { return s.compare(0, strlen(p), p) == 0; };
    if (pre("SLICE_"))
        return "SLICE";
    if (pre("RAMB36_"))
        return "BRAM";
    if (pre("RAMB18_") || pre("FIFO18_"))
        return "BRAM18";
    if (pre("DSP48_"))
        return "DSP";
    if (pre("BUFGCTRL_"))
        return "BUFG";
    if (pre("BUFHCE_"))
        return "BUFH";
    if (pre("MMCME2_") || pre("PLLE2_"))
        return "MMCM";
    if (pre("GTXE2_CHANNEL_") || pre("GTHE2_CHANNEL_"))
        return "GT_CHANNEL";
    if (pre("GTXE2_COMMON_") || pre("GTHE2_COMMON_"))
        return "GT_COMMON";
    if (pre("IOB_") || pre("IOB18_") || pre("IOB33_"))
        return "IO";
    return "";
}

// Enumerate every site in the chipdb once.
static std::vector<LefSite> build_sites(Context *ctx)
{
    std::map<std::string, size_t> idx;
    std::vector<LefSite> sites;
    for (BelId bel : ctx->getBels()) {
        std::string bn = ctx->getBelName(bel).str(ctx);
        size_t sl = bn.rfind('/');
        if (sl == std::string::npos)
            continue;
        std::string sname = bn.substr(0, sl), suffix = bn.substr(sl + 1);
        auto it = idx.find(sname);
        if (it == idx.end()) {
            LefSite s;
            s.name = sname;
            s.kind = site_kind_of_name(sname);
            SliceXY xy;
            if (parse_slice_xy(bn, xy)) {
                s.sx = xy.x;
                s.sy = xy.y;
            }
            idx[sname] = sites.size();
            sites.push_back(s);
            it = idx.find(sname);
        }
        LefSite &s = sites[it->second];
        // SLICEM is the site that can host distributed RAM: its LUT bel has a
        // real CLK pin.  Same test arch_place.cc uses to keep DRAM off SLICEL.
        if (suffix == "A6LUT") {
            s.rep = bel;
            if (ctx->getBelPinWire(bel, ctx->id("CLK")) != WireId())
                s.sm = true;
        }
        if (s.rep == BelId())
            s.rep = bel;
    }

    // Hard-block site indices are their OWN scale -- RAMB36_X11Y68 is nowhere
    // near SLICE_X11Y68 -- and only SLICE names carry the frame the placer
    // ranks in.  So map every non-SLICE site into the SLICE index frame
    // through its TILE location, which is what gen_floorplan.py does.  Left
    // unmapped they all land at (0,0), the anchor collapses onto the die
    // corner and the whole region goes with it.
    std::map<int, int> tilex2sx, tiley2sy;
    for (auto &s : sites) {
        if (s.kind != "SLICE" || s.rep == BelId())
            continue;
        Loc l = ctx->getBelLocation(s.rep);
        auto ix = tilex2sx.find(l.x);
        if (ix == tilex2sx.end() || s.sx < ix->second)
            tilex2sx[l.x] = s.sx; // two slices per CLB tile: take the lower
        auto iy = tiley2sy.find(l.y);
        if (iy == tiley2sy.end() || s.sy < iy->second)
            tiley2sy[l.y] = s.sy;
    }
    auto nearest = [](const std::map<int, int> &m, int k) {
        if (m.empty())
            return 0;
        auto it = m.lower_bound(k);
        if (it == m.end())
            return std::prev(it)->second;
        if (it == m.begin())
            return it->second;
        auto pv = std::prev(it);
        return (k - pv->first <= it->first - k) ? pv->second : it->second;
    };
    for (auto &s : sites) {
        if (s.kind == "SLICE" || s.rep == BelId())
            continue;
        Loc l = ctx->getBelLocation(s.rep);
        s.sx = nearest(tilex2sx, l.x);
        s.sy = nearest(tiley2sy, l.y);
    }
    return sites;
}

// LEF cell -> floorplan site kind (place_lef_core.ml kind_of_lef, verbatim,
// including its `_ -> "SLICE"` default).
static std::string kind_of_lef(const std::string &lef)
{
    if (lef.compare(0, 5, "SLICE") == 0)
        return "SLICE";
    if (lef == "RAMB36")
        return "BRAM";
    if (lef == "RAMB18")
        return "BRAM18";
    if (lef == "DSP48")
        return "DSP";
    if (lef == "IOB")
        return "IO";
    if (lef == "BUFG")
        return "BUFG";
    if (lef == "BUFH")
        return "BUFH";
    if (lef == "MMCM")
        return "MMCM";
    if (lef == "GT_CHANNEL")
        return "GT_CHANNEL";
    if (lef == "GT_COMMON")
        return "GT_COMMON";
    return "SLICE";
}

// place_lef's group_key: cut the name at ".genblk", ".ram_reg" or "[<digit>",
// so BRAMs of one macro share high-fanout control and land contiguously.
static std::string group_key(const std::string &n)
{
    size_t best = std::string::npos;
    for (const char *pat : {".genblk", ".ram_reg"}) {
        size_t p = n.find(pat);
        if (p != std::string::npos && p < best)
            best = p;
    }
    for (size_t i = 0; i + 1 < n.size(); i++)
        if (n[i] == '[' && isdigit((unsigned char)n[i + 1])) {
            if (i < best)
                best = i;
            break;
        }
    return best == std::string::npos ? n : n.substr(0, best);
}

// =========================================================================
// The placement itself, transplanted from place_lef_core.ml run_gen.
//
// Phase A  dedicated / hard blocks -> greedy nearest, become fixed anchors
// anchor   mean of placed hard blocks (or TOPO_ANCHOR_X/Y)
// region   the K free SLICE sites nearest the anchor, K = n_slice / fill
// Phase B  carry chains -> vertical runs within a column (rigid)
// Phase C  constructive placement, SLICEM-needing units first
// Phase D  simulated annealing over movable units
//
// The unit is the object throughout: a move relocates a whole packed SLICE,
// never an individual LUT or FF.  That is the entire point of the transplant.
// =========================================================================
static bool place_lef_place(Context *ctx, lefpack::PackResult &r, std::vector<LefSite> &sites)
{
    const size_t ncells = r.cells.size();
    const double fill = getenv_float("TOPO_REGION_FILL", 0.65f);
    const int seed = int(getenv_float("TOPO_SEED", 1.0f));
    ocaml_random::State rng;
    rng.init(seed);

    std::unordered_map<std::string, size_t> name2id;
    for (size_t i = 0; i < ncells; i++)
        name2id[r.cells[i].pc_name] = i;

    // ---- per-cell state -------------------------------------------------
    std::vector<int> pos_x(ncells, 0), pos_y(ncells, 0), cell_site(ncells, -1);
    std::vector<char> movable(ncells, 0), need_sm(ncells, 0), skip(ncells, 0);
    for (size_t i = 0; i < ncells; i++) {
        const std::string &lef = r.cells[i].pc_lef;
        // UNKNOWN: units (yosys $scopeinfo) carry no bels and have nothing to
        // place; kind_of_lef would call them SLICE and burn a site each.
        if (lef.compare(0, 8, "UNKNOWN:") == 0) {
            skip[i] = 1;
            continue;
        }
        movable[i] = (kind_of_lef(lef) == "SLICE" && lef != "SLICE_CARRY");
        need_sm[i] = (lef.compare(0, 6, "SLICEM") == 0);
    }

    std::unordered_map<std::string, int> occ; // site name -> cell
    auto fits = [&](size_t i, const LefSite &s) { return need_sm[i] ? s.sm : true; };
    auto bind = [&](size_t i, LefSite &s) {
        s.used = true;
        cell_site[i] = int(&s - sites.data());
        pos_x[i] = s.sx;
        pos_y[i] = s.sy;
        occ[s.name] = int(i);
    };
    auto is_placed = [&](size_t i) { return cell_site[i] >= 0; };

    // ---- nets (exclude clock: nets driven by BUFG/MMCM output) -----------
    std::set<int> clk_nets;
    for (auto &c : r.cells)
        if (c.pc_lef == "BUFG" || c.pc_lef == "MMCM")
            for (auto &pc : c.pc_conns)
                if ((pc.first == "O" || pc.first.compare(0, 6, "CLKOUT") == 0) &&
                    lefpack::is_net(pc.second))
                    clk_nets.insert(pc.second);
    std::unordered_map<int, int> net_of_key;
    std::vector<std::vector<int>> net_cells;
    for (size_t i = 0; i < ncells; i++)
        for (auto &pc : r.cells[i].pc_conns) {
            if (!lefpack::is_net(pc.second) || clk_nets.count(pc.second))
                continue;
            auto it = net_of_key.find(pc.second);
            int nid;
            if (it == net_of_key.end()) {
                nid = int(net_cells.size());
                net_of_key[pc.second] = nid;
                net_cells.emplace_back();
            } else
                nid = it->second;
            auto &l = net_cells[nid];
            if (std::find(l.begin(), l.end(), int(i)) == l.end())
                l.push_back(int(i));
        }
    const size_t nnets = net_cells.size();
    std::vector<std::vector<int>> cell_nets(ncells);
    for (size_t n = 0; n < nnets; n++)
        for (int i : net_cells[n])
            cell_nets[i].push_back(int(n));
    std::vector<double> net_w(nnets, 1.0);
    log_info("  nets: %zu (excluding %zu clock net(s))\n", nnets, clk_nets.size());

    auto net_hpwl = [&](int nid) {
        int mnx = INT_MAX, mxx = INT_MIN, mny = INT_MAX, mxy = INT_MIN, cnt = 0;
        for (int i : net_cells[nid])
            if (is_placed(i)) {
                cnt++;
                mnx = std::min(mnx, pos_x[i]);
                mxx = std::max(mxx, pos_x[i]);
                mny = std::min(mny, pos_y[i]);
                mxy = std::max(mxy, pos_y[i]);
            }
        return cnt >= 2 ? (mxx - mnx) + (mxy - mny) : 0;
    };

    // ---- Phase A: hard blocks -------------------------------------------
    // Greedy nearest free site of the right kind, each group's first member
    // near (110,100) and each subsequent one near the PREVIOUS member, so a
    // BRAM group's broadcast control nets stay short.
    auto nearest_free_of = [&](const std::string &kind, int tx, int ty) -> LefSite * {
        LefSite *best = nullptr;
        int bd = INT_MAX;
        for (auto &s : sites) {
            if (s.used || s.kind != kind)
                continue;
            int d = std::abs(s.sx - tx) + std::abs(s.sy - ty);
            if (d < bd) {
                bd = d;
                best = &s;
            }
        }
        return best;
    };
    std::vector<size_t> hard;
    for (size_t i = 0; i < ncells; i++)
        if (!skip[i] && kind_of_lef(r.cells[i].pc_lef) != "SLICE")
            hard.push_back(i);
    std::stable_sort(hard.begin(), hard.end(), [&](size_t a, size_t b) {
        auto ka = std::make_pair(kind_of_lef(r.cells[a].pc_lef), group_key(r.cells[a].pc_name));
        auto kb = std::make_pair(kind_of_lef(r.cells[b].pc_lef), group_key(r.cells[b].pc_name));
        return ka < kb;
    });
    std::map<std::string, std::pair<int, int>> last_pos;
    int nhard = 0, nhard_fail = 0;
    for (size_t i : hard) {
        std::string kind = kind_of_lef(r.cells[i].pc_lef);
        std::string gk = kind + ":" + group_key(r.cells[i].pc_name);
        auto lp = last_pos.find(gk);
        int tx = lp == last_pos.end() ? 110 : lp->second.first;
        int ty = lp == last_pos.end() ? 100 : lp->second.second;
        LefSite *s = nearest_free_of(kind, tx, ty);
        if (s) {
            bind(i, *s);
            last_pos[gk] = {s->sx, s->sy};
            nhard++;
        } else {
            log_warning("  no free %s site for %s\n", kind.c_str(), r.cells[i].pc_name.c_str());
            nhard_fail++;
        }
    }
    log_info("  phase A: %d hard block(s) placed, %d unplaceable\n", nhard, nhard_fail);

    // ---- anchor ---------------------------------------------------------
    // NOTE: place_lef invents its own IO sites (it ignores PACKAGE_PIN), so its
    // anchor is computed from fictitious pin positions.  Here the IO sites are
    // the chipdb's real ones, so the derived anchor legitimately differs.  It
    // is logged, and TOPO_ANCHOR_X/Y still overrides.
    int anchor_cx, anchor_cy;
    const char *ax = getenv("TOPO_ANCHOR_X"), *ay = getenv("TOPO_ANCHOR_Y");
    if (ax && ay) {
        anchor_cx = atoi(ax);
        anchor_cy = atoi(ay);
        log_info("  anchor: (%d,%d) from TOPO_ANCHOR_X/Y\n", anchor_cx, anchor_cy);
    } else {
        long sx = 0, sy = 0;
        int n = 0;
        for (size_t i = 0; i < ncells; i++)
            if (!skip[i] && kind_of_lef(r.cells[i].pc_lef) != "SLICE" && is_placed(i)) {
                sx += pos_x[i];
                sy += pos_y[i];
                n++;
            }
        anchor_cx = n == 0 ? 110 : int(sx / n);
        anchor_cy = n == 0 ? 100 : int(sy / n);
        log_info("  anchor: (%d,%d) = mean of %d placed hard block(s)\n", anchor_cx, anchor_cy, n);
    }

    // ---- region: the K free SLICE sites nearest the anchor ---------------
    // Rectangle (Chebyshev) by default with L1 as tiebreak; the diamond an L1
    // ranking gives wastes half its bounding box and tapers to ~1 row/column at
    // the tips, which is too few consecutive rows for the CARRY4 columns.
    int n_slice = 0;
    for (size_t i = 0; i < ncells; i++)
        if (!skip[i] && kind_of_lef(r.cells[i].pc_lef) == "SLICE")
            n_slice++;
    const std::string region_shape = getenv_str("TOPO_REGION_SHAPE", "rect");
    const double region_aspect = getenv_float("TOPO_REGION_ASPECT", 1.0f);
    auto region_rank = [&](const LefSite &s) {
        double dx = std::abs(s.sx - anchor_cx) * region_aspect;
        double dy = double(std::abs(s.sy - anchor_cy));
        return region_shape == "diamond" ? std::make_pair(dx + dy, 0.0)
                                         : std::make_pair(std::max(dx, dy), dx + dy);
    };
    std::vector<LefSite *> region;
    for (auto &s : sites)
        if (s.kind == "SLICE" && !s.used)
            region.push_back(&s);
    std::stable_sort(region.begin(), region.end(),
                     [&](LefSite *a, LefSite *b) { return region_rank(*a) < region_rank(*b); });
    size_t k = size_t(std::ceil(double(n_slice) / fill));
    if (k < region.size())
        region.resize(k);
    if (region.empty()) {
        log_error("place_lef: region is EMPTY -- no free SLICE sites\n");
        return false;
    }
    int rx0 = INT_MAX, rx1 = INT_MIN, ry0 = INT_MAX, ry1 = INT_MIN;
    long rcx = 0, rcy = 0;
    for (auto *s : region) {
        rx0 = std::min(rx0, s->sx);
        rx1 = std::max(rx1, s->sx);
        ry0 = std::min(ry0, s->sy);
        ry1 = std::max(ry1, s->sy);
        rcx += s->sx;
        rcy += s->sy;
    }
    const int region_cx = int(rcx / long(region.size())), region_cy = int(rcy / long(region.size()));
    log_info("  region: %zu sites for %d SLICE units (fill %.2f), bbox X%d..%d Y%d..%d\n",
             region.size(), n_slice, fill, rx0, rx1, ry0, ry1);

    // ---- Phase B: carry chains -> vertical runs --------------------------
    // A CARRY4's COUT->CIN is a dedicated point-to-point wire reaching only the
    // slice directly above, so a chain is rigid and must occupy consecutive
    // rows of ONE column.
    std::map<int, size_t> co2cell; // CO netkey -> unit
    for (size_t i = 0; i < ncells; i++)
        if (r.cells[i].pc_lef == "SLICE_CARRY")
            for (auto &pc : r.cells[i].pc_conns)
                if (pc.first == "CO") {
                    co2cell[pc.second] = i;
                    break;
                }
    auto conn_of = [&](size_t i, const char *pin) {
        for (auto &pc : r.cells[i].pc_conns)
            if (pc.first == pin)
                return pc.second;
        return int(lefpack::NK_NONE);
    };
    std::vector<size_t> carry_cells;
    for (size_t i = 0; i < ncells; i++)
        if (r.cells[i].pc_lef == "SLICE_CARRY")
            carry_cells.push_back(i);
    std::map<int, size_t> ci2cell;
    for (size_t i : carry_cells) {
        int ci = conn_of(i, "CI");
        if (ci != lefpack::NK_NONE && !ci2cell.count(ci))
            ci2cell[ci] = i;
    }
    std::vector<std::vector<size_t>> chains;
    for (size_t i : carry_cells) {
        int ci = conn_of(i, "CI");
        bool root = (ci == lefpack::NK_NONE) || !co2cell.count(ci);
        if (!root)
            continue;
        std::vector<size_t> ch{i};
        std::set<size_t> guard{i};
        size_t cur = i;
        for (;;) {
            int co = conn_of(cur, "CO");
            if (co == lefpack::NK_NONE)
                break;
            auto it = ci2cell.find(co);
            if (it == ci2cell.end() || guard.count(it->second))
                break;
            cur = it->second;
            guard.insert(cur);
            ch.push_back(cur);
        }
        chains.push_back(std::move(ch));
    }
    // region sites by column, ascending Y, for the run search
    std::map<int, std::vector<LefSite *>> slice_by_col;
    for (auto *s : region)
        slice_by_col[s->sx].push_back(s);
    for (auto &kv : slice_by_col)
        std::stable_sort(kv.second.begin(), kv.second.end(),
                         [](LefSite *a, LefSite *b) { return a->sy < b->sy; });
    auto run_in_col = [&](std::vector<LefSite *> &arr, size_t len) -> std::vector<LefSite *> {
        for (size_t i = 0; i + len <= arr.size(); i++) {
            bool ok = true;
            for (size_t j = 0; j < len && ok; j++) {
                if (arr[i + j]->used)
                    ok = false;
                if (j > 0 && arr[i + j]->sy != arr[i + j - 1]->sy + 1)
                    ok = false;
            }
            if (ok)
                return std::vector<LefSite *>(arr.begin() + i, arr.begin() + i + len);
        }
        return {};
    };
    // TOPO_CARRY_SPREAD: fill columns in X order up to a density cap, so
    // carries stay in a contiguous band without saturating any column's DMUX
    // switchbox.  Maximal spread cleared congestion but scattered carries over
    // ~70 columns and the 125 MHz datapath fell to ~27 MHz.
    const char *cs_env = getenv("TOPO_CARRY_SPREAD");
    const bool carry_spread = cs_env && strcmp(cs_env, "0") != 0 && cs_env[0] != '\0';
    const int carry_max_per_col = int(getenv_float("TOPO_CARRY_MAX_PER_COL", 32.0f));
    auto col_used = [&](std::vector<LefSite *> &arr) {
        int n = 0;
        for (auto *s : arr)
            if (s->used)
                n++;
        return n;
    };
    int nchain = 0, nchain_fail = 0, ncarry_placed = 0;
    for (auto &ch : chains) {
        std::vector<LefSite *> run;
        if (!carry_spread) {
            for (auto &kv : slice_by_col) {
                run = run_in_col(kv.second, ch.size());
                if (!run.empty())
                    break;
            }
        } else {
            for (auto &kv : slice_by_col) {
                if (col_used(kv.second) >= carry_max_per_col)
                    continue;
                run = run_in_col(kv.second, ch.size());
                if (!run.empty())
                    break;
            }
            if (run.empty()) { // every capped column full -> least-used column
                int best_used = INT_MAX;
                for (auto &kv : slice_by_col) {
                    int uc = col_used(kv.second);
                    if (uc >= best_used)
                        continue;
                    auto cand = run_in_col(kv.second, ch.size());
                    if (!cand.empty()) {
                        run = cand;
                        best_used = uc;
                    }
                }
            }
        }
        if (run.empty()) {
            log_warning("  no free carry column for chain of %zu\n", ch.size());
            nchain_fail++;
            continue;
        }
        for (size_t j = 0; j < ch.size(); j++) {
            bind(ch[j], *run[j]);
            ncarry_placed++;
        }
        nchain++;
    }
    log_info("  phase B: %d carry chain(s) placed (%d slices), %d unplaceable%s\n", nchain,
             ncarry_placed, nchain_fail, carry_spread ? ", spread on" : "");

    // ---- Phase C: constructive placement ---------------------------------
    auto centroid = [&](size_t i, int &tx, int &ty) {
        long sx = 0, sy = 0;
        int n = 0;
        for (int nid : cell_nets[i])
            for (int j : net_cells[nid])
                if (size_t(j) != i && is_placed(j)) {
                    sx += pos_x[j];
                    sy += pos_y[j];
                    n++;
                }
        tx = n == 0 ? region_cx : int(sx / n);
        ty = n == 0 ? region_cy : int(sy / n);
    };
    auto nearest_region_free = [&](size_t i, int tx, int ty) -> LefSite * {
        LefSite *best = nullptr;
        int bd = INT_MAX;
        for (auto *s : region) {
            if (s->used || !fits(i, *s))
                continue;
            int d = std::abs(s->sx - tx) + std::abs(s->sy - ty);
            if (d < bd) {
                bd = d;
                best = s;
            }
        }
        return best;
    };
    int nconstr = 0, nconstr_fail = 0;
    for (int want_sm = 1; want_sm >= 0; want_sm--) // SLICEM-needing units first
        for (size_t i = 0; i < ncells; i++) {
            if (skip[i] || kind_of_lef(r.cells[i].pc_lef) != "SLICE" || is_placed(i))
                continue;
            if (int(need_sm[i]) != want_sm)
                continue;
            int tx, ty;
            centroid(i, tx, ty);
            LefSite *s = nearest_region_free(i, tx, ty);
            if (s) {
                bind(i, *s);
                nconstr++;
            } else {
                log_warning("  no free region %s for %s\n", want_sm ? "SLICEM" : "SLICE",
                            r.cells[i].pc_name.c_str());
                nconstr_fail++;
            }
        }
    log_info("  phase C: %d unit(s) placed constructively, %d unplaceable\n", nconstr,
             nconstr_fail);

    // ---- Phase D: simulated annealing over movable units ------------------
    std::vector<int> mv;
    // Built by Array.iteri + prepend with NO List.rev, so the OCaml's move list
    // is in DESCENDING index order; the RNG draws from it by index, so the
    // order is part of the reproduction.
    for (size_t i = 0; i < ncells; i++)
        if (movable[i] && is_placed(i))
            mv.push_back(int(i));
    std::reverse(mv.begin(), mv.end());
    const int m = int(mv.size());
    if (m > 0) {
        const double cong_w = getenv_float("TOPO_CONG_W", 0.0f);
        const double coh_w = getenv_float("TOPO_COH_W", 0.0f);
        const bool cong_on = cong_w > 0.0;
        const bool coh_on = coh_w > 0.0;
        long moves = long(getenv_float("TOPO_SA_MOVES", 0.0f));
        if (moves <= 0)
            moves = std::max(200000L, 150L * m);
        const double ctemp = cong_on ? std::max(1.0, cong_w) : 1.0;
        const double t0 = getenv_float("TOPO_SA_T0", float(8.0 * ctemp));
        const double tend = getenv_float("TOPO_SA_TEND", float(0.05 * ctemp));
        const double alpha = std::pow(tend / t0, 1.0 / double(std::max(1L, moves)));
        double t = t0;

        // module cohesion: pull same-parent-module units to their centroid
        std::vector<int> mod_of(ncells, -1);
        std::vector<long> msx, msy;
        std::vector<int> mcnt;
        if (coh_on) {
            const int coh_depth = int(getenv_float("TOPO_COH_DEPTH", 0.0f));
            std::map<std::string, int> mod_ids;
            for (size_t i = 0; i < ncells; i++) {
                const std::string &n = r.cells[i].pc_name;
                std::string key;
                size_t nsep = 0;
                for (char c : n)
                    if (c == '.' || c == '/')
                        nsep++;
                if (coh_depth <= 0 || nsep <= size_t(coh_depth)) {
                    size_t cut = n.find_last_of("./");
                    key = (cut != std::string::npos && cut > 0) ? n.substr(0, cut) : n;
                } else {
                    size_t seen = 0, cut = n.size();
                    for (size_t j = 0; j < n.size(); j++)
                        if (n[j] == '.' || n[j] == '/') {
                            if (++seen >= size_t(coh_depth)) {
                                cut = j;
                                break;
                            }
                        }
                    key = n.substr(0, cut);
                }
                auto it = mod_ids.find(key);
                if (it == mod_ids.end()) {
                    int id = int(mod_ids.size());
                    mod_ids[key] = id;
                    mod_of[i] = id;
                } else
                    mod_of[i] = it->second;
            }
            msx.assign(mod_ids.size(), 0);
            msy.assign(mod_ids.size(), 0);
            mcnt.assign(mod_ids.size(), 0);
            for (size_t i = 0; i < ncells; i++)
                if (is_placed(i) && mod_of[i] >= 0) {
                    msx[mod_of[i]] += pos_x[i];
                    msy[mod_of[i]] += pos_y[i];
                    mcnt[mod_of[i]]++;
                }
            log_info("  cohesion: %zu modules, w=%.1f\n", mod_ids.size(), coh_w);
        }

        std::vector<int> stamp(nnets, 0);
        int stamp_ctr = 0;
        long accepted = 0;
        const long prog_every = std::max(1L, moves / 10);
        for (long mvno = 1; mvno <= moves; mvno++) {
            if (mvno % prog_every == 0)
                log_info("    SA %3ld%%  moves=%ld/%ld  accepted=%ld  temp=%.3f\n",
                         100 * mvno / moves, mvno, moves, accepted, t);
            int i = mv[rng.int_(m)];
            LefSite *s = region[rng.int_(int(region.size()))];
            LefSite &si = sites[cell_site[i]];
            if (s->name != si.name && fits(i, *s)) {
                auto oit = occ.find(s->name);
                int j = oit == occ.end() ? -1 : oit->second;
                if (j == -1 || (movable[j] && fits(j, si))) {
                    std::vector<int> moved;
                    std::vector<std::pair<int, int>> newpos, olds;
                    moved.push_back(i);
                    newpos.push_back({s->sx, s->sy});
                    if (j != -1) {
                        moved.push_back(j);
                        newpos.push_back({si.sx, si.sy});
                    }
                    // delta over the union of the affected units' nets
                    stamp_ctr++;
                    std::vector<int> nets;
                    for (int c : moved)
                        for (int nid : cell_nets[c])
                            if (stamp[nid] != stamp_ctr) {
                                stamp[nid] = stamp_ctr;
                                nets.push_back(nid);
                            }
                    double before = 0;
                    for (int nid : nets)
                        before += net_w[nid] * net_hpwl(nid);
                    for (int c : moved)
                        olds.push_back({pos_x[c], pos_y[c]});
                    for (size_t q = 0; q < moved.size(); q++) {
                        pos_x[moved[q]] = newpos[q].first;
                        pos_y[moved[q]] = newpos[q].second;
                    }
                    double after = 0;
                    for (int nid : nets)
                        after += net_w[nid] * net_hpwl(nid);
                    double dcoh = 0;
                    if (coh_on)
                        for (size_t q = 0; q < moved.size(); q++) {
                            int c = moved[q], mo = mod_of[c];
                            if (mo < 0 || mcnt[mo] <= 1)
                                continue;
                            int cx = int(msx[mo] / mcnt[mo]), cy = int(msy[mo] / mcnt[mo]);
                            dcoh += (std::abs(pos_x[c] - cx) + std::abs(pos_y[c] - cy)) -
                                    (std::abs(olds[q].first - cx) + std::abs(olds[q].second - cy));
                        }
                    double delta = (after - before) + coh_w * dcoh;
                    // `||` short-circuits, so a downhill move consumes NO random
                    // number -- getting this wrong desynchronises the sequence.
                    bool accept = delta <= 0.0 || rng.float_(1.0) < std::exp(-delta / t);
                    if (accept) {
                        accepted++;
                        if (coh_on)
                            for (size_t q = 0; q < moved.size(); q++) {
                                int c = moved[q], mo = mod_of[c];
                                if (mo < 0)
                                    continue;
                                msx[mo] += pos_x[c] - olds[q].first;
                                msy[mo] += pos_y[c] - olds[q].second;
                            }
                        if (j == -1) {
                            si.used = false;
                            occ.erase(si.name);
                            s->used = true;
                            cell_site[i] = int(s - sites.data());
                            occ[s->name] = i;
                        } else {
                            cell_site[i] = int(s - sites.data());
                            occ[s->name] = i;
                            cell_site[j] = int(&si - sites.data());
                            occ[si.name] = j;
                        }
                    } else {
                        for (size_t q = 0; q < moved.size(); q++) {
                            pos_x[moved[q]] = olds[q].first;
                            pos_y[moved[q]] = olds[q].second;
                        }
                    }
                }
            }
            t *= alpha;
        }
        log_info("  phase D: SA %ld/%ld moves accepted (%.1f%%)\n", accepted, moves,
                 100.0 * double(accepted) / double(moves));
    }

    // ---- emit -------------------------------------------------------------
    // GT and IO are genuinely PIN-DICTATED -- the XDC names a package pin and
    // nextpnr derives the site -- so a stamp of ours is only a second opinion.
    // BUFG/BUFH/MMCM are NOT: no XDC constrains them, and withholding the stamp
    // leaves the cell free for whichever placer runs to choose a clock site by
    // accident.  BUFG site choice has cost 13.7 ns of a 23.5 ns path here.
    const bool stamp_all = getenv("TOPO_STAMP_ALL") != nullptr;
    auto skip_kind = [&](const std::string &k) {
        return k == "GT_CHANNEL" || k == "GT_COMMON" || k == "IO";
    };
    std::string placed_path = getenv_str("PLACED_OUT", "");
    std::string bels_path = getenv_str("BELS_OUT", "");
    if (bels_path.empty())
        bels_path = "bels.txt";
    if (!placed_path.empty()) {
        std::ofstream o(placed_path);
        if (!o)
            log_error("place_lef: cannot write PLACED_OUT '%s'\n", placed_path.c_str());
        for (size_t i = 0; i < ncells; i++)
            if (is_placed(i))
                o << r.cells[i].pc_name << "\t" << r.cells[i].pc_lef << "\t"
                  << sites[cell_site[i]].name << "\n";
        log_info("  wrote %s\n", placed_path.c_str());
    }
    std::ofstream bo(bels_path);
    if (!bo)
        log_error("place_lef: cannot write BELS_OUT '%s'\n", bels_path.c_str());
    int nstamp = 0, nskipped = 0, nunplaced = 0;
    for (size_t i = 0; i < ncells; i++) {
        if (r.cells[i].pc_bels.empty())
            continue;
        std::string kind = kind_of_lef(r.cells[i].pc_lef);
        if (!stamp_all && skip_kind(kind)) {
            nskipped += int(r.cells[i].pc_bels.size());
            continue;
        }
        if (!is_placed(i)) {
            nunplaced += int(r.cells[i].pc_bels.size());
            continue;
        }
        for (auto &b : r.cells[i].pc_bels) {
            bo << b.first << "\t" << sites[cell_site[i]].name << "/" << b.second << "\n";
            nstamp++;
        }
    }
    log_info("  wrote %s: %d stamp(s), %d deferred to the XDC (GT/IO), %d unplaced\n",
             bels_path.c_str(), nstamp, nskipped, nunplaced);
    if (nunplaced > 0)
        log_warning("  %d primitive(s) got NO stamp because their unit is unplaced\n", nunplaced);
    return true;
}

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
    // Site census: the chipdb replacing floorplan.json.  Reported because a
    // kind that comes back empty would silently leave a whole unit class with
    // nowhere to go.
    std::vector<LefSite> sites = build_sites(ctx);
    std::map<std::string, int> kind_count;
    int slicem = 0;
    for (auto &s : sites) {
        kind_count[s.kind.empty() ? std::string("(unclassified)") : s.kind]++;
        if (s.sm)
            slicem++;
    }
    log_info("  chipdb sites: %zu total, %d SLICEM\n", sites.size(), slicem);
    for (auto &kv : kind_count)
        log_info("    kind %-14s %6d\n", kv.first.c_str(), kv.second);
    for (auto &kv : by_lef) {
        std::string want = kv.first.compare(0, 5, "SLICE") == 0 ? "SLICE"
                           : kv.first == "RAMB36"               ? "BRAM"
                           : kv.first == "RAMB18"               ? "BRAM18"
                           : kv.first == "DSP48"                ? "DSP"
                           : kv.first == "IOB"                  ? "IO"
                                                                : kv.first;
        if (kv.first.compare(0, 7, "UNKNOWN") == 0)
            continue;
        if (!kind_count.count(want))
            log_warning("  %d unit(s) of type %s need site kind '%s' -- the chipdb has NONE\n",
                        kv.second, kv.first.c_str(), want.c_str());
    }

    if (const char *out = getenv("PACK_LEF_UNITS_OUT")) {
        std::ofstream o(out);
        if (!o)
            log_error("place_lef pre-pass: cannot write '%s'\n", out);
        for (auto &c : r.cells)
            for (auto &b : c.pc_bels)
                o << b.first << "\t" << b.second << "\t" << c.pc_name << "\t" << c.pc_lef << "\n";
        log_info("  wrote unit membership to %s\n", out);
    }

    return place_lef_place(ctx, r, sites);
}

NEXTPNR_NAMESPACE_END
