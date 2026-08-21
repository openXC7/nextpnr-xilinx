/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  pack_to_lef_port: the SVS recognition packer (pack_to_lef.ml), transliterated
 *  exactly, as a library.
 *
 *  WHY IT IS HERE.  place_lef anneals PACKED UNITS whose slice contents are
 *  decided before placement; nextpnr places individual cells and lets slice
 *  co-tenancy emerge from legality checks.  Measured on ethmin the two agree on
 *  28.9% of the co-tenancy pairs pack_to_lef decides (85% where nextpnr's own
 *  packer fills in) while agreeing on the census almost exactly.  Deliberate
 *  grouping is the thing being transplanted, so it is copied verbatim.
 *
 *  DELIBERATELY FREE OF NEXTPNR HEADERS.  It operates on the RAW yosys JSON --
 *  the same input the OCaml takes -- so it must run BEFORE nextpnr's packer has
 *  turned LUT6/FDRE into SLICE_LUTX/SLICE_FFX.  Keeping it header-independent
 *  also lets tools/pack_to_lef build the identical code standalone, so the
 *  regression harness tests exactly what nextpnr runs.
 */

#ifndef PACK_TO_LEF_PORT_H
#define PACK_TO_LEF_PORT_H

#include <string>
#include <utility>
#include <vector>

namespace lefpack {

// netkey = Const of bool | Net of string * int.  On the yosys front end every
// Net is ("n<bitid>", 0), so an int carries it exactly.
static const int NK_GND = -1;  // Const false
static const int NK_VCC = -2;  // Const true
static const int NK_NONE = -3; // OCaml `None` (port absent / empty)

inline bool is_net(int k) { return k >= 0; }
std::string string_of_netkey(int k);

// One packed unit: what place_lef anneals as a single object, and what nextpnr
// must keep together if the transplant is to mean anything.
struct PackedCell
{
    std::string pc_name;
    std::string pc_lef;                                       // SLICE_LOGIC, SLICE_CARRY, ...
    std::vector<std::pair<std::string, int>> pc_conns;         // pin -> netkey
    std::vector<std::pair<std::string, std::string>> pc_bels;  // primitive inst -> BEL suffix
};

struct PackResult
{
    std::vector<PackedCell> cells;
    std::vector<std::pair<std::string, int>> report; // recognition counts, count-descending
    size_t n_instances = 0;
};

// Pack a yosys JSON netlist.  Picks the module with the most cells as the top,
// exactly as the OCaml's bmodule_of_yosys_tree does.  Honours PACK_SITE_IN,
// PACK_CRIT_FILE, PACK_CRIT_MIN and TOPO_LUT_FRACTURE from the environment.
PackResult pack_to_lef_json(const std::string &path);

// ---- netlist prepasses ---------------------------------------------------
// place_lef MUTATES THE NETLIST before packing and writes the result out
// (TOPO_FT_JSON); carry_stamp.py and the router then consume the MUTATED
// netlist, not the original.  That is why these are JSON->JSON in the OCaml,
// and why nothing has to be mirrored into nextpnr's ctx: the cells they add
// only need to exist in the netlist handed to the ROUTE run.
struct Netlist;

Netlist *netlist_load(const std::string &path);
void netlist_free(Netlist *n);
bool netlist_write(Netlist *n, const std::string &path);
PackResult pack_netlist(Netlist *n);

struct PrepassStats
{
    int muxdup = 0;       // wide-mux data pins given their own cloned LUT
    int mux_exclusive = 0; // ...of which were shared with a non-mux consumer
    int mux_skipped = 0;  // driver is not a LUT: UNROUTABLE BY CONSTRUCTION
    int muxf7_rep = 0;   // shared MUXF7 subtrees replicated for extra MUXF8 consumers
    int consts = 0;       // constant MUXF7/F8 inputs given a real LUT1
    int carry_chains = 0; // shared carry chains replicated
    int carry_rungs = 0;
    int init_fixed = 0;  // Verilog-literal params rewritten to bit-strings
};

// Runs, in place_lef's own order: split_degenerate_muxf, replicate_shared_muxf7,
// replicate_shared_carry, materialise_const_drivers, normalise_init.
PrepassStats netlist_prepasses(Netlist *n);

// ---- carry_stamp ---------------------------------------------------------
// The port of carry_stamp.py.  SVS stamps only the CARRY4 anchor, and
// nextpnr-xilinx has NO site-level LUT routethru, so a CARRY4 S-input driven by
// a non-LUT (a FF's Q via the AX bypass, or a GND const) cannot bind.  This
// lays out each stamped CARRY4's whole slice explicitly.  Not cosmetic: ~2140
// cells on ethmin.
struct CarryStampStats
{
    int n_buf = 0;   // S inputs given an inserted LUT1 buffer
    int n_slut = 0;  // S driving LUTs stamped into the carry's own slot
    int n_ff = 0;    // sum FFs stamped
    int n_di = 0;    // DI const/passthrough 5LUTs
    int n_fb = 0;    // same-slot FF->LUT feedback relays (CARRY_FB_NETS only)
    size_t total_cells = 0;
    bool collision = false;
    std::string collision_msg;
};

// `bels` is the placement, in the order the placer emitted it ("<SITE>/<BEL>"
// per primitive).  `slice_sites` supplies the legal SLICE site names for the
// neighbour-bel search -- from nextpnr's chipdb, replacing carry_stamp.py's
// CARRY_FLOORPLAN read.  Honours CARRY_STAMP_AVOID_CI and CARRY_FB_NETS.
// ---- emit_clock_xdc ------------------------------------------------------
// nextpnr can only create_clock on PORTS, but the interesting clocks are BUFG
// outputs several derivations downstream of one (GT refclk -> GTXE2 -> TXOUTCLK
// -> MMCM -> userclk2), which it cannot derive.  Without these, every clock
// falls back to --freq: measured on ethmin that reported the 125 MHz SGMII
// datapath as "65.35 MHz (PASS at 25.00 MHz)" -- missing its real target by 2x
// and calling it a PASS.  Every criticality the router exports is then relative
// to the wrong period.
//
// `spec` is TOPO_CLOCK_PERIODS: "bufg_userclk2=8.0,clk_sys_bufg=40.0,...".
// Keys are matched as SUBSTRINGS of BUFG/BUFGCTRL CELL names, which come from
// the design hierarchy and are stable, unlike synthesis-generated net names.
// Returns the number of create_clock lines written, -1 on failure.
int netlist_emit_clock_xdc(Netlist *n, const std::string &path, const std::string &spec);

CarryStampStats netlist_carry_stamp(Netlist *n,
                                    const std::vector<std::pair<std::string, std::string>> &bels,
                                    const std::vector<std::string> &slice_sites);

} // namespace lefpack

#endif
