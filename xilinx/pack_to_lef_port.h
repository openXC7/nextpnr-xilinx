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

} // namespace lefpack

#endif
