// pack_to_lef.cpp -- EXACT C++ transliteration of SVS deps/System-Verilog-suite/
// pack_to_lef.ml (the recognition packer place_lef.exe uses).
//
// WHY THIS EXISTS.  nextpnr places individual cells; place_lef places PACKED
// UNITS decided before placement.  Measured on ethmin, the two flows agree on
// only 28.9% of the slice co-tenancy pairs that pack_to_lef decides (85% where
// nextpnr's own packer fills in), while agreeing almost exactly on the census
// (1542 vs 1501 slices, 7801 vs 7759 cells).  Same cells, different partition.
// So the packer is ported EXACTLY first, as a standalone whose output can be
// diffed cell-for-cell against the OCaml, and only then integrated.
//
// FIDELITY NOTES -- the things that would silently diverge:
//   * m.instances ORDER drives every phase.  yosys JSON cell order is the
//     order; a std::map-backed JSON parser (json11) would alphabetise it, so
//     this file carries its own order-preserving parser.
//   * `sinks` lists are built by PREPENDING, so they iterate in reverse
//     instance order.  Replicated.
//   * conns/bels are built by prepending and reversed once at `add`, so the
//     emitted order is call order.  Replicated.
//   * net identity on the yosys path: bit-id integer.  bit_expr maps `Int id`
//     to BVar "n<id>" (width lookup fails -> width 1), so each bit is exactly
//     Net("n<id>", 0) -- a bijection with the bit-id.  Hence netkey == int
//     here, with GND/VCC as sentinels.  Anything not Int/"0"/"1" (i.e. "x",
//     "z") maps to Const false, as in the OCaml.
//   * phase 1a0 (r256 groups) uses Hashtbl.iter in the OCaml, whose order is
//     UNSPECIFIED.  We use first-encounter order: deterministic, and strictly
//     better-defined than the original.  ethmin has 0 such groups.

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "pack_to_lef_port.h"

namespace lefpack {

// ---------------------------------------------------------------- JSON ----
// Minimal recursive-descent parser.  Objects keep INSERTION ORDER (a vector of
// pairs), which is the whole point of not using json11 here.
struct JVal;
using JPtr = std::shared_ptr<JVal>;
struct JVal
{
    enum K { OBJ, ARR, STR, NUM, BOOL, NUL } k = NUL;
    std::vector<std::pair<std::string, JPtr>> obj;
    std::vector<JPtr> arr;
    std::string str;
    double num = 0;
    bool bol = false;

    JPtr member(const std::string &n) const
    {
        for (auto &kv : obj)
            if (kv.first == n)
                return kv.second;
        return nullptr;
    }
};

struct JParser
{
    const std::string &s;
    size_t i = 0;
    explicit JParser(const std::string &src) : s(src) {}

    void ws()
    {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r'))
            i++;
    }
    [[noreturn]] void die(const char *m)
    {
        fprintf(stderr, "JSON parse error at offset %zu: %s\n", i, m);
        exit(1);
    }
    std::string parse_string()
    {
        if (s[i] != '"')
            die("expected string");
        i++;
        std::string out;
        while (i < s.size() && s[i] != '"') {
            if (s[i] == '\\') {
                i++;
                switch (s[i]) {
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case 'r': out += '\r'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'u': {
                    // yosys does not emit these in cell names, but be safe:
                    // decode the BMP codepoint as UTF-8.
                    unsigned cp = std::stoul(s.substr(i + 1, 4), nullptr, 16);
                    i += 4;
                    if (cp < 0x80)
                        out += char(cp);
                    else if (cp < 0x800) {
                        out += char(0xC0 | (cp >> 6));
                        out += char(0x80 | (cp & 0x3F));
                    } else {
                        out += char(0xE0 | (cp >> 12));
                        out += char(0x80 | ((cp >> 6) & 0x3F));
                        out += char(0x80 | (cp & 0x3F));
                    }
                    break;
                }
                default: out += s[i];
                }
                i++;
            } else
                out += s[i++];
        }
        if (i >= s.size())
            die("unterminated string");
        i++;
        return out;
    }
    JPtr parse()
    {
        ws();
        auto v = std::make_shared<JVal>();
        if (i >= s.size())
            die("unexpected end");
        char c = s[i];
        if (c == '{') {
            v->k = JVal::OBJ;
            i++;
            ws();
            if (s[i] == '}') { i++; return v; }
            for (;;) {
                ws();
                std::string key = parse_string();
                ws();
                if (s[i] != ':')
                    die("expected :");
                i++;
                v->obj.emplace_back(key, parse());
                ws();
                if (s[i] == ',') { i++; continue; }
                if (s[i] == '}') { i++; break; }
                die("expected , or }");
            }
        } else if (c == '[') {
            v->k = JVal::ARR;
            i++;
            ws();
            if (s[i] == ']') { i++; return v; }
            for (;;) {
                v->arr.push_back(parse());
                ws();
                if (s[i] == ',') { i++; continue; }
                if (s[i] == ']') { i++; break; }
                die("expected , or ]");
            }
        } else if (c == '"') {
            v->k = JVal::STR;
            v->str = parse_string();
        } else if (c == 't') {
            v->k = JVal::BOOL; v->bol = true; i += 4;
        } else if (c == 'f') {
            v->k = JVal::BOOL; v->bol = false; i += 5;
        } else if (c == 'n') {
            v->k = JVal::NUL; i += 4;
        } else {
            v->k = JVal::NUM;
            size_t st = i;
            while (i < s.size() && (isdigit((unsigned char)s[i]) || s[i] == '-' || s[i] == '+' ||
                                    s[i] == '.' || s[i] == 'e' || s[i] == 'E'))
                i++;
            v->num = strtod(s.substr(st, i - st).c_str(), nullptr);
        }
        return v;
    }
};

// ------------------------------------------------------------- netkeys ----
// NK_GND / NK_VCC / NK_NONE and is_net() are declared in the header; this is
// their only definition.
std::string string_of_netkey(int k)
{
    if (k == NK_GND) return "GND";
    if (k == NK_VCC) return "VCC";
    if (k == NK_NONE) return "-";
    char b[32];
    snprintf(b, sizeof(b), "n%d[0]", k);
    return b;
}

// --------------------------------------------------------------- utils ----
static std::string upper(const std::string &s)
{
    std::string o = s;
    for (auto &c : o) c = toupper((unsigned char)c);
    return o;
}
// [starts_with s pre] : value first, prefix second -- matches the OCaml.
static bool starts_with(const std::string &s, const char *pre)
{
    size_t n = strlen(pre);
    return s.size() >= n && memcmp(s.data(), pre, n) == 0;
}
static std::string lane_letter(int k) { return std::string(1, char('A' + k)); }

static bool is_output(const std::string &mn, const std::string &port)
{
    std::string m = upper(mn), p = upper(port);
    if (starts_with(m, "CARRY4")) return p == "O" || p == "CO";
    if (starts_with(m, "FD")) return p == "Q";
    if (starts_with(m, "LUT")) return p == "O";
    if (starts_with(m, "RAMB")) return starts_with(p, "DO") || starts_with(p, "CASCADEOUT");
    if (starts_with(m, "DSP")) return p == "P" || p == "PCOUT" || p == "CARRYOUT";
    if (m == "MMCME2_ADV" || m == "PLLE2_ADV")
        return starts_with(p, "CLKOUT") || p == "CLKFBOUT" || p == "LOCKED";
    if (starts_with(m, "BUFG") || starts_with(m, "BUFH")) return p == "O";
    if (starts_with(m, "IBUF") || starts_with(m, "OBUF") || m == "IBUFDS") return p == "O";
    return p == "O" || p == "Q";
}

// io_map / hard_map / slicem_map, verbatim.
static bool io_map(const std::string &t, std::string &lef)
{
    if (t == "IBUF" || t == "OBUF" || t == "IBUFDS" || t == "OBUFDS" || t == "IOBUF" ||
        t == "IBUFDS_GTE2") { lef = "IOB"; return true; }
    if (t == "MMCME2_ADV" || t == "PLLE2_ADV") { lef = "MMCM"; return true; }
    if (t == "BUFG" || t == "BUFGCTRL") { lef = "BUFG"; return true; }
    if (t == "BUFH" || t == "BUFHCE") { lef = "BUFH"; return true; }
    // CHANNEL and COMMON are NOT interchangeable (see the OCaml comment).
    if (t == "GTXE2_CHANNEL" || t == "GTHE2_CHANNEL") { lef = "GT_CHANNEL"; return true; }
    if (t == "GTXE2_COMMON" || t == "GTHE2_COMMON") { lef = "GT_COMMON"; return true; }
    return false;
}
static bool hard_map(const std::string &t, std::string &lef, std::string &suffix)
{
    std::string u = upper(t);
    if (starts_with(u, "RAMB36")) { lef = "RAMB36"; suffix = "RAMB36E1"; return true; }
    if (starts_with(u, "RAMB18") || starts_with(u, "FIFO18")) { lef = "RAMB18"; suffix = "RAMB18E1"; return true; }
    if (starts_with(u, "DSP48")) { lef = "DSP48"; suffix = "DSP48E1"; return true; }
    return false;
}
static bool slicem_map(const std::string &t, std::string &lef, std::string &suffix)
{
    std::string u = upper(t);
    if (starts_with(u, "SRL")) { lef = "SLICEM_SRL"; suffix = "A6LUT"; return true; }
    if (starts_with(u, "RAMD") || starts_with(u, "RAMS") || starts_with(u, "RAM32") ||
        starts_with(u, "RAM64")) { lef = "SLICEM_DRAM"; suffix = "A6LUT"; return true; }
    if (starts_with(u, "MUXF7")) { lef = "SLICE_MUX"; suffix = "F7AMUX"; return true; }
    if (starts_with(u, "MUXF8")) { lef = "SLICE_MUX"; suffix = "F8MUX"; return true; }
    return false;
}

// ------------------------------------------------------------ netlist -----
struct Inst
{
    std::string name;
    std::string type;
    std::vector<std::pair<std::string, std::vector<int>>> ports; // JSON order
};

// PackedCell is declared in the header -- it is the transplant's payload, the
// unit place_lef anneals as one object.

struct Packer
{
    std::vector<Inst> insts;
    std::unordered_map<std::string, int> by_name;
    // driver: netkey -> (inst index, port, bit).  Hashtbl.replace => last wins.
    std::unordered_map<int, std::tuple<int, std::string, int>> drv;
    // sinks: built by PREPENDING, so front() is the LAST instance added.
    std::unordered_map<int, std::vector<std::tuple<int, std::string, int>>> sinks;
    std::unordered_set<std::string> absorbed;
    std::map<std::string, int> report;
    std::vector<PackedCell> packed;

    void bump(const std::string &k) { report[k]++; }
    const std::string &mtype(const std::string &n) { return insts[by_name.at(n)].type; }

    void add(const std::string &name, const std::string &lef,
             const std::vector<std::pair<std::string, int>> &conns,
             const std::vector<std::pair<std::string, std::string>> &bels = {})
    {
        packed.push_back({name, lef, conns, bels});
    }

    // List.assoc_opt on the port list: FIRST exact-name match.
    int port_bit(const std::string &iname, const std::string &pname, size_t idx)
    {
        auto it = by_name.find(iname);
        if (it == by_name.end()) return NK_NONE;
        for (auto &pc : insts[it->second].ports)
            if (pc.first == pname)
                return idx < pc.second.size() ? pc.second[idx] : NK_NONE;
        return NK_NONE;
    }
    int port_bit(int ii, const std::string &pname, size_t idx)
    {
        for (auto &pc : insts[ii].ports)
            if (pc.first == pname)
                return idx < pc.second.size() ? pc.second[idx] : NK_NONE;
        return NK_NONE;
    }

    void build_maps()
    {
        for (size_t k = 0; k < insts.size(); k++) by_name[insts[k].name] = int(k);
        for (size_t k = 0; k < insts.size(); k++) {
            auto &i = insts[k];
            for (auto &pc : i.ports) {
                for (size_t bi = 0; bi < pc.second.size(); bi++) {
                    int nk = pc.second[bi];
                    if (!is_net(nk)) continue;
                    if (is_output(i.type, pc.first))
                        drv[nk] = std::make_tuple(int(k), pc.first, int(bi));
                    else {
                        auto &v = sinks[nk];
                        v.insert(v.begin(), std::make_tuple(int(k), pc.first, int(bi)));
                    }
                }
            }
        }
    }

    void run();
    void phase0_site_guided();
    void phase1_carry();
    void phase1a0_r256_groups();
    void phase1b_ram256_macro();
    void phase1a_mux();
    void phase1c_dram_groups();
    void phase1b2_lut_ff_pairs();
    void phase2_dedicated_sites();
    void phase3a_leftover_luts();
    void phase3_leftover();
};

// ------------------------------------------------------- 0. site-guided ---
// PACK_SITE_IN: adopt an external tool's SLICE GROUPING verbatim.  Dormant in
// the production flow (place_route_open.sh never sets it) but ported for
// fidelity, because when it IS set it runs BEFORE every recognition path and
// claims CARRY4/DRAM/SRL too.
void Packer::phase0_site_guided()
{
    const char *path = getenv("PACK_SITE_IN");
    // site_of: inst -> (site, bel).  Uses Hashtbl.add for the collapsed parent
    // key (first wins), Hashtbl.replace for the leaf (last wins).
    std::unordered_map<std::string, std::pair<std::string, std::string>> site_of;
    if (path) {
        std::ifstream f(path);
        if (!f) {
            fprintf(stderr, "PACK_SITE_IN: %s: cannot open\n", path);
        } else {
            std::string line;
            while (std::getline(f, line)) {
                std::stringstream ss(line);
                std::string nm, sb;
                if (!std::getline(ss, nm, '\t')) continue;
                if (!std::getline(ss, sb, '\t')) continue;
                auto trim = [](std::string s) {
                    size_t a = s.find_first_not_of(" \t\r\n");
                    size_t b = s.find_last_not_of(" \t\r\n");
                    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
                };
                nm = trim(nm); sb = trim(sb);
                std::string site = sb, bel;
                size_t sl = sb.find('/');
                if (sl != std::string::npos) { site = sb.substr(0, sl); bel = sb.substr(sl + 1); }
                site_of[nm] = {site, bel};
                // Distributed RAM arrives as LEAF sub-bels while the netlist
                // holds only the PARENT; collapse so they group by site.
                size_t rs = nm.rfind('/');
                if (rs != std::string::npos) {
                    std::string p = nm.substr(0, rs);
                    if (!p.empty() && !site_of.count(p)) site_of[p] = {site, bel};
                }
            }
        }
    }
    if (site_of.empty()) return;

    // Only SLICE-resident types; BRAM/DSP/IO/MMCM/BUFG keep their own paths.
    auto slice_resident = [&](const std::string &t) {
        std::string u = upper(t), l, s;
        return (starts_with(u, "LUT") || u == "INV" || starts_with(u, "FD") ||
                starts_with(u, "CARRY4") || slicem_map(t, l, s)) &&
               !hard_map(t, l, s) && !io_map(t, l);
    };
    std::unordered_map<std::string, std::vector<int>> by_site;
    std::vector<std::string> order;
    for (size_t k = 0; k < insts.size(); k++) {
        auto &i = insts[k];
        if (absorbed.count(i.name) || !slice_resident(i.type)) continue;
        auto it = site_of.find(i.name);
        if (it == site_of.end()) continue;
        const std::string &s = it->second.first;
        if (!by_site.count(s)) order.push_back(s);
        by_site[s].push_back(int(k));
    }
    int nsite = 0, nprim = 0, ncarry = 0, nsm = 0;
    for (auto &s : order) {
        auto &ms = by_site[s];
        std::string l, sfx;
        bool has_sm = false, has_carry = false, has_srl = false;
        for (int ii : ms) {
            if (slicem_map(insts[ii].type, l, sfx)) has_sm = true;
            if (starts_with(upper(insts[ii].type), "CARRY4")) has_carry = true;
            if (starts_with(upper(insts[ii].type), "SRL")) has_srl = true;
        }
        std::string lef;
        if (has_sm) { nsm++; lef = has_srl ? "SLICEM_SRL" : "SLICEM_DRAM"; }
        else if (has_carry) { ncarry++; lef = "SLICE_CARRY"; }
        else lef = "SLICE_LOGIC";
        std::vector<std::pair<std::string, std::string>> bels;
        std::vector<std::pair<std::string, int>> conns;
        for (int ii : ms) {
            auto &inst = insts[ii];
            std::string bel = "A6LUT";
            auto it = site_of.find(inst.name);
            if (it != site_of.end() && !it->second.second.empty()) bel = it->second.second;
            bels.emplace_back(inst.name, bel);
            // Pin names are BEL-prefixed so two primitives in one slice cannot
            // collide; place_lef reads pc_conns only for NET identity.
            for (auto &pc : inst.ports)
                if (!pc.second.empty()) conns.emplace_back(bel + "_" + pc.first, pc.second[0]);
            absorbed.insert(inst.name);
            nprim++;
        }
        add("site_" + s, lef, conns, bels);
        nsite++;
        bump("site-guided slice");
    }
    printf("site-guided pack: %d primitives -> %d slice cells (%.2f per slice; %d carry, %d slicem)\n",
           nprim, nsite, double(nprim) / (nsite > 0 ? nsite : 1), ncarry, nsm);
}

// ------------------------------------- 1. CARRY4 -> SLICE_CARRY -----------
// Absorbs the S-driving LUTs and the sum FFs, so the cnt[i] -> S[i] feedback
// never reaches the router.  BEL-stamps ONLY the CARRY4 itself: stamping all
// four LUT slots leaves no room for the DI route-thru $LUTs nextpnr inserts.
void Packer::phase1_carry()
{
    for (size_t k = 0; k < insts.size(); k++) {
        auto &i = insts[k];
        if (!starts_with(upper(i.type), "CARRY4") || absorbed.count(i.name)) continue;
        std::vector<std::pair<std::string, int>> conns;
        auto put = [&](const std::string &key, int v) { conns.emplace_back(key, v); };
        std::vector<std::pair<std::string, std::string>> bels{{i.name, "CARRY4"}};
        int v;
        if ((v = port_bit(int(k), "CI", 0)) != NK_NONE) put("CI", v);
        if ((v = port_bit(int(k), "CYINIT", 0)) != NK_NONE) put("CYINIT", v);
        if ((v = port_bit(int(k), "CO", 3)) != NK_NONE) put("CO", v);
        absorbed.insert(i.name);
        bump("CARRY4->SLICE_CARRY");
        char buf[64];
        for (int kk = 0; kk <= 3; kk++) {
            snprintf(buf, sizeof(buf), "S%d", kk);
            if ((v = port_bit(int(k), "S", kk)) != NK_NONE) put(buf, v);
            snprintf(buf, sizeof(buf), "DI%d", kk);
            if ((v = port_bit(int(k), "DI", kk)) != NK_NONE) put(buf, v);
            int obit = port_bit(int(k), "O", kk);
            if (obit != NK_NONE) {
                snprintf(buf, sizeof(buf), "O%d", kk);
                put(buf, obit);
                // absorb sum FF: FDxE whose D == this O[k]
                auto sit = sinks.find(obit);
                if (sit != sinks.end()) {
                    for (auto &sk : sit->second) {
                        int sc = std::get<0>(sk);
                        const std::string &sp = std::get<1>(sk);
                        if (!starts_with(upper(insts[sc].type), "FD")) continue;
                        if (upper(sp) != "D") continue;
                        if (absorbed.count(insts[sc].name)) continue;
                        absorbed.insert(insts[sc].name);
                        bump("sum-FF absorbed");
                        int q;
                        if ((q = port_bit(sc, "Q", 0)) != NK_NONE) {
                            snprintf(buf, sizeof(buf), "Q%d", kk);
                            put(buf, q);
                        }
                        if ((q = port_bit(sc, "C", 0)) != NK_NONE) put("CLK", q);
                        if ((q = port_bit(sc, "CE", 0)) != NK_NONE) put("CE", q);
                        if ((q = port_bit(sc, "R", 0)) != NK_NONE) put("SR", q);
                        else if ((q = port_bit(sc, "S", 0)) != NK_NONE) put("SR", q);
                    }
                }
            }
            // absorb S-LUT: a LUT driving this S[k]
            int sbit = port_bit(int(k), "S", kk);
            if (sbit != NK_NONE) {
                auto dit = drv.find(sbit);
                if (dit != drv.end()) {
                    int dn = std::get<0>(dit->second);
                    if (starts_with(upper(insts[dn].type), "LUT") && !absorbed.count(insts[dn].name)) {
                        absorbed.insert(insts[dn].name);
                        bump("S-LUT absorbed");
                        for (auto &pc : insts[dn].ports) {
                            if (upper(pc.first) == "O") continue;
                            if (pc.second.empty()) continue;
                            snprintf(buf, sizeof(buf), "S%d_%s", kk, pc.first.c_str());
                            put(buf, pc.second[0]);
                        }
                    }
                }
            }
        }
        // Str.search_forward "_i_1$" -- anchored, so: strip a trailing "_i_1".
        std::string base = i.name;
        if (base.size() >= 4 && base.compare(base.size() - 4, 4, "_i_1") == 0)
            base = base.substr(0, base.size() - 4);
        add(base + "$carry", "SLICE_CARRY", conns, bels);
    }
}

// ---------------------- 1a0. decomposed RAM256X1S (r256_*) -> SLICEM_DRAM --
// THE LANE ORDER IS REVERSED, and not by choice: nextpnr's F7AMUX reads I0
// from the SECOND lane and I1 from the FIRST, and its F8MUX takes I1 from the
// A/B pair.  d->A, c->B, b->C, a->D with f7b on F7AMUX and f7a on F7BMUX.
void Packer::phase1a0_r256_groups()
{
    auto leaf_of = [](const std::string &n) {
        size_t p = n.rfind('.');
        return p == std::string::npos ? n : n.substr(p + 1);
    };
    auto parent_of = [](const std::string &n) {
        size_t p = n.rfind('.');
        return p == std::string::npos ? std::string() : n.substr(0, p);
    };
    std::unordered_map<std::string, std::vector<int>> groups;
    std::vector<std::string> order; // deterministic; OCaml uses Hashtbl.iter
    for (size_t k = 0; k < insts.size(); k++) {
        if (!starts_with(leaf_of(insts[k].name), "r256_")) continue;
        std::string key = parent_of(insts[k].name);
        if (!groups.count(key)) order.push_back(key);
        groups[key].push_back(int(k));
    }
    for (auto &key : order) {
        auto &ms = groups[key];
        auto find = [&](const char *sfx) -> int {
            std::string want = std::string("r256_") + sfx;
            for (int ii : ms)
                if (leaf_of(insts[ii].name) == want) return ii;
            return -1;
        };
        int ra = find("a"), rb = find("b"), rc = find("c"), rd = find("d");
        int f7a = find("f7a"), f7b = find("f7b"), f8 = find("f8");
        if (ra >= 0 && rb >= 0 && rc >= 0 && rd >= 0 && f7a >= 0 && f7b >= 0 && f8 >= 0 &&
            !absorbed.count(insts[ra].name)) {
            std::vector<std::pair<std::string, std::string>> bels{
                {insts[rd].name, "A6LUT"}, {insts[rc].name, "B6LUT"},
                {insts[rb].name, "C6LUT"}, {insts[ra].name, "D6LUT"},
                {insts[f7b].name, "F7AMUX"}, {insts[f7a].name, "F7BMUX"},
                {insts[f8].name, "F8MUX"}};
            std::vector<std::pair<std::string, int>> conns;
            auto put = [&](const std::string &p, int v) { conns.emplace_back(p, v); };
            int v;
            if ((v = port_bit(ra, "WCLK", 0)) != NK_NONE) put("WCLK", v);
            if ((v = port_bit(ra, "WE", 0)) != NK_NONE) put("WE", v);
            char buf[32];
            for (int kk = 0; kk <= 5; kk++) {
                snprintf(buf, sizeof(buf), "A%d", kk);
                if ((v = port_bit(ra, buf, 0)) != NK_NONE) put(buf, v);
            }
            int banks[4] = {ra, rb, rc, rd};
            for (int idx = 0; idx < 4; idx++) {
                if ((v = port_bit(banks[idx], "D", 0)) != NK_NONE) {
                    snprintf(buf, sizeof(buf), "DI%d", idx);
                    put(buf, v);
                }
                if ((v = port_bit(banks[idx], "O", 0)) != NK_NONE) {
                    snprintf(buf, sizeof(buf), "DO%d", idx);
                    put(buf, v);
                }
            }
            if ((v = port_bit(f8, "O", 0)) != NK_NONE) put("DO3", v);
            for (int ii : {ra, rb, rc, rd, f7a, f7b, f8}) absorbed.insert(insts[ii].name);
            bump("RAM256-group->SLICEM_DRAM");
            add(insts[ra].name + "$r256", "SLICEM_DRAM", conns, bels);
        } else {
            // An INCOMPLETE group must not pass quietly.
            std::string missing;
            const char *names[7] = {"a", "b", "c", "d", "f7a", "f7b", "f8"};
            int got[7] = {ra, rb, rc, rd, f7a, f7b, f8};
            for (int t = 0; t < 7; t++)
                if (got[t] < 0) {
                    if (!missing.empty()) missing += ",";
                    missing += std::string("r256_") + names[t];
                }
            bump("RAM256-group INCOMPLETE (scattered)");
            fprintf(stderr,
                    "[pack_to_lef] WARNING: r256 group '%s' has %zu of 7 members (missing: %s) -- "
                    "NOT packed into one SLICEM; its RAM64X1S will be scattered\n",
                    key.c_str(), ms.size(), missing.c_str());
        }
    }
}

// ------------------------- 1b. undecomposed RAM256X1S macro -> SLICEM_DRAM -
void Packer::phase1b_ram256_macro()
{
    for (size_t k = 0; k < insts.size(); k++) {
        auto &i = insts[k];
        if (!starts_with(upper(i.type), "RAM256X1S") || absorbed.count(i.name)) continue;
        std::vector<std::pair<std::string, int>> conns;
        auto put = [&](const std::string &p, int v) { conns.emplace_back(p, v); };
        int v;
        if ((v = port_bit(int(k), "WCLK", 0)) != NK_NONE) put("WCLK", v);
        if ((v = port_bit(int(k), "WE", 0)) != NK_NONE) put("WE", v);
        char buf[32];
        for (int kk = 0; kk <= 5; kk++) {
            if ((v = port_bit(int(k), "A", kk)) != NK_NONE) {
                snprintf(buf, sizeof(buf), "A%d", kk);
                put(buf, v);
            }
        }
        if ((v = port_bit(int(k), "D", 0)) != NK_NONE) put("DI0", v);
        if ((v = port_bit(int(k), "O", 0)) != NK_NONE) put("DO3", v);
        absorbed.insert(i.name);
        bump("RAM256X1S->SLICEM_DRAM");
        // z = height-1, the D lane -- what makes nextpnr's expansion land here.
        add(i.name + "$r256macro", "SLICEM_DRAM", conns, {{i.name, "D6LUT"}});
    }
}

// ------------------------------- 1a. MUXF7/MUXF8 -> SLICE_MUX -------------
void Packer::phase1a_mux()
{
    auto is_lut_t = [](const std::string &t) { return starts_with(upper(t), "LUT"); };

    // absorb MUXF7 [mn] as F7[la]MUX; its two driving LUT6 -> lanes la,lb.
    // Returns (bels, conns) UNREVERSED, exactly as the OCaml does.
    auto absorb_mux7 = [&](int mn, const std::string &la, const std::string &lb,
                           std::vector<std::pair<std::string, std::string>> &bels,
                           std::vector<std::pair<std::string, int>> &conns) {
        absorbed.insert(insts[mn].name);
        bels.insert(bels.begin(), {insts[mn].name, la == "A" ? "F7AMUX" : "F7BMUX"});
        int v;
        if ((v = port_bit(mn, "S", 0)) != NK_NONE) conns.insert(conns.begin(), {"F7" + la + "S", v});
        if ((v = port_bit(mn, "O", 0)) != NK_NONE) conns.insert(conns.begin(), {"F7" + la + "O", v});
        auto do_lut = [&](const char *inp, const std::string &lane) {
            int sbit = port_bit(mn, inp, 0);
            if (sbit == NK_NONE) return;
            auto dit = drv.find(sbit);
            if (dit == drv.end()) return;
            int dn = std::get<0>(dit->second);
            if (!is_lut_t(insts[dn].type) || absorbed.count(insts[dn].name)) return;
            absorbed.insert(insts[dn].name);
            bump("mux-LUT absorbed");
            bels.insert(bels.begin(), {insts[dn].name, lane + "6LUT"});
            for (auto &pc : insts[dn].ports) {
                if (upper(pc.first) == "O") continue;
                if (pc.second.empty()) continue;
                conns.insert(conns.begin(), {lane + pc.first, pc.second[0]});
            }
        };
        // nextpnr's F7[la]MUX reads I0 from the SECOND lane and I1 from the
        // FIRST.  Assigning I0->la crosses both mux inputs.
        do_lut("I0", lb);
        do_lut("I1", la);
    };

    // MUXF8 groups first (absorb the two feeding MUXF7 + their LUTs)
    for (size_t k = 0; k < insts.size(); k++) {
        auto &i = insts[k];
        if (!starts_with(upper(i.type), "MUXF8") || absorbed.count(i.name)) continue;
        std::vector<std::pair<std::string, std::string>> bels{{i.name, "F8MUX"}};
        std::vector<std::pair<std::string, int>> conns;
        int v;
        if ((v = port_bit(int(k), "S", 0)) != NK_NONE) conns.insert(conns.begin(), {"F8S", v});
        if ((v = port_bit(int(k), "O", 0)) != NK_NONE) conns.insert(conns.begin(), {"F8O", v});
        auto grab = [&](const char *inp, const std::string &la, const std::string &lb) {
            int mbit = port_bit(int(k), inp, 0);
            if (mbit == NK_NONE) return;
            auto dit = drv.find(mbit);
            if (dit == drv.end()) return;
            int mn = std::get<0>(dit->second);
            if (!starts_with(upper(insts[mn].type), "MUXF7") || absorbed.count(insts[mn].name)) return;
            std::vector<std::pair<std::string, std::string>> b;
            std::vector<std::pair<std::string, int>> c;
            absorb_mux7(mn, la, lb, b, c);
            bels.insert(bels.begin(), b.begin(), b.end()); // b @ !bels
            conns.insert(conns.begin(), c.begin(), c.end());
        };
        grab("I1", "A", "B");
        grab("I0", "C", "D");
        absorbed.insert(i.name);
        bump("MUXF8->SLICE_MUX");
        std::reverse(bels.begin(), bels.end());
        std::reverse(conns.begin(), conns.end());
        add(i.name + "$mux", "SLICE_MUX", conns, bels);
    }
    // standalone MUXF7 (not consumed by a MUXF8)
    for (size_t k = 0; k < insts.size(); k++) {
        auto &i = insts[k];
        if (!starts_with(upper(i.type), "MUXF7") || absorbed.count(i.name)) continue;
        std::vector<std::pair<std::string, std::string>> bels;
        std::vector<std::pair<std::string, int>> conns;
        absorb_mux7(int(k), "A", "B", bels, conns);
        bump("MUXF7->SLICE_MUX");
        std::reverse(bels.begin(), bels.end());
        std::reverse(conns.begin(), conns.end());
        add(i.name + "$mux", "SLICE_MUX", conns, bels);
    }
}

// --------------- 1c. RAM32X1D / RAM64X1D write-port grouping -> SLICEM ----
// Slot allocation mirrors nextpnr pack_dram.cc: z descends from D=3, each new
// slice burns the D6LUT on the write-address base cell, EXCEPT the first
// member's SPO folds into that base (z==2 fold).  Capacity is 3 DPO-only or 2
// dual-port per slice, not 4.  The _1 variants write on the FALLING edge and
// are grouped SEPARATELY (the key includes the type).
void Packer::phase1c_dram_groups()
{
    auto dram_kind = [](const std::string &t, int &abits, bool &inv) {
        std::string u = upper(t);
        if (u == "RAM32X1D") { abits = 5; inv = false; return true; }
        if (u == "RAM32X1D_1") { abits = 5; inv = true; return true; }
        if (u == "RAM64X1D") { abits = 6; inv = false; return true; }
        if (u == "RAM64X1D_1") { abits = 6; inv = true; return true; }
        return false;
    };
    using Key = std::tuple<std::string, int, int, std::vector<int>>;
    std::map<Key, std::vector<int>> dgroups;
    std::vector<Key> dg_order;
    for (size_t k = 0; k < insts.size(); k++) {
        int abits; bool inv;
        if (!dram_kind(insts[k].type, abits, inv)) continue;
        if (absorbed.count(insts[k].name)) continue;
        std::vector<int> wa;
        char buf[16];
        for (int t = 0; t < abits; t++) {
            snprintf(buf, sizeof(buf), "A%d", t);
            wa.push_back(port_bit(int(k), buf, 0));
        }
        Key key{upper(insts[k].type), port_bit(int(k), "WCLK", 0), port_bit(int(k), "WE", 0), wa};
        if (!dgroups.count(key)) dg_order.push_back(key);
        dgroups[key].push_back(int(k));
    }
    for (auto &key : dg_order) {
        auto &members = dgroups[key];
        const std::string &ty = std::get<0>(key);
        int wclk = std::get<1>(key), we = std::get<2>(key);
        const std::vector<int> &wa = std::get<3>(key);
        int abits; bool inv;
        dram_kind(ty, abits, inv);
        int slice_idx = 0, z = -1;
        // current slice under construction: (inst, sp_slot, dp_slot); -1 = None
        std::vector<std::tuple<int, int, int>> cur;
        auto flush = [&]() {
            if (!cur.empty()) {
                int i0 = std::get<0>(cur.front());
                std::vector<std::pair<std::string, int>> conns;
                std::vector<std::pair<std::string, std::string>> bels;
                auto put = [&](const std::string &kk, int v) { conns.emplace_back(kk, v); };
                if (wclk != NK_NONE) put("WCLK", wclk);
                if (we != NK_NONE) put("WE", we);
                char buf[32];
                for (size_t t = 0; t < wa.size(); t++)
                    if (wa[t] != NK_NONE) {
                        snprintf(buf, sizeof(buf), "WA%zu", t);
                        put(buf, wa[t]);
                    }
                if (inv) put("WCLK_INV", NK_VCC); // polarity marker, no HPWL
                for (auto &m : cur) {
                    int ii = std::get<0>(m), sp = std::get<1>(m), dp = std::get<2>(m);
                    int prim = sp >= 0 ? sp : (dp >= 0 ? dp : 3);
                    bels.emplace_back(insts[ii].name, lane_letter(prim) + "6LUT");
                    int v;
                    if ((v = port_bit(ii, "D", 0)) != NK_NONE) put(lane_letter(prim) + "D", v);
                    if (sp >= 0 && (v = port_bit(ii, "SPO", 0)) != NK_NONE)
                        put(lane_letter(sp) + "SPO", v);
                    if (dp >= 0) {
                        if ((v = port_bit(ii, "DPO", 0)) != NK_NONE) put(lane_letter(dp) + "DPO", v);
                        for (int t = 0; t < abits; t++) {
                            snprintf(buf, sizeof(buf), "DPRA%d", t);
                            if ((v = port_bit(ii, buf, 0)) != NK_NONE) {
                                snprintf(buf, sizeof(buf), "%sDPRA%d", lane_letter(dp).c_str(), t);
                                put(buf, v);
                            }
                        }
                    }
                }
                snprintf(buf, sizeof(buf), "$dram%s%d", inv ? "_n" : "", slice_idx);
                add(insts[i0].name + buf, "SLICEM_DRAM", conns, bels);
                slice_idx++;
                bump("DRAM-group->SLICEM_DRAM");
            }
            cur.clear();
            z = -1;
        };
        for (int ii : members) {
            auto has = [&](const char *p) { return is_net(port_bit(ii, p, 0)); };
            bool spo = has("SPO"), dpo = has("DPO");
            int zsz = (spo ? 1 : 0) + (dpo ? 1 : 0);
            if (zsz > 0) { // dead RAM (no read port) falls to the generic path
                if (z < 0 || z - zsz + 1 < 0) { flush(); z = 2; }
                int sp_slot = -1;
                if (spo) {
                    if (z == 2) sp_slot = 3; // fold into the D6LUT base
                    else { sp_slot = z; z--; }
                }
                int dp_slot = -1;
                if (dpo) { dp_slot = z; z--; }
                absorbed.insert(insts[ii].name);
                bump(ty + " grouped");
                cur.emplace_back(ii, sp_slot, dp_slot);
            }
        }
        flush();
    }
}

// ---------------------------- 1b. LUT + FF pairing -> SLICE_LOGIC ---------
// A 7-series slice shares ONE clock + ONE CE + ONE SRUSEDMUX, so FFs with
// differing control sets CANNOT share a slice.  CLOCK EDGE is part of the
// control set: a negedge flop cannot share a half-slice with posedge flops,
// and nothing complains until FASM -- after a full place AND route.
struct CtrlSet
{
    int c = NK_NONE, ce = NK_NONE, srn = NK_NONE;
    std::string srk;
    bool negedge = false;
    bool operator==(const CtrlSet &o) const
    {
        return c == o.c && ce == o.ce && srn == o.srn && srk == o.srk && negedge == o.negedge;
    }
    bool operator!=(const CtrlSet &o) const { return !(*this == o); }
};

void Packer::phase1b2_lut_ff_pairs()
{
    auto is_lut = [](const std::string &t) { return starts_with(upper(t), "LUT"); };
    auto is_ff = [](const std::string &t) { return starts_with(upper(t), "FD"); };

    int lane = 0, slice_no = 0;
    std::vector<std::pair<std::string, int>> slice_conns;
    std::vector<std::pair<std::string, std::string>> slice_bels;
    bool have_cs = false;
    CtrlSet cur_cs;
    auto flush_slice = [&]() {
        if (!slice_conns.empty()) {
            char buf[32];
            snprintf(buf, sizeof(buf), "logic_slice_%d", slice_no);
            std::vector<std::pair<std::string, std::string>> b(slice_bels.rbegin(), slice_bels.rend());
            std::vector<std::pair<std::string, int>> c(slice_conns.rbegin(), slice_conns.rend());
            add(buf, "SLICE_LOGIC", c, b);
            slice_no++;
            slice_conns.clear();
            slice_bels.clear();
            lane = 0;
            have_cs = false;
        }
    };
    auto ff_cs = [&](int ii) {
        CtrlSet cs;
        int v;
        if ((v = port_bit(ii, "R", 0)) != NK_NONE) { cs.srk = "R"; cs.srn = v; }
        else if ((v = port_bit(ii, "S", 0)) != NK_NONE) { cs.srk = "S"; cs.srn = v; }
        else if ((v = port_bit(ii, "PRE", 0)) != NK_NONE) { cs.srk = "PRE"; cs.srn = v; }
        else if ((v = port_bit(ii, "CLR", 0)) != NK_NONE) { cs.srk = "CLR"; cs.srn = v; }
        else { cs.srk = ""; cs.srn = NK_NONE; }
        std::string u = upper(insts[ii].type);
        cs.negedge = u.size() > 2 && u.compare(u.size() - 2, 2, "_1") == 0;
        cs.c = port_bit(ii, "C", 0);
        cs.ce = port_bit(ii, "CE", 0);
        return cs;
    };

    // PACK_CRIT_FILE: run the critical FFs FIRST and let them win the LUT.
    // Partitioning (not sorting) is deliberate -- a criticality sort would
    // interleave control sets and flush on nearly every FF.
    std::unordered_map<std::string, double> pack_crit;
    if (const char *path = getenv("PACK_CRIT_FILE")) {
        std::ifstream f(path);
        if (!f) fprintf(stderr, "PACK_CRIT_FILE: %s: cannot open\n", path);
        std::string line;
        while (std::getline(f, line)) {
            std::stringstream ss(line);
            std::string nm, val;
            if (!std::getline(ss, nm, '\t')) continue;
            if (!std::getline(ss, val, '\t')) continue;
            try { pack_crit[nm] = std::stod(val); } catch (...) {}
        }
    }
    std::vector<int> ff_pass_order;
    if (pack_crit.empty()) {
        for (size_t k = 0; k < insts.size(); k++) ff_pass_order.push_back(int(k));
    } else {
        double crit_min = 0.5;
        if (const char *s = getenv("PACK_CRIT_MIN")) { try { crit_min = std::stod(s); } catch (...) { crit_min = 0.5; } }
        auto crit_of = [&](int ii) {
            int nk = port_bit(ii, "D", 0);
            if (!is_net(nk)) return 0.0;
            auto dit = drv.find(nk);
            if (dit == drv.end()) return 0.0;
            auto cit = pack_crit.find(insts[std::get<0>(dit->second)].name);
            return cit == pack_crit.end() ? 0.0 : cit->second;
        };
        std::vector<int> hot, cold;
        int matched = 0, nff = 0;
        for (size_t k = 0; k < insts.size(); k++) {
            bool ff = is_ff(insts[k].type);
            if (ff) nff++;
            if (ff) {
                int nk = port_bit(int(k), "D", 0);
                if (is_net(nk)) {
                    auto dit = drv.find(nk);
                    if (dit != drv.end() && pack_crit.count(insts[std::get<0>(dit->second)].name)) matched++;
                }
            }
            if (ff && crit_of(int(k)) >= crit_min) hot.push_back(int(k));
            else cold.push_back(int(k));
        }
        // Report the JOIN RATE: two name spaces meet here and a silent 0%
        // match would look exactly like "criticality did not help".
        printf("crit-pack: %zu crit entries; %d/%d FFs have a known driver criticality; "
               "%zu above %.2f packed first\n",
               pack_crit.size(), matched, nff, hot.size(), crit_min);
        ff_pass_order = hot;
        ff_pass_order.insert(ff_pass_order.end(), cold.begin(), cold.end());
    }

    for (int ii : ff_pass_order) {
        auto &i = insts[ii];
        if (!is_ff(i.type) || absorbed.count(i.name)) continue;
        CtrlSet cs = ff_cs(ii);
        if (lane > 0 && !(have_cs && cur_cs == cs)) flush_slice();
        cur_cs = cs;
        have_cs = true;
        std::string l = lane_letter(lane);
        int dnet = port_bit(ii, "D", 0);
        // absorb the driving LUT if present and free
        if (is_net(dnet)) {
            auto dit = drv.find(dnet);
            if (dit != drv.end()) {
                int dn = std::get<0>(dit->second);
                if (is_lut(insts[dn].type) && !absorbed.count(insts[dn].name)) {
                    absorbed.insert(insts[dn].name);
                    bump("LUT+FF paired");
                    slice_bels.insert(slice_bels.begin(), {insts[dn].name, l + "6LUT"});
                    for (auto &pc : insts[dn].ports) {
                        if (upper(pc.first) == "O") continue;
                        if (pc.second.empty()) continue;
                        slice_conns.insert(slice_conns.begin(), {l + pc.first, pc.second[0]});
                    }
                }
            }
        }
        absorbed.insert(i.name);
        bump("FF packed");
        slice_bels.insert(slice_bels.begin(), {i.name, l + "FF"});
        int v;
        if ((v = port_bit(ii, "Q", 0)) != NK_NONE) slice_conns.insert(slice_conns.begin(), {l + "Q", v});
        if ((v = port_bit(ii, "C", 0)) != NK_NONE) slice_conns.insert(slice_conns.begin(), {"CLK", v});
        if ((v = port_bit(ii, "CE", 0)) != NK_NONE) slice_conns.insert(slice_conns.begin(), {"CE", v});
        if ((v = port_bit(ii, "R", 0)) != NK_NONE) slice_conns.insert(slice_conns.begin(), {"SR", v});
        else if ((v = port_bit(ii, "S", 0)) != NK_NONE) slice_conns.insert(slice_conns.begin(), {"SR", v});
        lane++;
        if (lane >= 4) flush_slice();
    }
    flush_slice();
}

// -------------------------- 2. dedicated sites: MMCM / BUFG / IO ----------
void Packer::phase2_dedicated_sites()
{
    for (size_t k = 0; k < insts.size(); k++) {
        auto &i = insts[k];
        if (absorbed.count(i.name) || i.type == "GND" || i.type == "VCC") continue;
        std::string lef;
        if (!io_map(i.type, lef)) continue;
        std::vector<std::pair<std::string, int>> conns;
        for (auto &pc : i.ports)
            if (!pc.second.empty()) conns.emplace_back(pc.first, pc.second[0]);
        // IO is XDC pin-constrained (nextpnr pack_io); only clock sites get an
        // explicit BEL suffix for the legaliser.
        std::vector<std::pair<std::string, std::string>> bels;
        if (lef == "BUFG") bels.emplace_back(i.name, "BUFGCTRL");
        else if (lef == "BUFH") bels.emplace_back(i.name, "BUFH");
        else if (lef == "MMCM") bels.emplace_back(i.name, "MMCME2_ADV");
        add(i.name + "$site", lef, conns, bels);
        absorbed.insert(i.name);
        bump(i.type + "->" + lef);
    }
}

// ------------------- 3a. leftover LUTs -> FILL THE SLICE (4 per slice) ----
// One LUT per slice put 3064 of 3486 LUT-bearing slices at exactly ONE LUT
// (mean 1.25 vs Vivado's 3.65 on the SAME netlist) and spread the design over
// 4692 slices where Vivado used 1669.  Fracturing (TOPO_LUT_FRACTURE=1) is OFF
// by default -- it MEASURED WORSE: 0 -> 342 skips, 120.35 -> 102.28 MHz.
void Packer::phase3a_leftover_luts()
{
    bool frac_on = false;
    if (const char *e = getenv("TOPO_LUT_FRACTURE")) frac_on = (strcmp(e, "1") == 0);

    struct LutItem
    {
        std::string name;
        std::vector<std::pair<std::string, int>> ports;
        std::vector<int> ins; // sort_uniq of the non-O port netkeys
    };
    std::vector<LutItem> lut_items;
    for (size_t k = 0; k < insts.size(); k++) {
        auto &i = insts[k];
        std::string l, s;
        if (absorbed.count(i.name) || i.type == "GND" || i.type == "VCC" ||
            hard_map(i.type, l, s) || slicem_map(i.type, l, s))
            continue;
        std::string u = upper(i.type);
        if (!(starts_with(u, "LUT") || u == "INV")) continue;
        LutItem it;
        it.name = i.name;
        for (auto &pc : i.ports)
            if (!pc.second.empty()) it.ports.emplace_back(pc.first, pc.second[0]);
        for (auto &pc : it.ports)
            if (upper(pc.first) != "O") it.ins.push_back(pc.second);
        std::sort(it.ins.begin(), it.ins.end());
        it.ins.erase(std::unique(it.ins.begin(), it.ins.end()), it.ins.end());
        lut_items.push_back(std::move(it));
    }
    std::unordered_map<std::string, size_t> idx_of;
    for (size_t k = 0; k < lut_items.size(); k++) idx_of[lut_items[k].name] = k;

    // greedy fracture pairing (widest first: a 5-input LUT has the fewest
    // legal partners, so let it choose before the 1- and 2-input ones)
    std::unordered_map<std::string, std::string> partner;
    std::unordered_set<std::string> taken;
    int npair = 0;
    if (frac_on) {
        std::unordered_map<int, std::vector<std::string>> by_net;
        for (auto &it : lut_items)
            if (it.ins.size() <= 5)
                for (int k : it.ins) by_net[k].insert(by_net[k].begin(), it.name);
        std::vector<size_t> order(lut_items.size());
        for (size_t k = 0; k < order.size(); k++) order[k] = k;
        std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            return lut_items[a].ins.size() > lut_items[b].ins.size();
        });
        for (size_t oi : order) {
            auto &it = lut_items[oi];
            if (it.ins.size() > 5 || taken.count(it.name)) continue;
            std::string best;
            size_t best_lu = 0;
            bool have = false;
            for (int k : it.ins) {
                auto bit = by_net.find(k);
                if (bit == by_net.end()) continue;
                for (auto &o : bit->second) {
                    if (o == it.name || taken.count(o)) continue;
                    auto oit = idx_of.find(o);
                    if (oit == idx_of.end()) continue;
                    auto &oi2 = lut_items[oit->second];
                    if (oi2.ins.size() > 5) continue;
                    std::vector<int> un = it.ins;
                    un.insert(un.end(), oi2.ins.begin(), oi2.ins.end());
                    std::sort(un.begin(), un.end());
                    un.erase(std::unique(un.begin(), un.end()), un.end());
                    if (un.size() > 5) continue;
                    if (have && best_lu <= un.size()) continue;
                    best = o;
                    best_lu = un.size();
                    have = true;
                }
            }
            if (have) {
                taken.insert(it.name);
                taken.insert(best);
                partner[it.name] = best;
                npair++;
            }
        }
    }

    // emit: each unit (a fractured pair or a lone LUT) takes one lane
    std::unordered_set<std::string> emitted;
    struct GroupItem
    {
        std::string name;
        std::vector<std::pair<std::string, int>> ports;
        bool has_second = false;
        std::string second_name;
        std::vector<std::pair<std::string, int>> second_ports;
    };
    std::vector<GroupItem> lut_group;
    int lut_slice = 0;
    auto flush_luts = [&]() {
        if (lut_group.empty()) return;
        std::vector<std::pair<std::string, std::string>> bels;
        std::vector<std::pair<std::string, int>> conns;
        for (size_t k = 0; k < lut_group.size(); k++) {
            std::string l = lane_letter(int(k));
            bels.emplace_back(lut_group[k].name, l + "6LUT");
            if (lut_group[k].has_second) bels.emplace_back(lut_group[k].second_name, l + "5LUT");
        }
        for (size_t k = 0; k < lut_group.size(); k++) {
            std::string l = lane_letter(int(k));
            for (auto &pc : lut_group[k].ports) conns.emplace_back(l + pc.first, pc.second);
            if (lut_group[k].has_second)
                for (auto &pc : lut_group[k].second_ports)
                    conns.emplace_back(l + "5" + pc.first, pc.second);
        }
        char buf[32];
        snprintf(buf, sizeof(buf), "lut_slice_%d", lut_slice);
        add(buf, "SLICE_LOGIC", conns, bels);
        lut_slice++;
        lut_group.clear();
    };
    for (auto &it : lut_items) {
        if (emitted.count(it.name)) continue;
        emitted.insert(it.name);
        GroupItem gi;
        gi.name = it.name;
        gi.ports = it.ports;
        auto pit = partner.find(it.name);
        if (pit != partner.end() && !emitted.count(pit->second)) {
            emitted.insert(pit->second);
            absorbed.insert(pit->second);
            bump("LUT fractured (O5)");
            gi.has_second = true;
            gi.second_name = pit->second;
            gi.second_ports = lut_items[idx_of[pit->second]].ports;
        }
        absorbed.insert(it.name);
        bump("LUT packed into shared slice");
        lut_group.push_back(std::move(gi));
        if (lut_group.size() >= 4) flush_luts();
    }
    flush_luts();
    if (npair > 0)
        printf("lut-fracture: %d pair(s) share a LUT6 site (%zu LUTs -> %zu sites)\n", npair,
               lut_items.size(), lut_items.size() - size_t(npair));
}

// ---------------- 3. leftover LUT / FF -> SLICE_LOGIC / SLICE_FF ----------
void Packer::phase3_leftover()
{
    for (size_t k = 0; k < insts.size(); k++) {
        auto &i = insts[k];
        if (absorbed.count(i.name) || i.type == "GND" || i.type == "VCC") continue;
        std::vector<std::pair<std::string, int>> conns;
        for (auto &pc : i.ports)
            if (!pc.second.empty()) conns.emplace_back(pc.first, pc.second[0]);
        std::string u = upper(i.type);
        std::string hlef, hsfx, mlef, msfx;
        if (hard_map(i.type, hlef, hsfx)) {
            add(i.name + "$hard", hlef, conns, {{i.name, hsfx}});
            bump(i.type + "->" + hlef);
        } else if (slicem_map(i.type, mlef, msfx)) {
            add(i.name + "$m", mlef, conns, {{i.name, msfx}});
            bump(i.type + "->" + mlef);
        } else if (starts_with(u, "LUT") || u == "INV") {
            add(i.name + "$logic", "SLICE_LOGIC", conns, {{i.name, "A6LUT"}});
            bump("LUT->SLICE_LOGIC");
        } else if (starts_with(u, "FD")) {
            add(i.name + "$ff", "SLICE_FF", conns, {{i.name, "AFF"}});
            bump("FF->SLICE_FF");
        } else {
            add(i.name + "$?", "UNKNOWN:" + i.type, conns);
            bump("UNMAPPED " + i.type);
        }
        absorbed.insert(i.name);
    }
}

void Packer::run()
{
    build_maps();
    phase0_site_guided();
    phase1_carry();
    phase1a0_r256_groups();
    phase1b_ram256_macro();
    phase1a_mux();
    phase1c_dram_groups();
    phase1b2_lut_ff_pairs();
    phase2_dedicated_sites();
    phase3a_leftover_luts();
    phase3_leftover();
}

// ===================== NETLIST PREPASSES ===================================
// place_lef mutates the netlist BEFORE packing and writes the result out; the
// downstream (carry_stamp.py, the router) consumes the mutated netlist.  These
// three are ported verbatim from place_lef_core.ml, in its own pipeline order.
// ===========================================================================

struct Netlist
{
    JPtr root;
};

// ---- small tree helpers ---------------------------------------------------
static JPtr jstr(const std::string &v) { auto j = std::make_shared<JVal>(); j->k = JVal::STR; j->str = v; return j; }
static JPtr jint(int v) { auto j = std::make_shared<JVal>(); j->k = JVal::NUM; j->num = v; return j; }
static JPtr jobj() { auto j = std::make_shared<JVal>(); j->k = JVal::OBJ; return j; }
static JPtr jarr() { auto j = std::make_shared<JVal>(); j->k = JVal::ARR; return j; }

static void set_member(JPtr o, const std::string &k, JPtr v)
{
    for (auto &kv : o->obj)
        if (kv.first == k) { kv.second = v; return; }
    o->obj.emplace_back(k, v);
}
static JPtr deep_copy(JPtr v)
{
    auto o = std::make_shared<JVal>();
    o->k = v->k; o->str = v->str; o->num = v->num; o->bol = v->bol;
    for (auto &kv : v->obj) o->obj.emplace_back(kv.first, deep_copy(kv.second));
    for (auto &e : v->arr) o->arr.push_back(deep_copy(e));
    return o;
}
// Only INTEGER bits; a "0"/"1"/"x"/"z" string is a constant, not a net.
static std::vector<int> bits_of(JPtr e)
{
    std::vector<int> out;
    if (e && e->k == JVal::ARR)
        for (auto &b : e->arr)
            if (b->k == JVal::NUM) out.push_back(int(b->num));
    return out;
}
static JPtr bit_list(const std::vector<int> &bs)
{
    JPtr a = jarr();
    for (int b : bs) a->arr.push_back(jint(b));
    return a;
}
static std::string type_of(JPtr cj)
{
    JPtr t = cj->member("type");
    return (t && t->k == JVal::STR) ? t->str : std::string();
}
static bool port_is_output(JPtr cj, const std::string &p)
{
    JPtr d = cj->member("port_directions");
    if (!d || d->k != JVal::OBJ) return false;
    JPtr v = d->member(p);
    return v && v->k == JVal::STR && v->str == "output";
}
static bool is_lut_type(const std::string &t) { return starts_with(upper(t), "LUT"); }
static bool is_widemux(const std::string &t) { return t == "MUXF7" || t == "MUXF8"; }

// Highest net id in use, so clones can take fresh ones.
static int max_bit(JPtr mj)
{
    int mx = 1;
    JPtr cells = mj->member("cells");
    if (cells && cells->k == JVal::OBJ)
        for (auto &c : cells->obj) {
            JPtr conns = c.second->member("connections");
            if (conns && conns->k == JVal::OBJ)
                for (auto &pc : conns->obj)
                    for (int b : bits_of(pc.second)) mx = std::max(mx, b);
        }
    JPtr nn = mj->member("netnames");
    if (nn && nn->k == JVal::OBJ)
        for (auto &n : nn->obj)
            for (int b : bits_of(n.second->member("bits"))) mx = std::max(mx, b);
    return mx;
}
static JPtr make_netname(const std::vector<int> &bs)
{
    JPtr n = jobj();
    set_member(n, "hide_name", jint(1));
    set_member(n, "bits", bit_list(bs));
    set_member(n, "attributes", jobj());
    return n;
}
static void append_cells(JPtr mj, std::vector<std::pair<std::string, JPtr>> &extra)
{
    if (extra.empty()) return;
    JPtr cells = mj->member("cells");
    for (auto &e : extra) cells->obj.emplace_back(e.first, e.second);
}
static void append_netnames(JPtr mj, std::vector<std::pair<std::string, JPtr>> &extra)
{
    if (extra.empty()) return;
    JPtr nn = mj->member("netnames");
    if (!nn || nn->k != JVal::OBJ) { nn = jobj(); set_member(mj, "netnames", nn); }
    for (auto &e : extra) nn->obj.emplace_back(e.first, e.second);
}

// ---- split_degenerate_muxf -----------------------------------------------
// Every wide-mux data pin needs a LUT of its OWN, in its own lane of its own
// slice.  Two pins sharing one driver is unsatisfiable whether they are I0/I1
// of the SAME mux (opt_merge collapsed an identical pair) or pins of two
// DIFFERENT muxes: a single instance cannot sit in two lanes.  First claimant
// keeps the original, every later one gets a clone.
//
// It also clones when the driver is shared with any NON-mux consumer, because
// CARRY4 absorbs its S-LUTs (pack rule 1) BEFORE the mux group (rule 1a), so a
// LUT shared between a MUXF7 data pin and a CARRY4.S pin is always lost to the
// carry chain and the mux pin then has to be fed from another slice -- which
// the dedicated F7/F8 path cannot do.
static void split_degenerate_muxf(JPtr mj, PrepassStats &st)
{
    JPtr cells = mj->member("cells");
    if (!cells || cells->k != JVal::OBJ || cells->obj.empty()) return;

    std::unordered_map<std::string, JPtr> byname;
    std::unordered_map<int, std::pair<std::string, std::string>> drv;
    for (auto &c : cells->obj) {
        byname[c.first] = c.second;
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        for (auto &pc : conns->obj)
            if (port_is_output(c.second, pc.first))
                for (int b : bits_of(pc.second)) drv[b] = {c.first, pc.first};
    }
    int mx = max_bit(mj);

    // consumers per bit, split into wide-mux data pins and everything else
    std::unordered_map<int, int> nuse, muxuse;
    for (auto &c : cells->obj) {
        bool ismux = is_widemux(type_of(c.second));
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        for (auto &pc : conns->obj) {
            if (port_is_output(c.second, pc.first)) continue;
            for (int b : bits_of(pc.second)) {
                nuse[b]++;
                if (ismux && (pc.first == "I0" || pc.first == "I1")) muxuse[b]++;
            }
        }
    }
    // a module port is a consumer too
    JPtr ports = mj->member("ports");
    if (ports && ports->k == JVal::OBJ)
        for (auto &p : ports->obj)
            for (int b : bits_of(p.second->member("bits"))) nuse[b]++;

    std::vector<std::pair<std::string, JPtr>> extra, extranets;
    auto clone_driver = [&](int b) -> int {
        auto it = drv.find(b);
        if (it == drv.end()) { st.mux_skipped++; return -1; }
        const std::string &dn = it->second.first, &dp = it->second.second;
        auto bit = byname.find(dn);
        if (bit == byname.end() || !is_lut_type(type_of(bit->second))) {
            // Driver is not a LUT -- a MUXF7 feeding a MUXF8 data pin, an FF, a
            // hard-block output.  Cloning it is not an option (that would change
            // the design).  The OCaml's MUXSPLIT_BUF branch is DEFAULT OFF and
            // stays unported: a bare LUT1 buffer measured WORSE (skips 4 -> 6,
            // userclk2 155 -> 116.67 MHz) because the constraint is not "a LUT
            // exists" but "a LUT in the mux's OWN slice", and a freshly-named
            // buffer is not associated with the mux so it lands elsewhere.
            st.mux_skipped++;
            return -1;
        }
        int fresh = ++mx;
        JPtr clone = deep_copy(bit->second);
        JPtr cc = clone->member("connections");
        if (cc && cc->k == JVal::OBJ) set_member(cc, dp, bit_list({fresh}));
        std::string nm = dn + "$muxdup";
        int k = 1;
        while (byname.count(nm)) { k++; nm = dn + "$muxdup" + std::to_string(k); }
        byname[nm] = clone;
        extra.emplace_back(nm, clone);
        extranets.emplace_back(nm + "." + dp, make_netname({fresh}));
        st.muxdup++;
        return fresh;
    };

    std::set<int> claimed;
    for (auto &c : cells->obj) {
        if (!is_widemux(type_of(c.second))) continue;
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        for (const char *pin : {"I0", "I1"}) {
            JPtr e = conns->member(pin);
            if (!e) continue;
            std::vector<int> bs = bits_of(e);
            if (bs.size() != 1) continue;
            int b = bs[0];
            int total = nuse.count(b) ? nuse[b] : 0;
            int nmux = muxuse.count(b) ? muxuse[b] : 0;
            bool stolen = (total - nmux) > 0;
            if (stolen || claimed.count(b)) {
                int fresh = clone_driver(b);
                if (fresh >= 0) {
                    if (stolen) st.mux_exclusive++;
                    claimed.insert(fresh);
                    set_member(conns, pin, bit_list({fresh}));
                }
            } else
                claimed.insert(b);
        }
    }
    append_cells(mj, extra);
    append_netnames(mj, extranets);
}

// ---- replicate_shared_muxf7 ----------------------------------------------
// Same fault as the shared carry chain, in the other dedicated-path structure.
// A MUXF7's output reaches a MUXF8 data pin over the F7->F8 path INSIDE one
// slice, so it can feed exactly one MUXF8.  When opt_merge shares a MUXF7
// between two MUXF8s, at most one is reachable and the other shows up as
//   SITEWIRE/.../F7BMUX_OUT -> SITEWIRE/.../F7AMUX_OUT
// Keep the first consumer on the original and give every later one a private
// clone of the MUXF7 and both its feeding LUTs.
static void replicate_shared_muxf7(JPtr mj, PrepassStats &st)
{
    JPtr cells = mj->member("cells");
    if (!cells || cells->k != JVal::OBJ || cells->obj.empty()) return;

    std::unordered_map<std::string, JPtr> byname;
    for (auto &c : cells->obj) byname[c.first] = c.second;
    int mx = max_bit(mj);

    std::unordered_map<int, std::pair<std::string, std::string>> drv;
    // MUXF8 consumers per bit, in CELL ORDER (the OCaml prepends then reverses)
    std::map<int, std::vector<std::pair<std::string, std::string>>> f8_users;
    for (auto &c : cells->obj) {
        bool is_f8 = type_of(c.second) == "MUXF8";
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        for (auto &pc : conns->obj) {
            bool out = port_is_output(c.second, pc.first);
            for (int b : bits_of(pc.second)) {
                if (out)
                    drv[b] = {c.first, pc.first};
                else if (is_f8 && (pc.first == "I0" || pc.first == "I1"))
                    f8_users[b].emplace_back(c.first, pc.first);
            }
        }
    }

    std::vector<std::pair<std::string, JPtr>> extra;
    std::vector<std::tuple<std::string, std::string, int>> rewire;
    // The OCaml iterates a Hashtbl here, whose order is UNSPECIFIED; each bit
    // is independent, so ascending bit order is equivalent and deterministic.
    for (auto &fu : f8_users) {
        int b = fu.first;
        auto &users = fu.second;
        if (users.size() <= 1) continue;
        auto dit = drv.find(b);
        if (dit == drv.end()) continue;
        const std::string &mn = dit->second.first;
        auto mit = byname.find(mn);
        if (mit == byname.end() || type_of(mit->second) != "MUXF7") continue;

        for (size_t i = 1; i < users.size(); i++) { // first consumer keeps the original
            int nb = ++mx;
            JPtr m7 = deep_copy(mit->second);
            JPtr conns = m7->member("connections");
            JPtr orig = mit->second->member("connections");
            if (conns && conns->k == JVal::OBJ) {
                set_member(conns, "O", bit_list({nb}));
                for (const char *pin : {"I0", "I1"}) {
                    std::vector<int> ib = bits_of(orig->member(pin));
                    if (ib.size() != 1) continue;
                    auto ldit = drv.find(ib[0]);
                    if (ldit == drv.end()) continue;
                    const std::string &ln = ldit->second.first;
                    auto lit = byname.find(ln);
                    if (lit == byname.end() || !is_lut_type(type_of(lit->second))) continue;
                    int lb = ++mx;
                    JPtr lclone = deep_copy(lit->second);
                    JPtr lc = lclone->member("connections");
                    if (lc && lc->k == JVal::OBJ)
                        for (auto &lp : lc->obj)
                            if (upper(lp.first) == "O") lp.second = bit_list({lb});
                    extra.emplace_back(ln + "_rep" + std::to_string(i) + "_" + pin, lclone);
                    set_member(conns, pin, bit_list({lb}));
                }
            }
            extra.emplace_back(mn + "_rep" + std::to_string(i) + "_m7", m7);
            rewire.emplace_back(users[i].first, users[i].second, nb);
            st.muxf7_rep++;
        }
    }
    for (auto &rw : rewire) {
        auto it = byname.find(std::get<0>(rw));
        if (it == byname.end()) continue;
        JPtr conns = it->second->member("connections");
        if (conns && conns->k == JVal::OBJ)
            set_member(conns, std::get<1>(rw), bit_list({std::get<2>(rw)}));
    }
    append_cells(mj, extra);
}

// ---- replicate_shared_carry ----------------------------------------------
// COUT->CIN is a dedicated point-to-point wire reaching only the slice directly
// above, so when a CARRY4's CO[3] feeds MORE THAN ONE downstream CI at most one
// of them can be reached through the cascade.  Give every extra user its own
// clone of the whole chain from the root down, S/DI driving LUTs included, so
// each cascade is private.
static void replicate_shared_carry(JPtr mj, PrepassStats &st)
{
    JPtr cells = mj->member("cells");
    if (!cells || cells->k != JVal::OBJ || cells->obj.empty()) return;

    std::unordered_map<std::string, JPtr> byname;
    for (auto &c : cells->obj) byname[c.first] = c.second;
    int mx = max_bit(mj);

    // driver map: bit -> (cell, port, index-within-port)
    std::unordered_map<int, std::tuple<std::string, std::string, int>> drv;
    for (auto &c : cells->obj) {
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        for (auto &pc : conns->obj)
            if (port_is_output(c.second, pc.first)) {
                std::vector<int> bs = bits_of(pc.second);
                for (size_t i = 0; i < bs.size(); i++)
                    drv[bs[i]] = std::make_tuple(c.first, pc.first, int(i));
            }
    }
    auto conn_bits = [&](const std::string &cn, const char *port) {
        std::vector<int> out;
        auto it = byname.find(cn);
        if (it == byname.end()) return out;
        JPtr conns = it->second->member("connections");
        if (!conns || conns->k != JVal::OBJ) return out;
        return bits_of(conns->member(port));
    };
    auto co3 = [&](const std::string &cn) {
        std::vector<int> b = conn_bits(cn, "CO");
        return b.size() == 4 ? b[3] : -1;
    };
    auto ci_of = [&](const std::string &cn) {
        std::vector<int> b = conn_bits(cn, "CI");
        return b.empty() ? -1 : b[0];
    };
    // CI consumers per bit
    std::unordered_map<int, std::vector<std::string>> ci_users;
    for (auto &c : cells->obj)
        if (type_of(c.second) == "CARRY4") {
            int b = ci_of(c.first);
            if (b >= 0) ci_users[b].insert(ci_users[b].begin(), c.first);
        }

    std::vector<std::pair<std::string, JPtr>> extra;
    std::vector<std::tuple<std::string, std::string, int>> rewire;
    for (auto &c : cells->obj) {
        if (type_of(c.second) != "CARRY4") continue;
        int b = co3(c.first);
        if (b < 0) continue;
        auto uit = ci_users.find(b);
        if (uit == ci_users.end() || uit->second.size() <= 1) continue;

        // walk back from this cell to the root of its cascade
        std::vector<std::string> chain;
        {
            std::string cur = c.first;
            std::set<std::string> guard;
            for (;;) {
                chain.insert(chain.begin(), cur);
                if (!guard.insert(cur).second) break;
                int cb = ci_of(cur);
                if (cb < 0) break;
                auto dit = drv.find(cb);
                if (dit == drv.end()) break;
                if (std::get<1>(dit->second) != "CO" || std::get<2>(dit->second) != 3) break;
                const std::string &pn = std::get<0>(dit->second);
                if (!byname.count(pn) || type_of(byname[pn]) != "CARRY4") break;
                if (guard.count(pn)) break;
                cur = pn;
            }
        }
        std::vector<std::string> users(uit->second.rbegin(), uit->second.rend());
        for (size_t ui = 0; ui < users.size(); ui++) {
            if (ui == 0) continue; // first user keeps the original cascade
            st.carry_chains++;
            int prev_co = -1;
            for (auto &rung : chain) {
                st.carry_rungs++;
                JPtr rj = byname[rung];
                JPtr rconns = rj->member("connections");
                if (!rconns || rconns->k != JVal::OBJ) continue;
                JPtr clone = deep_copy(rj);
                JPtr cc = clone->member("connections");
                // fresh CO bits for this clone
                std::vector<int> newco;
                for (size_t q = 0; q < bits_of(rconns->member("CO")).size(); q++) newco.push_back(++mx);
                if (!newco.empty()) set_member(cc, "CO", bit_list(newco));
                // sum outputs: fresh nets, unused by the clone
                {
                    std::vector<int> o = bits_of(rconns->member("O")), no;
                    for (size_t q = 0; q < o.size(); q++) no.push_back(++mx);
                    if (!no.empty()) set_member(cc, "O", bit_list(no));
                }
                // CI: chain to our own clone; the root keeps the original net
                if (prev_co >= 0) set_member(cc, "CI", bit_list({prev_co}));
                // clone the S/DI driving LUTs so the copy is independent
                for (const char *pin : {"S", "DI"}) {
                    JPtr e = rconns->member(pin);
                    if (!e) continue;
                    std::vector<int> nb;
                    for (int ib : bits_of(e)) {
                        auto dit = drv.find(ib);
                        if (dit == drv.end()) { nb.push_back(ib); continue; }
                        const std::string &ln = std::get<0>(dit->second);
                        auto lit = byname.find(ln);
                        if (lit == byname.end() || !is_lut_type(type_of(lit->second))) {
                            nb.push_back(ib);
                            continue;
                        }
                        int fresh = ++mx;
                        JPtr lclone = deep_copy(lit->second);
                        JPtr lc = lclone->member("connections");
                        if (lc && lc->k == JVal::OBJ)
                            for (auto &lp : lc->obj)
                                if (upper(lp.first) == "O") lp.second = bit_list({fresh});
                        extra.emplace_back(ln + "_carrep" + std::to_string(ui), lclone);
                        nb.push_back(fresh);
                    }
                    set_member(cc, pin, bit_list(nb));
                }
                if (!newco.empty()) prev_co = newco.back();
                extra.emplace_back(rung + "_carrep" + std::to_string(ui), clone);
            }
            if (prev_co >= 0) rewire.emplace_back(users[ui], "CI", prev_co);
        }
    }
    for (auto &rw : rewire) {
        auto it = byname.find(std::get<0>(rw));
        if (it == byname.end()) continue;
        JPtr conns = it->second->member("connections");
        if (conns && conns->k == JVal::OBJ)
            set_member(conns, std::get<1>(rw), bit_list({std::get<2>(rw)}));
    }
    append_cells(mj, extra);
}

// ---- materialise_const_drivers -------------------------------------------
// A MUXF7/MUXF8 whose I0/I1 is tied to a constant has no cell driving that pin,
// so nextpnr's packer INVENTS one ($PACKER_GND_NET$LUT$n) which place_lef never
// placed -- "ERROR: Found unbound cell".  Give the packer nothing to invent.
//
// I0 is driven from a FLIP-FLOP Q, not from the constant it replaced: tying it
// to the constant makes the timing engine see the const network arriving at a
// cell that drives it, and leaving it dangling invites the packer to tie it to
// a constant itself, recreating the cell this pass exists to remove.  INIT
// ignores I0, so any driven net works; pick the FF whose name shares the
// longest prefix with the mux so the extra load stays local.
static void materialise_const_drivers(JPtr mj, PrepassStats &st)
{
    JPtr cells = mj->member("cells");
    if (!cells || cells->k != JVal::OBJ || cells->obj.empty()) return;
    int mx = max_bit(mj);

    std::vector<std::pair<std::string, int>> ff_qs;
    for (auto &c : cells->obj) {
        if (!starts_with(type_of(c.second), "FD")) continue;
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        std::vector<int> q = bits_of(conns->member("Q"));
        if (q.size() == 1) ff_qs.emplace_back(c.first, q[0]);
    }
    auto pick_ff_for = [&](const std::string &cn) {
        int best = -1;
        size_t bestlen = 0;
        bool have = false;
        for (auto &f : ff_qs) {
            size_t n = std::min(cn.size(), f.first.size()), i = 0;
            while (i < n && cn[i] == f.first[i]) i++;
            if (!have || i > bestlen) { have = true; bestlen = i; best = f.second; }
        }
        return best;
    };

    std::vector<std::pair<std::string, JPtr>> extra;
    for (auto &c : cells->obj) {
        if (!is_widemux(type_of(c.second))) continue;
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        for (auto &pc : conns->obj) {
            if (pc.first != "I0" && pc.first != "I1") continue;
            JPtr e = pc.second;
            if (!e || e->k != JVal::ARR || e->arr.size() != 1) continue;
            if (e->arr[0]->k != JVal::STR) continue;
            const std::string &k = e->arr[0]->str;
            if (k != "0" && k != "1") continue;
            int nb = ++mx;
            JPtr lut = jobj();
            set_member(lut, "hide_name", jint(0));
            set_member(lut, "type", jstr("LUT1"));
            JPtr params = jobj();
            set_member(params, "INIT", jstr(k == "0" ? "2'h0" : "2'h3"));
            set_member(lut, "parameters", params);
            JPtr dirs = jobj();
            set_member(dirs, "I0", jstr("input"));
            set_member(dirs, "O", jstr("output"));
            set_member(lut, "port_directions", dirs);
            JPtr lc = jobj();
            int q = pick_ff_for(c.first);
            if (q >= 0)
                set_member(lc, "I0", bit_list({q}));
            else {
                JPtr a = jarr();
                a->arr.push_back(jstr(k));
                set_member(lc, "I0", a);
            }
            set_member(lc, "O", bit_list({nb}));
            set_member(lut, "connections", lc);
            extra.emplace_back(c.first + "_const_" + pc.first, lut);
            pc.second = bit_list({nb});
            st.consts++;
        }
    }
    append_cells(mj, extra);
}

// ---- normalise_init -------------------------------------------------------
// Turn every Verilog numeric literal parameter ("N'hV" / "N'bV" / "N'dV") into
// an MSB-first bit-string of N bits.  nextpnr stores numeric params as a string
// of [01xz] and asserts on anything else:
//   Assertion failure: str[i] == S0 || str[i] == S1 || str[i] == Sx || str[i] == Sz
//     (common/nextpnr.h:342)
// It aborts in POST-ROUTING LEGALISATION, long after a clean route, and
// truncates the fasm -- observed on ethmin after materialise_const_drivers,
// which writes INIT as 2'h0 and relies on this pass to rewrite it to "00".
// A genuine string param (an enum/mode) carries no apostrophe and is left alone.
static int normalise_init(JPtr mj)
{
    JPtr cells = mj->member("cells");
    if (!cells || cells->k != JVal::OBJ) return 0;
    auto hex_bits = [](const std::string &digits, std::string &out) {
        for (char c : digits) {
            int v;
            if (c >= '0' && c <= '9') v = c - '0';
            else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
            else if (c == 'x' || c == 'X' || c == 'z' || c == 'Z') v = 0;
            else return false;
            for (int k = 3; k >= 0; k--) out += ((v >> k) & 1) ? '1' : '0';
        }
        return true;
    };
    auto lit_to_bits = [&](const std::string &s, std::string &out) {
        size_t q = s.find('\'');
        if (q == std::string::npos || q == 0 || q + 1 >= s.size()) return false;
        int width = 0;
        try {
            width = std::stoi(s.substr(0, q));
        } catch (...) {
            return false;
        }
        if (width <= 0) return false;
        char base = char(tolower(s[q + 1]));
        std::string digits;
        for (size_t i = q + 2; i < s.size(); i++)
            if (s[i] != '_') digits += s[i];
        std::string raw;
        if (base == 'h') {
            if (!hex_bits(digits, raw)) return false;
        } else if (base == 'b') {
            for (char c : digits)
                raw += (c == 'x' || c == 'X' || c == 'z' || c == 'Z') ? '0' : c;
        } else if (base == 'd' && width <= 62) {
            int64_t v = 0;
            try {
                v = std::stoll(digits);
            } catch (...) {
                return false;
            }
            for (int i = 0; i < width; i++) raw += ((v >> (width - 1 - i)) & 1) ? '1' : '0';
        } else
            return false;
        int n = int(raw.size());
        out = (n >= width) ? raw.substr(n - width, width)
                           : std::string(width - n, '0') + raw;
        return true;
    };
    int nfix = 0;
    for (auto &c : cells->obj) {
        JPtr ps = c.second->member("parameters");
        if (!ps || ps->k != JVal::OBJ) continue;
        for (auto &pv : ps->obj) {
            if (pv.second->k != JVal::STR) continue;
            if (pv.second->str.find('\'') == std::string::npos) continue;
            std::string bits;
            if (lit_to_bits(pv.second->str, bits)) {
                pv.second = jstr(bits);
                nfix++;
            }
        }
    }
    return nfix;
}

// ===================== carry_stamp ========================================
// Port of carry_stamp.py.  Operates on the same tree, AFTER placement, using
// the placer's bels.  See the header for why it is not optional.
// ==========================================================================
static const char *SLOT = "ABCD";

CarryStampStats carry_stamp_tree(JPtr mj, const std::vector<std::pair<std::string, std::string>> &bels,
                                 const std::set<std::string> &known_sites)
{
    CarryStampStats st;
    JPtr cells = mj->member("cells");
    if (!cells || cells->k != JVal::OBJ) return st;

    // CARRY_STAMP_AVOID_CI: don't use the DEDICATED incoming carry (CI = the
    // previous CARRY4.CO) as the don't-care local input for a const-forced
    // S/DI LUT.  Reading it forces that CO onto general routing, where it
    // collides with the previous slice's sum O[k] on the single position-k
    // output mux (DMUX over-commit -> 128 unroutable nets).
    const char *aci = getenv("CARRY_STAMP_AVOID_CI");
    const bool AVOID_CI = aci != nullptr && aci[0] != '\0' && strcmp(aci, "0") != 0;

    std::unordered_map<std::string, JPtr> byname;
    for (auto &c : cells->obj) byname[c.first] = c.second;

    // 1) apply the plain BEL stamps from the placement
    for (auto &b : bels) {
        auto it = byname.find(b.first);
        if (it == byname.end()) continue;
        JPtr at = it->second->member("attributes");
        if (!at || at->k != JVal::OBJ) { at = jobj(); set_member(it->second, "attributes", at); }
        set_member(at, "BEL", jstr(b.second));
    }

    // GND/VCC-driven net bits.  With NEXTPNR_JSON_CONST_STRINGS=1 constants ALSO
    // appear as the string bits "0"/"1" directly on pins; treat both uniformly.
    std::set<int> gnd_bits, vcc_bits;
    for (auto &c : cells->obj) {
        std::string t = type_of(c.second);
        if (t != "GND" && t != "VCC") continue;
        std::set<int> &tgt = (t == "GND") ? gnd_bits : vcc_bits;
        JPtr conns = c.second->member("connections");
        if (conns && conns->k == JVal::OBJ)
            for (auto &pc : conns->obj)
                for (int b : bits_of(pc.second)) tgt.insert(b);
    }
    // a "bit" here is either an int net or a const string; -1 marks "not an int"
    auto is_gnd_bit = [&](JPtr e, size_t i) {
        if (!e || e->k != JVal::ARR || i >= e->arr.size()) return false;
        JPtr v = e->arr[i];
        if (v->k == JVal::STR) return v->str == "0";
        return v->k == JVal::NUM && gnd_bits.count(int(v->num)) > 0;
    };
    auto is_vcc_bit = [&](JPtr e, size_t i) {
        if (!e || e->k != JVal::ARR || i >= e->arr.size()) return false;
        JPtr v = e->arr[i];
        if (v->k == JVal::STR) return v->str == "1";
        return v->k == JVal::NUM && vcc_bits.count(int(v->num)) > 0;
    };
    auto int_at = [](JPtr e, size_t i) { // -1 if absent or not an int
        if (!e || e->k != JVal::ARR || i >= e->arr.size()) return -1;
        return e->arr[i]->k == JVal::NUM ? int(e->arr[i]->num) : -1;
    };

    // driver of each integer net bit -> (cell, type, port)
    std::unordered_map<int, std::tuple<std::string, std::string, std::string>> drv;
    for (auto &c : cells->obj) {
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        for (auto &pc : conns->obj)
            if (port_is_output(c.second, pc.first))
                for (int b : bits_of(pc.second))
                    drv[b] = std::make_tuple(c.first, type_of(c.second), pc.first);
    }

    // Global fallback net for const-LUT inputs: a const generator (INIT 00/11)
    // ignores its input, so any net routable on GENERAL interconnect works.
    // Highest-fanout NON-CLOCK net (clock nets ride dedicated routing and will
    // not reach a LUT data pin).
    std::map<int, int> fan;
    std::set<int> clk;
    for (auto &c : cells->obj) {
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        for (auto &pc : conns->obj)
            for (int b : bits_of(pc.second)) {
                fan[b]++;
                if (pc.first == "CLK" || pc.first == "C" || pc.first == "WCLK") clk.insert(b);
            }
    }
    int GLOBAL_FALLBACK_NET = -1;
    {
        int best = -1, bestn = 0;
        for (auto &kv : fan)
            if (!clk.count(kv.first) && kv.second > bestn) { bestn = kv.second; best = kv.first; }
        GLOBAL_FALLBACK_NET = best;
    }
    int maxbit = max_bit(mj);

    std::unordered_map<std::string, std::string> occupied; // bel -> cell
    std::set<std::string> carry_sites;
    for (auto &c : cells->obj) {
        JPtr at = c.second->member("attributes");
        JPtr b = at && at->k == JVal::OBJ ? at->member("BEL") : nullptr;
        if (!b || b->k != JVal::STR || b->str.empty()) continue;
        occupied[b->str] = c.first;
        if (b->str.size() > 7 && b->str.compare(b->str.size() - 7, 7, "/CARRY4") == 0)
            carry_sites.insert(b->str.substr(0, b->str.find('/')));
    }
    auto claim = [&](const std::string &bel, const std::string &who) {
        auto it = occupied.find(bel);
        if (it != occupied.end() && it->second != who) return false;
        occupied[bel] = who;
        return true;
    };

    // A CARRY4 slice's 6LUT slots are reserved for its OWN S/DI buffers; do not
    // borrow one for a neighbour's DIgnd const.
    auto free_neighbour_lut_global = [&](const std::string &site) -> std::string {
        int x, y;
        if (sscanf(site.c_str(), "SLICE_X%dY%d", &x, &y) != 2) return "";
        static const int d[10][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1},{2,0},{-2,0}};
        for (auto &dd : d) {
            std::string ns = "SLICE_X" + std::to_string(x + dd[0]) + "Y" + std::to_string(y + dd[1]);
            if (!known_sites.empty() && !known_sites.count(ns)) continue;
            if (carry_sites.count(ns)) continue;
            for (int sl = 0; sl < 4; sl++) {
                std::string bel = ns + "/" + SLOT[sl] + std::string("6LUT");
                if (!occupied.count(bel)) return bel;
            }
        }
        return "";
    };

    auto mk_lut = [&](const std::string &type, const std::vector<std::pair<std::string, JPtr>> &conns,
                      const std::string &init, const std::string &bel) {
        JPtr cell = jobj();
        set_member(cell, "type", jstr(type));
        JPtr dirs = jobj();
        for (auto &pc : conns) set_member(dirs, pc.first, jstr(pc.first == "O" ? "output" : "input"));
        set_member(cell, "port_directions", dirs);
        JPtr cc = jobj();
        for (auto &pc : conns) set_member(cc, pc.first, pc.second);
        set_member(cell, "connections", cc);
        JPtr par = jobj();
        set_member(par, "INIT", jstr(init));
        set_member(cell, "parameters", par);
        JPtr at = jobj();
        set_member(at, "BEL", jstr(bel));
        set_member(cell, "attributes", at);
        return cell;
    };
    auto cell_inputs = [&](const std::string &nm, std::vector<int> &out) {
        auto it = byname.find(nm);
        if (it == byname.end()) return;
        JPtr conns = it->second->member("connections");
        if (!conns || conns->k != JVal::OBJ) return;
        for (const char *pp : {"I0", "I1", "I2", "I3", "I4", "I5"}) {
            JPtr v = conns->member(pp);
            int b = int_at(v, 0);
            if (b >= 0) out.push_back(b);
        }
    };

    // sum-FF lookup: first FD* in CELL ORDER whose D[0] is this bit (the Python
    // scans cells in order and breaks on the first match)
    std::unordered_map<int, std::string> ff_by_d;
    for (auto &c : cells->obj) {
        if (!starts_with(type_of(c.second), "FD")) continue;
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        int d = int_at(conns->member("D"), 0);
        if (d >= 0 && !ff_by_d.count(d)) ff_by_d[d] = c.first;
    }

    std::vector<std::pair<std::string, JPtr>> new_cells;
    // snapshot: the Python iterates list(cells.items()), i.e. the ORIGINAL list
    std::vector<std::pair<std::string, JPtr>> snapshot = cells->obj;
    for (auto &c : snapshot) {
        if (type_of(c.second) != "CARRY4") continue;
        JPtr at = c.second->member("attributes");
        JPtr bj = at && at->k == JVal::OBJ ? at->member("BEL") : nullptr;
        if (!bj || bj->k != JVal::STR) continue;
        const std::string bel = bj->str;
        if (bel.size() < 7 || bel.compare(bel.size() - 7, 7, "/CARRY4") != 0) continue;
        const std::string site = bel.substr(0, bel.find('/'));
        const std::string &cn = c.first;
        JPtr conns = c.second->member("connections");
        if (!conns || conns->k != JVal::OBJ) continue;
        JPtr S = conns->member("S"), O = conns->member("O"), DI = conns->member("DI");

        // a real routable net already present at this slice, for const LUT inputs
        auto slice_local_net = [&]() -> int {
            int best = -1;
            if (S && S->k == JVal::ARR)
                for (size_t i = 0; i < S->arr.size(); i++) {
                    int sb2 = int_at(S, i);
                    if (sb2 < 0 || gnd_bits.count(sb2)) continue;
                    auto d2 = drv.find(sb2);
                    if (d2 != drv.end() && starts_with(std::get<1>(d2->second), "FD"))
                        return sb2; // FF-driven S bit: ideal
                    if (AVOID_CI && best < 0) best = sb2;
                }
            if (AVOID_CI) {
                if (best >= 0) return best;
                int v = int_at(conns->member("CYINIT"), 0);
                if (v >= 0 && !gnd_bits.count(v)) {
                    auto d = drv.find(v);
                    if (d == drv.end() || std::get<1>(d->second) != "CARRY4") return v;
                }
            } else {
                for (const char *pn : {"CYINIT", "CI"}) {
                    int v = int_at(conns->member(pn), 0);
                    if (v >= 0 && !gnd_bits.count(v)) return v;
                }
            }
            return GLOBAL_FALLBACK_NET;
        };

        int slot_in[4] = {-1, -1, -1, -1};
        // --- S inputs ---
        if (S && S->k == JVal::ARR) {
            JPtr newS = deep_copy(S);
            for (size_t k = 0; k < S->arr.size() && k < 4; k++) {
                std::string slot6 = site + "/" + SLOT[k] + std::string("6LUT");
                int sb = int_at(S, k);
                bool is_gnd = is_gnd_bit(S, k), is_vcc = is_vcc_bit(S, k);
                auto d = sb >= 0 ? drv.find(sb) : drv.end();
                if (d != drv.end() && starts_with(std::get<1>(d->second), "LUT")) {
                    // LUT-driven: stamp that S-LUT into THIS carry's slot -- but
                    // only if it is not already committed elsewhere.  CARRY4.S[k]
                    // is a DEDICATED same-slice connection, so one LUT cannot
                    // serve two carries; when it is placed elsewhere, fall
                    // through and buffer locally.
                    const std::string &dn = std::get<0>(d->second);
                    JPtr dat = byname[dn]->member("attributes");
                    JPtr dbel = dat && dat->k == JVal::OBJ ? dat->member("BEL") : nullptr;
                    bool free_here = !occupied.count(slot6) || occupied[slot6] == dn;
                    if ((!dbel || dbel->k != JVal::STR || dbel->str == slot6) && free_here) {
                        claim(slot6, dn);
                        if (!dat || dat->k != JVal::OBJ) {
                            dat = jobj();
                            set_member(byname[dn], "attributes", dat);
                        }
                        set_member(dat, "BEL", jstr(slot6));
                        std::vector<int> ins;
                        cell_inputs(dn, ins);
                        if (!ins.empty()) slot_in[k] = ins[0];
                        st.n_slut++;
                        continue;
                    }
                }
                int onet = ++maxbit;
                std::string bufname = cn + "$Srt$" + std::to_string(k);
                JPtr buf;
                if (is_gnd || is_vcc) {
                    // S=GND/VCC: const generator fed by a LOCAL net, INIT 00/11;
                    // needs no global GND/VCC routing.
                    int src = slice_local_net();
                    if (src < 0) continue;
                    buf = mk_lut("LUT1", {{"I0", bit_list({src})}, {"O", bit_list({onet})}},
                                 is_vcc ? "11" : "00", slot6);
                    slot_in[k] = src;
                } else {
                    if (sb < 0) continue;
                    // FF / external -> identity buffer passing the driver to S
                    buf = mk_lut("LUT1", {{"I0", bit_list({sb})}, {"O", bit_list({onet})}}, "10",
                                 slot6);
                    slot_in[k] = sb;
                }
                claim(slot6, bufname);
                new_cells.emplace_back(bufname, buf);
                byname[bufname] = buf;
                newS->arr[k] = jint(onet);
                st.n_buf++;
            }
            set_member(conns, "S", newS);
        }

        // --- DI ---
        std::vector<int> pending_gnd_di, pending_vcc_di;
        if (DI && DI->k == JVal::ARR) {
            JPtr newDI = deep_copy(DI);
            for (size_t k = 0; k < DI->arr.size() && k < 4; k++) {
                bool is_gnd = is_gnd_bit(DI, k), is_vcc = is_vcc_bit(DI, k);
                int db = int_at(DI, k);
                if (db < 0 && !is_gnd && !is_vcc) continue;
                auto d = db >= 0 ? drv.find(db) : drv.end();
                if (d != drv.end() && !is_gnd && !is_vcc) {
                    const std::string &dt = std::get<1>(d->second);
                    // LUT1-5-driven DI: nextpnr adopts the driver into the 5LUT.
                    // A LUT6 has no O5, so that one still needs a buffer.
                    if (dt == "LUT1" || dt == "LUT2" || dt == "LUT3" || dt == "LUT4" || dt == "LUT5")
                        continue;
                }
                std::string slot5 = site + "/" + SLOT[k] + std::string("5LUT");
                if (occupied.count(slot5)) continue;
                std::string slot6 = site + "/" + SLOT[k] + std::string("6LUT");
                std::vector<int> occ_ins;
                auto oit = occupied.find(slot6);
                if (oit != occupied.end()) cell_inputs(oit->second, occ_ins);
                std::set<int> occ_set(occ_ins.begin(), occ_ins.end());
                if (!is_gnd && !is_vcc) {
                    // fracture legality: the 5LUT shares A1-A5 with the 6LUT
                    if (!occ_set.count(db) && occ_set.size() + 1 > 5) continue;
                }
                int onet = ++maxbit;
                JPtr buf;
                std::string tag;
                if (is_gnd || is_vcc) {
                    // ILLEGAL when the occupant uses >=6 inputs (Vivado 18-608):
                    // the fractured LUT's O6 reads the upper INIT half with A6
                    // tied high while the 5LUT OVERWRITES the lower half,
                    // corrupting the LUT6 in hardware -- this silently broke the
                    // SGMII AN comparators and the link never came up.
                    if (occ_set.size() >= 6 || slot_in[k] < 0) {
                        (is_vcc ? pending_vcc_di : pending_gnd_di).push_back(int(k));
                        continue;
                    }
                    tag = is_vcc ? "DIvcc" : "DIgnd";
                    buf = mk_lut("LUT1", {{"I0", bit_list({slot_in[k]})}, {"O", bit_list({onet})}},
                                 is_vcc ? "11" : "00", slot5);
                } else {
                    // PIN-ALIGN with the 6LUT occupant: nextpnr pin-maps each
                    // fractured LUT's I0->A1, I1->A2..., so a different net on I0
                    // double-books sitewire A1 (the whole "->A1 unroutable" class).
                    tag = "DIrt";
                    if (occ_set.size() >= 6) continue;
                    auto pos = std::find(occ_ins.begin(), occ_ins.end(), db);
                    if (pos != occ_ins.end()) occ_ins.erase(pos, occ_ins.end());
                    if (occ_ins.size() + 1 > 5) continue;
                    size_t n_in = occ_ins.size() + 1;
                    std::string init = std::string(size_t(1) << (n_in - 1), '1') +
                                       std::string(size_t(1) << (n_in - 1), '0');
                    std::vector<std::pair<std::string, JPtr>> cc;
                    for (size_t i = 0; i < occ_ins.size(); i++)
                        cc.emplace_back("I" + std::to_string(i), bit_list({occ_ins[i]}));
                    cc.emplace_back("I" + std::to_string(n_in - 1), bit_list({db}));
                    cc.emplace_back("O", bit_list({onet}));
                    buf = mk_lut("LUT" + std::to_string(n_in), cc, init, slot5);
                }
                std::string bufname = cn + "$" + tag + "$" + std::to_string(k);
                claim(slot5, bufname);
                new_cells.emplace_back(bufname, buf);
                byname[bufname] = buf;
                newDI->arr[k] = jint(onet);
                st.n_di++;
            }
            // per-carry const in a NEIGHBOUR slice; DI enters via the AX bypass
            for (int pass = 0; pass < 2; pass++) {
                std::vector<int> &pend = pass ? pending_vcc_di : pending_gnd_di;
                if (pend.empty()) continue;
                int src = slice_local_net();
                std::string rbel = src >= 0 ? free_neighbour_lut_global(site) : "";
                if (rbel.empty()) continue;
                int onet = ++maxbit;
                std::string gname = cn + (pass ? "$DIvccx" : "$DIgndx");
                JPtr buf = mk_lut("LUT1", {{"I0", bit_list({src})}, {"O", bit_list({onet})}},
                                  pass ? "11" : "00", rbel);
                new_cells.emplace_back(gname, buf);
                byname[gname] = buf;
                occupied[rbel] = gname;
                for (int k : pend) {
                    newDI->arr[k] = jint(onet);
                    st.n_di++;
                }
            }
            set_member(conns, "DI", newDI);
        }

        // --- O outputs: sum FF (FD* consuming O[k] on D) ---
        if (O && O->k == JVal::ARR)
            for (size_t k = 0; k < O->arr.size() && k < 4; k++) {
                int ob = int_at(O, k);
                if (ob < 0) continue;
                auto fit = ff_by_d.find(ob);
                if (fit == ff_by_d.end()) continue;
                JPtr fc = byname[fit->second];
                JPtr fat = fc->member("attributes");
                if (fat && fat->k == JVal::OBJ && fat->member("BEL")) continue; // already placed
                std::string slotff = site + "/" + SLOT[k] + std::string("FF");
                if (!claim(slotff, fit->second)) continue; // slot taken; leave to nextpnr
                if (!fat || fat->k != JVal::OBJ) { fat = jobj(); set_member(fc, "attributes", fat); }
                set_member(fat, "BEL", jstr(slotff));
                st.n_ff++;
            }
    }

    // --- same-slot FF->LUT feedback relays ---------------------------------
    // The chipdb cannot route a same-slot Q->imux bounce (AQ->A1 fails at any
    // visit cap), so relay through an identity LUT1 in a NEIGHBOUR slice.
    // TARGETED ONLY: blanket relaying of all 388 same-slot feedbacks REGRESSED
    // 13 -> 65 skips, so only nets named in $CARRY_FB_NETS are relayed.
    std::set<std::string> fb_nets;
    if (const char *fbf = getenv("CARRY_FB_NETS")) {
        std::ifstream f(fbf);
        std::string line;
        while (std::getline(f, line)) {
            while (!line.empty() && isspace((unsigned char)line.back())) line.pop_back();
            if (!line.empty()) fb_nets.insert(line);
        }
    }
    if (!fb_nets.empty()) {
        std::set<std::string> ksites = known_sites;
        for (auto &kv : occupied) ksites.insert(kv.first.substr(0, kv.first.find('/')));
        std::unordered_map<int, std::string> bit2name;
        JPtr nn = mj->member("netnames");
        if (nn && nn->k == JVal::OBJ)
            for (auto &n : nn->obj)
                for (int b : bits_of(n.second->member("bits")))
                    if (!bit2name.count(b)) bit2name[b] = n.first;
        auto free_neighbour_lut = [&](const std::string &site) -> std::string {
            int x, y;
            if (sscanf(site.c_str(), "SLICE_X%dY%d", &x, &y) != 2) return "";
            static const int d[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}};
            for (auto &dd : d) {
                std::string ns = "SLICE_X" + std::to_string(x + dd[0]) + "Y" + std::to_string(y + dd[1]);
                if (!ksites.count(ns) || carry_sites.count(ns)) continue;
                for (int sl = 0; sl < 4; sl++) {
                    std::string bel = ns + "/" + SLOT[sl] + std::string("6LUT");
                    if (!occupied.count(bel)) return bel;
                }
            }
            return "";
        };
        std::vector<std::pair<std::string, JPtr>> scan = cells->obj;
        scan.insert(scan.end(), new_cells.begin(), new_cells.end());
        std::vector<std::pair<std::string, JPtr>> relays;
        for (auto &c : scan) {
            JPtr at = c.second->member("attributes");
            JPtr bj = at && at->k == JVal::OBJ ? at->member("BEL") : nullptr;
            if (!bj || bj->k != JVal::STR) continue;
            const std::string &b = bj->str;
            size_t sl = b.find('/');
            if (sl == std::string::npos || b.size() < 5 || b.compare(b.size() - 4, 4, "6LUT") != 0)
                continue;
            if (!starts_with(type_of(c.second), "LUT")) continue;
            std::string site = b.substr(0, sl);
            char slot_letter = b[sl + 1];
            JPtr conns = c.second->member("connections");
            if (!conns || conns->k != JVal::OBJ) continue;
            for (auto &pc : conns->obj) {
                if (pc.first == "O") continue;
                int v = int_at(pc.second, 0);
                if (v < 0) continue;
                auto d = drv.find(v);
                if (d == drv.end() || !starts_with(std::get<1>(d->second), "FD")) continue;
                auto dc = byname.find(std::get<0>(d->second));
                if (dc == byname.end()) continue;
                JPtr dat = dc->second->member("attributes");
                JPtr dbj = dat && dat->k == JVal::OBJ ? dat->member("BEL") : nullptr;
                std::string want = site + "/" + std::string(1, slot_letter) + "FF";
                if (!dbj || dbj->k != JVal::STR || dbj->str != want) continue;
                auto bn = bit2name.find(v);
                if (bn == bit2name.end() || !fb_nets.count(bn->second)) continue;
                std::string rbel = free_neighbour_lut(site);
                if (rbel.empty()) continue;
                int onet = ++maxbit;
                std::string rname = c.first + "$fbrelay$" + pc.first;
                relays.emplace_back(rname, mk_lut("LUT1",
                                                  {{"I0", bit_list({v})}, {"O", bit_list({onet})}},
                                                  "10", rbel));
                occupied[rbel] = rname;
                pc.second = bit_list({onet});
                st.n_fb++;
            }
        }
        for (auto &r : relays) cells->obj.push_back(r);
    }

    for (auto &nc : new_cells) cells->obj.push_back(nc);
    st.total_cells = cells->obj.size();
    return st;
}

CarryStampStats netlist_carry_stamp(Netlist *n,
                                    const std::vector<std::pair<std::string, std::string>> &bels,
                                    const std::vector<std::string> &slice_sites)
{
    CarryStampStats st;
    if (!n || !n->root) return st;
    JPtr mods = n->root->member("modules");
    if (!mods || mods->k != JVal::OBJ) return st;
    JPtr mj = nullptr;
    size_t best = 0;
    for (auto &kv : mods->obj) {
        JPtr c = kv.second->member("cells");
        size_t cnt = (c && c->k == JVal::OBJ) ? c->obj.size() : 0;
        if (!mj || cnt > best) { mj = kv.second; best = cnt; }
    }
    std::set<std::string> ks(slice_sites.begin(), slice_sites.end());
    return carry_stamp_tree(mj, bels, ks);
}

// ---- JSON writer ----------------------------------------------------------
static void json_escape(std::string &out, const std::string &s)
{
    for (char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\t': out += "\\t"; break;
        case '\r': out += "\\r"; break;
        default:
            if ((unsigned char)c < 0x20) {
                char b[8];
                snprintf(b, sizeof(b), "\\u%04x", c);
                out += b;
            } else
                out += c;
        }
    }
}
static void json_dump(std::string &out, JPtr v, int indent)
{
    std::string pad(indent * 2, ' '), pad2((indent + 1) * 2, ' ');
    switch (v->k) {
    case JVal::OBJ:
        if (v->obj.empty()) { out += "{}"; return; }
        out += "{\n";
        for (size_t i = 0; i < v->obj.size(); i++) {
            out += pad2 + "\"";
            json_escape(out, v->obj[i].first);
            out += "\": ";
            json_dump(out, v->obj[i].second, indent + 1);
            if (i + 1 < v->obj.size()) out += ",";
            out += "\n";
        }
        out += pad + "}";
        return;
    case JVal::ARR: {
        out += "[ ";
        for (size_t i = 0; i < v->arr.size(); i++) {
            json_dump(out, v->arr[i], indent + 1);
            if (i + 1 < v->arr.size()) out += ", ";
        }
        out += " ]";
        return;
    }
    case JVal::STR: out += "\""; json_escape(out, v->str); out += "\""; return;
    case JVal::NUM: {
        char b[32];
        if (v->num == double(int64_t(v->num)))
            snprintf(b, sizeof(b), "%lld", (long long)v->num);
        else
            snprintf(b, sizeof(b), "%g", v->num);
        out += b;
        return;
    }
    case JVal::BOOL: out += v->bol ? "true" : "false"; return;
    default: out += "null"; return;
    }
}

Netlist *netlist_load(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        fprintf(stderr, "pack_to_lef: cannot open %s\n", path.c_str());
        return nullptr;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string src = ss.str();
    JParser jp(src);
    Netlist *n = new Netlist;
    n->root = jp.parse();
    return n;
}
void netlist_free(Netlist *n) { delete n; }

bool netlist_write(Netlist *n, const std::string &path)
{
    if (!n || !n->root) return false;
    std::string out;
    json_dump(out, n->root, 0);
    out += "\n";
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << out;
    return bool(f);
}

PrepassStats netlist_prepasses(Netlist *n)
{
    PrepassStats st;
    if (!n || !n->root) return st;
    JPtr mods = n->root->member("modules");
    if (!mods || mods->k != JVal::OBJ) return st;
    // place_lef's own order (innermost first in run): split_degenerate_muxf,
    // then replicate_shared_carry, then materialise_const_drivers.
    for (auto &m : mods->obj) split_degenerate_muxf(m.second, st);
    for (auto &m : mods->obj) replicate_shared_muxf7(m.second, st);
    for (auto &m : mods->obj) replicate_shared_carry(m.second, st);
    for (auto &m : mods->obj) materialise_const_drivers(m.second, st);
    for (auto &m : mods->obj) st.init_fixed += normalise_init(m.second);
    return st;
}

// Pick the top (most cells) and pack it -- shared by pack_netlist and the
// file-based entry point.
static PackResult pack_tree(JPtr root)
{
    PackResult res;
    JPtr mods = root->member("modules");
    if (!mods || mods->k != JVal::OBJ) {
        fprintf(stderr, "pack_to_lef: no modules\n");
        return res;
    }
    JPtr mj = nullptr;
    size_t best = 0;
    for (auto &kv : mods->obj) {
        JPtr c = kv.second->member("cells");
        size_t n = (c && c->k == JVal::OBJ) ? c->obj.size() : 0;
        if (!mj || n > best) { mj = kv.second; best = n; }
    }
    JPtr cells = mj->member("cells");

    Packer P;
    if (cells && cells->k == JVal::OBJ) {
        for (auto &kv : cells->obj) {
            Inst in;
            in.name = kv.first;
            JPtr t = kv.second->member("type");
            in.type = t ? t->str : "";
            JPtr conns = kv.second->member("connections");
            if (conns && conns->k == JVal::OBJ) {
                for (auto &pc : conns->obj) {
                    std::vector<int> bits;
                    if (pc.second->k == JVal::ARR)
                        for (auto &b : pc.second->arr) {
                            if (b->k == JVal::NUM) bits.push_back(int(b->num));
                            else if (b->k == JVal::STR && b->str == "1") bits.push_back(NK_VCC);
                            else bits.push_back(NK_GND); // "0", "x", "z", anything
                        }
                    in.ports.emplace_back(pc.first, std::move(bits));
                }
            }
            P.insts.push_back(std::move(in));
        }
    }
    res.n_instances = P.insts.size();
    P.run();
    res.cells = std::move(P.packed);
    res.report.assign(P.report.begin(), P.report.end());
    std::stable_sort(res.report.begin(), res.report.end(),
                     [](const std::pair<std::string, int> &a, const std::pair<std::string, int> &b) {
                         return a.second > b.second;
                     });
    return res;
}

PackResult pack_netlist(Netlist *n)
{
    PackResult res;
    if (!n || !n->root) return res;
    return pack_tree(n->root);
}

// ------------------------------------------------------------- library ----
// Read a yosys JSON netlist and pack it, WITHOUT the prepasses -- this is the
// entry point tools/pack_to_lef regression-tests against the OCaml packer, so
// it must keep seeing the netlist exactly as pack_to_lef.ml would.
PackResult pack_to_lef_json(const std::string &path)
{
    PackResult res;
    Netlist *n = netlist_load(path);
    if (!n) return res;
    res = pack_tree(n->root);
    netlist_free(n);
    return res;
}

} // namespace lefpack
