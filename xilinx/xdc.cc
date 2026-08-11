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

#include "log.h"
#include "nextpnr.h"
NEXTPNR_NAMESPACE_BEGIN

void Arch::parseXdc(std::istream &in)
{

    if (!in)
        log_error("failed to open LPF file\n");
    std::string line;
    std::string linebuf;
    int lineno = 0;

    auto isempty = [](const std::string &str) {
        return std::all_of(str.begin(), str.end(), [](char c) { return std::isspace(c); });
    };
    auto strip_quotes = [](const std::string &str) {
        if (str.empty())
            return str;
        if (str.front() == '"') {
            if (str.back() == '"' && str.size() >= 2)
                return str.substr(1, str.size() - 2);
            return str;
        } else if (str.front() == '{') {
            if (str.back() == '}' && str.size() >= 2)
                return str.substr(1, str.size() - 2);
            return str;
        } else {
            return str;
        }
    };
    auto split_to_args = [](const std::string &str, bool group_brackets) {
        std::vector<std::string> split_args;
        std::string buffer;
        auto flush = [&]() {
            if (!buffer.empty())
                split_args.push_back(buffer);
            buffer.clear();
        };
        int brcount = 0;
        for (char c : str) {
            if ((c == '[' || c == '{') && group_brackets) {
                ++brcount;
            }
            if ((c == ']' || c == '}') && group_brackets) {
                --brcount;
                buffer += c;
                if (brcount == 0)
                    flush();
                continue;
            }
            if (std::isspace(c)) {
                if (brcount == 0) {
                    flush();
                    continue;
                }
            }
            buffer += c;
        }
        flush();
        return split_args;
    };

    auto trim_all = [](std::string s) {
        s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) {
            return c == ' ' || c == '\t';
        }), s.end());
        return s;
    };
    // Vivado calls a one-bit vector port "a[0]", but the JSON frontend collapses a
    // width-1, offset-0 vector to the bare name "a" (get_bit_name(),
    // frontend/frontend_base.h:367) because yosys' JSON carries nothing that tells
    // `wire [0:0] a` apart from `wire a`. An XDC written against Vivado names
    // therefore misses such a port, the constraint is dropped, and the design dies
    // later with "port a of type PAD has no IOSTANDARD property" -- a message that
    // points nowhere near the cause.  Strip a trailing "[0]" so the lookup can retry.
    auto debus_zero = [](const std::string &str) {
        if (str.size() > 3 && str.compare(str.size() - 3, 3, "[0]") == 0)
            return str.substr(0, str.size() - 3);
        return std::string();
    };
    auto get_cells = [&](std::string str) {
        std::vector<CellInfo *> tgt_cells;
        if (str.empty() || str.front() != '[')
            log_error("failed to parse target (on line %d)\n", lineno);
        str = str.substr(1, str.size() - 2);
        auto split = split_to_args(str, false);
        auto split_name = trim_all(str.substr(split.at(0).length()));
        if (split.size() < 1)
            log_error("failed to parse target (on line %d)\n", lineno);
        if (split.front() != "get_ports")
            log_warning("set_property: target %s not supported (on line %d)\n", split.front().c_str(), lineno);
        if (split.size() < 2)
            log_error("failed to parse target (on line %d)\n", lineno);
        IdString cellname = id(strip_quotes(split_name));
        if (!cells.count(cellname)) {
            std::string base = debus_zero(strip_quotes(split_name));
            if (!base.empty() && cells.count(id(base)))
                cellname = id(base);
        }
        if (cells.count(cellname))
            tgt_cells.push_back(cells.at(cellname).get());
        else
            log_warning("%s: no cell named '%s' (on line %d) - this target is ignored\n",
                        split.front().c_str(), cellname.c_str(this), lineno);
        return tgt_cells;
    };

    auto get_nets = [&](std::string str) {
        std::vector<NetInfo *> tgt_nets;
        if (str.empty() || str.front() != '[')
            log_error("failed to parse target (on line %d)\n", lineno);
        str = str.substr(1, str.size() - 2);
        auto split = split_to_args(str, false);
        auto split_name = trim_all(str.substr(split.at(0).length()));
        if (split.size() < 1)
            log_error("failed to parse target (on line %d)\n", lineno);
        if (split.front() != "get_ports" && split.front() != "get_nets")
            log_error("targets other than 'get_ports' or 'get_nets' are not supported (on line %d)\n", lineno);
        if (split.size() < 2)
            log_error("failed to parse target (on line %d)\n", lineno);
        IdString netname = id(strip_quotes(split_name));
        NetInfo *maybe_net = getNetByAlias(netname);
        if (maybe_net == nullptr) {
            std::string base = debus_zero(strip_quotes(split_name));
            if (!base.empty())
                maybe_net = getNetByAlias(id(base));
        }
        if (maybe_net != nullptr)
            tgt_nets.push_back(maybe_net);
        else
            log_warning("%s: no net or port named '%s' (on line %d) - this target is ignored\n",
                        split.front().c_str(), netname.c_str(this), lineno);
        return tgt_nets;
    };

    while (std::getline(in, line)) {
        ++lineno;
        // Trim comments, from # until end of the line
        size_t cstart = line.find('#');
        if (cstart != std::string::npos)
            line = line.substr(0, cstart);
        while (!line.empty() && std::isspace(line.back()))
            line.pop_back();
        if (!line.empty() && line.back() == ';')
            line.pop_back();
        if (isempty(line))
            continue;

        std::vector<std::string> arguments = split_to_args(line, true);
        if (arguments.empty())
            continue;
        std::string &cmd = arguments.front();
        if (cmd == "set_property") {
            std::vector<std::pair<std::string, std::string>> arg_pairs;
            if (arguments.size() != 4)
                log_error("expected four arguments to 'set_property' (on line %d)\n", lineno);
            else if (arguments.at(1) == "-dict") {
                std::vector<std::string> dict_args = split_to_args(strip_quotes(arguments.at(2)), false);
                if ((dict_args.size() % 2) != 0)
                    log_error("expected an even number of argument for dictionary (on line %d)\n", lineno);
                arg_pairs.reserve(dict_args.size() / 2);
                for (int cursor = 0; cursor + 1 < int(dict_args.size()); cursor += 2) {
                    arg_pairs.emplace_back(std::move(dict_args.at(cursor)), std::move(dict_args.at(cursor + 1)));
                }
            } else
                arg_pairs.emplace_back(std::move(arguments.at(1)), std::move(arguments.at(2)));
            if (arguments.at(1) == "INTERNAL_VREF")
                continue;
            if (arguments.at(3).size() > 2 && (arguments.at(3) == "[current_design]" || arguments.at(3) == "[current_project]")) {
                log_warning("[current_design] isn't supported, ignoring (on line %d)\n", lineno);
                continue;
            }
            std::vector<CellInfo *> dest = get_cells(arguments.at(3));
            for (auto c : dest)
                for (const auto &pair : arg_pairs)
                    c->attrs[id(pair.first)] = std::string(pair.second);
        } else if (cmd == "create_clock") {
            double period = 0;
            bool got_period = false;
            int cursor = 1;
            for (cursor = 1; cursor < int(arguments.size()); cursor++) {
                std::string opt = arguments.at(cursor);
                if (opt == "-add")
                    ;
                else if (opt == "-name" || opt == "-waveform")
                    cursor++;
                else if (opt == "-period") {
                    cursor++;
                    period = std::stod(arguments.at(cursor));
                    got_period = true;
                } else
                    break;
            }
            if (!got_period)
                log_error("found create_clock without period (on line %d)", lineno);
            if (cursor >= int(arguments.size())) {
                // virtual clock (no target ports/nets): not supported; skip it
                // instead of crashing with std::out_of_range
                log_warning("ignoring virtual clock (unsupported, on line %d)\n", lineno);
                continue;
            }
            std::vector<NetInfo *> dest = get_nets(arguments.at(cursor));
            if (dest.empty())
                log_warning("create_clock: target %s matched nothing, so the %.3f ns constraint "
                            "was NOT applied (on line %d). The clock domain keeps the default "
                            "target and will be reported as meeting timing at that default.\n",
                            arguments.at(cursor).c_str(), period, lineno);
            for (auto n : dest) {
                n->clkconstr = std::unique_ptr<ClockConstraint>(new ClockConstraint);
                n->clkconstr->period = getDelayFromNS(period);
                n->clkconstr->high.delay = n->clkconstr->period.delay / 2;
                n->clkconstr->low.delay = n->clkconstr->period.delay / 2;
            }
        } else if (cmd == "set_multicycle_path") {
            // set_multicycle_path <N> [-setup|-hold] -from [<sel>] -to [<sel>]
            // Tags the destination (capture) cells with a multicycle factor so the
            // timing engine can relax the setup requirement on those endpoints.
            // Previously this command was silently ignored. Supports a NAME glob in the
            // -to selector, e.g. -to [get_cells -hier -filter {NAME =~ *rf_reg*}]
            auto glob_match = [](const std::string &name, const std::string &pat) {
                // simple '*' wildcard match
                size_t n = 0, p = 0, star = std::string::npos, mark = 0;
                while (n < name.size()) {
                    if (p < pat.size() && (pat[p] == name[n] || pat[p] == '?')) { ++n; ++p; }
                    else if (p < pat.size() && pat[p] == '*') { star = p++; mark = n; }
                    else if (star != std::string::npos) { p = star + 1; n = ++mark; }
                    else return false;
                }
                while (p < pat.size() && pat[p] == '*') ++p;
                return p == pat.size();
            };
            // group_brackets=true keeps a whole "[get_cells ... {NAME =~ *pat*}]" as ONE argument.
            int mcp = 1; bool is_hold = false;
            std::string to_sel;
            for (int c = 1; c < int(arguments.size()); c++) {
                const std::string &a = arguments.at(c);
                if (a == "-hold") is_hold = true;
                else if (a == "-to" && c + 1 < int(arguments.size())) to_sel = arguments.at(c + 1);
                else if (!a.empty() && std::all_of(a.begin(), a.end(), ::isdigit)) mcp = std::stoi(a);
            }
            // extract the NAME glob from the -to selector (substring after "=~")
            std::string to_pat;
            size_t eq = to_sel.find("=~");
            if (eq != std::string::npos) to_pat = to_sel.substr(eq + 2);
            // trim whitespace and any leftover braces/brackets around the pattern
            auto clean = [](std::string s) {
                std::string o;
                for (char ch : s) if (ch != '{' && ch != '}' && ch != ']' && ch != '[' && !std::isspace(ch)) o += ch;
                return o;
            };
            to_pat = clean(to_pat);
            if (!is_hold && !to_pat.empty()) {
                int tagged = 0;
                for (auto &kv : cells) {
                    std::string cn = kv.first.str(this);
                    if (glob_match(cn, to_pat)) {
                        kv.second->attrs[id("NEXTPNR_MCP_SETUP")] = std::to_string(mcp);
                        ++tagged;
                    }
                }
                log_info("set_multicycle_path: setup multicycle %d tagged on %d cells matching '%s' (on line %d)\n",
                         mcp, tagged, to_pat.c_str(), lineno);
            } else {
                log_info("set_multicycle_path: parsed (hold or no -to glob) — no setup tag (on line %d)\n", lineno);
            }
        } else {
            log_info("ignoring unsupported XDC command '%s' (on line %d)\n", cmd.c_str(), lineno);
        }
    }
    if (!isempty(linebuf))
        log_error("unexpected end of XDC file\n");
}

NEXTPNR_NAMESPACE_END
