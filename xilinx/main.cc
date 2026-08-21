/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2018  Clifford Wolf <clifford@symbioticeda.com>
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

#ifdef MAIN_EXECUTABLE

#include <fstream>
#include "command.h"
#include "design_utils.h"
#include "jsonwrite.h"
#include "log.h"
#include "placer_lef.h"
#include "timing.h"

USING_NEXTPNR_NAMESPACE

class UspCommandHandler : public CommandHandler
{
  public:
    UspCommandHandler(int argc, char **argv);
    virtual ~UspCommandHandler(){};
    std::unique_ptr<Context> createContext(std::unordered_map<std::string, Property> &values) override;
    void setupArchContext(Context *ctx) override{};
    void customBitstream(Context *ctx) override;
    void customAfterLoad(Context *ctx) override;
    std::string customRewriteJson(Context *ctx, const std::string &filename) override;

  protected:
    po::options_description getArchOptions() override;
};

UspCommandHandler::UspCommandHandler(int argc, char **argv) : CommandHandler(argc, argv) {}

po::options_description UspCommandHandler::getArchOptions()
{
    po::options_description specific("Architecture specific options");
    specific.add_options()("chipdb", po::value<std::string>(), "name of chip database binary");
    specific.add_options()("xdc", po::value<std::vector<std::string>>(), "XDC-style constraints file");
    specific.add_options()("fasm", po::value<std::string>(), "fasm bitstream file to write");
    specific.add_options()("fixed-routes", po::value<std::string>(),
                           "frozen hard-macro routing to LOCK before routing (net + src->dst pips)");
    specific.add_options()("write-fixed-routes", po::value<std::string>(),
                           "after routing, dump fabric routing in --fixed-routes format");

    return specific;
}

void UspCommandHandler::customBitstream(Context *ctx)
{
    if (vm.count("fasm")) {
        std::string filename = vm["fasm"].as<std::string>();
        ctx->writeFasm(filename);
    }
    if (vm.count("write-fixed-routes"))
        ctx->writeFixedRoutes(vm["write-fixed-routes"].as<std::string>());
}

std::unique_ptr<Context> UspCommandHandler::createContext(std::unordered_map<std::string, Property> &values)
{
    ArchArgs chipArgs;
    if (!vm.count("chipdb")) {
        log_error("chip database binary must be provided\n");
    }
    chipArgs.chipdb = vm["chipdb"].as<std::string>();
    return std::unique_ptr<Context>(new Context(chipArgs));
}

void UspCommandHandler::customAfterLoad(Context *ctx)
{
    if (vm.count("xdc")) {
        std::vector<std::string> files = vm["xdc"].as<std::vector<std::string>>();
        for (const auto &filename : files) {
            std::ifstream in(filename);
            if (!in)
                log_error("failed to open XDC file '%s'\n", filename.c_str());
            ctx->parseXdc(in);
        }
    }
    if (vm.count("fixed-routes"))
        ctx->settings[ctx->id("fixed-routes")] = vm["fixed-routes"].as<std::string>();

}

// --placer lef: run the whole place_lef transplant HERE, before the netlist is
// parsed, and hand nextpnr the STAMPED netlist instead of the original.
//
// This is what makes it a single invocation.  The transplant's packer
// recognises LUT6/FDRE/CARRY4/MUXF7, so it cannot run in the placer slot --
// Arch::pack() would already have replaced those.  And its prepasses and
// carry_stamp ADD cells ($muxdup, _const_, _carrep, $Srt, $DIgnd, ...), which
// have to exist in ctx.  Parsing the transformed file gives both for free: no
// cell mirroring, and every BEL attribute arrives as an ordinary parsed
// attribute.  The placer setting is then rewritten to "sa", whose constraints
// pass binds the stamped cells at STRENGTH_USER.
std::string UspCommandHandler::customRewriteJson(Context *ctx, const std::string &filename)
{
    if (!ctx->settings.count(ctx->id("placer")) ||
        ctx->settings[ctx->id("placer")].as_string() != "lef")
        return filename;
    std::string stamped = place_lef_transplant(ctx, filename);
    if (stamped.empty())
        log_error("place_lef transplant failed.\n");
    log_info("place_lef: loading the stamped netlist %s (placer -> sa, which honours the "
             "stamps)\n",
             stamped.c_str());
    ctx->settings[ctx->id("placer")] = std::string("sa");
    return stamped;
}

int main(int argc, char *argv[])
{
    UspCommandHandler handler(argc, argv);
    return handler.exec();
}

#endif
