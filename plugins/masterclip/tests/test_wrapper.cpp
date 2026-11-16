// test_wrapper.cpp - tests the built MasterClip plugin (native .so) through the VST2 ABI.
//
//   test_wrapper <masterclip.so> <masterclip_append.so> [report.md]
//
// The catalog's standard checks (common/test/wrapper_checks.h) plus MasterClip specifics.
#include "../../../common/test/wrapper_checks.h"
#include "../presets/presets.h"

#include <cstring>
#include <limits>

using namespace cat::test;

int main(int argc, char** argv)
{
    if (argc < 3) { std::fprintf(stderr, "usage: test_wrapper <masterclip.so> <masterclip_append.so> [report.md]\n"); return 2; }
    Harness h("MasterClip plugin (wrapper) test report");

    CatalogSpec spec;
    spec.libPath = argv[1];
    spec.appendLibPath = argv[2];
    spec.uid = MPC_FOURCC('P', 's', 'M', 'c');
    spec.names = { "Reserved", "Input", "Ceiling", "Mode", "Oversampling", "Stereo Link", "Mix", "Output", "Match", "Clip",
                   "In L", "In R", "Out L", "Out R" };
    spec.meters = { 9, 10, 11, 12, 13 };
    spec.drive = { { 1, (12.0f + 12.0f) / 36.0f } };   // Input +12 dB
    runCatalogChecks(h, spec);

    h.section("MasterClip: value text");
    {
        VstPlugin p(spec.libPath);
        h.check(p.display(1) == "0.0 dB" && p.display(2) == "-0.3 dB" && p.display(6) == "100 %" && p.display(3) == "Classic" && p.display(9) == "0.0",
                "defaults with units", "Input '%s', Ceiling '%s', Mix '%s', Mode '%s', Clip '%s'",
                p.display(1).c_str(), p.display(2).c_str(), p.display(6).c_str(), p.display(3).c_str(), p.display(9).c_str());
        p.set(1, 1.0f); p.set(7, 0.25f);
        h.check(p.display(1) == "+24.0 dB" && p.display(7) == "-6.0 dB", "signed dB", "Input '%s', Output '%s'", p.display(1).c_str(), p.display(7).c_str());
        p.set(1, (12.0f + 12.0f) / 36.0f);
        std::vector<float> l = defaultTestSignal(), r = l;
        p.process(l, r, 128);
        h.check(p.display(9)[0] == '-', "clip readout", "0.2 s into a drum loop at Input +12 dB the readout shows '%s'", p.display(9).c_str());
    }

    h.section("MasterClip: malformed state is rejected without changing controls");
    {
        VstPlugin source(spec.libPath), target(spec.libPath);
        source.set(1, 0.85f);
        const auto valid = source.chunk();
        const auto unchanged = target.chunk();
        auto putU32 = [](std::vector<uint8_t>& bytes, size_t at, uint32_t value) {
            for (int k = 0; k < 4; ++k) bytes[at + (size_t)k] = (uint8_t)(value >> (8 * k));
        };
        auto putFloat = [&](std::vector<uint8_t>& bytes, size_t at, float value) {
            uint32_t bits; std::memcpy(&bits, &value, 4); putU32(bytes, at, bits);
        };
        size_t firstPlain = 12 + 1 + valid[12];
        size_t inputPlain = firstPlain + 8;
        inputPlain += 1 + valid[inputPlain];
        for (bool normalized : { false, true })
        {
            for (float value : { std::numeric_limits<float>::quiet_NaN(), INFINITY, -INFINITY })
            {
                auto bad = valid;
                putFloat(bad, inputPlain + (normalized ? 4 : 0), value);
                const bool rejected = !target.setChunk(bad);
                h.check(rejected && target.chunk() == unchanged, normalized ? "non-finite normalized state rejected" : "non-finite plain state rejected",
                        "invalid float rejected before any parameter changes");
            }
        }
        auto truncated = valid;
        truncated.pop_back();
        h.check(!target.setChunk(truncated) && target.chunk() == unchanged, "truncated extra state rejected atomically");
        auto future = valid;
        putU32(future, 4, 99);
        h.check(!target.setChunk(future) && target.chunk() == unchanged, "unsupported state version rejected atomically");
        std::vector<uint8_t> overflow = { 'M', 'P', 'C', 'P', 2, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255 };
        h.check(!target.setChunk(overflow) && target.chunk() == unchanged, "oversized extra length rejected", "UINT32_MAX cannot bypass the remaining-buffer check on ARM32");
        std::vector<uint8_t> legacy = { 'M', 'P', 'C', 'P', 1, 0, 0, 0, 1, 0, 0, 0, 5, 'i', 'n', 'p', 'u', 't' };
        legacy.resize(legacy.size() + 8, 0);
        putFloat(legacy, 18, 12.0f);
        h.check(target.setChunk(legacy) && std::fabs(target.get(1) - 2.0f / 3.0f) < 1e-6f, "version-1 state remains compatible");
    }

    h.section("MasterClip: non-finite host automation cannot poison controls");
    {
        VstPlugin p(spec.libPath);
        const float before = p.get(1);
        p.set(1, std::numeric_limits<float>::quiet_NaN());
        h.check(p.get(1) == before, "NaN automation ignored");
        p.set(1, INFINITY);
        h.check(p.get(1) == before, "infinite automation ignored");
    }

    h.section("MasterClip: factory presets");
    {
        VstPlugin p(spec.libPath);
        int n = 0; const masterclip::Preset* pr = masterclip::factoryPresets(n);
        h.check(p.numPrograms() == n, "program count", "%d programs", n);
        const char* modes[] = { "Hard", "Soft", "Warm", "Classic" }; const char* oss[] = { "Off", "2x", "4x" };
        for (int k = 0; k < n; ++k)
        {
            p.disp(effSetProgram, 0, k);
            char in[32], ce[32], mx[32];
            std::snprintf(in, sizeof(in), "%+.1f dB", (double)pr[k].p.inputDb); if (!std::strcmp(in, "+0.0 dB")) std::strcpy(in, "0.0 dB");
            std::snprintf(ce, sizeof(ce), "%.1f dB", (double)pr[k].p.ceilingDb);
            std::snprintf(mx, sizeof(mx), "%.0f %%", (double)pr[k].p.mixPct);
            const bool ok = p.programName(k) == pr[k].name && p.display(1) == in && p.display(2) == ce && p.display(3) == modes[pr[k].p.mode]
                            && p.display(4) == oss[pr[k].p.oversampling] && p.display(6) == mx && p.display(5) == (pr[k].p.link ? "On" : "Off") && p.display(8) == "Off";
            h.check(ok, pr[k].name, "recalls Mode %s, Input %s, Oversampling %s, Ceiling %s, Mix %s",
                    p.display(3).c_str(), p.display(1).c_str(), p.display(4).c_str(), p.display(2).c_str(), p.display(6).c_str());
        }
    }

    return h.finish(argc > 3 ? argv[3] : "test-output/masterclip/WRAPPER_REPORT.md");
}
