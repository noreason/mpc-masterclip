// test_wrapper.cpp - tests the built MasterClip plugin (native .so) through the VST2 ABI.
//
//   test_wrapper <masterclip.so> <masterclip_append.so> [report.md]
//
// The catalog's standard checks (common/test/wrapper_checks.h) plus MasterClip specifics.
#include "../../../common/test/wrapper_checks.h"
#include "../presets/presets.h"

#include <cstring>

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
