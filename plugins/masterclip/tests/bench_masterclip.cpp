// bench_masterclip.cpp - CPU cost of the MasterClip DSP core.
//
//   bench_masterclip [--block N] [--csv] [--cpu N]     (--cpu pins the process to one core, Linux)
//
// Prints nanoseconds per stereo frame and the implied share of one core at 44.1 kHz
// (% = ns/frame * 44100 / 1e7) for every Mode x Oversampling setting, worst case
// (Link off, Mix 50 % so the aligned dry path runs, signal clipping hard).
// Build optimised (-O2), never with sanitizers, for meaningful numbers. Cross-compile the same
// file for arm32/arm64 to run it on hardware.
#include "../dsp/masterclip_dsp.h"
#include "../../../common/test/bench.h"
#include "../../../common/test/signals.h"

#include <cstdio>
#include <cstring>
#include <memory>
#if defined(__linux__)
#include <sched.h>
#endif

using namespace masterclip;

int main(int argc, char** argv)
{
    int block = 128, cpu = -1; bool csv = false;
    for (int i = 1; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--block") && i + 1 < argc) block = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--cpu") && i + 1 < argc) cpu = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--csv")) csv = true;
    }
    const char* modes[] = { "Hard", "Soft", "Warm", "Classic" };
    const char* oss[] = { "Off", "2x", "4x" };
#if defined(__aarch64__)
    const char* arch = "arm64";
#elif defined(__arm__)
    const char* arch = "arm32";
#else
    const char* arch = "x86_64";
#endif

#if defined(__linux__)
    if (cpu >= 0)
    {
        cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0) { std::perror("sched_setaffinity"); return 1; }
    }
#endif
    auto dsp = std::make_unique<MasterClipDsp>();
    const int len = 44100;
    const std::vector<float> srcL = cat::test::drumLoop(1, 1.0), srcR = cat::test::whiteNoise(0.8, (int)srcL.size(), 4);
    std::vector<float> bufL(srcL.size()), bufR(srcR.size());
    (void)len;

    if (csv) std::printf("arch,block,mode,oversampling,mix,ns_per_frame,pct_one_core\n");
    else std::printf("MasterClip DSP benchmark (%s, block %d, cpu %d, stereo, Input +12 dB, Link off)\n\n"
                     "%-5s %-4s %-8s %10s %12s\n", arch, block, cpu, "Mode", "OS", "Mix", "ns/frame", "% of 1 core");
    for (int mode = 0; mode < kNumModes; ++mode)
        for (int os = 0; os < 3; ++os)
                for (int mix : { 100, 50 })
                {
                    Params p; p.mode = mode; p.oversampling = os; p.inputDb = 12; p.link = false; p.mixPct = (float)mix;
                    dsp->setParams(p); dsp->reset();
                    size_t pos = 0;
                    const double ns = cat::test::nsPerFrame([&](int n) {
                        if (pos + (size_t)n > bufL.size()) pos = 0;
                        std::memcpy(bufL.data() + pos, srcL.data() + pos, sizeof(float) * (size_t)n);
                        std::memcpy(bufR.data() + pos, srcR.data() + pos, sizeof(float) * (size_t)n);
                        dsp->setParams(p);
                        dsp->process(bufL.data() + pos, bufR.data() + pos, bufL.data() + pos, bufR.data() + pos, n);
                        pos += (size_t)n;
                    }, block);
                    const double pct = ns * 44100.0 / 1e7;
                    if (csv) std::printf("%s,%d,%s,%s,%d,%.2f,%.3f\n", arch, block, modes[mode], oss[os], mix, ns, pct);
                    else std::printf("%-5s %-4s %-8s %10.1f %11.2f%%\n", modes[mode], oss[os], mix == 100 ? "100%" : "50%", ns, pct);
                }
    return 0;
}
