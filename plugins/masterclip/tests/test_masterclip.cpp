// test_masterclip.cpp - desktop tests for the MasterClip DSP core.
//
//   test_masterclip [output-dir]      (default: test-output/masterclip)
//
// Writes REPORT.md and listening WAVs to the output dir. Exit code 0 only if every check passes.
// Built twice by scripts/test-all: under ASan+UBSan (correctness) and optimised (speed).
#include "../dsp/masterclip_dsp.h"
#include "../../../common/test/fft.h"
#include "../../../common/test/harness.h"
#include "../../../common/test/signals.h"
#include "../../../common/test/true_peak.h"
#include "../../../common/test/wav_writer.h"

#include <algorithm>
#include <complex>
#include <memory>
#include <string>
#include <sys/stat.h>

using namespace masterclip;
using namespace cat::test;
using cat::dsp::dbToGain;
using cat::dsp::gainToDbD;

namespace {

const char* kModeNames[] = { "Hard", "Soft", "Warm", "Classic" };
const char* kOsNames[] = { "Off", "2x", "4x" };
constexpr double kSr = 44100.0;

struct Stereo { std::vector<float> l, r; };

std::unique_ptr<MasterClipDsp> makeDsp() { return std::make_unique<MasterClipDsp>(); }

// Fresh state at params p (smoothers snapped), then process in blocks of `block` frames, in place.
Stereo render(MasterClipDsp& dsp, const Params& p, const Stereo& in, int block = 128)
{
    dsp.setParams(p);
    dsp.reset();
    Stereo out = in;
    for (size_t i = 0; i < out.l.size(); i += (size_t)block)
    {
        const int n = (int)std::min((size_t)block, out.l.size() - i);
        dsp.setParams(p);
        dsp.process(out.l.data() + i, out.r.data() + i, out.l.data() + i, out.r.data() + i, n);
    }
    return out;
}

float peak2(const Stereo& s, size_t from = 0) { return std::max(peakAbs(s.l, from), peakAbs(s.r, from)); }

// Stereo test signals: R is a related but different signal so linked/unlinked differ.
Stereo stereoSine(double hz, double amp, int n) { return { sine(hz, amp, n), sine(hz, amp * 0.8, n, kSr, 1.0) }; }
Stereo stereoNoise(double amp, int n, uint64_t seed) { return { whiteNoise(amp, n, seed), whiteNoise(amp, n, seed + 101) }; }
Stereo stereoDrums(int bars, double amp)
{
    Stereo s{ drumLoop(bars, amp), {} };
    s.r.assign(s.l.size(), 0.0f);
    for (size_t i = 3; i < s.l.size(); ++i) s.r[i] = 0.8f * s.l[i - 3];
    return s;
}

struct NamedSignal { std::string name; Stereo s; };

std::vector<NamedSignal> ceilingSignals()
{
    std::vector<NamedSignal> v;
    for (double hz : { 50.0, 100.0, 1000.0, 5000.0, 10000.0, 15000.0 })
        v.push_back({ "sine " + std::to_string((int)hz) + " Hz", stereoSine(hz, 1.0, 11025) });
    v.push_back({ "white noise", stereoNoise(1.0, 11025, 3) });
    v.push_back({ "drums", stereoDrums(1, 1.0) });
    return v;
}

// ---------------------------------------------------------------------------------------------
void testUnits(Harness& h)
{
    h.section("Building blocks");

    // fastTanh
    double maxErr = 0.0; bool mono = true, bounded = true; float prev = -2.0f;
    for (int i = -100000; i <= 100000; ++i)
    {
        const float x = (float)i * 1e-4f, y = cat::dsp::fastTanh(x);
        maxErr = std::max(maxErr, std::fabs((double)y - std::tanh((double)x)));
        if (y < prev - 1e-6f) mono = false;   // float rounding near 1.0 gives steps of a few ulp
        if (std::fabs(y) > 1.0f) bounded = false;
        prev = y;
    }
    h.check(maxErr < 1e-4 && mono && bounded, "fastTanh accuracy",
            "max |error| vs std::tanh on [-10,10] = %.2e (%.1f dB), monotonic to within 1e-6=%s, |y|<=1=%s",
            maxErr, 20 * std::log10(maxErr), mono ? "yes" : "no", bounded ? "yes" : "no");

    // Halfband stages: analytic frequency response of the designed coefficients.
    auto response = [](const cat::dsp::HalfbandCoefs& c, double f) {
        using C = std::complex<double>;
        const C z = std::polar(1.0, 2.0 * kPi * f), z2 = z * z;
        C a0 = 1.0, a1 = 1.0;
        for (int i = 0; i < c.n0; ++i) a0 *= ((double)c.branch0[i] + 1.0 / z2) / (1.0 + (double)c.branch0[i] / z2);
        for (int i = 0; i < c.n1; ++i) a1 *= ((double)c.branch1[i] + 1.0 / z2) / (1.0 + (double)c.branch1[i] / z2);
        return std::abs(0.5 * (a0 + a1 / z));
    };
    struct St { const char* name; const cat::dsp::HalfbandCoefs& c; double hiRate, fStop, need; };
    for (const St& st : { St{ "stage A (44.1k <-> 88.2k)", Oversampler::stageA(), 2 * kSr, 24100.0 / (2 * kSr), 100.0 },
                          St{ "stage B (88.2k <-> 176.4k)", Oversampler::stageB(), 4 * kSr, 0.25 + Oversampler::kStageBTransition / 2, 120.0 } })
    {
        double att = 1e9, rip = 0.0;
        for (int i = 0; i <= 4000; ++i) { const double f = 20000.0 / st.hiRate * i / 4000.0; rip = std::max(rip, std::fabs(20 * std::log10(response(st.c, f)))); }
        for (int i = 0; i <= 8000; ++i) { const double f = st.fStop + (0.5 - st.fStop) * i / 8000.0; att = std::min(att, -20 * std::log10(response(st.c, f))); }
        h.check(att >= st.need && rip < 1e-6, st.name, "%d coefficients, stopband >= %.1f dB from %.1f kHz, passband ripple to 20 kHz %.1e dB",
                st.c.n0 + st.c.n1, att, st.fStop * st.hiRate / 1000.0, rip);
    }

    // Up -> down round trip: gain flatness and group delay (this is the plugin's effective latency).
    for (int f : { 2, 4 })
    {
        cat::dsp::Oversampler os; os.init(f);
        std::vector<float> imp(8192, 0.0f), out(8192); imp[0] = 1.0f;
        for (int i = 0; i < 8192; i += 512) os.downsample(os.upsample(imp.data() + i, 512), out.data() + i, 512);
        std::vector<std::complex<double>> spec(8192);
        for (int i = 0; i < 8192; ++i) spec[(size_t)i] = out[(size_t)i];
        fft(spec);
        double worst = 0.0;
        for (int k = 1; k <= 20000 * 8192 / 44100; ++k) worst = std::max(worst, std::fabs(20 * std::log10(std::abs(spec[(size_t)k]))));
        auto gd = [&](double hz) {
            const int k = (int)std::lround(hz * 8192 / kSr);
            const double p1 = std::arg(spec[(size_t)k]), p2 = std::arg(spec[(size_t)k + 1]);
            double d = p2 - p1; while (d > kPi) d -= 2 * kPi; while (d < -kPi) d += 2 * kPi;
            return -d / (2 * kPi / 8192.0);
        };
        h.check(worst < 0.001, f == 2 ? "2x up/down round trip" : "4x up/down round trip",
                "flat to %.1e dB from 5 Hz to 20 kHz; group delay %.2f / %.2f / %.2f / %.2f samples at 100 Hz / 1 kHz / 10 kHz / 18 kHz",
                worst, gd(100), gd(1000), gd(10000), gd(18000));
    }
}

// ---------------------------------------------------------------------------------------------
void testCeiling(Harness& h)
{
    h.section("Ceiling (sample peak), Hard / Soft (and Warm) x Off/2x/4x x Link");
    auto dsp = makeDsp();
    const auto sigs = ceilingSignals();
    for (float ceilDb : { -0.3f, -6.0f, -12.0f })
    {
        for (int mode : { kHard, kSoft, kWarm, kClassic })
        {
            double worstDb = -1e9; std::string worstWhat; long runs = 0;
            for (int os = 0; os < 3; ++os)
                for (bool link : { true, false })
                        for (int in = -12; in <= 24; in += 3)
                            for (const auto& sg : sigs)
                            {
                                if (ceilDb != -0.3f && (in % 6 != 0)) continue;   // fewer combos for the extra ceilings
                                Params p; p.mode = mode; p.oversampling = os; p.link = link;
                                p.inputDb = (float)in; p.ceilingDb = ceilDb;
                                const Stereo o = render(*dsp, p, sg.s);
                                const double over = gainToDbD(peak2(o)) - gainToDbD(dbToGain(ceilDb));
                                ++runs;
                                if (over > worstDb)
                                {
                                    worstDb = over;
                                    char b[160]; std::snprintf(b, sizeof(b), "%s, %s, link %s, input %+d dB", sg.name.c_str(), kOsNames[os], link ? "on" : "off", in);
                                    worstWhat = b;
                                }
                            }
            const bool ok = worstDb <= 0.0;
            char name[64]; std::snprintf(name, sizeof(name), "%s, ceiling %.1f dBFS", kModeNames[mode], ceilDb);
            h.check(ok, name, "%ld renders, highest sample peak %.4f dB relative to the ceiling (%s)", runs, worstDb, worstWhat.c_str());
        }
    }
}

// ---------------------------------------------------------------------------------------------
struct TpResult { double est[3] = { -1e9, -1e9, -1e9 }; double ref = -1e9; };   // sines, noise, drums

TpResult worstTruePeak(MasterClipDsp& dsp, int mode, int os)
{
    struct S { int cls; Stereo s; };
    static std::vector<S> sigs;
    if (sigs.empty())
    {
        for (double hz : { 1000.0, 5000.0, 10000.0, 15000.0 }) sigs.push_back({ 0, stereoSine(hz, 1.0, 22050) });
        sigs.push_back({ 1, stereoNoise(1.0, 22050, 9) });
        sigs.push_back({ 2, stereoDrums(1, 1.0) });
    }
    TpResult r;
    double worstAll = -1e9; Stereo worstOut;
    const double c = dbToGain(-0.3f);
    for (int in : { 0, 6, 12, 18, 24 })
        for (const auto& sg : sigs)
        {
            Params p; p.mode = mode; p.oversampling = os; p.inputDb = (float)in;
            const Stereo o = render(dsp, p, sg.s);
            const double e = gainToDbD(std::max(truePeak4x(o.l), truePeak4x(o.r)) / c);
            r.est[sg.cls] = std::max(r.est[sg.cls], e);
            if (e > worstAll) { worstAll = e; worstOut = o; }
        }
    auto refWindow = [&](const std::vector<float>& x) {
        std::vector<double> local; TruePeak4x m; m.measure(x, &local);
        const size_t at = (size_t)(std::max_element(local.begin(), local.end()) - local.begin());
        const size_t lo = at > 600 ? at - 600 : 0, hi = std::min(x.size(), at + 600);
        return truePeakReference(std::vector<float>(x.begin() + (long)lo, x.begin() + (long)hi));
    };
    r.ref = gainToDbD(std::max(refWindow(worstOut.l), refWindow(worstOut.r)) / c);
    return r;
}

void testTruePeak(Harness& h)
{
    h.section("Inter-sample peaks (information: MasterClip is a clipper, not a true-peak limiter)");
    h.text("The ceiling guarantees the sample peak only. Worst true peak of the output relative to the ceiling "
           "(dB), BS.1770-4 style 4x estimator, per signal class, over Input 0..+24 dB in 6 dB steps, ceiling "
           "-0.3 dBFS, Link on, Mix 100 %%. Sines = 1, 5, 10, 15 kHz at 0 dBFS. `ref` = 64x reference "
           "interpolation of the overall worst render.\n\n| Mode | OS | sines | noise | drums | ref |\n|---|---|---|---|---|---|\n");
    auto dsp = makeDsp();
    double worst = -1e9;
    for (int mode = 0; mode < kNumModes; ++mode)
        for (int os = 0; os < 3; ++os)
        {
            const TpResult t = worstTruePeak(*dsp, mode, os);
            worst = std::max({ worst, t.est[0], t.est[1], t.est[2], t.ref });
            h.text("| %s | %s | %+.2f | %+.2f | %+.2f | %+.2f |\n", kModeNames[mode], kOsNames[os], t.est[0], t.est[1], t.est[2], t.ref);
        }
    h.text("\n");
    h.check(true, "inter-sample peak table produced", "worst %+.2f dB over the ceiling: put a true-peak limiter after MasterClip for streaming delivery", worst);
}

// ---------------------------------------------------------------------------------------------
void testAliasing(Harness& h)
{
    h.section("Aliasing");
    h.text("Spectrum of the last 65536 samples (Blackman-Harris 4-term). Everything in 20 Hz-20 kHz that is not "
           "the fundamental (or, for Warm, its 2nd harmonic) counts as aliasing. Worst alias bin relative to the "
           "fundamental. `clamp off` = the same render with the final sample-peak safety clamp disabled "
           "(measurement only): it shows what the oversampled clipper itself achieves; the difference is the "
           "cost of the strict sample-peak ceiling. Ceiling -0.3 dBFS, Link on.\n\n");
    auto dsp = makeDsp();
    const int N = 65536, settle = 8192;
    auto measure = [&](double hz, double amp, float inDb, int mode, int os, bool clamp) {
        Params p; p.mode = mode; p.oversampling = os; p.inputDb = inDb;
        dsp->setSafetyClampForMeasurement(clamp);
        const Stereo o = render(*dsp, p, stereoSine(hz, amp, N + settle));
        dsp->setSafetyClampForMeasurement(true);
        const auto mag = amplitudeSpectrum(o.l, settle, N);
        const double binHz = kSr / N;
        const int kf = (int)std::lround(hz / binHz);
        double fund = 0.0; for (int k = kf - 4; k <= kf + 4; ++k) fund = std::max(fund, mag[(size_t)k]);
        double worst = 0.0;
        for (int k = (int)(20.0 / binHz); k <= (int)(20000.0 / binHz); ++k)
        {
            bool harmonic = false;
            for (int m = 1; m * hz <= 20000.0; ++m) if (std::abs(k - (int)std::lround(m * hz / binHz)) <= 8) harmonic = true;
            if (!harmonic) worst = std::max(worst, mag[(size_t)k]);
        }
        return 20 * std::log10(worst / fund);
    };
    struct Case { const char* name; double hz, amp; float in; };
    double spec[kNumModes][3] = {};
    for (const Case& c : { Case{ "10 kHz sine at -6 dBFS, Input +18 dB (the specified test)", 10000, 0.5, 18 },
                           Case{ "10 kHz sine at -6 dBFS, Input +6 dB", 10000, 0.5, 6 },
                           Case{ "2.5 kHz sine at -6 dBFS, Input +18 dB", 2500, 0.5, 18 } })
    {
        h.text("**%s** (worst alias, dBc; clamp on / clamp off)\n\n| Mode | Off | 2x | 4x |\n|---|---|---|---|\n", c.name);
        for (int mode = 0; mode < kNumModes; ++mode)
        {
            double on[3], off[3];
            for (int os = 0; os < 3; ++os) { on[os] = measure(c.hz, c.amp, c.in, mode, os, true); off[os] = measure(c.hz, c.amp, c.in, mode, os, false); }
            if (c.in == 18 && c.hz == 10000) for (int os = 0; os < 3; ++os) spec[mode][os] = on[os];
            h.text("| %s | %.1f / %.1f | %.1f / %.1f | %.1f / %.1f |\n", kModeNames[mode], on[0], off[0], on[1], off[1], on[2], off[2]);
        }
        h.text("\n");
    }
    for (int mode : { kHard, kSoft })
        h.check(spec[mode][2] <= spec[mode][1] + 0.5 && spec[mode][1] < spec[mode][0], kModeNames[mode],
                "specified test (10 kHz, +18 dB), clamp on: Off %.1f dBc, 2x %.1f dBc, 4x %.1f dBc", spec[mode][0], spec[mode][1], spec[mode][2]);
}

// ---------------------------------------------------------------------------------------------
void testNull(Harness& h)
{
    h.section("Null: Mix 0 %");
    auto dsp = makeDsp();
    const Stereo sigs[] = { stereoNoise(0.9, 44100, 21), stereoDrums(1, 0.9) };
    for (int os = 0; os < 3; ++os)
    {
        double worstRef = -400.0, worstRaw = -400.0;
        for (const Stereo& s : sigs)
            for (int mode = 0; mode < kNumModes; ++mode)
            {
                Params p; p.mode = mode; p.oversampling = os; p.inputDb = 24.0f; p.mixPct = 0.0f;
                const Stereo o = render(*dsp, p, s, 96);
                // Reference = what an unclipped signal comes out as: same up/down filters, nothing in between.
                cat::dsp::Oversampler refL, refR; refL.init(os == 0 ? 1 : (os == 1 ? 2 : 4)); refR.init(refL.factor());
                Stereo ref = s;
                for (size_t i = 0; i < ref.l.size(); i += 96)
                {
                    const int n = (int)std::min<size_t>(96, ref.l.size() - i);
                    refL.downsample(refL.upsample(s.l.data() + i, n), ref.l.data() + i, n);
                    refR.downsample(refR.upsample(s.r.data() + i, n), ref.r.data() + i, n);
                }
                float dRef = 0.0f, dRaw = 0.0f;
                for (size_t i = 0; i < o.l.size(); ++i)
                {
                    dRef = std::max({ dRef, std::fabs(o.l[i] - ref.l[i]), std::fabs(o.r[i] - ref.r[i]) });
                    dRaw = std::max({ dRaw, std::fabs(o.l[i] - s.l[i]), std::fabs(o.r[i] - s.r[i]) });
                }
                worstRef = std::max(worstRef, gainToDbD(dRef));
                worstRaw = std::max(worstRaw, gainToDbD(dRaw));
            }
        char name[64]; std::snprintf(name, sizeof(name), "Mix 0 %%, oversampling %s", kOsNames[os]);
        if (os == 0)
            h.check(worstRaw < -120.0, name, "peak difference from the raw input %.1f dBFS (all modes, Input +24 dB)", worstRaw);
        else
            h.check(worstRef < -120.0, name, "peak difference from the input through the same %s up/down filters %.1f dBFS; "
                    "from the raw input %.1f dBFS (phase shift of the IIR filters, not a level change; see round-trip flatness)",
                    kOsNames[os], worstRef, worstRaw);
    }
}

// ---------------------------------------------------------------------------------------------
void testEdgeCases(Harness& h)
{
    h.section("Edge cases");
    auto dsp = makeDsp();

    // Silence in -> silence out, worst settings.
    {
        bool allZero = true;
        const Stereo z{ std::vector<float>(44100, 0.0f), std::vector<float>(44100, 0.0f) };
        for (int mode = 0; mode < kNumModes; ++mode)
            for (int os = 0; os < 3; ++os)
                for (bool link : { true, false })
                {
                    Params p; p.mode = mode; p.oversampling = os; p.link = link; p.inputDb = 24; p.outputDb = 12; p.mixPct = 50;
                    if (peak2(render(*dsp, p, z)) != 0.0f) allZero = false;
                }
        h.check(allZero, "silence in -> silence out", "every mode/oversampling/link, Input +24, Output +12, Mix 50 %%: output exactly 0.0");
    }

    // Warm: no DC offset, and it really adds even harmonics (Soft does not).
    for (int os = 0; os < 3; ++os)
    {
        const int n = 4 * 44100;
        const Stereo s = stereoSine(100.0, 0.5, n);
        Params p; p.mode = kWarm; p.oversampling = os; p.inputDb = 12;
        const Stereo o = render(*dsp, p, s);
        double sum = 0.0; for (int i = 2 * 44100; i < n; ++i) sum += o.l[(size_t)i];
        const double dc = std::fabs(sum / (2 * 44100));
        const auto mag = amplitudeSpectrum(o.l, 2 * 44100, 65536);
        const double b = 65536 / kSr;
        const double h1 = mag[(size_t)std::lround(100 * b)], h2 = mag[(size_t)std::lround(200 * b)];
        p.mode = kSoft;
        const Stereo os2 = render(*dsp, p, s);
        const auto mag2 = amplitudeSpectrum(os2.l, 2 * 44100, 65536);
        const double s2 = mag2[(size_t)std::lround(200 * b)] / mag2[(size_t)std::lround(100 * b)];
        char name[64]; std::snprintf(name, sizeof(name), "Warm %s: no DC offset", kOsNames[os]);
        h.check(gainToDbD(dc) < -80.0, name, "DC after 2 s = %.1f dBFS (100 Hz sine, -6 dBFS, Input +12); 2nd harmonic %.1f dBc (Soft: %.1f dBc)",
                gainToDbD(dc), gainToDbD(h2 / h1), gainToDbD(s2));
    }

    // NaN / Inf in never produces NaN out, and the plugin recovers.
    {
        bool finite = true, recovered = true; int events = 0;
        for (int mode = 0; mode < kNumModes; ++mode)
            for (int os = 0; os < 3; ++os)
            {
                auto d = makeDsp();
                Stereo s = stereoNoise(0.5, 3 * 44100, 5);
                const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
                s.l[1000] = nan; s.r[5000] = inf; s.l[9000] = -inf;
                for (int i = 20000; i < 20512; ++i) s.l[(size_t)i] = s.r[(size_t)i] = nan;   // a whole block of NaN
                s.l[30000] = 1e38f; s.r[30001] = -1e38f;                                     // huge but finite
                Params p; p.mode = mode; p.oversampling = os; p.inputDb = 24; p.mixPct = 50;
                const Stereo o = render(*d, p, s);
                for (size_t i = 0; i < o.l.size(); ++i)
                    if (!std::isfinite(o.l[i]) || !std::isfinite(o.r[i])) finite = false;
                if (peak2(o, 2 * 44100) < 0.01f) recovered = false;
                events += d->nonFiniteEvents();
            }
        h.check(finite, "NaN/Inf in never gives NaN/Inf out", "NaN, +Inf, -Inf samples, a 512-sample NaN block and +-1e38 in every mode/oversampling");
        h.check(recovered && events > 0, "recovers after NaN/Inf", "%d blocks muted in total; audio returns afterwards", events);
    }

    // Denormal input: no denormals out.
    {
        Stereo s{ std::vector<float>(44100, 1e-41f), std::vector<float>(44100, -3e-42f) };
        bool clean = true;
        for (int os = 0; os < 3; ++os)
        {
            Params p; p.mode = kWarm; p.oversampling = os; p.inputDb = 24; p.mixPct = 50;
            const Stereo o = render(*dsp, p, s);
            for (size_t i = 0; i < o.l.size(); ++i)
                if (std::fpclassify(o.l[i]) == FP_SUBNORMAL || std::fpclassify(o.r[i]) == FP_SUBNORMAL) clean = false;
        }
        h.check(clean, "denormal input", "denormal-valued input produces no denormal output (flush-to-zero active)");
    }

    // Block-size independence: identical output whatever the chunking.
    {
        const Stereo s = stereoDrums(1, 1.0);
        Params p; p.mode = kWarm; p.oversampling = kOs4x; p.inputDb = 9; p.mixPct = 70;
        const Stereo a = render(*dsp, p, s, 32), b = render(*dsp, p, s, 512), c = render(*dsp, p, s, 1);
        float d = 0.0f;
        for (size_t i = 0; i < a.l.size(); ++i) d = std::max({ d, std::fabs(a.l[i] - b.l[i]), std::fabs(a.l[i] - c.l[i]), std::fabs(a.r[i] - b.r[i]) });
        h.check(d == 0.0f, "block-size independence", "blocks of 1, 32 and 512 frames give bit-identical output (max diff %g)", (double)d);
    }
}

// ---------------------------------------------------------------------------------------------
void testNoClicks(Harness& h)
{
    h.section("No clicks: abrupt parameter changes between blocks");
    h.text("Input: 110 Hz sine, -12 dBFS (R: -14 dBFS, shifted). Parameters jump every 0.1 s through a script "
           "of extreme settings for 6 s. Each change is checked over the following 50 ms: the largest "
           "sample-to-sample step must stay within 1.25 x the larger steady-state step of the two settings it moves "
           "between, + 0.001. A discontinuity (click) would exceed that.\n\n");
    // The script of settings (each is held for 0.1 s, then the next is applied in one block).
    std::vector<Params> script;
    {
        Params p; script.push_back(p);
        p.inputDb = 24; script.push_back(p);
        p.ceilingDb = -12; script.push_back(p);
        p.mode = kHard; script.push_back(p);
        p.oversampling = kOs4x; script.push_back(p);
        p.link = false; script.push_back(p);
        p.mode = kWarm; script.push_back(p);
        p.link = true; script.push_back(p);
        p.oversampling = kOsOff; script.push_back(p);
        p.mixPct = 0; script.push_back(p);
        p.outputDb = 12; script.push_back(p);
        p.mixPct = 100; script.push_back(p);
        p.match = true; script.push_back(p);
        p.inputDb = -12; script.push_back(p);
        p.ceilingDb = 0; script.push_back(p);
        p.oversampling = kOs2x; p.mode = kSoft; p.link = true; script.push_back(p);
        p.outputDb = -12; p.match = false; script.push_back(p);
        p.mixPct = 37; script.push_back(p);
        p.mode = kHard; p.oversampling = kOs4x; p.inputDb = 24; script.push_back(p);   // several at once
    }
    auto dsp = makeDsp();
    const int n = 6 * 44100;
    const Stereo in{ sine(110.0, 0.25, n), sine(110.0, 0.2, n, kSr, 0.7) };
    auto maxStep = [](const std::vector<float>& x, size_t from) {
        float m = 0.0f; for (size_t i = from + 1; i < x.size(); ++i) m = std::max(m, std::fabs(x[i] - x[i - 1])); return m;
    };
    // Steady-state worst step of every setting in the script.
    std::vector<float> steady;
    for (const Params& p : script)
    {
        const Stereo o = render(*dsp, p, Stereo{ sine(110.0, 0.25, 22050), sine(110.0, 0.2, 22050, kSr, 0.7) });
        steady.push_back(std::max(maxStep(o.l, 2205), maxStep(o.r, 2205)));
    }
    const int hold = (int)(0.1 * kSr);
    for (int block : { 32, 96, 128, 512 })
    {
        dsp->setParams(script[0]); dsp->reset();
        Stereo o = in;
        std::vector<int> changeAt;
        int last = 0;
        for (int i = 0; i < n; i += block)
        {
            const int m = std::min(block, n - i);
            const int idx = (i / hold) % (int)script.size();
            if (i > 0 && idx != last) changeAt.push_back(i);
            last = idx;
            dsp->setParams(script[(size_t)idx]);
            dsp->process(o.l.data() + i, o.r.data() + i, o.l.data() + i, o.r.data() + i, m);
        }
        // Each change is judged in the 50 ms after it, against the two settings it moves between.
        float worstRatio = 0.0f; int worstIdx = -1; float worstStep = 0.0f, worstBound = 0.0f;
        for (size_t c = 0; c < changeAt.size(); ++c)
        {
            const int at = changeAt[c];
            const int to = (at / hold) % (int)script.size(), from = (to + (int)script.size() - 1) % (int)script.size();
            const float bound = 1.25f * std::max(steady[(size_t)from], steady[(size_t)to]) + 0.001f;
            float step = 0.0f;
            for (int i = at; i < std::min(n, at + 2205); ++i)
                step = std::max({ step, std::fabs(o.l[(size_t)i] - o.l[(size_t)i - 1]), std::fabs(o.r[(size_t)i] - o.r[(size_t)i - 1]) });
            if (step / bound > worstRatio) { worstRatio = step / bound; worstIdx = to; worstStep = step; worstBound = bound; }
        }
        char name[64]; std::snprintf(name, sizeof(name), "parameter jumps, block %d", block);
        h.check(worstRatio <= 1.0f, name, "%zu changes; tightest: change to script step %d, step %.4f vs bound %.4f (%.0f %% of bound)",
                changeAt.size(), worstIdx, worstStep, worstBound, 100.0 * worstRatio);
    }
}

// ---------------------------------------------------------------------------------------------
void testFuzz(Harness& h)
{
    h.section("Fuzz: random parameters and block sizes");
    auto dsp = makeDsp();
    Rng r(2026);
    const int n = 20 * 44100;
    Stereo s = stereoNoise(2.0, n, 77);
    bool finite = true;
    int i = 0;
    Params p;
    while (i < n)
    {
        const int block = 1 + (int)(r.uniform() * 512.0);
        const int m = std::min(block, n - i);
        if (r.uniform() < 0.3)
        {
            p.inputDb = (float)(-20 + r.uniform() * 50);     // deliberately out of range too
            p.ceilingDb = (float)(-15 + r.uniform() * 18);
            p.mode = (int)(r.uniform() * 4) - 0;
            p.oversampling = (int)(r.uniform() * 4) - 1;
            p.link = r.uniform() < 0.5; p.match = r.uniform() < 0.5;
            p.mixPct = (float)(-10 + r.uniform() * 120); p.outputDb = (float)(-15 + r.uniform() * 30);
        }
        dsp->setParams(p);
        dsp->process(s.l.data() + i, s.r.data() + i, s.l.data() + i, s.r.data() + i, m);
        for (int k = i; k < i + m; ++k) if (!std::isfinite(s.l[(size_t)k]) || !std::isfinite(s.r[(size_t)k])) finite = false;
        i += m;
    }
    h.check(finite, "random parameters, block sizes 1..512, out-of-range values", "20 s, all output finite (run under ASan/UBSan for memory errors)");
}

// ---------------------------------------------------------------------------------------------
void renderListening(Harness& h, const std::string& dir)
{
    h.section("Listening files");
    auto dsp = makeDsp();
    const Stereo drums = stereoDrums(4, 0.7);                                // -3 dBFS peak
    const Stereo sweep{ logSweep(20, 20000, 0.5, 10 * 44100), logSweep(20, 20000, 0.5, 10 * 44100) };
    struct Cfg { const char* name; Params p; };
    std::vector<Cfg> cfgs;
    { Params p; p.mode = kSoft; p.inputDb = 6; cfgs.push_back({ "soft_+6dB_2x", p }); }
    { Params p; p.mode = kHard; p.inputDb = 9; p.oversampling = kOs4x; cfgs.push_back({ "hard_+9dB_4x", p }); }
    { Params p; p.mode = kHard; p.inputDb = 12; p.oversampling = kOsOff; cfgs.push_back({ "hard_+12dB_off_aliasing", p }); }
    { Params p; p.mode = kWarm; p.inputDb = 9; p.mixPct = 70; cfgs.push_back({ "warm_+9dB_2x_mix70", p }); }
    { Params p; p.mode = kWarm; p.inputDb = 6; p.match = true; cfgs.push_back({ "warm_+6dB_2x_match", p }); }
    writeWavFloat(dir + "/drums_dry.wav", drums.l, &drums.r);
    writeWavFloat(dir + "/sweep_dry.wav", sweep.l, &sweep.r);
    for (const Cfg& c : cfgs)
    {
        const Stereo a = render(*dsp, c.p, drums), b = render(*dsp, c.p, sweep);
        writeWavFloat(dir + "/drums_" + c.name + ".wav", a.l, &a.r);
        writeWavFloat(dir + "/sweep_" + c.name + ".wav", b.l, &b.r);
    }
    h.check(true, "listening WAVs written", "%s/{drums,sweep}_*.wav (32-bit float, 44.1 kHz): dry plus %zu settings", dir.c_str(), cfgs.size());
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Classic: the FL-style knee. Below the knee (25/32 of the ceiling) the signal is untouched; above it the
// curve leaves the line at the same slope, keeps rising, and approaches the ceiling without reaching past it.
void testClassic(Harness& h)
{
    h.section("Classic (FL-style knee)");
    auto dsp = makeDsp();
    const int n = 44100;
    Params p;   // the defaults: Classic, ceiling -0.3 dBFS
    h.check(p.mode == kClassic, "Classic is the default mode", "");
    p.oversampling = kOsOff;
    p.link = false;
    const float c = std::pow(10.0f, p.ceilingDb / 20.0f), knee = kClassicKnee * c;

    // Below the knee: a 100 Hz sine peaking just under it comes out unchanged.
    const Stereo below = stereoSine(100.0, 0.98 * knee, n);
    const Stereo out = render(*dsp, p, below);
    double worst = 0.0;
    for (int i = n / 2; i < n; ++i) worst = std::fmax(worst, std::fabs((double)out.l[(size_t)i] - below.l[(size_t)i]));
    h.check(worst < 1e-6, "untouched below the knee", "largest difference %.2e (sine peaking at 98 %% of the knee)", worst);

    // The curve itself, through the processor with steady (DC) input: no corner, monotonic, under the ceiling.
    auto steady = [&](float a) {
        const Stereo dc{ std::vector<float>((size_t)n, a), std::vector<float>((size_t)n, a) };
        return render(*dsp, p, dc).l[(size_t)n - 1];
    };
    const float eps = 1e-3f * knee;
    const float slope = (steady(knee + eps) - steady(knee)) / eps;
    h.check(std::fabs(slope - 1.0f) < 0.01f, "no corner at the knee", "slope just above the knee %.4f (1 = the straight line)", slope);
    float prev = 0.0f, top = 0.0f;
    bool rising = true;
    for (float a = 0.5f * c; a <= 16.0f * c; a *= 1.05f)
    {
        const float y = steady(a);
        rising = rising && y >= prev;
        prev = y;
        top = std::fmax(top, y);
    }
    h.check(rising && top <= c, "rises smoothly toward the ceiling, never past it", "highest output %.6f, ceiling %.6f (input up to +24 dB over)", top, c);
    h.check(steady(1.0f * c) > knee && steady(1.0f * c) < c, "at the ceiling level it is already bending", "input = ceiling -> %.4f (knee %.4f)", steady(c), knee);
}

int main(int argc, char** argv)
{
    const std::string dir = argc > 1 ? argv[1] : "test-output/masterclip";
    ::mkdir("test-output", 0755);
    ::mkdir(dir.c_str(), 0755);
    Harness h("MasterClip DSP test report");
    testUnits(h);
    testCeiling(h);
    testClassic(h);
    testTruePeak(h);
    testAliasing(h);
    testNull(h);
    testEdgeCases(h);
    testNoClicks(h);
    testFuzz(h);
    renderListening(h, dir);
    return h.finish(dir + "/REPORT.md");
}
