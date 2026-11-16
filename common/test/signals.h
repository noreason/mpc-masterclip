// signals.h - deterministic test signal generators (mono, float, 44.1 kHz unless stated).
#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

namespace cat::test {

constexpr double kPi = 3.14159265358979323846;

struct Rng   // xorshift64*, deterministic across platforms
{
    uint64_t s;
    explicit Rng(uint64_t seed = 0x9E3779B97F4A7C15ull) : s(seed ? seed : 1) {}
    uint64_t next() { s ^= s >> 12; s ^= s << 25; s ^= s >> 27; return s * 2685821657736338717ull; }
    double uniform() { return (double)(next() >> 11) * (1.0 / 9007199254740992.0); }   // [0, 1)
    double bipolar() { return 2.0 * uniform() - 1.0; }
};

inline std::vector<float> sine(double hz, double amp, int n, double sr = 44100.0, double phase = 0.0)
{
    std::vector<float> x((size_t)n);
    for (int i = 0; i < n; ++i) x[(size_t)i] = (float)(amp * std::sin(2.0 * kPi * hz * i / sr + phase));
    return x;
}

// Uniform white noise scaled to the given sample peak.
inline std::vector<float> whiteNoise(double peak, int n, uint64_t seed = 1)
{
    Rng r(seed);
    std::vector<float> x((size_t)n);
    for (auto& v : x) v = (float)(peak * r.bipolar());
    return x;
}

// Exponential sine sweep, f0 -> f1 over n samples.
inline std::vector<float> logSweep(double f0, double f1, double amp, int n, double sr = 44100.0)
{
    std::vector<float> x((size_t)n);
    const double T = n / sr, k = std::log(f1 / f0);
    for (int i = 0; i < n; ++i)
    {
        const double t = i / sr;
        x[(size_t)i] = (float)(amp * std::sin(2.0 * kPi * f0 * T / k * (std::exp(t / T * k) - 1.0)));
    }
    return x;
}

// A synthetic drum loop (kick, snare, closed hat) at `bpm`, normalised to `peak`.
// Sharp attacks and a sub-heavy kick make it a realistic stress test for a clipper.
inline std::vector<float> drumLoop(int bars, double peak, double bpm = 96.0, double sr = 44100.0, uint64_t seed = 7)
{
    const int stepLen = (int)std::lround(sr * 60.0 / bpm / 4.0);    // 16th notes
    const int n = bars * 16 * stepLen;
    std::vector<double> x((size_t)n, 0.0);
    Rng r(seed);
    auto add = [&](int start, int len, auto&& gen) {
        for (int i = 0; i < len && start + i < n; ++i) x[(size_t)(start + i)] += gen(i);
    };
    for (int s = 0; s < bars * 16; ++s)
    {
        const int at = s * stepLen, beat = s % 16;
        if (beat == 0 || beat == 8 || beat == 10)   // kick: 150 -> 45 Hz pitch drop, click on attack
        {
            double ph = 0.0;
            add(at, (int)(0.45 * sr), [&](int i) {
                const double t = i / sr, f = 45.0 + 105.0 * std::exp(-t * 30.0);
                ph += 2.0 * kPi * f / sr;
                return 1.0 * std::sin(ph) * std::exp(-t * 6.0) + (i < 40 ? 0.5 * (1.0 - i / 40.0) : 0.0);
            });
        }
        if (beat == 4 || beat == 12)                 // snare: 190 Hz body + noise
        {
            add(at, (int)(0.25 * sr), [&](int i) {
                const double t = i / sr;
                return 0.55 * std::sin(2.0 * kPi * 190.0 * t) * std::exp(-t * 25.0) + 0.6 * r.bipolar() * std::exp(-t * 18.0);
            });
        }
        // closed hat on every 8th: first-difference (high-passed) noise
        if ((beat & 1) == 0)
        {
            double prev = 0.0;
            add(at, (int)(0.05 * sr), [&](int i) {
                const double v = r.bipolar(), y = v - prev; prev = v;
                return 0.25 * y * std::exp(-(i / sr) * 90.0);
            });
        }
    }
    double pk = 0.0; for (double v : x) pk = std::fmax(pk, std::fabs(v));
    std::vector<float> out((size_t)n);
    for (int i = 0; i < n; ++i) out[(size_t)i] = (float)(x[(size_t)i] * peak / pk);
    return out;
}

inline float peakAbs(const std::vector<float>& x, size_t from = 0)
{
    float p = 0.0f;
    for (size_t i = from; i < x.size(); ++i) p = std::fmax(p, std::fabs(x[i]));
    return p;
}

} // namespace cat::test
