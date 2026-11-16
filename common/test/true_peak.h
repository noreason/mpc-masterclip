// true_peak.h - inter-sample (true) peak measurement for tests.
//
// truePeak4x(): the ITU-R BS.1770-4 Annex 2 method: 4x oversampling with a 48-tap
//   (12 taps per phase) interpolating FIR, absolute value, maximum. The filter here is our own
//   Kaiser-windowed sinc of that size (beta 5), not the coefficient table printed in the
//   Recommendation. Like any 4x/48-tap estimator it can under-read the true reconstructed peak
//   by up to ~0.7 dB for content near Nyquist.
// truePeakReference(): a much finer estimate (64x, 128 taps/phase, Kaiser beta 10) evaluated
//   wherever the 4x estimate comes within 3 dB of the signal's maximum. Used to report how much
//   the 4x method misses. Offline only (O(N * taps * phases)).
// Both treat the signal as zero outside its bounds and return a linear peak value.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace cat::test {

inline double besselI0(double x)
{
    double s = 1.0, t = 1.0;
    for (int k = 1; k < 50; ++k) { t *= (x / (2.0 * k)) * (x / (2.0 * k)); s += t; if (t < 1e-17 * s) break; }
    return s;
}

// Value of the band-limited reconstruction at fractional position pos (in samples), using a
// Kaiser-windowed sinc kernel spanning +-halfWidth input samples.
inline double interpAt(const std::vector<float>& x, double pos, int halfWidth, double beta)
{
    const long base = (long)std::floor(pos);
    const double i0b = besselI0(beta);
    double acc = 0.0;
    for (long k = base - halfWidth + 1; k <= base + halfWidth; ++k)
    {
        if (k < 0 || k >= (long)x.size()) continue;
        const double d = pos - (double)k;
        const double r = d / (double)halfWidth;
        if (std::fabs(r) >= 1.0) continue;
        const double sinc = std::fabs(d) < 1e-12 ? 1.0 : std::sin(3.14159265358979323846 * d) / (3.14159265358979323846 * d);
        acc += x[(size_t)k] * sinc * besselI0(beta * std::sqrt(1.0 - r * r)) / i0b;
    }
    return acc;
}

class TruePeak4x
{
public:
    TruePeak4x()
    {
        // 4 phases x 12 taps, Kaiser beta 5, cutoff at the original Nyquist.
        const double i0b = besselI0(5.0);
        for (int p = 0; p < 4; ++p)
            for (int t = 0; t < 12; ++t)
            {
                const double d = (double)(5 - t) + p / 4.0;   // output point (n + p/4) minus input tap (n - 5 + t)
                const double r = d / 6.0;
                const double sinc = std::fabs(d) < 1e-12 ? 1.0 : std::sin(3.14159265358979323846 * d) / (3.14159265358979323846 * d);
                const double w = std::fabs(r) < 1.0 ? besselI0(5.0 * std::sqrt(1.0 - r * r)) / i0b : 0.0;
                h_[p][t] = sinc * w;
            }
    }
    // Max over all samples and the 4 interpolation phases (phase 0 = the samples themselves).
    double measure(const std::vector<float>& x, std::vector<double>* perSample = nullptr) const
    {
        double peak = 0.0;
        if (perSample) perSample->assign(x.size(), 0.0);
        for (size_t n = 0; n < x.size(); ++n)
        {
            double local = 0.0;
            for (int p = 0; p < 4; ++p)
            {
                double acc = 0.0;
                for (int t = 0; t < 12; ++t)
                {
                    const long k = (long)n - 5 + t;
                    if (k >= 0 && k < (long)x.size()) acc += h_[p][t] * x[(size_t)k];
                }
                local = std::max(local, std::fabs(acc));
            }
            // A Kaiser-windowed sinc passes through the samples exactly at phase 0.
            local = std::max(local, (double)std::fabs(x[n]));
            if (perSample) (*perSample)[n] = local;
            peak = std::max(peak, local);
        }
        return peak;
    }

private:
    double h_[4][12];
};

inline double truePeak4x(const std::vector<float>& x) { static const TruePeak4x m; return m.measure(x); }

inline double truePeakReference(const std::vector<float>& x)
{
    static const TruePeak4x est;
    std::vector<double> local;
    const double p4 = est.measure(x, &local);
    double peak = p4;
    const double gate = p4 * 0.7079;   // -3 dB
    for (size_t n = 0; n + 1 < x.size(); ++n)
    {
        if (std::max(local[n], local[n + 1]) < gate) continue;
        for (int s = 0; s < 64; ++s)
            peak = std::max(peak, std::fabs(interpAt(x, (double)n + s / 64.0, 128, 10.0)));
    }
    return peak;
}

} // namespace cat::test
