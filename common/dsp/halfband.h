// halfband.h - polyphase IIR (allpass) halfband filters for 2x up/down sampling.
//
// Structure (Valenzuela & Constantinides, 1983): an elliptic halfband lowpass written as
//     H(z) = 1/2 * [ A0(z^2) + z^-1 * A1(z^2) ]
// where A0 and A1 are cascades of first-order allpass sections  (a + z^-1) / (1 + a z^-1)
// running at the LOW rate. Each coefficient costs one multiply per low-rate sample. The filter
// is IIR and not linear-phase; its group delay is small (a few samples at low frequencies, see
// tests) and nothing is "reported" as latency, which is the point: the MPC's plugin delay
// compensation is unconfirmed (docs/SDK_NOTES.md section 4).
//
// Coefficients are computed here from the closed-form elliptic design via Jacobi theta
// functions (nome series), written from the published math, not copied from any library.
// Inputs: number of coefficients N (filter order 2N+1) and transition bandwidth `tb` as a
// fraction of the HIGH sample rate (passband edge = 0.25 - tb/2, stopband edge = 0.25 + tb/2).
// The tests verify the resulting response (stopband attenuation, passband ripple) numerically.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace cat::dsp {

inline std::vector<double> designHalfbandAllpass(int numCoefs, double transition)
{
    const double pi = 3.14159265358979323846;
    // Selectivity of the halfband elliptic filter: k = tan^2(wp / 2), wp = pi/2 - pi*tb.
    const double t = std::tan((1.0 - 2.0 * transition) * pi / 4.0);
    const double k = t * t;
    // Nome q of the modulus k' = sqrt(1 - k^2) (series for the nome, converges very fast).
    const double kp4 = std::pow(1.0 - k * k, 0.25);
    const double e = 0.5 * (1.0 - kp4) / (1.0 + kp4);
    const double e4 = e * e * e * e;
    const double q = e * (1.0 + e4 * (2.0 + e4 * (15.0 + 150.0 * e4)));
    const int order = 2 * numCoefs + 1;

    std::vector<double> a;
    for (int c = 1; c <= numCoefs; ++c)
    {
        const double arg = c * pi / order;
        // theta_1 / theta_4 ratio -> sqrt(k) * sn(...) at the pole positions.
        double num = 0.0, den = 0.0;
        for (int m = 0; m < 12; ++m)
        {
            const double sgn = (m & 1) ? -1.0 : 1.0;
            num += sgn * std::pow(q, (double)(m * (m + 1))) * std::sin((2 * m + 1) * arg);
        }
        for (int m = 1; m < 12; ++m)
        {
            const double sgn = (m & 1) ? -1.0 : 1.0;
            den += sgn * std::pow(q, (double)(m * m)) * std::cos(2 * m * arg);
        }
        const double ww = 2.0 * std::pow(q, 0.25) * num / (1.0 + 2.0 * den);
        const double w2 = ww * ww;
        const double x = std::sqrt((1.0 - w2 * k) * (1.0 - w2 / k)) / (1.0 + w2);
        a.push_back((1.0 - x) / (1.0 + x));
    }
    std::sort(a.begin(), a.end());
    return a;
}

// Up to this many coefficients per halfband (both branches together).
constexpr int kMaxHalfbandCoefs = 16;

// One cascade of first-order allpass sections on the low-rate grid.
struct AllpassChain
{
    float a[kMaxHalfbandCoefs / 2 + 1] = {};
    float x1[kMaxHalfbandCoefs / 2 + 1] = {};
    float y1[kMaxHalfbandCoefs / 2 + 1] = {};
    int n = 0;

    void reset() { for (int i = 0; i < n; ++i) x1[i] = y1[i] = 0.0f; }
    float process(float x)
    {
        for (int i = 0; i < n; ++i)
        {
            const float y = a[i] * (x - y1[i]) + x1[i];
            x1[i] = x; y1[i] = y;
            x = y;
        }
        return x;
    }
};

// Coefficient set shared by an upsampler and a downsampler of one stage.
struct HalfbandCoefs
{
    float branch0[kMaxHalfbandCoefs / 2 + 1] = {};
    float branch1[kMaxHalfbandCoefs / 2 + 1] = {};
    int n0 = 0, n1 = 0;

    static HalfbandCoefs design(int numCoefs, double transition)
    {
        HalfbandCoefs c;
        const std::vector<double> a = designHalfbandAllpass(std::min(numCoefs, kMaxHalfbandCoefs), transition);
        // Sorted coefficients alternate between the two branches.
        for (size_t i = 0; i < a.size(); ++i)
        {
            if ((i & 1) == 0) c.branch0[c.n0++] = (float)a[i];
            else              c.branch1[c.n1++] = (float)a[i];
        }
        return c;
    }
};

// 2x interpolator: one low-rate sample in, two high-rate samples out (gain 1 in the passband).
class HalfbandUp2
{
public:
    void setCoefs(const HalfbandCoefs& c)
    {
        p0_.n = c.n0; p1_.n = c.n1;
        for (int i = 0; i < c.n0; ++i) p0_.a[i] = c.branch0[i];
        for (int i = 0; i < c.n1; ++i) p1_.a[i] = c.branch1[i];
        reset();
    }
    void reset() { p0_.reset(); p1_.reset(); }
    // out[0] is the earlier high-rate sample.
    void process(float x, float* out)
    {
        out[0] = p0_.process(x);
        out[1] = p1_.process(x);
    }
    void processBlock(const float* in, float* out, int n)
    {
        for (int i = 0; i < n; ++i) process(in[i], out + 2 * i);
    }

private:
    AllpassChain p0_, p1_;
};

// 2x decimator: two high-rate samples in, one low-rate sample out.
class HalfbandDown2
{
public:
    void setCoefs(const HalfbandCoefs& c)
    {
        p0_.n = c.n0; p1_.n = c.n1;
        for (int i = 0; i < c.n0; ++i) p0_.a[i] = c.branch0[i];
        for (int i = 0; i < c.n1; ++i) p1_.a[i] = c.branch1[i];
        reset();
    }
    void reset() { p0_.reset(); p1_.reset(); prevOdd_ = 0.0f; }
    // in[0] is the earlier high-rate sample.
    float process(const float* in)
    {
        const float y = 0.5f * (p0_.process(in[0]) + p1_.process(prevOdd_));
        prevOdd_ = in[1];
        return y;
    }
    void processBlock(const float* in, float* out, int n)
    {
        for (int i = 0; i < n; ++i) out[i] = process(in + 2 * i);
    }

private:
    AllpassChain p0_, p1_;
    float prevOdd_ = 0.0f;
};

} // namespace cat::dsp
