// fft.h - in-place iterative radix-2 complex FFT (double) and a windowed magnitude spectrum.
#pragma once
#include <cmath>
#include <complex>
#include <vector>

namespace cat::test {

inline void fft(std::vector<std::complex<double>>& a)
{
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i)
    {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1)
    {
        const double ang = -2.0 * 3.14159265358979323846 / (double)len;
        const std::complex<double> wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len)
        {
            std::complex<double> w(1.0, 0.0);
            for (size_t k = 0; k < len / 2; ++k)
            {
                const std::complex<double> u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v; a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

// Amplitude spectrum (bins 0..n/2) of x[start .. start+n) with a 4-term Blackman-Harris window
// (-92 dB sidelobes), scaled so a full-scale sine reads 1.0 at its bin. n must be a power of 2.
inline std::vector<double> amplitudeSpectrum(const std::vector<float>& x, size_t start, size_t n)
{
    std::vector<std::complex<double>> a(n);
    const double c0 = 0.35875, c1 = 0.48829, c2 = 0.14128, c3 = 0.01168;
    double wsum = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const double t = 2.0 * 3.14159265358979323846 * (double)i / (double)n;
        const double w = c0 - c1 * std::cos(t) + c2 * std::cos(2 * t) - c3 * std::cos(3 * t);
        wsum += w;
        a[i] = std::complex<double>(x[start + i] * w, 0.0);
    }
    fft(a);
    std::vector<double> mag(n / 2 + 1);
    for (size_t k = 0; k <= n / 2; ++k) mag[k] = 2.0 * std::abs(a[k]) / wsum;
    return mag;
}

} // namespace cat::test
