// fast_math.h - cheap transcendental approximations and finite checks.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

namespace cat::dsp {

// tanh via the 7/6 Lambert continued-fraction (Pade) approximant:
//   tanh(x) ~= x (135135 + 17325 x^2 + 378 x^4 + x^6) / (135135 + 62370 x^2 + 3150 x^4 + 28 x^6)
// The input is clamped to +-4.9718, where the approximant reaches 1.0, and the result to [-1, 1].
// Measured against std::tanh over [-10, 10] (see tests): max abs error 9.6e-5 (-80 dB),
// monotonic, |y| <= 1 always (so c * fastTanh(x / c) can never exceed c).
// Cost: 7 multiplies, 6 adds, 1 divide, 2 compares. No libm call.
inline float fastTanh(float x)
{
    constexpr float kClamp = 4.9718f;
    x = x < -kClamp ? -kClamp : (x > kClamp ? kClamp : x);
    const float x2 = x * x;
    const float num = x * (135135.0f + x2 * (17325.0f + x2 * (378.0f + x2)));
    const float den = 135135.0f + x2 * (62370.0f + x2 * (3150.0f + 28.0f * x2));
    const float y = num / den;
    return y < -1.0f ? -1.0f : (y > 1.0f ? 1.0f : y);
}

// True for NaN and +-Inf. Works under -ffast-math too (bit test, not x != x).
inline bool isNonFinite(float x)
{
    uint32_t u; std::memcpy(&u, &x, 4);
    return (u & 0x7F800000u) == 0x7F800000u;
}

// Scans a buffer; true if any sample is NaN or Inf.
inline bool anyNonFinite(const float* x, int n)
{
    uint32_t acc = 0;
    for (int i = 0; i < n; ++i)
    {
        uint32_t u; std::memcpy(&u, &x[i], 4);
        acc |= ((u & 0x7F800000u) == 0x7F800000u) ? 1u : 0u;
    }
    return acc != 0;
}

} // namespace cat::dsp
