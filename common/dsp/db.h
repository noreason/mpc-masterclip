// db.h - level conversions and tiny math helpers. Header-only, no dependencies.
#pragma once
#include <cmath>

namespace cat::dsp {

template <typename T> constexpr T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline float dbToGain(float db) { return std::pow(10.0f, db * 0.05f); }
inline double dbToGainD(double db) { return std::pow(10.0, db * 0.05); }
inline float gainToDb(float g, float floorDb = -200.0f)
{
    return g > 0.0f ? std::fmax(20.0f * std::log10(g), floorDb) : floorDb;
}
inline double gainToDbD(double g, double floorDb = -300.0)
{
    return g > 0.0 ? std::fmax(20.0 * std::log10(g), floorDb) : floorDb;
}

} // namespace cat::dsp
