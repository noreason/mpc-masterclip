// presets.h - MasterClip factory presets (shared by the plugin wrapper and the tests).
//
// The MPC selects program 0 right after loading a plugin, so program 0 is "Default" (the
// parameter defaults), not a character preset. Unlisted settings keep their defaults
// (Ceiling -0.3 dBFS, Link on, Mix 100 %, Output 0 dB, Match off).
#pragma once
#include "../dsp/masterclip_dsp.h"

namespace masterclip {

struct Preset { const char* name; Params p; };

inline Params presetParams(int mode, float inputDb, int os, float ceilingDb = -0.3f, float mixPct = 100.0f)
{
    Params p;
    p.mode = mode; p.inputDb = inputDb; p.oversampling = os; p.ceilingDb = ceilingDb; p.mixPct = mixPct;
    return p;
}

// The setting most producers reach for first: the Classic curve at full scale, no oversampling (its bright
// grit included) and each channel clipped on its own, like the FL Studio clipper's default.
inline Params classicPunch()
{
    Params p = presetParams(kClassic, 0.0f, kOsOff, 0.0f);
    p.link = false;
    return p;
}

inline const Preset* factoryPresets(int& count)
{
    static const Preset kPresets[] = {
        { "Default",           Params{} },
        { "Classic Punch",     classicPunch() },
        { "Master Safety",     presetParams(kHard, 0.0f, kOs4x, -1.0f) },
        { "Drum Bus Glue",     presetParams(kSoft, 3.0f, kOs2x) },
        { "Break Smash",       presetParams(kWarm, 9.0f, kOs2x, -0.3f, 70.0f) },
        { "808 Grit",          presetParams(kWarm, 6.0f, kOs2x) },
        { "Transparent Catch", presetParams(kSoft, 1.5f, kOs4x) },
    };
    count = (int)(sizeof(kPresets) / sizeof(kPresets[0]));
    return kPresets;
}

} // namespace masterclip
