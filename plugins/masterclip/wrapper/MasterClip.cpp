// MasterClip.cpp - MPC Plugin SDK wrapper around the MasterClip DSP core.
//
// Effect, 2 in / 2 out, no MIDI, no latency. All DSP lives in ../dsp/masterclip_dsp.h; the catalog
// base class (common/mpc/plugin_base.h) provides reserved0, meters + notifier, presets, unit text,
// state and the entry point. This file maps parameters to the core and declares the presets.
//
// PARAMETER ORDER AND IDS ARE PERMANENT (saved projects, Q-Links and presets use them).
// Append new parameters at the end only.
//   0  reserved0  no-op (catalog convention; added by CatalogPlugin)
//   1  input      Input    -12 .. +24 dB    drive into the clipper
//   2  ceiling    Ceiling  -12 .. 0 dBFS    sample-peak ceiling (always enforced)
//   3  mode       Mode     Hard/Soft/Warm/Classic (Classic: the FL-style knee curve; the default)
//   4  os         Oversampling  Off/2x/4x   zero-latency IIR oversampling
//   5  link       Stereo Link   Off/On      linked: one gain from max(|L|,|R|) for both channels
//   6  mix        Mix      0 .. 100 %       parallel blend with the phase-aligned dry signal
//   7  output     Output   -12 .. +12 dB
//   8  match      Match    Off/On           output offset by -Input for level-matched comparison
//   9  clip_meter Clip     0 .. 12 dB       READ-ONLY meter: peak clip amount, 300 ms hold, written by
//                 the audio thread, announced from the host's idle call (MeterNotifier)
//  10  in_l       In L     0 .. 60          READ-ONLY meters: sample peak in dBFS + 48 (0 = -48 dBFS or
//  11  in_r       In R                      less, 48 = 0 dBFS, 60 = +12 dBFS: the MPC mixes in floating
//  12  out_l      Out L                     point, so hot tracks arrive above 0 dBFS), 300 ms hold then
//                                           20 dB/s fall. Input is measured before processing (the MPC
//                                           processes in place) and before the Input control.
//  13  out_r      Out R                     The text of In L / Out L is the louder side ("-3.2 dB"), for
//                                           the readout under each meter pair.
//
// Build switches: CAT_DEVICE_LOG, CAT_APPEND_TEST, CAT_NO_METER (see plugin_base.h).
#include "mpc_plugin.h"
#include "../../../common/mpc/plugin_base.h"
#include "../../../common/dsp/peak_hold.h"
#include "../dsp/masterclip_dsp.h"
#include "../presets/presets.h"

#include <cmath>
#include <cstdio>

using namespace masterclip;

class MasterClipPlugin : public cat::mpcx::CatalogPlugin
{
public:
    enum Index { kReserved0, kInput, kCeiling, kMode, kOs, kLink, kMix, kOutput, kMatch, kClipMeter, kInL, kInR, kOutL, kOutR };

    static mpc::PluginInfo makeInfo()
    {
        mpc::PluginInfo i;
        i.name = "MasterClip";
        i.vendor = "PluginBooth";
        i.uniqueId = MPC_FOURCC('P', 's', 'M', 'c');   // == "uid" in plugin.json
        i.version = 1000;                               // 1.0.0.0
        i.isInstrument = false;
        i.numInputs = 2;
        i.numOutputs = 2;
        i.latencySamples = 0;
        i.tailSamples = 1;                              // no tail
        return i;
    }

    MasterClipPlugin() : CatalogPlugin(makeInfo())
    {
        using namespace mpc;
        addParam(floatParam("input", "Input", -12.0f, 24.0f, 0.0f, "dB", 1));
        addParam(floatParam("ceiling", "Ceiling", -12.0f, 0.0f, -0.3f, "dB", 1));
        addParam(choiceParam("mode", "Mode", { "Hard", "Soft", "Warm", "Classic" }, kClassic));
        addParam(choiceParam("os", "Oversampling", { "Off", "2x", "4x" }, kOs2x));
        addParam(boolParam("link", "Stereo Link", true));
        addParam(floatParam("mix", "Mix", 0.0f, 100.0f, 100.0f, "%", 0));
        addParam(floatParam("output", "Output", -12.0f, 12.0f, 0.0f, "dB", 1));
        addParam(boolParam("match", "Match", false));
        addMeter("clip_meter", "Clip", kMeterRangeDb);
        addMeter("in_l", "In L", kLevelRangeDb, "", 1, 0.002f);
        addMeter("in_r", "In R", kLevelRangeDb, "", 1, 0.002f);
        addMeter("out_l", "Out L", kLevelRangeDb, "", 1, 0.002f);
        addMeter("out_r", "Out R", kLevelRangeDb, "", 1, 0.002f);

        int n = 0;
        const Preset* presets = factoryPresets(n);
        for (int k = 0; k < n; ++k)
        {
            const Params& p = presets[k].p;
            addPreset(presets[k].name, { { kInput, p.inputDb }, { kCeiling, p.ceilingDb }, { kMode, (float)p.mode },
                                         { kOs, (float)p.oversampling }, { kLink, p.link ? 1.0f : 0.0f },
                                         { kMix, p.mixPct }, { kOutput, p.outputDb }, { kMatch, p.match ? 1.0f : 0.0f } });
        }
        finishParams();
    }

    Params currentParams() const
    {
        Params p;
        p.inputDb = get(kInput);
        p.ceilingDb = get(kCeiling);
        p.mode = getInt(kMode);
        p.oversampling = getInt(kOs);
        p.link = getBool(kLink);
        p.mixPct = get(kMix);
        p.outputDb = get(kOutput);
        p.match = getBool(kMatch);
        return p;
    }

    void prepareDsp(double sampleRate, int /*maxBlockSize*/) override
    {
        dsp_.setParams(currentParams());
        dsp_.prepare(sampleRate);   // designs filters, then reset(): smoothers snap to current values
        meterHold_.setTimes(300.0f, 30.0f);
        meterHold_.reset();
        for (cat::dsp::PeakHold& h : levelHold_) { h.setTimes(300.0f, 20.0f); h.reset(); }
    }

    // Resume, state or program load (possibly while audio runs): clear filters, fade in over 20 ms.
    void resetDsp() override
    {
        dsp_.setParams(currentParams());
        dsp_.reset(true);
    }

    void process(mpc::ProcessContext& ctx) override
    {
        if (ctx.numInputs < 2 || ctx.numOutputs < 2 || ctx.inputs == nullptr) return;
        dsp_.setParams(currentParams());
        const float seconds = (float)ctx.numFrames / (float)ctx.sampleRate;
        const float inL = levelAboveFloor(ctx.inputs[0], ctx.numFrames), inR = levelAboveFloor(ctx.inputs[1], ctx.numFrames);
        dsp_.process(ctx.inputs[0], ctx.inputs[1], ctx.outputs[0], ctx.outputs[1], ctx.numFrames);
        setMeter(kClipMeter, meterHold_.push(dsp_.clipAmountDb(), seconds));
        setMeter(kInL, levelHold_[0].push(inL, seconds));
        setMeter(kInR, levelHold_[1].push(inR, seconds));
        setMeter(kOutL, levelHold_[2].push(levelAboveFloor(ctx.outputs[0], ctx.numFrames), seconds));
        setMeter(kOutR, levelHold_[3].push(levelAboveFloor(ctx.outputs[1], ctx.numFrames), seconds));
    }

    // A block's sample peak as dBFS + kLevelFloorDb (0 .. kLevelRangeDb), 0 at or below -48 dBFS (NaN/Inf read as 0).
    static float levelAboveFloor(const float* x, int n)
    {
        float peak = 0.0f;
        for (int i = 0; i < n; ++i)
        {
            const float a = std::fabs(x[i]);
            if (a > peak && a < 1e6f) peak = a;
        }
        const float db = peak > 1e-9f ? 20.0f * std::log10(peak) : -200.0f;
        return std::fmax(0.0f, std::fmin(kLevelRangeDb, db + kLevelFloorDb));
    }

    bool formatParam(int index, float v, std::string& out) override
    {
        if (index == kInL || index == kOutL)
        {
            // The readout under a meter pair: the louder side, held like the meters.
            const float louder = std::fmax(v, get(index + 1));
            char b[24];
            const double db = (double)(louder - kLevelFloorDb);
            if (louder < 0.05f) std::snprintf(b, sizeof(b), "-inf");
            else std::snprintf(b, sizeof(b), db >= 0.05 ? "%+.1f dB" : "%.1f dB", db < 0.0 && db > -0.05 ? 0.0 : db);
            out = b;
            return true;
        }
        if (index == kInR || index == kOutR)
        {
            char b[24];
            std::snprintf(b, sizeof(b), "%.1f dB", (double)(v - kLevelFloorDb));
            out = b;
            return true;
        }
        if (index != kClipMeter) return false;
        char b[16];
        std::snprintf(b, sizeof(b), "%.1f", v > 0.05f ? -(double)v : 0.0);   // "-4.5"; "DB CLIPPED" is on the panel
        out = b;
        return true;
    }

private:
    static constexpr float kMeterRangeDb = 12.0f;
    static constexpr float kLevelFloorDb = 48.0f;          // the meters' bottom: -48 dBFS
    static constexpr float kLevelRangeDb = 60.0f;          // ... up to +12 dBFS
    MasterClipDsp dsp_;
    cat::dsp::PeakHold meterHold_;
    cat::dsp::PeakHold levelHold_[4];
};

CAT_PLUGIN_ENTRY(MasterClipPlugin)
