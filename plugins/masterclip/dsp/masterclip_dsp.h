// masterclip_dsp.h - MasterClip DSP core. Plain C++17, header-only, no SDK dependency.
//
// Signal flow (per stereo block, chunked to <= 512 frames):
//
//   in ──┬─> x Input ─> [up 1/2/4x] ─> (Warm: asymmetric pre-shape) ─> clipper ─> [down] ─┐
//        │                              Hard: min(|x|, c)                                   │
//        │                              Soft: c * tanh(x / c)                               │
//        │                              Classic: x below the knee k = 25/32 c, then           │
//        │                                k + (c - k) tanh((|x| - k) / (c - k)) (no corner)   │
//        │                              Link: gain from max(|L|,|R|), applied to both       │
//        │   ┌──────────────────────────────────────────────────────────────────────────────┘
//        │   └─> (Warm: DC blocker) ─> safety clamp at c (sample peak, always on) ─> wet
//        │
//        └─> [up -> down, same filters, no clipping] ─> dry (phase-aligned with wet)
//
//   out = (wet * mix + dry * (1 - mix)) * Output * (Match ? 1/Input : 1)
//
//   c = Ceiling. The final clamp guarantees the SAMPLE peak never exceeds c. It is not a true-peak
//   limiter: inter-sample peaks can exceed c (measured in tests/RESULTS.md).
//
// Real-time rules: no allocation (all buffers are fixed-size members), no locks, no I/O,
// no logging. process() handles any frame count (chunks of kMaxBlock). Denormals are flushed
// for the duration of process(). If NaN/Inf appears in the output, that block is replaced by
// silence and all filter state is cleared.
//
// Smoothing: every continuous parameter ramps over 20 ms: Input and Output gain linearly in dB
// (so Match stays exact during a ramp), Ceiling and Mix linearly. Discrete parameters
// (Mode, Link, Oversampling) cross-fade between the old and new configuration over 20 ms by
// running both; Match is part of the smoothed output gain. A change requested during a cross-fade is queued, never jumped.
//
// Dry path alignment: the oversampling filters are minimum-phase-like IIR (no reported
// latency), so their phase shift is frequency dependent. A plain delay cannot align the dry
// signal; instead the dry signal goes through an identical up/down filter pair with nothing in
// between, so parallel blends do not comb-filter. At Oversampling Off the dry path is the raw
// input. At 2x/4x the dry path is the input through that near-allpass pair (level flat to within
// 3e-6 dB to 20 kHz, phase shifted). Mix 0 % is therefore NOT a bypass at 2x/4x.
#pragma once

#include "../../../common/dsp/db.h"
#include "../../../common/dsp/dc_blocker.h"
#include "../../../common/dsp/denormal.h"
#include "../../../common/dsp/fast_math.h"
#include "../../../common/dsp/oversampler.h"
#include "../../../common/dsp/ramp.h"

#include <atomic>
#include <cmath>
#include <cstring>

namespace masterclip {

using namespace cat::dsp;

enum Mode { kHard = 0, kSoft = 1, kWarm = 2, kClassic = 3, kNumModes = 4 };

// Classic: the soft clip most producers know from FL Studio's stock clipper, matched to its published
// behaviour (no FL code): untouched below a knee, then a bend that leaves the line at the same slope and
// approaches the ceiling without reaching past it. The knee is the default one measured on FL's exports
// (Donkerwerk SOFT CLIP, "-2.1442 dB" below full scale = 0.78125 = 25/32 of the ceiling).
constexpr float kClassicKnee = 25.0f / 32.0f;
enum Oversampling { kOsOff = 0, kOs2x = 1, kOs4x = 2 };

struct Params
{
    float inputDb = 0.0f;       // -12 .. +24
    float ceilingDb = -0.3f;    // -12 .. 0 dBFS
    int   mode = kClassic;      // Hard / Soft / Warm / Classic
    int   oversampling = kOs2x; // Off / 2x / 4x
    bool  link = true;
    float mixPct = 100.0f;      // 0 .. 100
    float outputDb = 0.0f;      // -12 .. +12
    bool  match = false;        // output offset by -Input
};

class MasterClipDsp
{
public:
    static constexpr int kMaxBlock = Oversampler::kMaxBlock;
    static constexpr double kRampMs = 20.0;
    static constexpr double kDcCutoffHz = 5.0;
    // Asymmetry of the Warm pre-shaper: x + kWarmAsym * c * tanh(x / c)^2 (even harmonics).
    static constexpr float kWarmAsym = 0.12f;

    MasterClipDsp() { prepare(44100.0); }

    // Not real-time: designs filters. Call before processing and on sample-rate change.
    void prepare(double sampleRate)
    {
        sampleRate_ = sampleRate;
        for (int k = 0; k < 3; ++k)
        {
            const int f = k == kOsOff ? 1 : (k == kOs2x ? 2 : 4);
            for (int ch = 0; ch < 2; ++ch) { chains_[k].wet[ch].init(f); chains_[k].dry[ch].init(f); }
        }
        for (int ch = 0; ch < 2; ++ch) { dc_[ch].setCutoff(kDcCutoffHz, sampleRate); }
        for (LinearRamp* r : { &ceil_, &mix_, &warmW_, &cfgW_, &osW_, &fade_ })
            r->setRampTime(kRampMs, sampleRate);
        inGain_.setRampTime(kRampMs, sampleRate);
        outGain_.setRampTime(kRampMs, sampleRate);
        reset();
    }

    // Clears all audio state and snaps every smoother/cross-fade to the current parameters.
    // fadeIn: ramp the output up from silence over 20 ms (use when audio may already be running,
    // e.g. a preset change or project load, so clearing the filters cannot click).
    void reset(bool fadeIn = false)
    {
        clearFilters();
        fade_.snap(fadeIn ? 0.0f : 1.0f);
        fade_.setTarget(1.0f);
        applyTargets(params_);
        for (LinearRamp* r : { &ceil_, &mix_, &warmW_ })
            r->snap(r->target());
        inGain_.snap(inGain_.target());
        outGain_.snap(outGain_.target());
        cfgOld_ = cfgNew_ = cfgPending_ = ClipCfg{ params_.mode, params_.link };
        cfgW_.snap(1.0f); cfgPendingSet_ = false;
        osOld_ = osNew_ = osPending_ = params_.oversampling; osW_.snap(1.0f); osPendingSet_ = false;
        dryRunning_ = mix_.current() < 1.0f;
    }

    // Call once per block (audio thread OK). Values are clamped to their legal ranges.
    void setParams(const Params& in)
    {
        Params p = in;
        p.inputDb = clampv(p.inputDb, -12.0f, 24.0f);
        p.ceilingDb = clampv(p.ceilingDb, -12.0f, 0.0f);
        p.mode = clampv(p.mode, 0, kNumModes - 1);
        p.oversampling = clampv(p.oversampling, 0, 2);
        p.mixPct = clampv(p.mixPct, 0.0f, 100.0f);
        p.outputDb = clampv(p.outputDb, -12.0f, 12.0f);
        params_ = p;
        applyTargets(p);

        const ClipCfg want{ p.mode, p.link };
        if (cfgW_.ramping() || cfgW_.current() < 1.0f) { cfgPending_ = want; cfgPendingSet_ = !(want == cfgNew_); }
        else if (!(want == cfgNew_)) startCfgFade(want);

        if (osW_.ramping() || osW_.current() < 1.0f) { osPending_ = p.oversampling; osPendingSet_ = p.oversampling != osNew_; }
        else if (p.oversampling != osNew_) startOsFade(p.oversampling);
    }

    const Params& params() const { return params_; }
    // MEASUREMENT ONLY: disables the final sample-peak clamp so tests can separate what the
    // oversampled clipper does from what the strict ceiling costs. Never used by the plugin.
    void setSafetyClampForMeasurement(bool on) { safetyClamp_ = on; }

    // Peak "clip amount" of the last processed block: how far the driven signal went above
    // the ceiling, in dB (0 = not clipping). Lock-free, readable from any thread.
    float clipAmountDb() const { return clipDb_.load(std::memory_order_relaxed); }
    // Number of blocks that were muted because NaN/Inf appeared.
    int nonFiniteEvents() const { return nonFiniteEvents_; }

    // In-place safe (outL may equal inL, outR may equal inR). Any n >= 0.
    void process(const float* inL, const float* inR, float* outL, float* outR, int n)
    {
        ScopedFlushDenormals ftz;
        float peakOver = 0.0f;
        int done = 0;
        while (done < n)
        {
            const int m = (n - done) < kMaxBlock ? (n - done) : kMaxBlock;
            peakOver = std::fmax(peakOver, processChunk(inL + done, inR + done, outL + done, outR + done, m));
            done += m;
        }
        clipDb_.store(peakOver > 1.0f ? 20.0f * std::log10(peakOver) : 0.0f, std::memory_order_relaxed);
    }

private:
    struct ClipCfg
    {
        int mode = kSoft; bool link = true;
        bool operator==(const ClipCfg& o) const { return mode == o.mode && link == o.link; }
    };
    struct Chain { Oversampler wet[2], dry[2]; };

    void clearFilters()
    {
        for (Chain& c : chains_) for (int ch = 0; ch < 2; ++ch) { c.wet[ch].reset(); c.dry[ch].reset(); }
        for (DcBlocker& d : dc_) d.reset();
    }

    void applyTargets(const Params& p)
    {
        inGain_.setTarget(dbToGain(p.inputDb));
        ceil_.setTarget(dbToGain(p.ceilingDb));
        mix_.setTarget(p.mixPct * 0.01f);
        outGain_.setTarget(dbToGain(p.outputDb - (p.match ? p.inputDb : 0.0f)));
        warmW_.setTarget(p.mode == kWarm ? 1.0f : 0.0f);
    }

    void startCfgFade(const ClipCfg& to) { cfgOld_ = cfgNew_; cfgNew_ = to; cfgW_.snap(0.0f); cfgW_.setTarget(1.0f); cfgPendingSet_ = false; }
    void startOsFade(int to)
    {
        osOld_ = osNew_; osNew_ = to;
        for (int ch = 0; ch < 2; ++ch) { chains_[to].wet[ch].reset(); chains_[to].dry[ch].reset(); }
        osW_.snap(0.0f); osW_.setTarget(1.0f); osPendingSet_ = false;
    }

    // ---- the clipper, at the oversampled rate ------------------------------------------
    static inline float curve(int mode, float m, float c, float invC)
    {
        // m >= 0. Returns the clipped magnitude (<= c).
        if (mode == kHard) return m < c ? m : c;
        if (mode == kClassic)
        {
            const float k = kClassicKnee * c, room = c - k;
            return m <= k ? m : k + room * fastTanh((m - k) / room);
        }
        return c * fastTanh(m * invC);
    }
    static inline float warmShape(float x, float c, float invC)
    {
        const float t = fastTanh(x * invC);
        return x + kWarmAsym * c * t * t;
    }
    static inline void clipPair(const ClipCfg& cfg, float& l, float& r, float c, float invC)
    {
        if (cfg.mode == kWarm) { l = warmShape(l, c, invC); r = warmShape(r, c, invC); }
        if (cfg.link)
        {
            const float m = std::fmax(std::fabs(l), std::fabs(r));
            if (m > 1e-20f)
            {
                const float g = curve(cfg.mode, m, c, invC) / m;
                l *= g; r *= g;
            }
        }
        else
        {
            l = std::copysign(curve(cfg.mode, std::fabs(l), c, invC), l);
            r = std::copysign(curve(cfg.mode, std::fabs(r), c, invC), r);
        }
    }

    // Runs one oversampling chain over the chunk: wetOut/dryOut (base rate, L and R).
    void runChain(int os, int m, float* wetL, float* wetR, float* dryOutL, float* dryOutR, bool needDry)
    {
        Chain& ch = chains_[os];
        const int f = ch.wet[0].factor();
        float* hiL = ch.wet[0].upsample(preL_, m);
        float* hiR = ch.wet[1].upsample(preR_, m);
        const bool fading = cfgFading_;
        for (int i = 0; i < m; ++i)
        {
            const float c = ceilBuf_[i], invC = 1.0f / c;
            const float w = cfgWBuf_[i];
            for (int k = 0; k < f; ++k)
            {
                const int j = i * f + k;
                float l = hiL[j], r = hiR[j];
                if (!fading) clipPair(cfgNew_, l, r, c, invC);
                else
                {
                    float lo = l, ro = r;
                    clipPair(cfgOld_, lo, ro, c, invC);
                    clipPair(cfgNew_, l, r, c, invC);
                    l = lo + w * (l - lo); r = ro + w * (r - ro);
                }
                hiL[j] = l; hiR[j] = r;
            }
        }
        ch.wet[0].downsample(hiL, wetL, m);
        ch.wet[1].downsample(hiR, wetR, m);
        if (needDry)
        {
            ch.dry[0].downsample(ch.dry[0].upsample(dryL_, m), dryOutL, m);
            ch.dry[1].downsample(ch.dry[1].upsample(dryR_, m), dryOutR, m);
        }
    }

    float processChunk(const float* inL, const float* inR, float* outL, float* outR, int m)
    {
        // 1. Copy the input first: the host processes in place.
        std::memcpy(dryL_, inL, sizeof(float) * (size_t)m);
        std::memcpy(dryR_, inR, sizeof(float) * (size_t)m);

        // 2. Per-sample parameter trajectories.
        cfgFading_ = cfgW_.ramping() || cfgW_.current() < 1.0f;
        const bool osFading = osW_.ramping() || osW_.current() < 1.0f;
        const bool needDry = mix_.current() < 1.0f || mix_.target() < 1.0f;
        if (needDry && !dryRunning_)
            for (Chain& c : chains_) for (int ch = 0; ch < 2; ++ch) c.dry[ch].reset();
        dryRunning_ = needDry;

        float peakPre = 0.0f;
        for (int i = 0; i < m; ++i)
        {
            const float g = inGain_.next();
            ceilBuf_[i] = ceil_.next();
            mixBuf_[i] = mix_.next();
            outBuf_[i] = outGain_.next() * fade_.next();
            warmBuf_[i] = warmW_.next();
            cfgWBuf_[i] = cfgW_.next();
            osWBuf_[i] = osW_.next();
            preL_[i] = dryL_[i] * g;
            preR_[i] = dryR_[i] * g;
            peakPre = std::fmax(peakPre, std::fmax(std::fabs(preL_[i]), std::fabs(preR_[i])) / ceilBuf_[i]);
        }

        // 3. Oversampled clipping (two chains while the oversampling factor cross-fades).
        runChain(osNew_, m, wetL_, wetR_, dAL_, dAR_, needDry);
        if (osFading)
        {
            runChain(osOld_, m, wet2L_, wet2R_, dA2L_, dA2R_, needDry);
            for (int i = 0; i < m; ++i)
            {
                const float w = osWBuf_[i];
                wetL_[i] = wet2L_[i] + w * (wetL_[i] - wet2L_[i]);
                wetR_[i] = wet2R_[i] + w * (wetR_[i] - wet2R_[i]);
                if (needDry)
                {
                    dAL_[i] = dA2L_[i] + w * (dAL_[i] - dA2L_[i]);
                    dAR_[i] = dA2R_[i] + w * (dAR_[i] - dA2R_[i]);
                }
            }
        }

        // 4. DC blocker, selected by the smoothed Warm weight.
        for (int i = 0; i < m; ++i)
        {
            const float l = wetL_[i], r = wetR_[i];
            const float bl = dc_[0].process(l), br = dc_[1].process(r);
            const float ww = warmBuf_[i];
            wetL_[i] = l + ww * (bl - l); wetR_[i] = r + ww * (br - r);
        }

        // 5. Sample-peak safety clamp at the ceiling, mix with the aligned dry path, output gain.
        for (int i = 0; i < m; ++i)
        {
            float l = wetL_[i], r = wetR_[i];
            const float c = ceilBuf_[i];
            if (safetyClamp_)
            {
                l = l > c ? c : (l < -c ? -c : l);
                r = r > c ? c : (r < -c ? -c : r);
            }
            const float mx = mixBuf_[i], og = outBuf_[i];
            float yl = l * mx, yr = r * mx;
            if (needDry) { yl += dAL_[i] * (1.0f - mx); yr += dAR_[i] * (1.0f - mx); }
            outL[i] = yl * og;
            outR[i] = yr * og;
        }

        // 6. Never let NaN/Inf out: silence the block and clear all filter state.
        if (anyNonFinite(outL, m) || anyNonFinite(outR, m))
        {
            std::memset(outL, 0, sizeof(float) * (size_t)m);
            if (outR != outL) std::memset(outR, 0, sizeof(float) * (size_t)m);
            clearFilters();
            ++nonFiniteEvents_;
            peakPre = 0.0f;
        }

        // 7. Finish queued discrete changes once the running cross-fade has ended.
        if (!(cfgW_.ramping() || cfgW_.current() < 1.0f) && cfgPendingSet_) startCfgFade(cfgPending_);
        if (!(osW_.ramping() || osW_.current() < 1.0f) && osPendingSet_) startOsFade(osPending_);
        return isNonFinite(peakPre) ? 0.0f : peakPre;
    }

    double sampleRate_ = 44100.0;
    Params params_;

    ExpRamp inGain_, outGain_;   // ramp in dB, so Match cancels exactly while both move
    LinearRamp ceil_, mix_, warmW_, cfgW_, osW_, fade_;
    bool safetyClamp_ = true;
    ClipCfg cfgOld_, cfgNew_, cfgPending_;
    bool cfgPendingSet_ = false, cfgFading_ = false;
    int osOld_ = kOs2x, osNew_ = kOs2x, osPending_ = kOs2x;
    bool osPendingSet_ = false, dryRunning_ = false;

    Chain chains_[3];
    DcBlocker dc_[2];
    std::atomic<float> clipDb_{0.0f};
    int nonFiniteEvents_ = 0;

    float dryL_[kMaxBlock] = {}, dryR_[kMaxBlock] = {};
    float preL_[kMaxBlock] = {}, preR_[kMaxBlock] = {};
    float wetL_[kMaxBlock] = {}, wetR_[kMaxBlock] = {}, wet2L_[kMaxBlock] = {}, wet2R_[kMaxBlock] = {};
    float dAL_[kMaxBlock] = {}, dAR_[kMaxBlock] = {}, dA2L_[kMaxBlock] = {}, dA2R_[kMaxBlock] = {};
    float ceilBuf_[kMaxBlock] = {};
    float mixBuf_[kMaxBlock] = {}, outBuf_[kMaxBlock] = {};
    float warmBuf_[kMaxBlock] = {}, cfgWBuf_[kMaxBlock] = {}, osWBuf_[kMaxBlock] = {};
};

} // namespace masterclip
