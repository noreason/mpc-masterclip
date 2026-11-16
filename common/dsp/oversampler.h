// oversampler.h - 1x / 2x / 4x mono oversampler built from polyphase IIR halfbands.
//
// Stage A (base <-> 2x): 8 coefficients, transition 0.0465 of the 2x rate
//   -> passband to 20.0 kHz at 44.1 kHz, stopband from 24.1 kHz, >= 104 dB (verified in tests).
// Stage B (2x <-> 4x): 4 coefficients, transition 0.27 of the 4x rate -> >= 124 dB.
// Fixed-size buffers (kMaxBlock base-rate frames per call): no allocation anywhere.
#pragma once
#include "halfband.h"

namespace cat::dsp {

class Oversampler
{
public:
    static constexpr int kMaxBlock = 512;
    static constexpr int kMaxFactor = 4;
    static constexpr int kStageACoefs = 8;
    static constexpr double kStageATransition = 0.0465;
    static constexpr int kStageBCoefs = 4;
    static constexpr double kStageBTransition = 0.27;

    // Designs coefficients (uses libm; call outside the audio thread).
    static const HalfbandCoefs& stageA() { static const HalfbandCoefs c = HalfbandCoefs::design(kStageACoefs, kStageATransition); return c; }
    static const HalfbandCoefs& stageB() { static const HalfbandCoefs c = HalfbandCoefs::design(kStageBCoefs, kStageBTransition); return c; }

    void init(int factor)   // 1, 2 or 4. Not real-time safe the first time (static design).
    {
        factor_ = factor == 4 ? 4 : (factor == 2 ? 2 : 1);
        upA_.setCoefs(stageA()); downA_.setCoefs(stageA());
        upB_.setCoefs(stageB()); downB_.setCoefs(stageB());
        reset();
    }
    void reset() { upA_.reset(); downA_.reset(); upB_.reset(); downB_.reset(); }
    int factor() const { return factor_; }

    // Upsamples n (<= kMaxBlock) base-rate samples. Returns the internal high-rate buffer
    // (n * factor samples), which the caller may modify in place before downsample().
    float* upsample(const float* in, int n)
    {
        if (factor_ == 1) { for (int i = 0; i < n; ++i) hi_[i] = in[i]; return hi_; }
        if (factor_ == 2) { upA_.processBlock(in, hi_, n); return hi_; }
        upA_.processBlock(in, mid_, n);
        upB_.processBlock(mid_, hi_, 2 * n);
        return hi_;
    }

    // Downsamples n * factor high-rate samples (normally the buffer upsample() returned) to n.
    void downsample(const float* hi, float* out, int n)
    {
        if (factor_ == 1) { for (int i = 0; i < n; ++i) out[i] = hi[i]; return; }
        if (factor_ == 2) { downA_.processBlock(hi, out, n); return; }
        downB_.processBlock(hi, mid_, 2 * n);
        downA_.processBlock(mid_, out, n);
    }

private:
    int factor_ = 1;
    HalfbandUp2 upA_, upB_;
    HalfbandDown2 downA_, downB_;
    float mid_[kMaxBlock * 2] = {};
    float hi_[kMaxBlock * kMaxFactor] = {};
};

} // namespace cat::dsp
