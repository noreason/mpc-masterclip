// ramp.h - linear parameter smoother.
//
// A new target starts a straight-line ramp that reaches it in exactly `rampSamples` samples and
// then holds it exactly (no asymptotic tail, so "Mix 100 %" really is 1.0f and "Output 0 dB"
// really is 1.0f). A linear ramp also bounds the per-sample change, which is what makes the
// no-click tests provable: |step| <= |target - start| / rampSamples.
#pragma once
#include <cmath>

namespace cat::dsp {

class LinearRamp
{
public:
    void setRampSamples(int n) { rampSamples_ = n < 1 ? 1 : n; }
    void setRampTime(double ms, double sampleRate) { setRampSamples((int)std::lround(ms * 0.001 * sampleRate)); }

    // Jump straight to v (no ramp). Use in prepare()/reset().
    void snap(float v) { cur_ = target_ = v; step_ = 0.0f; left_ = 0; }

    void setTarget(float v)
    {
        if (v == target_) return;
        target_ = v;
        left_ = rampSamples_;
        step_ = (target_ - cur_) / (float)rampSamples_;
    }

    float next()
    {
        if (left_ > 0)
        {
            if (--left_ == 0) cur_ = target_;
            else cur_ += step_;
        }
        return cur_;
    }

    bool ramping() const { return left_ > 0; }
    float current() const { return cur_; }
    float target() const { return target_; }

private:
    float cur_ = 0.0f, target_ = 0.0f, step_ = 0.0f;
    int left_ = 0, rampSamples_ = 1;
};

} // namespace cat::dsp

namespace cat::dsp {

// Ramps a positive gain linearly in dB (multiplicatively) over `rampSamples`, ending exactly on
// the target. Two of these moving in opposite directions over the same time keep their product
// exactly constant, which is what a "Match" (input/output compensation) control needs.
// One std::pow per target change, one multiply per sample.
class ExpRamp
{
public:
    void setRampSamples(int n) { rampSamples_ = n < 1 ? 1 : n; }
    void setRampTime(double ms, double sampleRate) { setRampSamples((int)std::lround(ms * 0.001 * sampleRate)); }
    void snap(float v) { cur_ = target_ = v; ratio_ = 1.0f; left_ = 0; }
    void setTarget(float v)   // v > 0
    {
        if (v == target_) return;
        target_ = v;
        left_ = rampSamples_;
        ratio_ = (float)std::pow((double)target_ / (double)cur_, 1.0 / (double)rampSamples_);
    }
    float next()
    {
        if (left_ > 0)
        {
            if (--left_ == 0) cur_ = target_;
            else cur_ *= ratio_;
        }
        return cur_;
    }
    bool ramping() const { return left_ > 0; }
    float current() const { return cur_; }
    float target() const { return target_; }

private:
    float cur_ = 1.0f, target_ = 1.0f, ratio_ = 1.0f;
    int left_ = 0, rampSamples_ = 1;
};

} // namespace cat::dsp
