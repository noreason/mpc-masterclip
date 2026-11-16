// peak_hold.h - display ballistics for meters (per block, not per sample).
//
// Instant attack; a new peak is held for `holdMs`, then the value falls at `fallDbPerSec`.
// Values are in dB-like units >= 0 (e.g. "dB clipped"). Real-time safe: no allocation, no libm.
#pragma once

namespace cat::dsp {

class PeakHold
{
public:
    void setTimes(float holdMs, float fallDbPerSec) { holdS_ = holdMs * 0.001f; fall_ = fallDbPerSec; }
    void reset() { value_ = 0.0f; holdLeft_ = 0.0f; }
    // Feeds one block's peak (dB) and its duration (s). Returns the value to display.
    float push(float peak, float blockSeconds)
    {
        if (!(peak >= 0.0f)) peak = 0.0f;   // also rejects NaN
        if (peak >= value_) { value_ = peak; holdLeft_ = holdS_; }
        else if (holdLeft_ > 0.0f) holdLeft_ -= blockSeconds;
        else
        {
            value_ -= fall_ * blockSeconds;
            if (value_ < peak) value_ = peak;
        }
        return value_;
    }
    float value() const { return value_; }

private:
    float value_ = 0.0f, holdLeft_ = 0.0f, holdS_ = 0.3f, fall_ = 30.0f;
};

} // namespace cat::dsp
