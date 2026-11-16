// dc_blocker.h - first-order DC blocking high-pass: y[n] = x[n] - x[n-1] + R * y[n-1].
#pragma once
#include <cmath>

namespace cat::dsp {

class DcBlocker
{
public:
    // cutoffHz ~ (1 - R) * fs / (2 pi). 5 Hz at 44.1 kHz gives R = 0.99929.
    void setCutoff(double cutoffHz, double sampleRate)
    {
        r_ = (float)std::exp(-2.0 * 3.14159265358979323846 * cutoffHz / sampleRate);
    }
    void reset() { x1_ = y1_ = 0.0f; }
    float process(float x)
    {
        const float y = x - x1_ + r_ * y1_;
        x1_ = x; y1_ = y;
        return y;
    }

private:
    float r_ = 0.9993f, x1_ = 0.0f, y1_ = 0.0f;
};

} // namespace cat::dsp
