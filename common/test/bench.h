// bench.h - wall-clock micro-benchmark helper: nanoseconds per (stereo) frame.
#pragma once
#include <algorithm>
#include <chrono>
#include <vector>

namespace cat::test {

// Calls fn(frames) repeatedly for at least `minSeconds`, `reps` times, and returns the best
// (lowest) ns per frame: the least disturbed run.
template <typename Fn>
double nsPerFrame(Fn&& fn, int framesPerCall, double minSeconds = 0.3, int reps = 5)
{
    using clk = std::chrono::steady_clock;
    double best = 1e30;
    for (int r = 0; r < reps; ++r)
    {
        long frames = 0;
        const auto t0 = clk::now();
        double el = 0.0;
        do
        {
            for (int k = 0; k < 64; ++k) { fn(framesPerCall); frames += framesPerCall; }
            el = std::chrono::duration<double>(clk::now() - t0).count();
        } while (el < minSeconds);
        best = std::min(best, el * 1e9 / (double)frames);
    }
    return best;
}

} // namespace cat::test
