// denormal.h - flush denormals to zero for the lifetime of a scope.
//
// The SDK framework already does this around every process() call on the MPC; the DSP core
// still sets it itself so desktop tests and benchmarks run under the same FP rules as the device.
//   arm32  : FPSCR.FZ (bit 24)        - also makes NEON and VFP flush
//   arm64  : FPCR.FZ  (bit 24)
//   x86-64 : MXCSR FTZ | DAZ (0x8040)
#pragma once
#include <cstdint>

namespace cat::dsp {

class ScopedFlushDenormals
{
public:
    ScopedFlushDenormals()
    {
#if defined(__aarch64__)
        uint64_t v; __asm__ __volatile__("mrs %0, fpcr" : "=r"(v));
        saved_ = v; v |= (1ull << 24);
        __asm__ __volatile__("msr fpcr, %0" : : "r"(v));
#elif defined(__arm__)
        uint32_t v; __asm__ __volatile__("vmrs %0, fpscr" : "=r"(v));
        saved_ = v; v |= (1u << 24);
        __asm__ __volatile__("vmsr fpscr, %0" : : "r"(v));
#elif defined(__x86_64__) || defined(__SSE__)
        uint32_t v; __asm__ __volatile__("stmxcsr %0" : "=m"(v));
        saved_ = v; v |= 0x8040u;
        __asm__ __volatile__("ldmxcsr %0" : : "m"(v));
#endif
    }
    ~ScopedFlushDenormals()
    {
#if defined(__aarch64__)
        uint64_t v = saved_; __asm__ __volatile__("msr fpcr, %0" : : "r"(v));
#elif defined(__arm__)
        uint32_t v = (uint32_t)saved_; __asm__ __volatile__("vmsr fpscr, %0" : : "r"(v));
#elif defined(__x86_64__) || defined(__SSE__)
        uint32_t v = (uint32_t)saved_; __asm__ __volatile__("ldmxcsr %0" : : "m"(v));
#endif
    }
    ScopedFlushDenormals(const ScopedFlushDenormals&) = delete;
    ScopedFlushDenormals& operator=(const ScopedFlushDenormals&) = delete;

private:
    uint64_t saved_ = 0;
};

} // namespace cat::dsp
