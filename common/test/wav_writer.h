// wav_writer.h - minimal 32-bit float WAV writer (mono or stereo). Test/offline use only.
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace cat::test {

inline bool writeWavFloat(const std::string& path, const std::vector<float>& left,
                          const std::vector<float>* right = nullptr, int sampleRate = 44100)
{
    const uint16_t ch = right ? 2 : 1;
    const uint32_t frames = (uint32_t)left.size();
    const uint32_t dataBytes = frames * ch * 4;
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    auto u32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto u16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f); u32(36 + dataBytes); std::fwrite("WAVE", 1, 4, f);
    std::fwrite("fmt ", 1, 4, f); u32(16); u16(3 /* IEEE float */); u16(ch);
    u32((uint32_t)sampleRate); u32((uint32_t)sampleRate * ch * 4); u16((uint16_t)(ch * 4)); u16(32);
    std::fwrite("data", 1, 4, f); u32(dataBytes);
    for (uint32_t i = 0; i < frames; ++i)
    {
        std::fwrite(&left[i], 4, 1, f);
        if (right) std::fwrite(&(*right)[i], 4, 1, f);
    }
    return std::fclose(f) == 0;
}

} // namespace cat::test
