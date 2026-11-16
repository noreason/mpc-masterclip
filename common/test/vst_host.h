// vst_host.h - minimal VST2 host for desktop tests and tools (dlopen a built plugin .so and drive it
// through the ABI the way the MPC does). Test/tool code only.
//
// The host callback records every plugin->host call with the calling thread, so tests can prove
// that the audio thread never calls the host (except audioMasterGetTime, which the SDK framework
// calls every block and the MPC expects).
#pragma once
#include "mpc_vst2_abi.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace cat::test {

struct HostLog
{
    std::atomic<long> automate[1024] = {};      // audioMasterAutomate per parameter index
    std::atomic<long> updateDisplay{0}, ioChanged{0}, needIdle{0}, other{0};
    std::atomic<long> audioThreadCalls{0};    // host calls made from the audio thread (except GetTime)
    std::atomic<std::thread::id> audioThread{};
    std::atomic<bool> grantIdle{true};        // answer audioMasterNeedIdle with 1 (the MPC answers 0 but still idles)
    void reset()
    {
        for (auto& a : automate) a = 0;
        updateDisplay = ioChanged = needIdle = other = audioThreadCalls = 0;
    }
};

inline HostLog& hostLog() { static HostLog l; return l; }

inline VstIntPtr testHostCallback(AEffect*, VstInt32 opcode, VstInt32 index, VstIntPtr, void* ptr, float)
{
    HostLog& L = hostLog();
    const bool onAudio = std::this_thread::get_id() == L.audioThread.load();
    switch (opcode)
    {
        case audioMasterVersion: return 2400;
        case audioMasterGetVendorString: case audioMasterGetProductString:
            if (ptr) std::strcpy(static_cast<char*>(ptr), "MPC 3");
            return 1;
        case audioMasterGetTime: return 0;   // no transport info (allowed from the audio thread)
        case audioMasterAutomate:
            if (index >= 0 && index < 1024) L.automate[index]++;
            break;
        case audioMasterUpdateDisplay: L.updateDisplay++; break;
        case audioMasterIOChanged: L.ioChanged++; break;
        case audioMasterNeedIdle_Deprecated: L.needIdle++; if (onAudio) L.audioThreadCalls++; return L.grantIdle ? 1 : 0;
        default: L.other++; break;
    }
    if (onAudio) L.audioThreadCalls++;
    return 0;
}

class VstPlugin
{
public:
    explicit VstPlugin(const std::string& path, double sr = 44100.0, int block = 128)
    {
        lib_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!lib_) { std::fprintf(stderr, "dlopen %s: %s\n", path.c_str(), dlerror()); std::exit(2); }
        using Main = AEffect* (*)(audioMasterCallback);
        Main m = reinterpret_cast<Main>(dlsym(lib_, "VSTPluginMain"));
        if (!m) { std::fprintf(stderr, "%s: no VSTPluginMain\n", path.c_str()); std::exit(2); }
        e = m(testHostCallback);
        if (!e) { std::fprintf(stderr, "%s: VSTPluginMain returned null\n", path.c_str()); std::exit(2); }
        disp(effOpen);
        disp(effSetSampleRate, 0, 0, nullptr, (float)sr);
        disp(effSetBlockSize, 0, block);
        disp(effMainsChanged, 0, 1);
    }
    ~VstPlugin() { disp(effMainsChanged, 0, 0); disp(effClose); dlclose(lib_); }
    VstPlugin(const VstPlugin&) = delete;
    VstPlugin& operator=(const VstPlugin&) = delete;

    VstIntPtr disp(VstInt32 op, VstInt32 idx = 0, VstIntPtr v = 0, void* p = nullptr, float o = 0.0f) { return e->dispatcher(e, op, idx, v, p, o); }
    int numParams() const { return e->numParams; }
    int numPrograms() const { return e->numPrograms; }
    float get(int i) { return e->getParameter(e, i); }
    void set(int i, float v) { e->setParameter(e, i, v); }
    std::string str(VstInt32 op, int i) { char b[256] = {}; disp(op, i, 0, b); return b; }
    std::string name(int i) { return str(effGetParamName, i); }
    std::string display(int i) { return str(effGetParamDisplay, i); }
    std::string programName(int k) { char b[256] = {}; disp(effGetProgramNameIndexed, k, 0, b); return b; }
    bool automatable(int i) { return disp(effCanBeAutomated, i) == 1; }
    std::vector<uint8_t> chunk() { void* d = nullptr; const VstIntPtr n = disp(effGetChunk, 0, 0, &d); return std::vector<uint8_t>((uint8_t*)d, (uint8_t*)d + n); }
    bool setChunk(std::vector<uint8_t> c) { return disp(effSetChunk, 0, (VstIntPtr)c.size(), c.data()) == 1; }
    void idle() { disp(effIdle_Deprecated); }

    // In-place stereo processing in blocks, like the MPC.
    void process(std::vector<float>& l, std::vector<float>& r, int block)
    {
        for (size_t i = 0; i < l.size(); i += (size_t)block)
        {
            const int n = (int)std::min<size_t>((size_t)block, l.size() - i);
            float* io[2] = { l.data() + i, r.data() + i };
            hostLog().audioThread = std::this_thread::get_id();   // host calls made now count as audio-thread calls
            e->processReplacing(e, io, io, n);
            hostLog().audioThread = std::thread::id();
        }
    }

    AEffect* e = nullptr;

private:
    void* lib_ = nullptr;
};

} // namespace cat::test
