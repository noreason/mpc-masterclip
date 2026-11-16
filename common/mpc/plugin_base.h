// plugin_base.h - PluginStand catalog base class on top of the SDK's mpc::Plugin.
//
// Everything every catalog plugin needs, learned on MasterClip (docs/PLAYBOOK.md):
//   * Parameter 0 is the reserved no-op `reserved0` (the MPC nudges parameter 0 on every prepare).
//     The constructor adds it; plugin parameters start at index 1.
//   * Meters: addMeter() declares a read-only, non-automatable parameter; setMeter() (audio thread)
//     stores the value; the MeterNotifier announces changes from the host's idle call (never from the
//     audio thread), because the MPC screen only redraws parameters it is told about.
//     Meters must come after every user parameter.
//   * Presets: addPreset("Name", {{index, plain}, ...}) fills every user parameter from its default
//     and applies the overrides; meters are never touched. Add "Default" first: the MPC selects
//     program 0 right after loading.
//   * Value text with units (the MPC does not append unit labels): "+3.0 dB" (signed for dB ranges
//     that go negative, "0.0 dB" at zero), "70 %", "120 Hz". Override formatParam() for custom text.
//   * State: one version byte of extra state (guarantees loadExtraState() runs); plugin extra state via
//     saveExtra()/loadExtra().
//   * Announced parameters: addAnnounced() declares a user parameter that the plugin also moves itself
//     (a playhead); the notifier tells the host about it like a meter, with its own epsilon.
//   * Long value text: override maxDisplayChars(index) (default 31, the framework's limit). The MPC
//     reads up to 100 characters of value text (Crate Digger, MPC OS 5.0); capped at 63 here.
//   * hostLoading(): true while the host restores a chunk or selects a program. Parameter writes seen then
//     are restored values, not user actions (keeps trigger buttons from firing on project load).
//   * Build switches (scripts/build-all --define X=1):
//       CAT_DEVICE_LOG   log restored state and the meter notifier's route to stderr (MPC journal)
//       CAT_APPEND_TEST  append one extra parameter at the end (device test: appending in a later version)
//       CAT_NO_METER     never write or announce meters (CPU comparison)
//
// Usage:
//   class MyPlugin : public cat::mpcx::CatalogPlugin {
//     MyPlugin() : CatalogPlugin(info()) { kGain = addParam(...); kMeter = addMeter("out_meter", "Out", 24.0f);
//                                          addPreset("Default", {}); ...; finishParams(); }
//     void prepareDsp(double sr, int maxBlock) override; void resetDsp() override;
//     void process(mpc::ProcessContext&) override;
//   };
//   CAT_PLUGIN_ENTRY(MyPlugin)
#pragma once
#include "mpc_plugin.h"
#include "meter_notifier.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

namespace cat::mpcx {

class CatalogPlugin : public mpc::Plugin
{
public:
    struct PresetValue { int index; float plain; };

    explicit CatalogPlugin(const mpc::PluginInfo& info) : mpc::Plugin(info)
    {
        mpc::Param r = mpc::floatParam("reserved0", "Reserved", 0.0f, 1.0f, 0.0f);
        r.automatable = false;
        mpc::Plugin::addParam(r);
#if defined(CAT_DEVICE_LOG)
        notifier_.setVerbose(true);
#endif
    }

    // ---- declaration (constructor only) -------------------------------------------------------
    int addParam(const mpc::Param& p)
    {
        if (firstMeter_ >= 0) mpc::log("%s: parameter '%s' declared after a meter; meters must be last", info().name, p.id.c_str());
        return mpc::Plugin::addParam(p);
    }

    // A read-only meter parameter (0 .. maxValue, plain units), hidden from automation.
    // keepalive = false: announce changes only (see MeterNotifier::addParam).
    int addMeter(const char* id, const char* name, float maxValue, const char* unit = "dB", int decimals = 1, float epsilon = -1.0f,
                 bool keepalive = true)
    {
        mpc::Param m = mpc::floatParam(id, name, 0.0f, maxValue, 0.0f, unit, decimals);
        m.automatable = false;
        const int idx = mpc::Plugin::addParam(m);
        if (firstMeter_ < 0) firstMeter_ = idx;
        isMeter_.resize((size_t)idx + 1, false);
        isMeter_[(size_t)idx] = true;
        notifier_.addParam(idx, epsilon, keepalive);
        return idx;
    }

    // A user parameter the plugin also writes itself (a playhead). Announced like a meter, with `epsilon`
    // (normalised) as the smallest move worth a redraw. Declare it before the meters, and make it
    // non-automatable (rt_host fails announcements of automatable parameters: they write automation).
    int addAnnounced(const mpc::Param& p, float epsilon)
    {
        const int idx = addParam(p);
        notifier_.addParam(idx, epsilon);
        return idx;
    }

    void addPreset(const char* name, std::initializer_list<PresetValue> overrides)
    {
        const int n = firstMeter_ >= 0 ? firstMeter_ : numParams();
        std::vector<float> v((size_t)n);
        for (int i = 0; i < n; ++i) v[(size_t)i] = param(i).def;
        for (const PresetValue& o : overrides)
        {
            if (o.index <= 0 || o.index >= n) { mpc::log("%s: preset '%s' sets index %d (not a user parameter)", info().name, name, o.index); continue; }
            v[(size_t)o.index] = o.plain;
        }
        addProgram(name, std::move(v));
    }

    // Call at the end of the derived constructor.
    void finishParams()
    {
#if defined(CAT_APPEND_TEST)
        mpc::Plugin::addParam(mpc::floatParam("append_test", "Append Test", 0.0f, 1.0f, 0.5f));
#endif
        if (programs_.empty() || programs_[0].name != "Default")
            mpc::log("%s: program 0 should be \"Default\" (the MPC selects it after loading)", info().name);
    }

    // The plugin set a user parameter because of a tap on its own screen: tell the host (from idle).
    void announceEdit(int index) { notifier_.requestEdit(index); }

    bool isMeter(int i) const { return i >= 0 && (size_t)i < isMeter_.size() && isMeter_[(size_t)i]; }

    // ---- audio thread ---------------------------------------------------------------------------
    // Stores a meter value (plain units). A relaxed atomic store; the host is never called here.
    void setMeter(int index, float plain)
    {
#if !defined(CAT_NO_METER)
        setNormalised(index, param(index).toNormalised(plain));
#else
        (void)index; (void)plain;
#endif
    }

    // ---- lifecycle --------------------------------------------------------------------------------
    virtual void prepareDsp(double sampleRate, int maxBlockSize) = 0;
    virtual void resetDsp() {}

    void prepare(double sampleRate, int maxBlockSize) final
    {
        prepareDsp(sampleRate, maxBlockSize);
#if !defined(CAT_NO_METER)
        notifier_.start();   // UI thread; no-op without meters
#endif
    }
    // Resume, state load and preset change (possibly while audio runs): fade in inside resetDsp().
    void reset() final { resetDsp(); }

    // ---- value text -------------------------------------------------------------------------------
    virtual bool formatParam(int index, float plain, std::string& out) { (void)index; (void)plain; (void)out; return false; }

    bool formatValue(int index, float v, std::string& out) final
    {
        if (index == 0) { out = "-"; return true; }
        if (formatParam(index, v, out)) return true;
        const mpc::Param& p = param(index);
        if (p.kind != mpc::Param::Kind::Float && p.kind != mpc::Param::Kind::Int) return false;
        char b[48];
        const bool signedDb = p.unit == "dB" && p.min < 0.0f;
        std::snprintf(b, sizeof(b), signedDb ? "%+.*f" : "%.*f", p.decimals, (double)v);
        std::string num = b;
        if (num.size() > 1 && (num[0] == '+' || num[0] == '-') && std::strspn(num.c_str() + 1, "0.") == num.size() - 1)
            num = num.substr(1);                       // "+0.0" / "-0.0" -> "0.0"
        out = p.unit.empty() ? num : num + " " + p.unit;
        return true;
    }

    // ---- state ------------------------------------------------------------------------------------
    virtual void saveExtra(std::vector<uint8_t>& out) { (void)out; }
    virtual void loadExtra(const uint8_t* data, size_t size) { (void)data; (void)size; }

    void saveExtraState(std::vector<uint8_t>& out) final
    {
        out.push_back(1);   // catalog extra-state version
        saveExtra(out);
    }
    void loadExtraState(const uint8_t* data, size_t size) final
    {
        if (size >= 1 && data[0] == 1) loadExtra(data + 1, size - 1);
#if defined(CAT_DEVICE_LOG)
        std::string s;   // UI thread, after the parameters were restored: safe to log
        for (int i = 0; i < numParams(); ++i) s += param(i).id + "=" + displayText(i) + (i + 1 < numParams() ? ", " : "");
        mpc::log("%s state loaded (%d params in this build, extra %zu bytes): %s", info().name, numParams(), size, s.c_str());
#endif
    }

    // ---- long value text and the host-loading guard ---------------------------------------------------
    virtual int maxDisplayChars(int index) const { (void)index; return 31; }
    bool hostLoading() const { return hostLoading_.load(std::memory_order_acquire) > 0; }

    template <class T> friend AEffect* createCatalogEffect(audioMasterCallback host);

    // Wraps the notifier's dispatcher: long value text, and the loading guard around chunk/program loads.
    static VstIntPtr catalogDispatcher(AEffect* e, VstInt32 op, VstInt32 index, VstIntPtr value, void* ptr, float opt)
    {
        // Only installed by createCatalogEffect<T>, so e->object is always a CatalogPlugin.
        CatalogPlugin* p = e->object ? static_cast<CatalogPlugin*>(static_cast<mpc::Plugin*>(e->object)) : nullptr;
        if (p && op == effGetParamDisplay && ptr && index >= 0 && index < p->numParams() && p->maxDisplayChars(index) > 31)
        {
            const std::string s = p->displayText(index);
            const size_t len = std::min<size_t>(s.size(), (size_t)std::min(p->maxDisplayChars(index), 63));
            std::memcpy(ptr, s.data(), len);
            static_cast<char*>(ptr)[len] = 0;
            return 0;
        }
        if (p && (op == effSetChunk || op == effSetProgram))
        {
            p->hostLoading_.fetch_add(1, std::memory_order_acq_rel);
            const VstIntPtr r = MeterNotifier::hookDispatcher(e, op, index, value, ptr, opt);
            p->hostLoading_.fetch_sub(1, std::memory_order_acq_rel);
            return r;
        }
        return MeterNotifier::hookDispatcher(e, op, index, value, ptr, opt);
    }

protected:
    MeterNotifier notifier_{ *this };

private:
    std::atomic<int> hostLoading_{0};
    int firstMeter_ = -1;
    std::vector<bool> isMeter_;
};

// The SDK's MPC_PLUGIN_ENTRY body plus the meter notifier's idle hook.
template <class T>
AEffect* createCatalogEffect(audioMasterCallback host)
{
    if (host != nullptr && host(nullptr, audioMasterVersion, 0, 0, nullptr, 0.0f) == 0)
        return nullptr;   // VST1-only host
    T* plugin = new T();
    AEffect* e = mpc::detail::createEffect(plugin, host);
    if (e != nullptr)
    {
        MeterNotifier::installIdleHook(e, &plugin->notifier_);
        e->dispatcher = &CatalogPlugin::catalogDispatcher;
    }
    return e;
}

} // namespace cat::mpcx

#define CAT_PLUGIN_ENTRY(PluginClass)                                                              \
    extern "C" MPC_EXPORT AEffect* VSTPluginMain(audioMasterCallback host)                         \
    { return cat::mpcx::createCatalogEffect<PluginClass>(host); }
