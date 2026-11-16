// meter_notifier.h - tells the MPC that read-only meter parameters changed, never from the audio thread.
//
// Why: on MPC OS 3.10.1.12 the plugin screen reads parameter values when the edit screen opens and
// afterwards only redraws a parameter the host was told about. Writing a meter parameter from the
// audio thread (plain atomic store) is therefore invisible (docs/UI_NOTES.md). The audio thread must
// not call the host, so the notification comes from one of two non-audio contexts:
//   1. idle: the plugin asks for idle calls (audioMasterNeedIdle). If the host then calls effIdle /
//      effEditIdle on its UI thread, meters are notified from there.
//   2. thread: otherwise a small plugin-owned thread wakes every `periodMs` and notifies.
// Either way only parameters whose value moved by more than `epsilon` (or every second) are sent,
// with audioMasterAutomate(index, normalised value). Concurrent sends from both contexts are excluded.
// With verbose = true (development builds) the first use of each route and every host opcode seen are
// logged to stderr (the MPC journal); release builds log nothing.
//
// Device result (MPC Key 37 G2, MPC OS 3.10.1.12): the host answers audioMasterNeedIdle with 0 but then
// calls effIdle (opcode 53) on its UI thread, so notifications come from the host's idle. The thread
// only covers the moment before the first idle call.
//
// Use: own one MeterNotifier per plugin instance, call start() from prepare() (UI thread), install
// installIdleHook() on the AEffect right after creation, stop() in the destructor.
#pragma once
#include "mpc_plugin.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace cat::mpcx {

class MeterNotifier
{
public:
    MeterNotifier(mpc::Plugin& plugin, std::vector<int> meterParams = {}, bool verbose = false, int periodMs = 66, float epsilon = 0.004f)
        : plugin_(plugin), params_(std::move(meterParams)), eps_(params_.size(), -1.0f), keep_(params_.size(), true), verbose_(verbose), periodMs_(periodMs), epsilon_(epsilon) {}
    ~MeterNotifier() { stop(); }

    // Constructor time only (before start()).
    // epsilon < 0: the notifier's default. A smaller epsilon redraws finer moves (a playhead).
    // keepalive: also re-send it every second when it hasn't changed (false for large banks of meters,
    // e.g. a waveform: the MPC reads every value when the plugin screen opens, so only changes matter).
    void addParam(int index, float epsilon = -1.0f, bool keepalive = true) { params_.push_back(index); eps_.push_back(epsilon); keep_.push_back(keepalive); }
    void setVerbose(bool v) { verbose_ = v; }

    // A user edit made on the plugin's own controls (e.g. picking a value from a list the plugin draws):
    // the plugin has set the parameter; the host is told (audioMasterAutomate) from idle, like a plugin
    // editor reports a knob turn. Any thread, lock-free. Parameters 0..63 only.
    void requestEdit(int index) { if (index >= 0 && index < kMaxEdits) edits_[index].store(true, std::memory_order_release); }
    bool empty() const { return params_.empty(); }

    // UI thread (prepare). Requests idle calls once and starts the fallback thread. No-op without meters.
    void start()
    {
        if (params_.empty()) return;
        if (last_.size() != params_.size()) { last_.assign(params_.size(), -1.0f); lastSent_.assign(params_.size(), 0); }
        if (!idleRequested_ && plugin_.host_ && plugin_.effect_)
        {
            idleRequested_ = true;
            const VstIntPtr r = plugin_.host_(plugin_.effect_, audioMasterNeedIdle_Deprecated, 0, 0, nullptr, 0.0f);
            if (verbose_) mpc::log("meter: requested idle calls (audioMasterNeedIdle -> %ld)", (long)r);
        }
        if (!thread_.joinable())
        {
            quit_ = false;
            thread_ = std::thread([this] { run(); });
        }
    }

    void stop()
    {
        { std::lock_guard<std::mutex> g(m_); quit_ = true; }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    // Host idle (UI thread).
    void onIdle()
    {
        lastIdle_.store(nowMs(), std::memory_order_relaxed);
        if (!loggedIdle_.exchange(true) && verbose_) mpc::log("meter: host idle calls arrive; notifying from idle");
        send(false);
    }

    // Called by the dispatcher hook for every opcode (any thread, including audio): only sets a bit.
    void sawOpcode(VstInt32 op)
    {
        if (op >= 0 && op < 64) seen_.fetch_or(1ull << op, std::memory_order_relaxed);
    }

    // Replaces the effect's dispatcher with one that forwards idle to `self` and records opcodes.
    // `self` is found through a small registry keyed by AEffect* (the framework owns e->object).
    static void installIdleHook(AEffect* e, MeterNotifier* self)
    {
        registry().store(e, self);
        e->dispatcher = &hookDispatcher;
    }

private:
    struct Registry
    {
        static constexpr int kSlots = 64;
        std::atomic<AEffect*> keys[kSlots] = {};
        std::atomic<MeterNotifier*> vals[kSlots] = {};
        void store(AEffect* e, MeterNotifier* n)
        {
            for (int i = 0; i < kSlots; ++i)
            {
                AEffect* expect = nullptr;
                if (keys[i].compare_exchange_strong(expect, e)) { vals[i].store(n); return; }
            }
        }
        MeterNotifier* find(AEffect* e)
        {
            for (int i = 0; i < kSlots; ++i) if (keys[i].load() == e) return vals[i].load();
            return nullptr;
        }
        void erase(AEffect* e)
        {
            for (int i = 0; i < kSlots; ++i) if (keys[i].load() == e) { vals[i].store(nullptr); keys[i].store(nullptr); }
        }
    };
    static Registry& registry() { static Registry r; return r; }

public:
    // Public so a wrapping dispatcher (CatalogPlugin) can forward to it.
    static VstIntPtr hookDispatcher(AEffect* e, VstInt32 op, VstInt32 index, VstIntPtr value, void* ptr, float opt)
    {
        MeterNotifier* n = registry().find(e);
        if (n)
        {
            n->sawOpcode(op);
            if (op == effIdle_Deprecated || op == effEditIdle) n->onIdle();
            if (op == effClose) registry().erase(e);   // the framework deletes the plugin (and us) next
        }
        return mpc::detail::dispatcher(e, op, index, value, ptr, opt);
    }

private:
    static int64_t nowMs()
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }

    void run()
    {
        uint64_t logged = 0;
        std::unique_lock<std::mutex> lk(m_);
        while (!quit_)
        {
            cv_.wait_for(lk, std::chrono::milliseconds(periodMs_));
            if (quit_) break;
            lk.unlock();
            if (nowMs() - lastIdle_.load(std::memory_order_relaxed) > 4 * periodMs_)
            {
                if (!loggedThread_.exchange(true) && verbose_) mpc::log("meter: no host idle; notifying from the meter thread every %d ms", periodMs_);
                send(true);
            }
            const uint64_t s = seen_.load(std::memory_order_relaxed);
            if (verbose_ && s != logged)
            {
                for (int op = 0; op < 64; ++op)
                    if ((s & ~logged) & (1ull << op)) mpc::log("meter: host sent opcode %d", op);
                logged = s;
            }
            lk.lock();
        }
    }

    void send(bool fromThread)
    {
        bool expect = false;
        if (!busy_.compare_exchange_strong(expect, true)) return;   // the other context is sending
        if (plugin_.host_ && plugin_.effect_)
        {
            const int64_t now = nowMs();
            for (int i = 0; i < kMaxEdits; ++i)
                if (edits_[i].exchange(false, std::memory_order_acq_rel))
                    plugin_.host_(plugin_.effect_, audioMasterAutomate, i, 0, nullptr, plugin_.getNormalised(i));
            for (size_t i = 0; i < params_.size(); ++i)
            {
                const float v = plugin_.getNormalised(params_[i]);
                const float d = v - last_[i];
                const float eps = (i < eps_.size() && eps_[i] >= 0.0f) ? eps_[i] : epsilon_;
                if (d > eps || d < -eps || (keep_[i] && now - lastSent_[i] > 1000))
                {
                    plugin_.host_(plugin_.effect_, audioMasterAutomate, params_[i], 0, nullptr, v);
                    last_[i] = v; lastSent_[i] = now;
                    if (!loggedFirstSend_.exchange(true) && verbose_)
                        mpc::log("meter: first notification sent from %s (param %d = %.3f)", fromThread ? "thread" : "idle", params_[i], (double)v);
                }
            }
        }
        busy_.store(false);
    }

    mpc::Plugin& plugin_;
    std::vector<int> params_;
    std::vector<float> eps_;   // per parameter; < 0 = epsilon_
    std::vector<bool> keep_;   // per parameter: re-send every second
    static constexpr int kMaxEdits = 64;
    std::atomic<bool> edits_[kMaxEdits] = {};
    bool verbose_;
    int periodMs_;
    float epsilon_;
    std::vector<float> last_;
    std::vector<int64_t> lastSent_;
    std::thread thread_;
    std::mutex m_;
    std::condition_variable cv_;
    bool quit_ = false, idleRequested_ = false;
    std::atomic<bool> busy_{false}, loggedIdle_{false}, loggedThread_{false}, loggedFirstSend_{false};
    std::atomic<int64_t> lastIdle_{0};
    std::atomic<uint64_t> seen_{0};
};

} // namespace cat::mpcx
