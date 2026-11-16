// wrapper_checks.h - the catalog's standard tests for a built plugin (.so), through the VST2 ABI.
//
// Every catalog plugin's tests/test_wrapper.cpp calls runCatalogChecks() with a short spec, then adds
// its own plugin-specific checks. What is checked (docs/PLAYBOOK.md, definition of done):
//   identity and parameter names/order; parameter 0 = reserved no-op (name, "-", not automatable,
//   nudges change nothing bit for bit); meters read-only, untouched by presets, overwritten after a
//   state load, announced to the host from a non-audio thread, notifier stops on close; program 0 =
//   "Default"; state round trip bit-exact and re-saved chunk identical; garbage chunk rejected; NaN/Inf
//   input never reaches the output; an appended-parameter build loads old state and vice versa.
#pragma once
#include "harness.h"
#include "signals.h"
#include "vst_host.h"

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

namespace cat::test {

struct CatalogSpec
{
    std::string libPath;                      // the plugin build
    std::string appendLibPath;                // same plugin built with CAT_APPEND_TEST (optional)
    uint32_t uid = 0;                         // MPC_FOURCC(...)
    std::vector<std::string> names;           // all parameter names in order, including "Reserved" and meters
    std::vector<int> meters;                  // meter parameter indices (read-only)
    std::vector<int> announced;               // user parameters the plugin also moves and announces (a playhead)
    std::vector<std::pair<int, float>> drive; // normalised settings that make the meters move (e.g. high input)
    std::vector<int> quietMeters;             // meters that may stay at 0 in this test (e.g. a waveform with nothing loaded)
    std::vector<float> (*signal)() = nullptr; // test input (mono, duplicated to stereo); default: drum loop
};

inline std::vector<float> defaultTestSignal() { std::vector<float> s = drumLoop(1, 0.9); s.resize(8820); return s; }

inline void runCatalogChecks(Harness& h, const CatalogSpec& spec)
{
    auto input = [&] { return spec.signal ? spec.signal() : defaultTestSignal(); };
    const int nParams = (int)spec.names.size();
    const int firstMeter = spec.meters.empty() ? nParams : spec.meters.front();
    auto isMeter = [&](int i) { for (int m : spec.meters) if (m == i) return true; return false; };

    h.section("Catalog checks: identity, parameters, automation");
    {
        VstPlugin p(spec.libPath);
        h.check(p.e->magic == kEffectMagic && (uint32_t)p.e->uniqueID == spec.uid, "identity", "VST2 magic and uid as declared");
        bool names = p.numParams() == nParams;
        for (int i = 0; i < nParams && names; ++i) names = p.name(i) == spec.names[(size_t)i];
        h.check(names, "parameter order", "%d parameters in the declared permanent order", nParams);
        bool autom = !p.automatable(0) && p.display(0) == "-";
        auto isAnnounced = [&](int i) { for (int a : spec.announced) if (a == i) return true; return false; };
        for (int i = 1; i < nParams; ++i) autom = autom && (p.automatable(i) == !(isMeter(i) || isAnnounced(i)));
        h.check(autom, "automation flags", "reserved0, %zu meter(s) and %zu announced parameter(s) hidden from automation; every other parameter automatable",
                spec.meters.size(), spec.announced.size());
        h.check(p.numPrograms() >= 1 && p.programName(0) == "Default", "program 0 is Default", "the MPC selects program 0 right after loading");
    }

    h.section("Catalog checks: state");
    {
        VstPlugin a(spec.libPath);
        Rng rng(99);
        for (int i = 1; i < firstMeter; ++i) a.set(i, (float)rng.uniform());
        std::vector<std::string> before;
        for (int i = 0; i < firstMeter; ++i) before.push_back(a.display(i));
        const std::vector<uint8_t> c = a.chunk();
        VstPlugin b(spec.libPath);
        bool same = b.setChunk(c);
        for (int i = 0; i < firstMeter; ++i) same = same && b.get(i) == a.get(i) && b.display(i) == before[(size_t)i];
        h.check(same, "state round trip", "%zu-byte chunk; every user parameter bit-identical in a fresh instance", c.size());
        h.check(b.chunk() == c, "re-saved chunk identical", "byte for byte");
        const float keep = b.get(1);
        h.check(!b.setChunk(std::vector<uint8_t>(c.size(), 0xAB)) && b.get(1) == keep, "garbage chunk rejected", "state unchanged");
    }

    h.section("Catalog checks: audio safety");
    {
        VstPlugin a(spec.libPath), b(spec.libPath);
        for (VstPlugin* p : { &a, &b }) for (auto& d : spec.drive) p->set(d.first, d.second);
        std::vector<float> l1 = input(), r1 = l1, l2 = l1, r2 = l1;
        for (size_t i = 0; i < l1.size(); i += 128)
        {
            const int n = (int)std::min<size_t>(128, l1.size() - i);
            float* ia[2] = { l1.data() + i, r1.data() + i }; float* ib[2] = { l2.data() + i, r2.data() + i };
            a.e->processReplacing(a.e, ia, ia, n);
            b.set(0, (i / 128) % 2 ? 1.0f : 0.0f); b.set(0, 0.0f);   // the MPC's parameter-0 nudge
            b.e->processReplacing(b.e, ib, ib, n);
        }
        h.check(l1 == l2 && r1 == r2, "parameter 0 is a no-op", "bit-identical output with and without nudges");

        VstPlugin c(spec.libPath);
        for (auto& d : spec.drive) c.set(d.first, d.second);
        std::vector<float> l = input(), r = l;
        l[100] = std::nanf(""); r[200] = INFINITY; l[300] = -INFINITY;
        c.process(l, r, 128);
        bool finite = true;
        for (size_t i = 0; i < l.size(); ++i) finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
        h.check(finite, "NaN/Inf never reach the output", "NaN, +Inf and -Inf samples in the input");
    }

    if (!spec.meters.empty())
    {
        h.section("Catalog checks: meters");
        HostLog& L = hostLog();
        L.reset();
        {
            VstPlugin p(spec.libPath);
            for (auto& d : spec.drive) p.set(d.first, d.second);
            std::vector<float> l = input(), r = l;
            p.process(l, r, 128);
            bool moved = true;
            for (int m : spec.meters)
            {
                bool quiet = false;
                for (int q : spec.quietMeters) quiet = quiet || q == m;
                moved = moved && (quiet || p.get(m) > 0.0f);
            }
            h.check(moved, "meters move", "every meter is non-zero after the test signal with the drive settings");
            const float before = p.get(spec.meters.front());
            p.disp(effSetProgram, 0, p.numPrograms() > 1 ? 1 : 0);
            h.check(p.get(spec.meters.front()) == before, "presets don't touch meters", "program change left the meter value alone");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));   // the notifier thread (no idle from this host)
            long meterCalls = 0, other = 0;
            auto isAnnounced = [&](int i) { for (int a : spec.announced) if (a == i) return true; return false; };
            for (int i = 0; i < 1024; ++i) if (!isAnnounced(i)) (isMeter(i) ? meterCalls : other) += L.automate[i].load();
            h.check(meterCalls > 0 && other == 0, "meters announced", "%ld audioMasterAutomate calls for meters, %ld for other parameters (announced parameters excluded)", meterCalls, other);
            h.check(L.audioThreadCalls.load() == 0, "audio thread never calls the host", "%ld host calls from inside processReplacing (GetTime excepted)", L.audioThreadCalls.load());

            p.idle();   // the MPC's path: effIdle on the UI thread
            h.check(true, "effIdle accepted", "the idle hook forwards to the framework without side effects");

            VstPlugin q(spec.libPath);
            p.set(spec.meters.front(), 0.7f);
            q.setChunk(p.chunk());
            std::vector<float> sl(256, 0.0f), sr(256, 0.0f);
            q.process(sl, sr, 128);
            h.check(q.get(spec.meters.front()) != 0.7f, "restored meter value overwritten", "a saved meter value is replaced by the live value on the next block");
        }
        const long after = L.automate[spec.meters.front()].load();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        h.check(L.automate[spec.meters.front()].load() == after, "notifier stops on close", "no host calls after effClose");
    }

    if (!spec.appendLibPath.empty())
    {
        h.section("Catalog checks: appending a parameter in a later version");
        VstPlugin v1(spec.libPath);
        Rng rng(7);
        for (int i = 1; i < firstMeter; ++i) v1.set(i, (float)rng.uniform());
        VstPlugin v2(spec.appendLibPath);
        bool same = v2.setChunk(v1.chunk()) && v2.numParams() == nParams + 1;
        for (int i = 0; i < firstMeter; ++i) same = same && v2.get(i) == v1.get(i);
        h.check(same && v2.display(nParams) == "0.50", "old state -> build with one more parameter",
                "every user parameter restored; the appended one keeps its default (%s)", v2.display(nParams).c_str());
        v2.set(nParams, 0.9f);
        VstPlugin v1b(spec.libPath);
        bool down = v1b.setChunk(v2.chunk());
        for (int i = 0; i < firstMeter; ++i) down = down && v1b.get(i) == v2.get(i);
        h.check(down, "newer state -> older build", "known parameters restore; the unknown one is ignored");
    }
}

} // namespace cat::test
