// harness.h - tiny test harness: named checks, a markdown report, non-zero exit on failure.
#pragma once
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

namespace cat::test {

class Harness
{
public:
    explicit Harness(std::string title) { md_ += "# " + title + "\n\n"; }

    void section(const std::string& name)
    {
        std::printf("\n== %s ==\n", name.c_str());
        md_ += "\n## " + name + "\n\n";
    }
    // A pass/fail check. detail is shown either way.
    bool check(bool ok, const char* name, const char* fmt = "", ...)
    {
        char buf[1024]; va_list ap; va_start(ap, fmt); std::vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
        std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name, buf[0] ? ": " : "", buf);
        md_ += std::string("- **") + (ok ? "PASS" : "FAIL") + "** " + name + (buf[0] ? std::string(": ") + buf : "") + "\n";
        (ok ? passed_ : failed_)++;
        if (!ok) failures_.push_back(name);
        return ok;
    }
    // Free text / tables for the report (also echoed to stdout).
    void text(const char* fmt, ...)
    {
        char buf[8192]; va_list ap; va_start(ap, fmt); std::vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
        std::printf("%s", buf);
        md_ += buf;
    }
    int finish(const std::string& reportPath)
    {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "\n---\n**%d passed, %d failed**\n", passed_, failed_);
        md_ += buf;
        std::printf("%s", buf + 5);
        for (const auto& f : failures_) std::printf("  failed: %s\n", f.c_str());
        if (FILE* f = std::fopen(reportPath.c_str(), "w")) { std::fputs(md_.c_str(), f); std::fclose(f); }
        return failed_ == 0 ? 0 : 1;
    }

private:
    std::string md_;
    int passed_ = 0, failed_ = 0;
    std::vector<std::string> failures_;
};

} // namespace cat::test
