// ============================================================================
//  logger.h — small header-only logging library for the BVH physics demo
// ============================================================================
//
//  Features
//    * TRACE / DEBUG / INFO / WARN / ERROR levels (runtime-filterable)
//    * Console sink with per-level ANSI colors (VT mode auto-enabled on Windows)
//    * Optional file sink: thread-safe, flushed on WARN+ and at shutdown
//    * Millisecond timestamps + "file:line" provenance
//    * printf-style formatting, zero external dependencies (STL only, C++20)
//    * Compile-time cut-off:  #define BVHLOG_COMPILE_LEVEL 2
//        -> TRACE(0) / DEBUG(1) calls then compile out of the binary entirely
//    * RAII scope timer:  BVHLOG_TIMER("label");  logs elapsed time on exit
//
//  Usage
//      #include "logger.h"
//
//      int main() {
//          bvhlog::init("bvh_run.log", bvhlog::Level::Info);   // console + file
//          LOG_INFO("objects=%d pairs=%d", n, p);
//          {
//              BVHLOG_TIMER("heavy step");     // logs elapsed time on scope exit
//              ...
//          }
//          bvhlog::shutdown();                 // summary line, closes file
//      }
// ============================================================================

#pragma once

#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <string>

// ---- compile-time level cut-off (0 = keep everything) ----------------------
#ifndef BVHLOG_COMPILE_LEVEL
#define BVHLOG_COMPILE_LEVEL 0
#endif

// ---- printf format-string checking where the compiler supports it ----------
#if defined(__MINGW32__) || defined(__MINGW64__)
#define BVHLOG_PRINTF(fmt_idx, first_arg) __attribute__((format(gnu_printf, fmt_idx, first_arg)))
#elif defined(__GNUC__) || defined(__clang__)
#define BVHLOG_PRINTF(fmt_idx, first_arg) __attribute__((format(printf, fmt_idx, first_arg)))
#else
#define BVHLOG_PRINTF(fmt_idx, first_arg)
#endif

// ---- Windows console (VT/ANSI) support, without min/max macro pollution ----
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#else
#  include <unistd.h>
#endif

namespace bvhlog {

enum class Level : int { Trace = 0, Debug = 1, Info = 2, Warn = 3, Error = 4, Off = 5 };

struct Config {
    Level min_level  = Level::Info; // runtime filter (applies to console + file)
    bool  console    = true;        // write to stdout/stderr
    bool  colors     = true;        // ANSI colors (auto-disabled if redirected)
    bool  timestamps = true;        // "YYYY-MM-DD HH:MM:SS.mmm " prefix
};

namespace detail {

// ---------------------------------------------------------------- internals
inline const char* levelTag(Level l) noexcept {
    switch (l) {
        case Level::Trace: return "TRACE";
        case Level::Debug: return "DEBUG";
        case Level::Info:  return "INFO ";
        case Level::Warn:  return "WARN ";
        case Level::Error: return "ERROR";
        default:           return "OFF  ";
    }
}

inline const char* levelColor(Level l) noexcept {
    switch (l) {
        case Level::Trace: return "\x1b[90m"; // bright black
        case Level::Debug: return "\x1b[36m"; // cyan
        case Level::Info:  return "\x1b[32m"; // green
        case Level::Warn:  return "\x1b[33m"; // yellow
        case Level::Error: return "\x1b[31m"; // red
        default:           return "\x1b[0m";
    }
}

inline const char* baseName(const char* path) noexcept {
    if (!path || !*path) return "?";
    const char* base = path;
    for (const char* p = path; *p; ++p)
        if (*p == '/' || *p == '\\') base = p + 1;
    return base;
}

struct Sink {
    std::mutex    mtx;
    Config        cfg{};
    FILE*         file     = nullptr;
    std::uint64_t counts[6]{}; // indexed by Level (Trace..Off)
    int           vt_state = -1; // -1 unknown, 0 no ANSI, 1 ANSI enabled
};

inline Sink& sink() noexcept {
    static Sink s;
    return s;
}

// Detect console + enable VT processing once (no-op when output is piped).
inline bool prepareConsole(Sink& s) noexcept {
    if (s.vt_state >= 0) return s.vt_state == 1;
#ifdef _WIN32
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (h == INVALID_HANDLE_VALUE || !GetConsoleMode(h, &mode)) {
        s.vt_state = 0;
        return false;
    }
    s.vt_state = SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING) ? 1 : 0;
    return s.vt_state == 1;
#else
    s.vt_state = isatty(fileno(stdout)) ? 1 : 0;
    return s.vt_state == 1;
#endif
}

// "YYYY-MM-DD HH:MM:SS.mmm " (trailing space included)
inline void timestamp(char* buf, std::size_t n) noexcept {
    using namespace std::chrono;
    const auto      now = system_clock::now();
    const std::time_t t = system_clock::to_time_t(now);
    const int ms =
        static_cast<int>(duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000);
    std::tm tmv{};
#ifdef _WIN32
    localtime_s(&tmv, &t);
#else
    localtime_r(&t, &tmv);
#endif
    std::snprintf(buf, n, "%04d-%02d-%02d %02d:%02d:%02d.%03d ",
                  tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                  tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ms);
}

inline std::string vformat(const char* fmt, va_list ap) {
    if (!fmt) return {};
    va_list ap2;
    va_copy(ap2, ap);
    const int len = std::vsnprintf(nullptr, 0, fmt, ap2);
    va_end(ap2);
    if (len <= 0) return fmt;
    std::string out(static_cast<std::size_t>(len) + 1, '\0');
    std::vsnprintf(out.data(), out.size(), fmt, ap);
    out.resize(static_cast<std::size_t>(len));
    return out;
}

// Counts the record and applies the runtime filter (call BEFORE formatting).
// Caller must hold sink().mtx.
inline bool beginUnlocked(Level lvl) {
    Sink&     s  = sink();
    const int li = static_cast<int>(lvl);
    if (li >= 0 && li < 6) ++s.counts[li];
    return li >= static_cast<int>(s.cfg.min_level);
}

// Formats one already-prepared message onto the configured sinks.
// Caller must hold sink().mtx and must pass a record accepted by beginUnlocked.
inline void writeUnlocked(Level lvl, const char* file, int line, const std::string& msg) {
    Sink& s = sink();

    char ts[48] = "";
    if (s.cfg.timestamps) timestamp(ts, sizeof ts);

    char head[224];
    std::snprintf(head, sizeof head, "%s[%s] %s:%d | ", ts, levelTag(lvl),
                  baseName(file), line);

    if (s.cfg.console) {
        FILE* out = (lvl >= Level::Warn) ? stderr : stdout;
        if (s.cfg.colors && prepareConsole(s))
            std::fprintf(out, "%s%s%s\n", levelColor(lvl), head, msg.c_str());
        else
            std::fprintf(out, "%s%s\n", head, msg.c_str());
        if (lvl >= Level::Warn) std::fflush(out);
    }
    if (s.file) {
        std::fprintf(s.file, "%s%s\n", head, msg.c_str());
        if (lvl >= Level::Warn) std::fflush(s.file);
    }
}

// Caller must hold sink().mtx.
inline void logUnlocked(Level lvl, const char* file, int line, const char* fmt, va_list ap) {
    if (!beginUnlocked(lvl)) return; // filtered before formatting
    writeUnlocked(lvl, file, line, vformat(fmt, ap));
}

inline FILE* openFile(const char* path) {
#ifdef _MSC_VER
    FILE* f = nullptr;
    if (fopen_s(&f, path, "w") != 0) return nullptr;
    return f;
#else
    return std::fopen(path, "w");
#endif
}

} // namespace detail

// ------------------------------------------------------------------ logging
inline void logv(Level lvl, const char* file, int line, const char* fmt, va_list ap) {
    detail::Sink& s = detail::sink();
    std::lock_guard<std::mutex> lock(s.mtx);
    detail::logUnlocked(lvl, file, line, fmt, ap);
}

inline void logf(Level lvl, const char* file, int line, const char* fmt, ...)
    BVHLOG_PRINTF(4, 5);

inline void logf(Level lvl, const char* file, int line, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    logv(lvl, file, line, fmt, ap);
    va_end(ap);
}

// ------------------------------------------------------------------ control
// Opens the optional file sink and sets the runtime level. Returns false if
// the requested file could not be opened (logging continues on console).
inline bool init(const char* file_path = nullptr, Level min_level = Level::Info) {
    bool ok = true;
    {
        detail::Sink& s = detail::sink();
        std::lock_guard<std::mutex> lock(s.mtx);
        s.cfg.min_level = min_level;
        if (file_path && *file_path) {
            if (s.file) { std::fclose(s.file); s.file = nullptr; }
            s.file = detail::openFile(file_path);
            ok = (s.file != nullptr);
        }
    }
    if (!ok)
        logf(Level::Error, "logger.h", 0, "cannot open log file '%s'", file_path);
    return ok;
}

inline void setLevel(Level l) {
    detail::Sink& s = detail::sink();
    std::lock_guard<std::mutex> lock(s.mtx);
    s.cfg.min_level = l;
}

inline Level getLevel() {
    detail::Sink& s = detail::sink();
    std::lock_guard<std::mutex> lock(s.mtx);
    return s.cfg.min_level;
}

inline void setConsole(bool on) {
    detail::Sink& s = detail::sink();
    std::lock_guard<std::mutex> lock(s.mtx);
    s.cfg.console = on;
}

inline void flush() {
    detail::Sink& s = detail::sink();
    std::lock_guard<std::mutex> lock(s.mtx);
    std::fflush(stdout);
    std::fflush(stderr);
    if (s.file) std::fflush(s.file);
}

struct Counts {
    std::uint64_t trace, debug, info, warn, error;
};

inline Counts counts() {
    detail::Sink& s = detail::sink();
    std::lock_guard<std::mutex> lock(s.mtx);
    return {s.counts[0], s.counts[1], s.counts[2], s.counts[3], s.counts[4]};
}

// Summary line + file close. Console logging still works afterwards.
inline void shutdown() {
    detail::Sink& s = detail::sink();
    std::lock_guard<std::mutex> lock(s.mtx);
    if (detail::beginUnlocked(Level::Info)) {
        char buf[192];
        std::snprintf(buf, sizeof buf,
            "log summary: trace=%llu debug=%llu info=%llu warn=%llu error=%llu",
            static_cast<unsigned long long>(s.counts[0]),
            static_cast<unsigned long long>(s.counts[1]),
            static_cast<unsigned long long>(s.counts[2]),
            static_cast<unsigned long long>(s.counts[3]),
            static_cast<unsigned long long>(s.counts[4]));
        detail::writeUnlocked(Level::Info, "logger.h", 0, buf);
    }
    if (s.file) { std::fflush(s.file); std::fclose(s.file); s.file = nullptr; }
    std::fflush(stdout);
    std::fflush(stderr);
}

// -------------------------------------------------------------- scope timer
// Logs ">> label {" on entry and "<< label done (X.XXX ms)" on exit.
class ScopedTimer {
public:
    ScopedTimer(const char* label, Level lvl, const char* file, int line)
        : label_(label), lvl_(lvl), file_(file), line_(line),
          start_(std::chrono::steady_clock::now()) {
        logf(lvl_, file_, line_, ">> %s {", label_);
    }
    ~ScopedTimer() {
        const double us = std::chrono::duration<double, std::micro>(
                              std::chrono::steady_clock::now() - start_).count();
        logf(lvl_, file_, line_, "<< %s done (%.3f ms)", label_, us / 1000.0);
    }
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    const char*              label_;
    Level                    lvl_;
    const char*              file_;
    int                      line_;
    std::chrono::steady_clock::time_point start_;
};

} // namespace bvhlog

// ------------------------------------------------------------------ macros
#define BVHLOG_CONCAT_IMPL(a, b) a##b
#define BVHLOG_CONCAT(a, b) BVHLOG_CONCAT_IMPL(a, b)

#define BVHLOG_AT(level_enum, ...)                                        \
    do { ::bvhlog::logf((level_enum), __FILE__, __LINE__, __VA_ARGS__); } while (0)

#if BVHLOG_COMPILE_LEVEL <= 0
#define BVHLOG_TRACE(...) BVHLOG_AT(::bvhlog::Level::Trace, __VA_ARGS__)
#else
#define BVHLOG_TRACE(...) ((void)0)
#endif

#if BVHLOG_COMPILE_LEVEL <= 1
#define BVHLOG_DEBUG(...) BVHLOG_AT(::bvhlog::Level::Debug, __VA_ARGS__)
#else
#define BVHLOG_DEBUG(...) ((void)0)
#endif

#if BVHLOG_COMPILE_LEVEL <= 2
#define BVHLOG_INFO(...) BVHLOG_AT(::bvhlog::Level::Info, __VA_ARGS__)
#else
#define BVHLOG_INFO(...) ((void)0)
#endif

#if BVHLOG_COMPILE_LEVEL <= 3
#define BVHLOG_WARN(...) BVHLOG_AT(::bvhlog::Level::Warn, __VA_ARGS__)
#else
#define BVHLOG_WARN(...) ((void)0)
#endif

#if BVHLOG_COMPILE_LEVEL <= 4
#define BVHLOG_ERROR(...) BVHLOG_AT(::bvhlog::Level::Error, __VA_ARGS__)
#else
#define BVHLOG_ERROR(...) ((void)0)
#endif

// Pretty aliases (kept safe against names some system headers define).
#ifndef LOG_TRACE
#define LOG_TRACE(...) BVHLOG_TRACE(__VA_ARGS__)
#endif
#ifndef LOG_DEBUG
#define LOG_DEBUG(...) BVHLOG_DEBUG(__VA_ARGS__)
#endif
#ifndef LOG_INFO
#define LOG_INFO(...) BVHLOG_INFO(__VA_ARGS__)
#endif
#ifndef LOG_WARN
#define LOG_WARN(...) BVHLOG_WARN(__VA_ARGS__)
#endif
#ifndef LOG_ERROR
#define LOG_ERROR(...) BVHLOG_ERROR(__VA_ARGS__)
#endif

// Scope timer: BVHLOG_TIMER("build");   (or BVHLOG_TIMER_AT(Level::Debug, ...))
#define BVHLOG_TIMER(label) BVHLOG_TIMER_AT(::bvhlog::Level::Info, label)
#define BVHLOG_TIMER_AT(level_enum, label)                                  \
    ::bvhlog::ScopedTimer BVHLOG_CONCAT(_bvhlog_timer_, __LINE__)(          \
        (label), (level_enum), __FILE__, __LINE__)
