/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef FP32_XMERA_FSW_ALGORITHMS_CUSTOMEXCEPTIONS_H
#define FP32_XMERA_FSW_ALGORITHMS_CUSTOMEXCEPTIONS_H

// Give a clear compile-time error if <exception> isn't present (when supported).
#if defined(__has_include)
#if !__has_include(<exception>)
#error "This target requires <exception>; none found."
#endif
#endif

#include <cstddef>
#include <exception>

// FSW_EXC_MAX_MSG must be a macro: it sizes a fixed-length char array, so it
// must be a compile-time integer, AND it must be overridable from -D flags so
// targets can pick a per-build buffer size without editing this header.
#ifndef FSW_EXC_MAX_MSG  // NOLINT(cppcoreguidelines-macro-usage)
#define FSW_EXC_MAX_MSG 256
#endif

namespace fsw {

class exception : public std::exception {
   public:
    const char* what() const noexcept override { return this->message[0] ? this->message : "fsw::exception"; }

   protected:
    explicit exception(const char* msg) noexcept { safe_copy(this->message, FSW_EXC_MAX_MSG, msg); }
    char message[FSW_EXC_MAX_MSG];

   private:
    static void safe_copy(char* dst, std::size_t cap, const char* src) noexcept {
        if (!dst || cap == 0) return;
        if (!src) {
            dst[0] = '\0';
            return;
        }
        std::size_t i = 0;
        while (i + 1 < cap && src[i] != '\0') {
            dst[i] = src[i];
            ++i;
        }
        dst[i] = '\0';
    }
};

class invalid_argument : public exception {
   public:
    explicit invalid_argument(const char* msg) noexcept : exception(msg) {}
    const char* what() const noexcept override { return this->message[0] ? this->message : "invalid argument"; }
};

}  // namespace fsw

// FSW_REQUIRE_EXCEPTIONS must be a macro: it drives an #error directive,
// which is preprocessor-only and cannot be expressed as a constexpr value.
#ifndef FSW_REQUIRE_EXCEPTIONS  // NOLINT(cppcoreguidelines-macro-usage)
#define FSW_REQUIRE_EXCEPTIONS 0
#endif

#if FSW_REQUIRE_EXCEPTIONS && !defined(__cpp_exceptions)
#error "Exceptions are required for this target (define __cpp_exceptions)."
#endif

// FSW_THROW_INVALID_ARGUMENT must be a macro: it switches between `throw`
// (which is rejected at parse time under -fno-exceptions, even in a dead
// branch) and a trap intrinsic. A constexpr template function cannot do this.
#if defined(__cpp_exceptions)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define FSW_THROW_INVALID_ARGUMENT(msg_cstr) throw fsw::invalid_argument((msg_cstr))
#else
#if defined(__GNUC__) || defined(__clang__)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define FSW_THROW_INVALID_ARGUMENT(msg_cstr) \
    do {                                     \
        (void)(msg_cstr);                    \
        __builtin_trap();                    \
    } while (0)
#elif defined(_MSC_VER)
#include <intrin.h>
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define FSW_THROW_INVALID_ARGUMENT(msg_cstr) \
    do {                                     \
        (void)(msg_cstr);                    \
        __debugbreak();                      \
    } while (0)
#else
#error "FSW_THROW_INVALID_ARGUMENT: unsupported compiler; add a trap intrinsic for this toolchain."
#endif
#endif

#endif  // FP32_XMERA_FSW_ALGORITHMS_CUSTOMEXCEPTIONS_H
