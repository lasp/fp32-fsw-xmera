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

#ifndef FS_EXC_MAX_MSG
#define FS_EXC_MAX_MSG 256  // override if you need longer messages
#endif

namespace fsw {

class invalid_argument : public std::exception {
   public:
    explicit invalid_argument(const char* msg) noexcept { safe_copy(this->message, FS_EXC_MAX_MSG, msg); }

    const char* what() const noexcept override { return this->message[0] ? this->message : "invalid argument"; }

   private:
    char message[FS_EXC_MAX_MSG];

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

}  // namespace fsw

// Require exceptions if you want (set to 1), otherwise we'll still compile.
#ifndef FS_REQUIRE_EXCEPTIONS
#define FS_REQUIRE_EXCEPTIONS 0
#endif

#if FS_REQUIRE_EXCEPTIONS && !defined(__cpp_exceptions)
#error "Exceptions are required for this target (define __cpp_exceptions)."
#endif

#if defined(__cpp_exceptions)
#define FS_THROW_INVALID_ARGUMENT(msg_cstr) throw fsw::invalid_argument((msg_cstr))
#else
   // No <cstdio>/<cstdlib>. Use a trap so this stays freestanding.
#if defined(__GNUC__) || defined(__clang__)
#define FS_THROW_INVALID_ARGUMENT(msg_cstr) \
    do {                                    \
        (void)(msg_cstr);                   \
        __builtin_trap();                   \
    } while (0)
#elif defined(_MSC_VER)
#include <intrin.h>
#define FS_THROW_INVALID_ARGUMENT(msg_cstr) \
    do {                                    \
        (void)(msg_cstr);                   \
        __debugbreak();                     \
    } while (0)
#else
#define FS_THROW_INVALID_ARGUMENT(msg_cstr) \
    do {                                    \
        (void)(msg_cstr);                   \
        for (;;) {                          \
        }                                   \
    } while (0)
#endif
#endif

#endif  // FP32_XMERA_FSW_ALGORITHMS_CUSTOMEXCEPTIONS_H
