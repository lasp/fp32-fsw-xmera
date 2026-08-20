#ifndef MSG_DEFINITIONS_H
#define MSG_DEFINITIONS_H

#include <mission/parameters.h>

// The plain C boundary types size their arrays from the mission/parameters.h macros, because a C
// array bound has to be an integer constant expression and a const variable is not one in C. Every
// C++ translation unit should reach for the typed constants below instead.
#ifdef __cplusplus

#include <cstdint>

inline constexpr std::uint32_t kMaxNumCssSensors = MAX_NUM_CSS_SENSORS;
inline constexpr std::uint32_t kMaxThrusterCount = MAX_EFF_CNT;
inline constexpr std::uint32_t kMaxNumRw = RW_EFF_CNT;  //!< [-] maximum number of reaction wheels
inline constexpr std::uint32_t kMimuCount = 3U;

// FP32 algorithm specific constants
inline constexpr std::uint32_t kMaxMeasurementNumber = 5U;
inline constexpr std::uint32_t kMaxMeasurementVector = 5U;

#endif  // __cplusplus

#endif  // MSG_DEFINITIONS_H
