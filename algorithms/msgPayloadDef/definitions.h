#ifndef MSG_DEFINITIONS_H
#define MSG_DEFINITIONS_H

#include <cstdint>

#define MAX_NUM_CSS_SENSORS 32
inline constexpr std::uint32_t kMaxNumCssSensors = MAX_NUM_CSS_SENSORS;
#define MAX_EFF_CNT 36
inline constexpr std::uint32_t kMaxThrusterCount = MAX_EFF_CNT;
inline constexpr std::uint32_t kMimuCount = 3U;
#define RW_EFF_CNT 36

#endif  // MSG_DEFINITIONS_H
