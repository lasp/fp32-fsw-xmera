#include "averageMimuDataTestHelpers.hpp"
#include <fuzztest/fuzztest.h>
#include <array>
#include <vector>

FUZZ_TEST(averageMimuDataFuzz, regressionTestaverageMimuData)
    .WithDomains(fuzztest::InRange(0.0f, 1.0f),                         // timeDelta
                 fuzztest::InRange(0.0f, 1.0f),                         // factor0
                 fuzztest::InRange(0.0f, 1.0f),                         // factor1
                 fuzztest::InRange(0.0f, 1.0f),                         // factor2
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6f, 1e6f)),  // gyro0
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6f, 1e6f)),  // accel0
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6f, 1e6f)),  // gyro1
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6f, 1e6f)),  // accel1
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6f, 1e6f)),  // gyro2
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6f, 1e6f)),  // accel2
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6f, 1e6f)),  // gyro3
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6f, 1e6f))   // accel3
    );
