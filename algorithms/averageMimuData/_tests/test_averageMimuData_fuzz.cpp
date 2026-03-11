#include "averageMimuDataTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(averageMimuDataFuzz, regressionTestAverageMimuData)
    .WithDomains(fuzztest::InRange<std::size_t>(0U, MAX_ACC_BUF_PKT),
                 fuzztest::InRange(0.0F, 1.0F),
                 fuzztest::ArrayOf<MAX_ACC_BUF_PKT>(
                     fuzztest::StructOf<InputData>(fuzztest::InRange<uint64_t>(0U, 10U * SEC2NANO),
                                                   fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6F, 1e6F)),
                                                   fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6F, 1e6F)))));
