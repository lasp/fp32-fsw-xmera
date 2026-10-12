#include "averageRwSpeedDataTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

#include <limits>

namespace {
auto sampleDomain() {
    return fuzztest::StructOf<RwSpeedSample>(fuzztest::Arbitrary<std::uint64_t>(),
                                             fuzztest::ArrayOf<kMaxNumRw>(fuzztest::InRange(-1e4F, 1e4F)));
}

auto windowDomain() {
    return fuzztest::InRange(std::numeric_limits<float>::denorm_min(),
                             AverageRwSpeedDataAlgorithm::kMaxAveragingWindowSec);
}
}  // namespace

FUZZ_TEST(averageRwSpeedDataFuzz, regressionTestAverageRwSpeedData).WithDomains(windowDomain(), sampleDomain());

FUZZ_TEST(averageRwSpeedDataFuzz, sequencedRegressionTestAverageRwSpeedData)
    .WithDomains(windowDomain(), fuzztest::VectorOf(sampleDomain()).WithMaxSize(40));
