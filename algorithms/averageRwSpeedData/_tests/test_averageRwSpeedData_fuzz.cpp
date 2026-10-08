#include "averageRwSpeedDataTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

namespace {
auto sampleDomain() {
    return fuzztest::StructOf<RwSpeedSample>(fuzztest::Arbitrary<std::uint64_t>(),
                                             fuzztest::ArrayOf<kMaxNumRw>(fuzztest::InRange(-1e4F, 1e4F)));
}

auto windowDomain() { return fuzztest::InRange(0.0F, AverageRwSpeedDataAlgorithm::kMaxAveragingWindowSec); }
}  // namespace

FUZZ_TEST(averageRwSpeedDataFuzz, regressionTestAverageRwSpeedData).WithDomains(windowDomain(), sampleDomain());

FUZZ_TEST(averageRwSpeedDataFuzz, sequencedRegressionTestAverageRwSpeedData)
    .WithDomains(windowDomain(), fuzztest::VectorOf(sampleDomain()).WithMaxSize(40));
