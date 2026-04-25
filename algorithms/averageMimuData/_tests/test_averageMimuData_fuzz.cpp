#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "averageMimuDataTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

namespace {
auto sampleDomain() {
    return fuzztest::StructOf<Sample>(fuzztest::Arbitrary<uint64_t>(),
                                      xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                                      xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));
}

auto inputPktsDataDomain() {
    return fuzztest::StructOf<InputPktsData>(
        fuzztest::ArrayOf<MAX_MIMU_PKT>(fuzztest::Arbitrary<bool>()),
        fuzztest::ArrayOf<MAX_MIMU_PKT>(fuzztest::ArrayOf<MAX_MIMU_SAMPLES_PER_PKT>(sampleDomain())));
}
}  // namespace

FUZZ_TEST(averageMimuDataFuzz, regressionTestAverageMimuData)
    .WithDomains(fuzztest::InRange(0.0F, 1e6F), inputPktsDataDomain());
