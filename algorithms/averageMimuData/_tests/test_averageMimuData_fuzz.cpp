#include "../architecture/testUtilities/eigenFuzzDomains.hpp"
#include "averageMimuDataTestHelpers.hpp"
#include "averageMimuDataTypes.h"
#include <fuzztest/fuzztest.h>

namespace {
auto sampleDomain() {
    return fuzztest::StructOf<Sample>(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                                      xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));
}

auto inputPacketDomain() {
    return fuzztest::StructOf<InputPacket>(
        fuzztest::Arbitrary<bool>(),
        fuzztest::Arbitrary<uint64_t>(),
        fuzztest::ArrayOf<MAX_MIMU_SAMPLES_PER_PKT_C>(sampleDomain()));
}

auto inputPktsDataDomain() {
    return fuzztest::StructOf<InputPktsData>(
        fuzztest::ArrayOf<MAX_MIMU_PKT_C>(inputPacketDomain()));
}
}  // namespace

FUZZ_TEST(averageMimuDataFuzz, regressionTestAverageMimuData)
    .WithDomains(fuzztest::InRange(0.0F, AverageMimuDataAlgorithm::kMaxAveragingWindowSec),
                 inputPktsDataDomain());

FUZZ_TEST(averageMimuDataFuzz, sequencedRegressionTestAverageMimuData)
    .WithDomains(fuzztest::InRange(0.0F, AverageMimuDataAlgorithm::kMaxAveragingWindowSec),
                 fuzztest::VectorOf(inputPktsDataDomain()).WithSize(8));
