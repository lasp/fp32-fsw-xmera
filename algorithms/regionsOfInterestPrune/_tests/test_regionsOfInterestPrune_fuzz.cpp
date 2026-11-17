#include "regionsOfInterestPruneTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Fuzz tests
// ---------------------------------------------------------------------------
//
// Frames are built from fuzzed raw arrays (see frameFromRaw): counts in [0, 2] so zeros split the
// arrays into many spans, arbitrary uint32 intensities, tile sizes 1..16 and small lit counts so the
// tile filter both keeps and drops boxes. Arrays are capped at 64 entries to keep iterations fast.
// The reference point covers the frame and a margin beyond it; alpha covers the cost range [0, 2] and
// negative values (cost bypassed).

namespace {
void fuzzOutputIsWellFormed(std::vector<uint16_t> rowSums,
                            std::vector<uint16_t> colSums,
                            std::vector<uint32_t> rowIntensities,
                            std::vector<uint32_t> colIntensities,
                            uint32_t regionSize,
                            std::vector<uint32_t> litCounts,
                            uint32_t x0,
                            uint32_t y0,
                            float alpha) {
    const Frame f = frameFromRaw(std::move(rowSums),
                                 std::move(colSums),
                                 std::move(rowIntensities),
                                 std::move(colIntensities),
                                 regionSize,
                                 std::move(litCounts));
    propertyRegionsOfInterestPruneOutputIsWellFormed(f, x0, y0, alpha);
}

void fuzzMatchesReference(std::vector<uint16_t> rowSums,
                          std::vector<uint16_t> colSums,
                          std::vector<uint32_t> rowIntensities,
                          std::vector<uint32_t> colIntensities,
                          uint32_t regionSize,
                          std::vector<uint32_t> litCounts,
                          uint32_t x0,
                          uint32_t y0,
                          float alpha) {
    const Frame f = frameFromRaw(std::move(rowSums),
                                 std::move(colSums),
                                 std::move(rowIntensities),
                                 std::move(colIntensities),
                                 regionSize,
                                 std::move(litCounts));
    (void)testRegionsOfInterestPrune(f, x0, y0, alpha);
}

auto countsDomain() { return fuzztest::VectorOf(fuzztest::InRange<uint16_t>(0U, 2U)).WithMaxSize(64U); }
auto intensitiesDomain() { return fuzztest::VectorOf(fuzztest::Arbitrary<uint32_t>()).WithMaxSize(64U); }
auto litCountsDomain() { return fuzztest::VectorOf(fuzztest::InRange<uint32_t>(0U, 3U)).WithMaxSize(4096U); }
}  // namespace

FUZZ_TEST(RegionsOfInterestPruneFuzz, fuzzOutputIsWellFormed)
    .WithDomains(countsDomain(),
                 countsDomain(),
                 intensitiesDomain(),
                 intensitiesDomain(),
                 fuzztest::InRange<uint32_t>(1U, 16U),
                 litCountsDomain(),
                 fuzztest::InRange<uint32_t>(0U, 80U),
                 fuzztest::InRange<uint32_t>(0U, 80U),
                 fuzztest::InRange<float>(-2.0F, 2.0F));

FUZZ_TEST(RegionsOfInterestPruneFuzz, fuzzMatchesReference)
    .WithDomains(countsDomain(),
                 countsDomain(),
                 intensitiesDomain(),
                 intensitiesDomain(),
                 fuzztest::InRange<uint32_t>(1U, 16U),
                 litCountsDomain(),
                 fuzztest::InRange<uint32_t>(0U, 80U),
                 fuzztest::InRange<uint32_t>(0U, 80U),
                 fuzztest::InRange<float>(-2.0F, 2.0F));
