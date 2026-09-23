#include "regionsOfInterestPruneTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------
//
// Row/col sum arrays are capped at 100 entries to keep each fuzz iteration fast while still
// exercising a wide range of span counts/shapes; maxRowSpans/maxColSpans are capped at 20 since the
// pre-filter's behavior at very large values is already covered by the dedicated
// HugeMaxSpansKeepsAllAvailableSpans regression test. testRegionsOfInterestPrune (exact-match against
// the reference) is deliberately not fuzzed here — arbitrary arrays are highly likely to produce ties,
// and the tie-break order at the pre-filter/final-sort boundary is implementation-defined (see the
// tie-break note in regionsOfInterestPruneTestHelpers.hpp), so only the tie-independent invariants
// below are fuzzed.

FUZZ_TEST(RegionsOfInterestPrunePropertyFuzz, propertyRegionsOfInterestPruneOutputIsWellFormed)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange<uint16_t>(0U, 65535U)).WithMaxSize(100U),
                 fuzztest::VectorOf(fuzztest::InRange<uint16_t>(0U, 65535U)).WithMaxSize(100U),
                 fuzztest::InRange<uint32_t>(1U, 20U),
                 fuzztest::InRange<uint32_t>(1U, 20U));

FUZZ_TEST(RegionsOfInterestPrunePropertyFuzz, propertyRegionsOfInterestPruneCountBoundedByPixelSums)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange<uint16_t>(0U, 65535U)).WithMaxSize(100U),
                 fuzztest::VectorOf(fuzztest::InRange<uint16_t>(0U, 65535U)).WithMaxSize(100U),
                 fuzztest::InRange<uint32_t>(1U, 20U),
                 fuzztest::InRange<uint32_t>(1U, 20U));
