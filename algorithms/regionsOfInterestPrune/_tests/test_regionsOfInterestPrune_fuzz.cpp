#include "regionsOfInterestPruneTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------
//
// Row/col sum arrays are capped at 100 entries to keep each fuzz iteration fast while still
// exercising a wide range of span counts/shapes; maxRowSpans/maxColSpans span their full valid range
// [1, ROW_SPANS_MAX] / [1, COL_SPANS_MAX]. testRegionsOfInterestPrune (exact-match against
// the reference) is deliberately not fuzzed here — arbitrary arrays are highly likely to produce ties,
// and the tie-break order at the final-sort boundary is implementation-defined (see the
// tie-break note in regionsOfInterestPruneTestHelpers.hpp), so only the tie-independent invariants
// below are fuzzed.

FUZZ_TEST(RegionsOfInterestPrunePropertyFuzz, propertyRegionsOfInterestPruneOutputIsWellFormed)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange<uint16_t>(0U, 65535U)).WithMaxSize(100U),
                 fuzztest::VectorOf(fuzztest::InRange<uint16_t>(0U, 65535U)).WithMaxSize(100U),
                 fuzztest::InRange<uint32_t>(1U, ROW_SPANS_MAX),
                 fuzztest::InRange<uint32_t>(1U, COL_SPANS_MAX));

FUZZ_TEST(RegionsOfInterestPrunePropertyFuzz, propertyRegionsOfInterestPruneCountBoundedByPixelSums)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange<uint16_t>(0U, 65535U)).WithMaxSize(100U),
                 fuzztest::VectorOf(fuzztest::InRange<uint16_t>(0U, 65535U)).WithMaxSize(100U),
                 fuzztest::InRange<uint32_t>(1U, ROW_SPANS_MAX),
                 fuzztest::InRange<uint32_t>(1U, COL_SPANS_MAX));
