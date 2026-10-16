#include "regionsOfInterestPruneTestHelpers.hpp"

// ---------------------------------------------------------------------------
// Regression tests — algorithm vs. independent reference implementation
// ---------------------------------------------------------------------------

// One row span, one col span; no pre-filter pressure (maxRowSpans/maxColSpans exceed the span count).
TEST(RegionsOfInterestPruneTest, SingleSpanEachAxis) {
    testRegionsOfInterestPrune(/* rowSums */ {0, 0, 5, 7, 0},  // span start=2, len=2, sum=12
                               /* colSums */ {0, 3, 4, 0, 0},  // span start=1, len=2, sum=7
                               /* maxRowSpans */ 3,
                               /* maxColSpans */ 3);
}

// Three row spans (sums 50, 30, 10) pruned to top 2; three col spans (sums 20, 5, 1) pruned to top 2.
// Exercises the pre-filter on both axes together with the cross-product and the
// count-desc/distance-to-center-asc/area-asc sort tie-break.
TEST(RegionsOfInterestPruneTest, PreFilterBothAxesThenCrossProduct) {
    testRegionsOfInterestPrune(
        /* rowSums */ {0, 10, 10, 10, 10, 10, 0, 10, 10, 10, 0, 10},  // spans: (1,5,50) (7,3,30) (11,1,10)
        /* colSums */ {0, 0, 5, 5, 5, 5, 0, 0, 5, 0, 1},              // spans: (2,4,20) (8,1,5) (10,1,1)
        /* maxRowSpans */ 2,
        /* maxColSpans */ 2);
}

// Same spans as above but maxRowSpans=maxColSpans=1: only the single highest-sum row/col span
// survives the pre-filter, so the sole candidate must be the true global rank-1 (argmax(R) x
// argmax(C)) — the invariant the pre-filter's doc comment claims.
TEST(RegionsOfInterestPruneTest, Rank1PreservedUnderAggressivePreFilter) {
    testRegionsOfInterestPrune(/* rowSums */ {0, 10, 10, 10, 10, 10, 0, 10, 10, 10, 0, 10},
                               /* colSums */ {0, 0, 5, 5, 5, 5, 0, 0, 5, 0, 1},
                               /* maxRowSpans */ 1,
                               /* maxColSpans */ 1);
}

// No above-threshold pixels in rowSums: no row spans, so the cross-product is empty regardless of
// colSums.
TEST(RegionsOfInterestPruneTest, NoAboveThresholdPixelsYieldsNoCandidates) {
    testRegionsOfInterestPrune(/* rowSums */ {0, 0, 0, 0, 0},
                               /* colSums */ {0, 5, 5, 0},
                               /* maxRowSpans */ 3,
                               /* maxColSpans */ 3);
}

// ---------------------------------------------------------------------------
// Setup test (config validation)
// ---------------------------------------------------------------------------

TEST(RegionsOfInterestPruneTest, SetupTest) { testRegionsOfInterestPruneSetup(); }

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// 25-candidate cross-product (5 row spans x 5 col spans) must truncate to exactly
// ROI_CANDIDATES_MAX, never more.
TEST(RegionsOfInterestPruneTest, NumCandidatesNeverExceedsMax) {
    const auto grid = fiveRowFiveColSpans();
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(5U, 5U)};
    const RoiCandidates out = alg.update(grid.rowSums.data(),
                                         static_cast<uint32_t>(grid.rowSums.size()),
                                         grid.colSums.data(),
                                         static_cast<uint32_t>(grid.colSums.size()));
    EXPECT_EQ(out.numCandidates, ROI_CANDIDATES_MAX);
}

// Published candidates are sorted by count in non-increasing order.
TEST(RegionsOfInterestPruneTest, CandidatesSortedByCountDescending) {
    const auto grid = fiveRowFiveColSpans();
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(5U, 5U)};
    const RoiCandidates out = alg.update(grid.rowSums.data(),
                                         static_cast<uint32_t>(grid.rowSums.size()),
                                         grid.colSums.data(),
                                         static_cast<uint32_t>(grid.colSums.size()));
    for (uint32_t i = 1; i < out.numCandidates; ++i) {
        EXPECT_GE(out.candidates[i - 1].count, out.candidates[i].count);
    }
}

// Every candidate's bounding box lies fully within the input arrays.
TEST(RegionsOfInterestPruneTest, CandidatesStayWithinInputBounds) {
    const auto grid = fiveRowFiveColSpans();
    const auto numRows = static_cast<uint32_t>(grid.rowSums.size());
    const auto numCols = static_cast<uint32_t>(grid.colSums.size());
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(5U, 5U)};
    const RoiCandidates out = alg.update(grid.rowSums.data(), numRows, grid.colSums.data(), numCols);
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        const auto& c = out.candidates[i];
        EXPECT_LE(c.row + c.height, numRows) << "candidate " << i;
        EXPECT_LE(c.col + c.width, numCols) << "candidate " << i;
    }
}

// A candidate's published count is a genuine upper bound: it never exceeds the actual sum of raw
// input pixels over the candidate's own row/col footprint.
TEST(RegionsOfInterestPruneTest, CandidateCountNeverExceedsActualPixelSums) {
    const auto grid = fiveRowFiveColSpans();
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(5U, 5U)};
    const RoiCandidates out = alg.update(grid.rowSums.data(),
                                         static_cast<uint32_t>(grid.rowSums.size()),
                                         grid.colSums.data(),
                                         static_cast<uint32_t>(grid.colSums.size()));
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        const auto& c = out.candidates[i];
        EXPECT_LE(c.count, rangeSum(grid.rowSums, c.row, c.height)) << "candidate " << i;
        EXPECT_LE(c.count, rangeSum(grid.colSums, c.col, c.width)) << "candidate " << i;
    }
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Zero-length row/col sum arrays: no spans, no candidates, no crash.
TEST(RegionsOfInterestPruneTest, EmptyInputProducesNoCandidates) {
    const std::vector<uint16_t> rowSums;
    const std::vector<uint16_t> colSums;
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(3U, 3U)};
    RoiCandidates out{};
    EXPECT_NO_THROW(out = alg.update(rowSums.data(), 0U, colSums.data(), 0U));
    EXPECT_EQ(out.numCandidates, 0U);
}

// Degenerate geometry: the whole row/col array is one contiguous span (no gaps at all).
TEST(RegionsOfInterestPruneTest, EntireArrayAboveThreshold) {
    testRegionsOfInterestPrune(/* rowSums */ {5, 5, 5, 5},  // one span: start=0, len=4, sum=20
                               /* colSums */ {3, 3, 3},     // one span: start=0, len=3, sum=9
                               /* maxRowSpans */ 3,
                               /* maxColSpans */ 3);
}

// Degenerate geometry: 1x1 input arrays.
TEST(RegionsOfInterestPruneTest, SinglePixelArrays) {
    testRegionsOfInterestPrune(/* rowSums */ {7},
                               /* colSums */ {4},
                               /* maxRowSpans */ 1,
                               /* maxColSpans */ 1);
}

// 130 single-pixel row spans (separated by gaps) exceed MAX_SPANS (128): findSpans silently drops
// spans beyond the cap (see the header's MAX_SPANS comment). With maxRowSpans set far above 130, the
// pre-filter itself does no further pruning, so any surviving candidate must come from one of the
// first 128 spans (positions 0, 2, ..., 254) — never from the two dropped spans at 256/258. All 130
// row spans share the same sum, so which specific 16 of the resulting 128 candidates get published is
// tie-break-order-dependent, but the bound below (row <= 254) and the truncated count are not.
TEST(RegionsOfInterestPruneTest, SpansBeyondMaxSpansCapAreDropped) {
    std::vector<uint16_t> rowSums(259, 0);
    for (uint32_t k = 0; k < 130; ++k) {
        rowSums[2 * k] = 7;
    }
    const std::vector<uint16_t> colSums = {9};

    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(1000U, 1U)};
    RoiCandidates out{};
    EXPECT_NO_THROW(out = alg.update(rowSums.data(),
                                     static_cast<uint32_t>(rowSums.size()),
                                     colSums.data(),
                                     static_cast<uint32_t>(colSums.size())));

    EXPECT_EQ(out.numCandidates, ROI_CANDIDATES_MAX);
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        EXPECT_LE(out.candidates[i].row, 254U) << "candidate " << i << " references a dropped span";
    }
}

// maxRowSpans/maxColSpans far exceeding the actual span count must not prune anything and must not
// misbehave (exercises the keep = std::min(keep, n) boundary at a large keep value).
TEST(RegionsOfInterestPruneTest, HugeMaxSpansKeepsAllAvailableSpans) {
    testRegionsOfInterestPrune(
        /* rowSums */ {10, 10, 10, 0, 6},  // spans: (0,3,30) (4,1,6)
        /* colSums */ {8, 8, 0, 3, 3, 3},  // spans: (0,2,16) (3,3,9)
        /* maxRowSpans */ 1000000U,
        /* maxColSpans */ 1000000U);
}

// ---------------------------------------------------------------------------
// Config test — round-trip at the top of the valid range (no upper bound is enforced)
// ---------------------------------------------------------------------------

TEST(RegionsOfInterestPruneTest, ConfigAcceptsLargeMaxSpanValues) {
    RegionsOfInterestPruneConfig cfg = RegionsOfInterestPruneConfig::create(1U, 1U);
    EXPECT_NO_THROW(cfg = RegionsOfInterestPruneConfig::create(UINT32_MAX, UINT32_MAX));
    EXPECT_EQ(cfg.getMaxRowSpans(), UINT32_MAX);
    EXPECT_EQ(cfg.getMaxColSpans(), UINT32_MAX);
}
