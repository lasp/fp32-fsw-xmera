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
// Exercises the top-span selection on both axes together with the cross-product and the
// count-desc/distance-to-center-asc/area-asc sort tie-break.
TEST(RegionsOfInterestPruneTest, TopSpansBothAxesThenCrossProduct) {
    testRegionsOfInterestPrune(
        /* rowSums */ {0, 10, 10, 10, 10, 10, 0, 10, 10, 10, 0, 10},  // spans: (1,5,50) (7,3,30) (11,1,10)
        /* colSums */ {0, 0, 5, 5, 5, 5, 0, 0, 5, 0, 1},              // spans: (2,4,20) (8,1,5) (10,1,1)
        /* maxRowSpans */ 2,
        /* maxColSpans */ 2);
}

// Same spans as above but maxRowSpans=maxColSpans=1: only the single highest-sum row/col span is
// kept by the scan, so the sole candidate must be the true global rank-1 (argmax(R) x argmax(C)) —
// the invariant the Step 1 documentation claims.
TEST(RegionsOfInterestPruneTest, Rank1PreservedWithSingleTopSpan) {
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

// 5 row spans x 5 col spans pre-filtered to ROW_SPANS_MAX x COL_SPANS_MAX must produce exactly
// MAX_RAW_CANDIDATES candidates, never more than ROI_CANDIDATES_MAX.
TEST(RegionsOfInterestPruneTest, NumCandidatesNeverExceedsMax) {
    const auto grid = fiveRowFiveColSpans();
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(ROW_SPANS_MAX, COL_SPANS_MAX)};
    const RoiCandidates out = alg.update(grid.rowSums.data(),
                                         static_cast<uint32_t>(grid.rowSums.size()),
                                         grid.colSums.data(),
                                         static_cast<uint32_t>(grid.colSums.size()));
    EXPECT_EQ(out.numCandidates, MAX_RAW_CANDIDATES);
    EXPECT_LE(out.numCandidates, ROI_CANDIDATES_MAX);
}

// Published candidates are sorted by count in non-increasing order.
TEST(RegionsOfInterestPruneTest, CandidatesSortedByCountDescending) {
    const auto grid = fiveRowFiveColSpans();
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(ROW_SPANS_MAX, COL_SPANS_MAX)};
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
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(ROW_SPANS_MAX, COL_SPANS_MAX)};
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
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(ROW_SPANS_MAX, COL_SPANS_MAX)};
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

// 150 single-pixel row spans with strictly increasing sums 1..150: the top ROW_SPANS_MAX are the
// last four, all past index 128. The running top-K scan inspects every span, so spans late in the
// array are never dropped (no fixed cap on the number of spans scanned).
TEST(RegionsOfInterestPruneTest, LateHighSumSpansAreKept) {
    std::vector<uint16_t> rowSums(300, 0);
    for (uint32_t k = 0; k < 150; ++k) {
        rowSums[2 * k] = static_cast<uint16_t>(k + 1);
    }
    testRegionsOfInterestPrune(rowSums,
                               /* colSums */ {200},
                               /* maxRowSpans */ ROW_SPANS_MAX,
                               /* maxColSpans */ 1);
}

// Equal sums at the keep cutoff: the scan only replaces the smallest kept span on a strictly greater
// sum, so the earlier span wins the tie (matches the reference's stable sort).
TEST(RegionsOfInterestPruneTest, EarlierSpanWinsTieAtCutoff) {
    testRegionsOfInterestPrune(/* rowSums */ {9, 0, 5, 0, 5, 0, 5},  // spans sums 9,5,5,5 -> keep 9 + first 5
                               /* colSums */ {20},
                               /* maxRowSpans */ 2,
                               /* maxColSpans */ 1);
}

// maxRowSpans/maxColSpans at their upper bound but exceeding the actual span count must not prune
// anything and must not misbehave (exercises the keep = std::min(keep, n) boundary).
TEST(RegionsOfInterestPruneTest, MaxSpansAboveSpanCountKeepsAllAvailableSpans) {
    testRegionsOfInterestPrune(
        /* rowSums */ {10, 10, 10, 0, 6},  // spans: (0,3,30) (4,1,6)
        /* colSums */ {8, 8, 0, 3, 3, 3},  // spans: (0,2,16) (3,3,9)
        /* maxRowSpans */ ROW_SPANS_MAX,
        /* maxColSpans */ COL_SPANS_MAX);
}

// ---------------------------------------------------------------------------
// Config test — round-trip at the top of the valid range, rejection just above it
// ---------------------------------------------------------------------------

TEST(RegionsOfInterestPruneTest, ConfigEnforcesMaxSpanUpperBound) {
    RegionsOfInterestPruneConfig cfg = RegionsOfInterestPruneConfig::create(1U, 1U);
    EXPECT_NO_THROW(cfg = RegionsOfInterestPruneConfig::create(ROW_SPANS_MAX, COL_SPANS_MAX));
    EXPECT_EQ(cfg.getMaxRowSpans(), ROW_SPANS_MAX);
    EXPECT_EQ(cfg.getMaxColSpans(), COL_SPANS_MAX);

    EXPECT_THROW((void)RegionsOfInterestPruneConfig::create(ROW_SPANS_MAX + 1U, 1U), fsw::invalid_argument);
    EXPECT_THROW((void)RegionsOfInterestPruneConfig::create(1U, COL_SPANS_MAX + 1U), fsw::invalid_argument);
    EXPECT_THROW((void)RegionsOfInterestPruneConfig::create(UINT32_MAX, UINT32_MAX), fsw::invalid_argument);
}
