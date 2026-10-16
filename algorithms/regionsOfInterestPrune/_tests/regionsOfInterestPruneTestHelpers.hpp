#ifndef TEST_REGIONSOFINTERESTPRUNE_H
#define TEST_REGIONSOFINTERESTPRUNE_H

#include "regionsOfInterestPruneAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <vector>

// ---------------------------------------------------------------------------
// Reference implementation
// ---------------------------------------------------------------------------
//
// Independently reimplements the span / pre-filter / cross-product / sort pipeline using
// std::vector, sharing no code with the algorithm under test.
//
// Tie-break note: the algorithm's pre-filter (topIndices) uses std::ranges::partial_sort, which is
// not stable, so which spans win a tie in accumulator sum at the maxRowSpans/maxColSpans cutoff is
// implementation-defined. Likewise the final sort (packOutput) is a plain std::ranges::sort, so a
// full tie on (count, distance-to-center, area) is also implementation-defined. Callers of
// referenceRegionsOfInterestPrune intended for exact-match regression comparisons must choose inputs
// with no ties at either boundary.

struct RefSpan {
    uint32_t start{};
    uint32_t length{};
    uint32_t sum{};
};

// Locate maximal contiguous non-zero runs in `s` and accumulate the sum of each run.
inline std::vector<RefSpan> referenceFindSpans(const std::vector<uint16_t>& s) {
    std::vector<RefSpan> spans;
    uint32_t i = 0;
    while (i < s.size()) {
        if (s[i] != 0) {
            uint32_t j = i;
            uint32_t sum = 0;
            while (j < s.size() && s[j] != 0) {
                sum += s[j];
                ++j;
            }
            spans.push_back({i, j - i, sum});
            i = j;
        } else {
            ++i;
        }
    }
    return spans;
}

// Keep the `keep` spans with the highest sum (stable on ties).
inline std::vector<RefSpan> referenceTopSpans(std::vector<RefSpan> spans, uint32_t keep) {
    std::stable_sort(spans.begin(), spans.end(), [](const RefSpan& a, const RefSpan& b) { return a.sum > b.sum; });
    if (spans.size() > keep) {
        spans.resize(keep);
    }
    return spans;
}

// Squared Euclidean distance from a candidate box's center to a reference center point. Mirrors
// RegionsOfInterestPruneAlgorithm's internal squaredDistanceToCenter (kept in float here too, so
// tie-break decisions match the algorithm bit-for-bit rather than diverging on double vs. float
// rounding).
inline float referenceSquaredDistanceToCenter(const RoiCandidateEntry& e, float centerRow, float centerCol) {
    const float rowCenter = static_cast<float>(e.row) + (static_cast<float>(e.height) / 2.0F);
    const float colCenter = static_cast<float>(e.col) + (static_cast<float>(e.width) / 2.0F);
    const float dRow = rowCenter - centerRow;
    const float dCol = colCenter - centerCol;
    return (dRow * dRow) + (dCol * dCol);
}

// Independently reimplements RegionsOfInterestPruneAlgorithm::update(): span detection, top-K
// pre-filter, cross-product with count = min(R[k], C[l]), then sort by count desc / distance-to-image-
// center asc / area asc and truncate to ROI_CANDIDATES_MAX.
inline RoiCandidates referenceRegionsOfInterestPrune(const std::vector<uint16_t>& rowSums,
                                                     const std::vector<uint16_t>& colSums,
                                                     uint32_t maxRowSpans,
                                                     uint32_t maxColSpans) {
    const auto rowSpans = referenceTopSpans(referenceFindSpans(rowSums), maxRowSpans);
    const auto colSpans = referenceTopSpans(referenceFindSpans(colSums), maxColSpans);

    std::vector<RoiCandidateEntry> candidates;
    for (const auto& r : rowSpans) {
        for (const auto& c : colSpans) {
            candidates.push_back({r.start, c.start, r.length, c.length, std::min(r.sum, c.sum)});
        }
    }

    const float centerRow = static_cast<float>(rowSums.size()) / 2.0F;
    const float centerCol = static_cast<float>(colSums.size()) / 2.0F;
    std::stable_sort(candidates.begin(),
                     candidates.end(),
                     [centerRow, centerCol](const RoiCandidateEntry& a, const RoiCandidateEntry& b) {
                         if (a.count != b.count) {
                             return a.count > b.count;
                         }
                         const float distA = referenceSquaredDistanceToCenter(a, centerRow, centerCol);
                         const float distB = referenceSquaredDistanceToCenter(b, centerRow, centerCol);
                         if (distA != distB) {
                             return distA < distB;
                         }
                         return a.height * a.width < b.height * b.width;
                     });

    RoiCandidates out{};
    out.numCandidates = std::min(static_cast<uint32_t>(candidates.size()), ROI_CANDIDATES_MAX);
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        out.candidates[i] = candidates[i];
    }
    return out;
}

// ---------------------------------------------------------------------------
// Regression test helper
// ---------------------------------------------------------------------------

// Drive the algorithm on (rowSums, colSums) and compare its output to the independent reference.
// Inputs should have no ties at the maxRowSpans/maxColSpans cutoff (see tie-break note above) so the
// comparison is exact.
inline void testRegionsOfInterestPrune(const std::vector<uint16_t>& rowSums,
                                       const std::vector<uint16_t>& colSums,
                                       uint32_t maxRowSpans,
                                       uint32_t maxColSpans) {
    const RegionsOfInterestPruneConfig cfg = RegionsOfInterestPruneConfig::create(maxRowSpans, maxColSpans);
    const RegionsOfInterestPruneAlgorithm alg{cfg};

    RoiCandidates out{};
    EXPECT_NO_THROW(out = alg.update(rowSums.data(),
                                     static_cast<uint32_t>(rowSums.size()),
                                     colSums.data(),
                                     static_cast<uint32_t>(colSums.size())));

    const RoiCandidates ref = referenceRegionsOfInterestPrune(rowSums, colSums, maxRowSpans, maxColSpans);

    EXPECT_EQ(out.numCandidates, ref.numCandidates);
    const uint32_t n = std::min(out.numCandidates, ref.numCandidates);
    for (uint32_t i = 0; i < n; ++i) {
        EXPECT_EQ(out.candidates[i].row, ref.candidates[i].row) << "candidate " << i;
        EXPECT_EQ(out.candidates[i].col, ref.candidates[i].col) << "candidate " << i;
        EXPECT_EQ(out.candidates[i].height, ref.candidates[i].height) << "candidate " << i;
        EXPECT_EQ(out.candidates[i].width, ref.candidates[i].width) << "candidate " << i;
        EXPECT_EQ(out.candidates[i].count, ref.candidates[i].count) << "candidate " << i;
    }
}

// ---------------------------------------------------------------------------
// Config validation helper
// ---------------------------------------------------------------------------

inline void testRegionsOfInterestPruneSetup() {
    EXPECT_NO_THROW((void)RegionsOfInterestPruneConfig::create(1U, 1U));
    EXPECT_NO_THROW((void)RegionsOfInterestPruneConfig::create(DEFAULT_MAX_ROW_SPANS, DEFAULT_MAX_COL_SPANS));

    EXPECT_THROW((void)RegionsOfInterestPruneConfig::create(0U, 1U), fsw::invalid_argument);
    EXPECT_THROW((void)RegionsOfInterestPruneConfig::create(1U, 0U), fsw::invalid_argument);
    EXPECT_THROW((void)RegionsOfInterestPruneConfig::create(0U, 0U), fsw::invalid_argument);

    const RegionsOfInterestPruneConfig cfg = RegionsOfInterestPruneConfig::create(5U, 7U);
    EXPECT_EQ(cfg.getMaxRowSpans(), 5U);
    EXPECT_EQ(cfg.getMaxColSpans(), 7U);
}

// ---------------------------------------------------------------------------
// Property/edge-case helpers
// ---------------------------------------------------------------------------

struct GridInputs {
    std::vector<uint16_t> rowSums;
    std::vector<uint16_t> colSums;
};

// 5 row spans (sums 100,90,80,70,60) x 5 col spans (sums 50,40,30,20,10): every col sum is below
// every row sum, so count = min(row,col) = col always, and the 25-candidate cross-product exceeds
// ROI_CANDIDATES_MAX (16) — used by the property tests below, which only check bound/order/count
// invariants that hold regardless of which candidates win a count tie.
inline GridInputs fiveRowFiveColSpans() {
    return {
        {100, 0, 90, 0, 80, 0, 70, 0, 60},
        {50, 0, 40, 0, 30, 0, 20, 0, 10},
    };
}

// Sum of v[start .. start+length).
inline uint32_t rangeSum(const std::vector<uint16_t>& v, uint32_t start, uint32_t length) {
    uint32_t sum = 0;
    for (uint32_t i = start; i < start + length; ++i) {
        sum += v[i];
    }
    return sum;
}

// ---------------------------------------------------------------------------
// Fuzz-safe property helpers
// ---------------------------------------------------------------------------
//
// Unlike testRegionsOfInterestPrune (exact-match against the reference), these only check invariants
// that hold for ANY input regardless of which candidates win a tie at the pre-filter or final-sort
// boundary (see the tie-break note above) — safe to fuzz with arbitrary row/col sum arrays, which are
// highly likely to contain ties.

// Output is well-formed: never throws, never exceeds ROI_CANDIDATES_MAX, sorted by count descending,
// and every candidate's bounding box lies within the input arrays.
inline void propertyRegionsOfInterestPruneOutputIsWellFormed(const std::vector<uint16_t>& rowSums,
                                                             const std::vector<uint16_t>& colSums,
                                                             uint32_t maxRowSpans,
                                                             uint32_t maxColSpans) {
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(maxRowSpans, maxColSpans)};

    RoiCandidates out{};
    EXPECT_NO_THROW(out = alg.update(rowSums.data(),
                                     static_cast<uint32_t>(rowSums.size()),
                                     colSums.data(),
                                     static_cast<uint32_t>(colSums.size())));

    EXPECT_LE(out.numCandidates, ROI_CANDIDATES_MAX);

    const auto numRows = static_cast<uint32_t>(rowSums.size());
    const auto numCols = static_cast<uint32_t>(colSums.size());
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        const auto& c = out.candidates[i];
        EXPECT_LE(c.row + c.height, numRows) << "candidate " << i;
        EXPECT_LE(c.col + c.width, numCols) << "candidate " << i;
        if (i > 0) {
            EXPECT_GE(out.candidates[i - 1].count, c.count) << "candidate " << i;
        }
    }
}

// A candidate's published count never exceeds the actual raw pixel sum over its own row/col
// footprint — it is a genuine upper bound, not an overstatement. The bounds checks are ASSERT (fatal)
// because rangeSum() would read out of bounds if a candidate's box ever extended past the input.
inline void propertyRegionsOfInterestPruneCountBoundedByPixelSums(const std::vector<uint16_t>& rowSums,
                                                                  const std::vector<uint16_t>& colSums,
                                                                  uint32_t maxRowSpans,
                                                                  uint32_t maxColSpans) {
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(maxRowSpans, maxColSpans)};

    RoiCandidates out{};
    EXPECT_NO_THROW(out = alg.update(rowSums.data(),
                                     static_cast<uint32_t>(rowSums.size()),
                                     colSums.data(),
                                     static_cast<uint32_t>(colSums.size())));

    const auto numRows = static_cast<uint32_t>(rowSums.size());
    const auto numCols = static_cast<uint32_t>(colSums.size());
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        const auto& c = out.candidates[i];
        ASSERT_LE(c.row + c.height, numRows) << "candidate " << i;
        ASSERT_LE(c.col + c.width, numCols) << "candidate " << i;
        EXPECT_LE(c.count, rangeSum(rowSums, c.row, c.height)) << "candidate " << i;
        EXPECT_LE(c.count, rangeSum(colSums, c.col, c.width)) << "candidate " << i;
    }
}

#endif  // TEST_REGIONSOFINTERESTPRUNE_H
