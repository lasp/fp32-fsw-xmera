#ifndef TEST_REGIONSOFINTERESTPRUNE_H
#define TEST_REGIONSOFINTERESTPRUNE_H

#include "regionsOfInterestPruneAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

// ---------------------------------------------------------------------------
// Test frames
// ---------------------------------------------------------------------------
//
// A Frame holds everything update() consumes for one image: per-row/col above-threshold counts and
// intensities, and the bin grid of per-tile lit counts. makeFrame() builds all of them consistently
// from a list of rectangular objects, so tests describe scenes ("two objects on a diagonal") rather
// than hand-written arrays. Tests that need inconsistent inputs (e.g. a broken grid) edit the fields.

//! A rectangular above-threshold object: every pixel in it is lit with blurred value `value`.
struct TestObject {
    uint32_t row{};
    uint32_t col{};
    uint32_t height{};
    uint32_t width{};
    uint32_t value{};
};

struct Frame {
    uint32_t numRows{};
    uint32_t numCols{};
    std::vector<uint16_t> rowSums;
    std::vector<uint16_t> colSums;
    std::vector<uint32_t> rowIntensities;
    std::vector<uint32_t> colIntensities;
    uint32_t regionSize{};
    uint32_t numRegionRows{};
    uint32_t numRegionCols{};
    std::vector<uint32_t> litCounts;  //!< row-major, numRegionRows * numRegionCols entries

    RoiBinGrid binGrid() const {
        return RoiBinGrid{.regionSize = regionSize,
                          .numRegionRows = numRegionRows,
                          .numRegionCols = numRegionCols,
                          .litCounts = std::span<const uint32_t>(litCounts.data(), litCounts.size())};
    }
};

//! Build a frame of numRows x numCols pixels with the given objects (later objects overwrite earlier ones
//! where they overlap) and a bin grid of regionSize-pixel tiles.
inline Frame makeFrame(uint32_t numRows,
                       uint32_t numCols,
                       uint32_t regionSize,
                       const std::vector<TestObject>& objects) {
    std::vector<uint32_t> pixels(static_cast<size_t>(numRows) * numCols, 0U);
    for (const auto& o : objects) {
        for (uint32_t r = o.row; r < o.row + o.height && r < numRows; ++r) {
            for (uint32_t c = o.col; c < o.col + o.width && c < numCols; ++c) {
                pixels[(static_cast<size_t>(r) * numCols) + c] = o.value;
            }
        }
    }

    Frame f;
    f.numRows = numRows;
    f.numCols = numCols;
    f.rowSums.assign(numRows, 0U);
    f.colSums.assign(numCols, 0U);
    f.rowIntensities.assign(numRows, 0U);
    f.colIntensities.assign(numCols, 0U);
    f.regionSize = regionSize;
    f.numRegionRows = (numRows + regionSize - 1U) / regionSize;
    f.numRegionCols = (numCols + regionSize - 1U) / regionSize;
    f.litCounts.assign(static_cast<size_t>(f.numRegionRows) * f.numRegionCols, 0U);
    for (uint32_t r = 0; r < numRows; ++r) {
        for (uint32_t c = 0; c < numCols; ++c) {
            const uint32_t v = pixels[(static_cast<size_t>(r) * numCols) + c];
            if (v == 0U) {
                continue;
            }
            ++f.rowSums[r];
            ++f.colSums[c];
            f.rowIntensities[r] += v;
            f.colIntensities[c] += v;
            ++f.litCounts[(static_cast<size_t>(r / regionSize) * f.numRegionCols) + (c / regionSize)];
        }
    }
    return f;
}

inline RoiCandidates runAlgorithm(const Frame& f, const Eigen::Vector2i& centralPoint, float alpha) {
    const RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(centralPoint, alpha)};
    return alg.update(Eigen::Vector2<uint32_t>(f.numCols, f.numRows),
                      f.rowSums.data(),
                      f.rowIntensities.data(),
                      f.colSums.data(),
                      f.colIntensities.data(),
                      f.binGrid());
}

// ---------------------------------------------------------------------------
// Reference implementation
// ---------------------------------------------------------------------------
//
// Independently reimplements the six pipeline steps with std::vector and stable sorts, sharing no
// code with the algorithm under test:
//   1. contiguous non-zero count spans, each summing its counts (pixels) and intensities; keep the
//      ROW_SPANS_MAX / COL_SPANS_MAX whose midpoints are closest to the reference row / col, centralPoint.y() /
//      centralPoint.x() (stable: an earlier span wins a distance tie),
//   2. row x col cross-product, intensity = min(row-span intensity, col-span intensity),
//   2b. with 2 row and 2 col spans, score the diagonal pairing {(r0,c0), (r1,c1)} and the anti-diagonal
//      pairing {(r0,c1), (r1,c0)} by the summed relative mismatch of their spans' pixels and intensities;
//      drop the worse pairing's two boxes when the better score < GHOST_PAIRING_MARGIN x the worse,
//      otherwise keep all four,
//   3. drop boxes whose covered tiles sum to 0 lit pixels (litPixelCount = that sum, saturated);
//      an unusable grid yields nothing,
//   4. scaledIntensity = intensity / area^(alpha - 1); left at 0 when alpha < 0 (cost function bypassed),
//   5. stable sort by box-center distance to centralPoint ascending, then scaledIntensity descending (bypass:
//      distance only, so equally close boxes keep the order they were formed in: row span, then col span),
//   6. keep the first ROI_CANDIDATES_OUT_MAX.
// The distance, mismatch and scaledIntensity expressions use the same float operations as the
// algorithm so comparisons can be exact. The reference returns the full ranked list before step 6.

struct RefSpan {
    uint32_t start{};
    uint32_t length{};
    uint64_t pixels{};
    uint64_t intensity{};
    float distance{};
};

inline std::vector<RefSpan> referenceClosestSpans(const std::vector<uint16_t>& counts,
                                                  const std::vector<uint32_t>& intensities,
                                                  float reference,
                                                  uint32_t keep) {
    std::vector<RefSpan> spans;
    uint32_t i = 0;
    while (i < counts.size()) {
        if (counts[i] == 0) {
            ++i;
            continue;
        }
        RefSpan s{.start = i};
        while (i < counts.size() && counts[i] != 0) {
            s.pixels += counts[i];
            s.intensity += intensities[i];
            ++i;
        }
        s.length = i - s.start;
        s.distance = fabsf((static_cast<float>(s.start) + (static_cast<float>(s.length) / 2.0F)) - reference);
        spans.push_back(s);
    }
    std::stable_sort(
        spans.begin(), spans.end(), [](const RefSpan& a, const RefSpan& b) { return a.distance < b.distance; });
    if (spans.size() > keep) {
        spans.resize(keep);
    }
    return spans;
}

inline float referenceRelativeMismatch(uint64_t a, uint64_t b) {
    const uint64_t larger = std::max(a, b);
    if (larger == 0U) {
        return 0.0F;
    }
    return static_cast<float>(a > b ? a - b : b - a) / static_cast<float>(larger);
}

inline float referencePairMismatch(const RefSpan& row, const RefSpan& col) {
    return referenceRelativeMismatch(row.pixels, col.pixels) + referenceRelativeMismatch(row.intensity, col.intensity);
}

inline float referenceSquaredDistance(const RoiCandidateEntry& e, float referenceRow, float referenceCol) {
    const float dRow = (static_cast<float>(e.row) + (static_cast<float>(e.height) / 2.0F)) - referenceRow;
    const float dCol = (static_cast<float>(e.col) + (static_cast<float>(e.width) / 2.0F)) - referenceCol;
    return (dRow * dRow) + (dCol * dCol);
}

inline bool referenceGridUsable(const Frame& f) {
    if (f.regionSize == 0U) {
        return false;
    }
    const uint64_t rows = (static_cast<uint64_t>(f.numRows) + f.regionSize - 1U) / f.regionSize;
    const uint64_t cols = (static_cast<uint64_t>(f.numCols) + f.regionSize - 1U) / f.regionSize;
    return f.numRegionRows == rows && f.numRegionCols == cols &&
           static_cast<uint64_t>(f.numRegionRows) * f.numRegionCols == f.litCounts.size();
}

//! Sum of litCounts over the tiles a box covers, saturated at UINT32_MAX (grid must be usable).
inline uint32_t referenceLitTileSum(const Frame& f, const RoiCandidateEntry& e) {
    uint64_t sum = 0;
    for (uint32_t tr = e.row / f.regionSize; tr <= (e.row + e.height - 1U) / f.regionSize; ++tr) {
        for (uint32_t tc = e.col / f.regionSize; tc <= (e.col + e.width - 1U) / f.regionSize; ++tc) {
            sum += f.litCounts[(static_cast<size_t>(tr) * f.numRegionCols) + tc];
        }
    }
    return static_cast<uint32_t>(std::min<uint64_t>(sum, std::numeric_limits<uint32_t>::max()));
}

//! Steps 1-5: every surviving candidate, ranked (step 6's cut is applied by the caller).
inline std::vector<RoiCandidateEntry> referenceRegionsOfInterestPrune(const Frame& f,
                                                                      const Eigen::Vector2i& centralPoint,
                                                                      float alpha) {
    const auto referenceRow = static_cast<float>(centralPoint.y());
    const auto referenceCol = static_cast<float>(centralPoint.x());
    const auto rowSpans = referenceClosestSpans(f.rowSums, f.rowIntensities, referenceRow, ROW_SPANS_MAX);
    const auto colSpans = referenceClosestSpans(f.colSums, f.colIntensities, referenceCol, COL_SPANS_MAX);
    if (!referenceGridUsable(f)) {
        return {};
    }

    const bool bypassCost = alpha < 0.0F;

    // Step 2b: which (row span, col span) pairs are ghosts.
    bool dropDiagonal = false;
    bool dropAntiDiagonal = false;
    if (rowSpans.size() == 2U && colSpans.size() == 2U) {
        const float diagonal =
            referencePairMismatch(rowSpans[0], colSpans[0]) + referencePairMismatch(rowSpans[1], colSpans[1]);
        const float antiDiagonal =
            referencePairMismatch(rowSpans[0], colSpans[1]) + referencePairMismatch(rowSpans[1], colSpans[0]);
        if (diagonal < GHOST_PAIRING_MARGIN * antiDiagonal) {
            dropAntiDiagonal = true;
        } else if (antiDiagonal < GHOST_PAIRING_MARGIN * diagonal) {
            dropDiagonal = true;
        }
    }

    std::vector<RoiCandidateEntry> out;
    for (size_t a = 0; a < rowSpans.size(); ++a) {
        for (size_t b = 0; b < colSpans.size(); ++b) {
            const bool onDiagonal = (a == b);
            if ((onDiagonal && dropDiagonal) || (!onDiagonal && dropAntiDiagonal)) {
                continue;
            }
            const RefSpan& r = rowSpans[a];
            const RefSpan& c = colSpans[b];
            RoiCandidateEntry e{.row = r.start, .col = c.start, .height = r.length, .width = c.length};
            e.litPixelCount = referenceLitTileSum(f, e);
            if (e.litPixelCount == 0U) {
                continue;
            }
            e.intensity = std::min(r.intensity, c.intensity);
            if (!bypassCost) {
                const auto area = static_cast<float>(e.width) * static_cast<float>(e.height);
                e.scaledIntensity = static_cast<float>(e.intensity) / powf(area, alpha - 1.0F);
            }
            out.push_back(e);
        }
    }
    std::stable_sort(out.begin(), out.end(), [&](const RoiCandidateEntry& a, const RoiCandidateEntry& b) {
        const float distanceA = referenceSquaredDistance(a, referenceRow, referenceCol);
        const float distanceB = referenceSquaredDistance(b, referenceRow, referenceCol);
        if (distanceA != distanceB) {
            return distanceA < distanceB;
        }
        return !bypassCost && a.scaledIntensity > b.scaledIntensity;
    });
    return out;
}

// ---------------------------------------------------------------------------
// Regression test helper
// ---------------------------------------------------------------------------

inline bool sameCandidate(const RoiCandidateEntry& a, const RoiCandidateEntry& b) {
    return a.row == b.row && a.col == b.col && a.height == b.height && a.width == b.width &&
           a.litPixelCount == b.litPixelCount && a.intensity == b.intensity && a.scaledIntensity == b.scaledIntensity;
}

//! The output must be exactly the reference's first ROI_CANDIDATES_OUT_MAX candidates, in the same order (both
//! rank with a stable sort, so equally ranked boxes keep the order they were formed in), and every slot after
//! numCandidates must be zero.
inline void expectSameCandidates(const RoiCandidates& out, const std::vector<RoiCandidateEntry>& rankedReference) {
    const size_t expectedCount = std::min<size_t>(rankedReference.size(), ROI_CANDIDATES_OUT_MAX);
    ASSERT_EQ(out.numCandidates, expectedCount);
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        const RoiCandidateEntry& got = out.candidates[i];
        const RoiCandidateEntry& want = rankedReference[i];
        EXPECT_TRUE(sameCandidate(got, want)) << "candidate " << i << ": got row " << got.row << " col " << got.col
                                              << " " << got.width << "x" << got.height << ", reference row " << want.row
                                              << " col " << want.col << " " << want.width << "x" << want.height;
    }
    for (uint32_t i = out.numCandidates; i < ROI_CANDIDATES_MAX; ++i) {
        EXPECT_TRUE(sameCandidate(out.candidates[i], RoiCandidateEntry{})) << "slot " << i << " should be zero";
    }
}

//! Run the algorithm on a frame and require it to match the independent reference exactly.
inline RoiCandidates testRegionsOfInterestPrune(const Frame& f, const Eigen::Vector2i& centralPoint, float alpha) {
    RoiCandidates out{};
    EXPECT_NO_THROW(out = runAlgorithm(f, centralPoint, alpha));
    expectSameCandidates(out, referenceRegionsOfInterestPrune(f, centralPoint, alpha));
    return out;
}

// ---------------------------------------------------------------------------
// Config validation helper
// ---------------------------------------------------------------------------

inline void testRegionsOfInterestPruneSetup() {
    constexpr auto kMaxCol = static_cast<int32_t>(MAX_IMAGE_COLS);
    constexpr auto kMaxRow = static_cast<int32_t>(MAX_IMAGE_ROWS);
    const Eigen::Vector2i kDefaultCentralPoint{DEFAULT_CENTRAL_POINT_X, DEFAULT_CENTRAL_POINT_Y};
    EXPECT_NO_THROW((void)RegionsOfInterestPruneConfig::create(Eigen::Vector2i(0, 0), 1.0F));
    EXPECT_NO_THROW((void)RegionsOfInterestPruneConfig::create(Eigen::Vector2i(kMaxCol, kMaxRow), 1.0F));
    // alpha: [0, 2] uses the cost function; any negative value, -infinity included, bypasses it.
    for (const float alpha : {0.0F, 2.0F, -0.001F, -1.0F, -1000.0F, -std::numeric_limits<float>::infinity()}) {
        EXPECT_NO_THROW((void)RegionsOfInterestPruneConfig::create(kDefaultCentralPoint, alpha)) << "alpha " << alpha;
    }

    // Each coordinate is rejected above its image limit and, since Eigen::Vector2i is signed, below 0.
    for (const Eigen::Vector2i& bad : {Eigen::Vector2i(kMaxCol + 1, DEFAULT_CENTRAL_POINT_Y),
                                       Eigen::Vector2i(DEFAULT_CENTRAL_POINT_X, kMaxRow + 1),
                                       Eigen::Vector2i(-1, DEFAULT_CENTRAL_POINT_Y),
                                       Eigen::Vector2i(DEFAULT_CENTRAL_POINT_X, -1)}) {
        EXPECT_THROW((void)RegionsOfInterestPruneConfig::create(bad, 1.0F), fsw::invalid_argument)
            << "centralPoint " << bad.transpose();
    }
    for (const float alpha :
         {2.01F, 5.0F, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
        EXPECT_THROW((void)RegionsOfInterestPruneConfig::create(kDefaultCentralPoint, alpha), fsw::invalid_argument)
            << "alpha " << alpha;
    }

    const RegionsOfInterestPruneConfig cfg = RegionsOfInterestPruneConfig::create(Eigen::Vector2i(12, 34), 0.5F);
    EXPECT_EQ(cfg.getCentralPoint().x(), 12);
    EXPECT_EQ(cfg.getCentralPoint().y(), 34);
    EXPECT_EQ(cfg.getAlphaAreaPruneScaling(), 0.5F);
}

// ---------------------------------------------------------------------------
// Fuzz-safe property helpers
// ---------------------------------------------------------------------------
//
// Invariants that hold for ANY frame, checked against the frame itself rather than the reference.

inline uint64_t rangeSum(const std::vector<uint32_t>& v, uint32_t start, uint32_t length) {
    uint64_t sum = 0;
    for (uint32_t i = start; i < start + length; ++i) {
        sum += v[i];
    }
    return sum;
}

//! Output is well-formed: at most ROI_CANDIDATES_OUT_MAX boxes, each non-empty and inside the image, each
//! touching a lit tile with litPixelCount equal to its covered-tile sum, intensity equal to
//! min(row-span, col-span) intensity, a finite non-negative scaledIntensity (0 when alpha < 0 bypasses the
//! cost), sorted by distance ascending then scaledIntensity descending (bypass: distance only), and every
//! slot after numCandidates zero.
inline void propertyRegionsOfInterestPruneOutputIsWellFormed(const Frame& f,
                                                             const Eigen::Vector2i& centralPoint,
                                                             float alpha) {
    RoiCandidates out{};
    EXPECT_NO_THROW(out = runAlgorithm(f, centralPoint, alpha));
    ASSERT_LE(out.numCandidates, ROI_CANDIDATES_OUT_MAX);
    if (out.numCandidates > 0U) {
        ASSERT_TRUE(referenceGridUsable(f)) << "candidates returned for an unusable grid";
    }

    const auto refRow = static_cast<float>(centralPoint.y());
    const auto refCol = static_cast<float>(centralPoint.x());
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        const auto& c = out.candidates[i];
        ASSERT_GT(c.height, 0U) << "candidate " << i;
        ASSERT_GT(c.width, 0U) << "candidate " << i;
        ASSERT_LE(c.row + c.height, f.numRows) << "candidate " << i;
        ASSERT_LE(c.col + c.width, f.numCols) << "candidate " << i;
        EXPECT_GT(c.litPixelCount, 0U) << "candidate " << i;
        EXPECT_EQ(c.litPixelCount, referenceLitTileSum(f, c)) << "candidate " << i;
        EXPECT_EQ(c.intensity,
                  std::min(rangeSum(f.rowIntensities, c.row, c.height), rangeSum(f.colIntensities, c.col, c.width)))
            << "candidate " << i;
        EXPECT_TRUE(std::isfinite(c.scaledIntensity)) << "candidate " << i;
        EXPECT_GE(c.scaledIntensity, 0.0F) << "candidate " << i;
        if (alpha < 0.0F) {
            EXPECT_EQ(c.scaledIntensity, 0.0F) << "candidate " << i << " (cost bypassed)";
        }
        if (i > 0) {
            const auto& p = out.candidates[i - 1];
            const float distanceP = referenceSquaredDistance(p, refRow, refCol);
            const float distanceC = referenceSquaredDistance(c, refRow, refCol);
            EXPECT_LE(distanceP, distanceC) << "candidate " << i;
            if (distanceP == distanceC && alpha >= 0.0F) {
                EXPECT_GE(p.scaledIntensity, c.scaledIntensity) << "candidate " << i;
            }
        }
    }
    for (uint32_t i = out.numCandidates; i < ROI_CANDIDATES_MAX; ++i) {
        EXPECT_TRUE(sameCandidate(out.candidates[i], RoiCandidateEntry{})) << "slot " << i << " should be zero";
    }
}

//! Build a frame from fuzzed raw arrays: counts decide the spans; intensities and lit counts are
//! resized to match, so any combination of sizes is a valid (if arbitrary) frame.
inline Frame frameFromRaw(std::vector<uint16_t> rowSums,
                          std::vector<uint16_t> colSums,
                          std::vector<uint32_t> rowIntensities,
                          std::vector<uint32_t> colIntensities,
                          uint32_t regionSize,
                          std::vector<uint32_t> litCounts) {
    Frame f;
    f.numRows = static_cast<uint32_t>(rowSums.size());
    f.numCols = static_cast<uint32_t>(colSums.size());
    rowIntensities.resize(rowSums.size(), 0U);
    colIntensities.resize(colSums.size(), 0U);
    f.rowSums = std::move(rowSums);
    f.colSums = std::move(colSums);
    f.rowIntensities = std::move(rowIntensities);
    f.colIntensities = std::move(colIntensities);
    f.regionSize = regionSize;
    f.numRegionRows = (f.numRows + regionSize - 1U) / regionSize;
    f.numRegionCols = (f.numCols + regionSize - 1U) / regionSize;
    litCounts.resize(static_cast<size_t>(f.numRegionRows) * f.numRegionCols, 0U);
    f.litCounts = std::move(litCounts);
    return f;
}

#endif  // TEST_REGIONSOFINTERESTPRUNE_H
