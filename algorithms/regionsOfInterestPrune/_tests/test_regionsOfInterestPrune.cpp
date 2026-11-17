#include "regionsOfInterestPruneTestHelpers.hpp"

#include <set>
#include <utility>

// Most tests use a 100 x 100 frame with 10 px tiles and the reference point at its center.
namespace {
constexpr uint32_t kRows = 100;
constexpr uint32_t kCols = 100;
constexpr uint32_t kTile = 10;
constexpr uint32_t kX0 = 50;
constexpr uint32_t kY0 = 50;
//! alphaAreaPruneScaling whose area exponent (alpha - 1) is 0: scaledIntensity is the plain intensity.
constexpr float kIntensityOnly = 1.0F;

//! Published box corners as (row, col) pairs.
std::set<std::pair<uint32_t, uint32_t>> corners(const RoiCandidates& out) {
    std::set<std::pair<uint32_t, uint32_t>> s;
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        s.emplace(out.candidates[i].row, out.candidates[i].col);
    }
    return s;
}

//! Five 2 x 2 objects on the diagonal at rows/cols 10, 30, 50, 70, 90, each in its own tile.
Frame fiveObjectDiagonal() {
    return makeFrame(
        kRows,
        kCols,
        kTile,
        {{10, 10, 2, 2, 100}, {30, 30, 2, 2, 100}, {50, 50, 2, 2, 100}, {70, 70, 2, 2, 100}, {90, 90, 2, 2, 100}});
}

//! A small 2 x 2 box (intensity 400, rows 10-11, cols 70-71) and a large 8 x 8 box (intensity 3200,
//! rows 85-92, cols 25-32) whose centers are exactly the same distance from (kX0, kY0): offsets (-39, +21)
//! and (+39, -21). Distance cannot order them, so the scaledIntensity tie-break decides.
Frame equalDistanceSmallAndLarge() { return makeFrame(kRows, kCols, kTile, {{10, 70, 2, 2, 100}, {85, 25, 8, 8, 50}}); }
constexpr uint32_t kSmallRow = 10;
constexpr uint32_t kLargeRow = 85;
}  // namespace

// ---------------------------------------------------------------------------
// Regression: one object end to end
// ---------------------------------------------------------------------------

// A 5 x 6 object of value 100 crossing a tile border: one candidate with intensity 5*6*100 and
// litPixelCount equal to the lit pixels of both tiles it covers.
TEST(RegionsOfInterestPruneTest, SingleObjectIsFound) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{40, 45, 5, 6, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 1U);
    const auto& c = out.candidates[0];
    EXPECT_EQ(c.row, 40U);
    EXPECT_EQ(c.col, 45U);
    EXPECT_EQ(c.height, 5U);
    EXPECT_EQ(c.width, 6U);
    EXPECT_EQ(c.intensity, 3000U);
    EXPECT_EQ(c.scaledIntensity, 3000.0F);
    EXPECT_EQ(c.litPixelCount, 30U);  // tile (4,4): 5x5 pixels, tile (4,5): 5x1 pixels
}

// ---------------------------------------------------------------------------
// Step 1: closest spans
// ---------------------------------------------------------------------------

// Five spans per axis, reference at the center object: the two closest (30 at distance 19, 50 at 1) are
// kept; 70 (distance 21), 10 and 90 are not.
TEST(RegionsOfInterestPruneTest, ClosestSpansAreKept) {
    const RoiCandidates out = testRegionsOfInterestPrune(fiveObjectDiagonal(), kX0, kY0, kIntensityOnly);
    const std::set<std::pair<uint32_t, uint32_t>> expected{{30, 30}, {50, 50}};
    EXPECT_EQ(corners(out), expected);
}

// Moving the reference point to a corner changes which spans are closest.
TEST(RegionsOfInterestPruneTest, ReferencePointSelectsSpans) {
    const Frame f = fiveObjectDiagonal();
    const std::set<std::pair<uint32_t, uint32_t>> topLeft{{10, 10}, {30, 30}};
    const std::set<std::pair<uint32_t, uint32_t>> bottomRight{{70, 70}, {90, 90}};
    EXPECT_EQ(corners(testRegionsOfInterestPrune(f, 0U, 0U, kIntensityOnly)), topLeft);
    EXPECT_EQ(corners(testRegionsOfInterestPrune(f, 99U, 99U, kIntensityOnly)), bottomRight);
}

// Row spans at distances 0, 10, 10, 20, 20 from y0 = 50: the second slot is a tie between the span
// at row 39 (earlier) and row 59 (later); the earlier one is kept.
TEST(RegionsOfInterestPruneTest, EarlierSpanWinsDistanceTieAtCutoff) {
    const Frame f = makeFrame(
        kRows,
        kCols,
        kTile,
        {{49, 10, 2, 2, 100}, {39, 10, 2, 2, 100}, {59, 10, 2, 2, 100}, {29, 10, 2, 2, 100}, {69, 10, 2, 2, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    const std::set<std::pair<uint32_t, uint32_t>> expected{{39, 10}, {49, 10}};
    EXPECT_EQ(corners(out), expected);
}

// ---------------------------------------------------------------------------
// Step 2b: ghost removal
// ---------------------------------------------------------------------------
//
// The scenes put both objects in one lit tile, so the tile filter cannot remove the ghosts: whatever
// is removed was removed by the ghost step.

// Diagonal pairing is real: object B (2x2) is closest on both axes, object A (3x3) second on both. The
// ghosts pair B's rows with A's cols (4 vs 9 pixels); one of them would otherwise be R2, ahead of A.
TEST(RegionsOfInterestPruneTest, GhostStepDropsAntiDiagonalGhosts) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{41, 41, 3, 3, 100}, {46, 46, 2, 2, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(std::make_pair(out.candidates[0].row, out.candidates[0].col), std::make_pair(46U, 46U));
    EXPECT_EQ(std::make_pair(out.candidates[1].row, out.candidates[1].col), std::make_pair(41U, 41U));
}

// Anti-diagonal pairing is real: object A (3x3) has the closest rows but the farther cols, object B (2x2)
// the reverse. The ghost (A's rows x B's cols) is the box closest to the reference and would be R1.
TEST(RegionsOfInterestPruneTest, GhostStepDropsDiagonalGhosts) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{47, 41, 3, 3, 100}, {42, 46, 2, 2, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(std::make_pair(out.candidates[0].row, out.candidates[0].col), std::make_pair(42U, 46U));
    EXPECT_EQ(std::make_pair(out.candidates[1].row, out.candidates[1].col), std::make_pair(47U, 41U));
}

// Known limit: two identical objects in the same tile. Their spans match either pairing equally, so
// the ghost step keeps all four boxes, and the ghosts lie in the same lit tile, so the tile filter keeps
// them too. R1 is the real object closest to the reference; R2 is a ghost, ahead of the other real object.
TEST(RegionsOfInterestPruneTest, IdenticalObjectsInOneTileKeepGhosts) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{20, 20, 3, 3, 100}, {26, 26, 3, 3, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(std::make_pair(out.candidates[0].row, out.candidates[0].col), std::make_pair(26U, 26U));
    const auto r2 = std::make_pair(out.candidates[1].row, out.candidates[1].col);
    EXPECT_TRUE(r2 == std::make_pair(20U, 26U) || r2 == std::make_pair(26U, 20U))
        << "R2 at (" << r2.first << ", " << r2.second << ") should be a ghost";
}

// The ghost step drops a pairing only when its mismatch is strictly more than 1 / GHOST_PAIRING_MARGIN
// times the other's. Two 1x1 objects at (49, 49) and (60, 60) in one tile, with the span pixel counts
// edited (intensities stay equal, so only pixels mismatch): rows 1 and x, cols 2 and x. The real
// (diagonal) pairing scores 0.5; the ghost pairing scores (x-1)/x + (x-2)/x:
//   x = 3: ghost pairing 1.0, real 0.5 = exactly the margin -> ambiguous, all four kept, R2 is a ghost;
//   x = 4: ghost pairing 1.25 -> the ghosts are dropped and R2 is the second real object.
TEST(RegionsOfInterestPruneTest, GhostStepMarginIsStrict) {
    const auto frameWith = [](uint16_t x) {
        Frame f = makeFrame(kRows, kCols, kRows, {{49, 49, 1, 1, 100}, {60, 60, 1, 1, 100}});
        f.rowSums[49] = 1U;
        f.rowSums[60] = x;
        f.colSums[49] = 2U;
        f.colSums[60] = x;
        return f;
    };
    const RoiCandidates atMargin = testRegionsOfInterestPrune(frameWith(3U), kX0, kY0, kIntensityOnly);
    ASSERT_EQ(atMargin.numCandidates, 2U);
    EXPECT_EQ(std::make_pair(atMargin.candidates[0].row, atMargin.candidates[0].col), std::make_pair(49U, 49U));
    const auto ghost = std::make_pair(atMargin.candidates[1].row, atMargin.candidates[1].col);
    EXPECT_TRUE(ghost == std::make_pair(49U, 60U) || ghost == std::make_pair(60U, 49U))
        << "R2 at (" << ghost.first << ", " << ghost.second << ") should be a ghost";

    const RoiCandidates pastMargin = testRegionsOfInterestPrune(frameWith(4U), kX0, kY0, kIntensityOnly);
    ASSERT_EQ(pastMargin.numCandidates, 2U);
    EXPECT_EQ(std::make_pair(pastMargin.candidates[1].row, pastMargin.candidates[1].col), std::make_pair(60U, 60U));
}

// With fewer than 2 spans on an axis no ghost can form, so the step does not run: two objects on the
// same rows (1 row span x 2 col spans) or the same cols (2 x 1) both survive, although their spans'
// pixel counts and intensities differ.
TEST(RegionsOfInterestPruneTest, GhostStepSkippedBelowTwoByTwoSpans) {
    const Frame sameRows = makeFrame(kRows, kCols, kTile, {{45, 40, 4, 4, 100}, {45, 55, 4, 2, 50}});
    const std::set<std::pair<uint32_t, uint32_t>> rowsExpected{{45, 40}, {45, 55}};
    EXPECT_EQ(corners(testRegionsOfInterestPrune(sameRows, kX0, kY0, kIntensityOnly)), rowsExpected);

    const Frame sameCols = makeFrame(kRows, kCols, kTile, {{40, 45, 4, 4, 100}, {55, 45, 2, 4, 50}});
    const std::set<std::pair<uint32_t, uint32_t>> colsExpected{{40, 45}, {55, 45}};
    EXPECT_EQ(corners(testRegionsOfInterestPrune(sameCols, kX0, kY0, kIntensityOnly)), colsExpected);
}

// ---------------------------------------------------------------------------
// Step 3: lit-tile filter
// ---------------------------------------------------------------------------

// Two objects on a diagonal give four boxes; the two ghost boxes cover only unlit tiles and are dropped.
TEST(RegionsOfInterestPruneTest, GhostBoxesOnUnlitTilesAreDropped) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{20, 20, 4, 4, 100}, {70, 70, 4, 4, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    const std::set<std::pair<uint32_t, uint32_t>> expected{{20, 20}, {70, 70}};
    EXPECT_EQ(corners(out), expected);
}

// A box crossing a tile border is kept when any covered tile is lit, and dropped when only an
// unrelated tile is lit.
TEST(RegionsOfInterestPruneTest, FilterChecksOnlyTheCoveredTiles) {
    Frame f = makeFrame(kRows, kCols, kTile, {{18, 30, 4, 4, 100}});  // tile rows 1 and 2, tile col 3
    std::fill(f.litCounts.begin(), f.litCounts.end(), 0U);
    f.litCounts[(2 * f.numRegionCols) + 3] = 5U;
    RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 1U);
    EXPECT_EQ(out.candidates[0].litPixelCount, 5U);

    std::fill(f.litCounts.begin(), f.litCounts.end(), 0U);
    f.litCounts[(8 * f.numRegionCols) + 8] = 5U;
    out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    EXPECT_EQ(out.numCandidates, 0U);
}

TEST(RegionsOfInterestPruneTest, NoLitTilesYieldsNoCandidates) {
    Frame f = makeFrame(kRows, kCols, kTile, {{20, 20, 4, 4, 100}});
    std::fill(f.litCounts.begin(), f.litCounts.end(), 0U);
    EXPECT_EQ(testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly).numCandidates, 0U);
}

// A grid that does not match the image (zero tile size, wrong tile rows/cols, wrong count length, or a
// tile size inconsistent with the grid dimensions) cannot confirm any box.
TEST(RegionsOfInterestPruneTest, UnusableGridYieldsNoCandidates) {
    const Frame good = makeFrame(kRows, kCols, kTile, {{20, 20, 4, 4, 100}});
    ASSERT_EQ(testRegionsOfInterestPrune(good, kX0, kY0, kIntensityOnly).numCandidates, 1U);

    Frame f = good;
    f.regionSize = 0U;
    EXPECT_EQ(testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly).numCandidates, 0U) << "regionSize 0";

    f = good;
    f.numRegionRows += 1U;
    EXPECT_EQ(testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly).numCandidates, 0U) << "extra tile row";

    f = good;
    f.numRegionCols -= 1U;
    EXPECT_EQ(testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly).numCandidates, 0U) << "missing tile col";

    f = good;
    f.litCounts.pop_back();
    EXPECT_EQ(testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly).numCandidates, 0U) << "short litCounts";

    f = good;
    f.regionSize = 20U;
    EXPECT_EQ(testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly).numCandidates, 0U) << "tile size vs grid";
}

// litPixelCount is the sum over whole covered tiles: a box inside one tile also counts other lit
// pixels in that tile, and a box crossing a tile corner sums all four tiles.
TEST(RegionsOfInterestPruneTest, LitPixelCountIsSumOfCoveredTiles) {
    // 4x4 object plus a 1-pixel object in the same tile (4,4): every box in the tile reports 16 + 1.
    const Frame shared = makeFrame(kRows, kCols, kTile, {{42, 42, 4, 4, 100}, {47, 47, 1, 1, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(shared, kX0, kY0, kIntensityOnly);
    ASSERT_GT(out.numCandidates, 0U);
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        EXPECT_EQ(out.candidates[i].litPixelCount, 17U) << "candidate " << i;
    }

    // 4x4 object over the corner of tiles (4,4), (4,5), (5,4), (5,5): 4 lit pixels in each.
    const Frame corner = makeFrame(kRows, kCols, kTile, {{48, 48, 4, 4, 100}});
    const RoiCandidates c = testRegionsOfInterestPrune(corner, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(c.numCandidates, 1U);
    EXPECT_EQ(c.candidates[0].litPixelCount, 16U);
}

TEST(RegionsOfInterestPruneTest, LitPixelCountSaturates) {
    Frame f = makeFrame(kRows, kCols, kTile, {{48, 48, 4, 4, 100}});
    std::fill(f.litCounts.begin(), f.litCounts.end(), std::numeric_limits<uint32_t>::max());
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 1U);
    EXPECT_EQ(out.candidates[0].litPixelCount, std::numeric_limits<uint32_t>::max());
}

// ---------------------------------------------------------------------------
// Step 4: scaled intensity
// ---------------------------------------------------------------------------

// alpha 1: exponent alpha - 1 = 0, so scaledIntensity is the plain intensity.
TEST(RegionsOfInterestPruneTest, AlphaOneScaledIntensityEqualsIntensity) {
    const RoiCandidates out = testRegionsOfInterestPrune(fiveObjectDiagonal(), kX0, kY0, kIntensityOnly);
    ASSERT_GT(out.numCandidates, 0U);
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        EXPECT_EQ(out.candidates[i].scaledIntensity, static_cast<float>(out.candidates[i].intensity));
    }
}

// Alpha only matters when two boxes are equally far from the reference. Small 2x2 box (intensity 400) vs
// large 8x8 box (intensity 3200) at the same distance: alpha 1 (exponent 0) and 0 (exponent -1, multiplies by
// the area: 1600 vs 204800) rank the large box first; alpha 2 (exponent 1) divides by the area (100 vs 50)
// and ranks the small box first.
TEST(RegionsOfInterestPruneTest, AlphaBreaksDistanceTies) {
    const Frame f = equalDistanceSmallAndLarge();
    for (const float alpha : {kIntensityOnly, 0.0F}) {
        const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, alpha);
        ASSERT_EQ(out.numCandidates, 2U);
        EXPECT_EQ(out.candidates[0].row, kLargeRow) << "alpha " << alpha;
    }
    const RoiCandidates atZero = testRegionsOfInterestPrune(f, kX0, kY0, 0.0F);
    EXPECT_EQ(atZero.candidates[0].scaledIntensity, 204800.0F);
    EXPECT_EQ(atZero.candidates[1].scaledIntensity, 1600.0F);

    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, 2.0F);
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(out.candidates[0].row, kSmallRow);
    EXPECT_EQ(out.candidates[0].scaledIntensity, 100.0F);
    EXPECT_EQ(out.candidates[1].scaledIntensity, 50.0F);
}

// Alpha sweep over [0, 2] (exponent alpha - 1 over [-1, 1]) on the equal-distance scene. The two boxes score
// 400 / 4^e and 3200 / 64^e with e = alpha - 1, equal where 16^e = 8, i.e. e = 0.75, alpha = 1.75: below it
// the large box is R1, above it the small box. At exactly 1.75 the scores tie and the boxes keep the order
// they were formed in, which is a span-order detail rather than a cost result, so it is not checked here.
TEST(RegionsOfInterestPruneTest, AlphaSweepSwitchesToSmallBoxAboveCrossover) {
    const Frame f = equalDistanceSmallAndLarge();
    for (const float alpha : {0.0F, 0.25F, 0.5F, 0.75F, 1.0F, 1.25F, 1.5F, 1.7F}) {
        const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, alpha);
        ASSERT_EQ(out.numCandidates, 2U) << "alpha " << alpha;
        EXPECT_EQ(out.candidates[0].row, kLargeRow) << "alpha " << alpha;
    }
    for (const float alpha : {1.8F, 1.9F, 2.0F}) {
        const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, alpha);
        ASSERT_EQ(out.numCandidates, 2U) << "alpha " << alpha;
        EXPECT_EQ(out.candidates[0].row, kSmallRow) << "alpha " << alpha;
    }
}

// Bypass: any negative alpha switches the cost function off. On the equal-distance scene, where the cost
// decides with alpha in [0, 2], the boxes now keep the order they were formed in: the row spans tie on
// distance, so the earlier one (the small box's, row 10) comes first, and the large box second, although it
// is 8x brighter. scaledIntensity is not computed (0).
TEST(RegionsOfInterestPruneTest, NegativeAlphaBypassesCostFunction) {
    const Frame f = equalDistanceSmallAndLarge();
    for (const float alpha : {-0.001F, -1.0F, -5.0F, -1000.0F, -std::numeric_limits<float>::infinity()}) {
        const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, alpha);
        ASSERT_EQ(out.numCandidates, 2U) << "alpha " << alpha;
        EXPECT_EQ(out.candidates[0].row, kSmallRow) << "alpha " << alpha;
        EXPECT_EQ(out.candidates[1].row, kLargeRow) << "alpha " << alpha;
        EXPECT_EQ(out.candidates[0].scaledIntensity, 0.0F) << "alpha " << alpha;
        EXPECT_EQ(out.candidates[1].scaledIntensity, 0.0F) << "alpha " << alpha;
    }
}

// Bypass changes only the tie-break: boxes at different distances are still ranked closest first.
TEST(RegionsOfInterestPruneTest, NegativeAlphaStillRanksClosestFirst) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{10, 10, 4, 4, 300}, {40, 40, 4, 4, 200}, {70, 70, 4, 4, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, 72U, 72U, -1.0F);
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(out.candidates[0].row, 70U);
    EXPECT_EQ(out.candidates[1].row, 40U);
}

// Boxes with lit counts but zero intensity get a scaledIntensity of exactly 0 (never NaN) for any alpha.
TEST(RegionsOfInterestPruneTest, ZeroIntensityGivesZeroScaledIntensity) {
    Frame f = makeFrame(kRows, kCols, kTile, {{20, 20, 4, 4, 100}});
    std::fill(f.rowIntensities.begin(), f.rowIntensities.end(), 0U);
    std::fill(f.colIntensities.begin(), f.colIntensities.end(), 0U);
    for (const float alpha : {-1.0F, 0.0F, 1.0F, 2.0F}) {
        const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, alpha);
        ASSERT_EQ(out.numCandidates, 1U);
        EXPECT_EQ(out.candidates[0].intensity, 0U);
        EXPECT_EQ(out.candidates[0].scaledIntensity, 0.0F) << "alpha " << alpha;
    }
}

// Span intensity sums above 2^32 do not wrap: a 3x3 object at 1.4e9 per pixel (sum 1.26e10) still
// outranks a 2x2 one (sum 5.6e9); a 32-bit sum would reverse them.
TEST(RegionsOfInterestPruneTest, IntensitySumsDoNotWrap) {
    constexpr uint32_t kValue = 1'400'000'000U;
    const Frame f = makeFrame(kRows, kCols, kTile, {{10, 10, 2, 2, kValue}, {60, 60, 3, 3, kValue}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(out.candidates[0].row, 60U);
    EXPECT_EQ(out.candidates[0].intensity, 9ULL * kValue);
    EXPECT_EQ(out.candidates[1].intensity, 4ULL * kValue);
}

// ---------------------------------------------------------------------------
// Step 5: ranking
// ---------------------------------------------------------------------------

// Ranked by distance, not intensity: the reference point sits on the dimmest object, which ranks first,
// ahead of the brighter object at 40 (the brightest, at 10, is not among the closest spans).
TEST(RegionsOfInterestPruneTest, RankedByDistance) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{10, 10, 4, 4, 300}, {40, 40, 4, 4, 200}, {70, 70, 4, 4, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, 72U, 72U, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(out.candidates[0].row, 70U);
    EXPECT_EQ(out.candidates[1].row, 40U);
}

// Equal scaledIntensity: the box closer to the reference point comes first.
TEST(RegionsOfInterestPruneTest, EqualScaledIntensityCloserIsFirst) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{10, 10, 2, 2, 100}, {48, 60, 2, 2, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(out.candidates[0].scaledIntensity, out.candidates[1].scaledIntensity);
    EXPECT_EQ(out.candidates[0].row, 48U);
}

// Box size is not a tie-break: a large close box and a small far box with equal intensity (400) are
// ordered by distance alone.
TEST(RegionsOfInterestPruneTest, BoxSizeIsNotATieBreak) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{46, 46, 4, 4, 25}, {10, 80, 2, 2, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(out.candidates[0].row, 46U);
    EXPECT_EQ(out.candidates[1].row, 10U);
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

TEST(RegionsOfInterestPruneTest, EmptyFrameYieldsNoCandidates) {
    const Frame f = makeFrame(0U, 0U, kTile, {});
    EXPECT_EQ(testRegionsOfInterestPrune(f, 0U, 0U, kIntensityOnly).numCandidates, 0U);
}

TEST(RegionsOfInterestPruneTest, DarkFrameYieldsNoCandidates) {
    const Frame f = makeFrame(kRows, kCols, kTile, {});
    EXPECT_EQ(testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly).numCandidates, 0U);
}

TEST(RegionsOfInterestPruneTest, EntireFrameLit) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{0, 0, kRows, kCols, 7}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 1U);
    EXPECT_EQ(out.candidates[0].height, kRows);
    EXPECT_EQ(out.candidates[0].width, kCols);
    EXPECT_EQ(out.candidates[0].litPixelCount, kRows * kCols);
}

// Objects touching the image edges: boxes start at row/col 0 and end on the last row/col.
TEST(RegionsOfInterestPruneTest, ObjectsOnImageBorder) {
    const Frame f = makeFrame(kRows, kCols, kTile, {{0, 0, 3, 3, 100}, {kRows - 3, kCols - 3, 3, 3, 100}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, kX0, kY0, kIntensityOnly);
    const std::set<std::pair<uint32_t, uint32_t>> expected{{0, 0}, {kRows - 3, kCols - 3}};
    EXPECT_EQ(corners(out), expected);
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        EXPECT_EQ(out.candidates[i].height, 3U) << "candidate " << i;
        EXPECT_EQ(out.candidates[i].width, 3U) << "candidate " << i;
    }
}

TEST(RegionsOfInterestPruneTest, SinglePixelFrame) {
    const Frame f = makeFrame(1U, 1U, 64U, {{0, 0, 1, 1, 9}});
    const RoiCandidates out = testRegionsOfInterestPrune(f, 0U, 0U, kIntensityOnly);
    ASSERT_EQ(out.numCandidates, 1U);
    EXPECT_EQ(out.candidates[0].litPixelCount, 1U);
}

// A 5x5 grid of objects: the 2 x 2 closest spans form 4 real boxes, and only the first
// ROI_CANDIDATES_OUT_MAX are returned (the helper checks the cut slots are zero).
TEST(RegionsOfInterestPruneTest, AtMostTwoCandidatesReturned) {
    std::vector<TestObject> objects;
    for (const uint32_t r : {10U, 30U, 50U, 70U, 90U}) {
        for (const uint32_t c : {10U, 30U, 50U, 70U, 90U}) {
            objects.push_back({r, c, 2, 2, 100});
        }
    }
    const RoiCandidates out =
        testRegionsOfInterestPrune(makeFrame(kRows, kCols, kTile, objects), kX0, kY0, kIntensityOnly);
    EXPECT_EQ(out.numCandidates, ROI_CANDIDATES_OUT_MAX);
}

// ---------------------------------------------------------------------------
// Flyby scenes: is the target R1?
// ---------------------------------------------------------------------------
//
// Small scenes in the A1 camera window (682 x 500 px, 64 px tiles, reference at its center) with an
// 11 x 11 target near the reference, as in the A1 flyby. R1 may be larger than the target (merged spans
// stretch it); downstream only uses its center and pixel count, so it is correct as long as the
// target's center is inside it.

namespace {
constexpr uint32_t kSceneRows = 500;
constexpr uint32_t kSceneCols = 682;
constexpr uint32_t kSceneTile = 64;
constexpr uint32_t kSceneX0 = 341;
constexpr uint32_t kSceneY0 = 250;
constexpr float kSceneAlpha = 2.0F;  // exponent 1: intensity / area

//! The target's center lies inside R1.
::testing::AssertionResult targetIsR1(const RoiCandidates& out, const TestObject& target) {
    const uint32_t row = target.row + (target.height / 2U);
    const uint32_t col = target.col + (target.width / 2U);
    if (out.numCandidates > 0U) {
        const RoiCandidateEntry& r1 = out.candidates[0];
        if (row >= r1.row && row < r1.row + r1.height && col >= r1.col && col < r1.col + r1.width) {
            return ::testing::AssertionSuccess();
        }
    }
    auto failure = ::testing::AssertionFailure();
    failure << "target center (row " << row << ", col " << col << ") is not inside R1";
    for (uint32_t i = 0; i < out.numCandidates; ++i) {
        const RoiCandidateEntry& e = out.candidates[i];
        failure << "\n  R" << (i + 1) << ": row " << e.row << " col " << e.col << " " << e.width << "x" << e.height;
    }
    return failure;
}

RoiCandidates runScene(const std::vector<TestObject>& objects) {
    return testRegionsOfInterestPrune(
        makeFrame(kSceneRows, kSceneCols, kSceneTile, objects), kSceneX0, kSceneY0, kSceneAlpha);
}
}  // namespace

TEST(RegionsOfInterestPruneTest, SceneTargetAtReference) {
    const TestObject target{245, 336, 11, 11, 200};
    EXPECT_TRUE(targetIsR1(runScene({target}), target));
}

// Target 20 px left of the reference; a streak far away on other rows and columns.
TEST(RegionsOfInterestPruneTest, SceneOffCenterTargetWithFarStreak) {
    const TestObject target{241, 315, 11, 11, 200};
    const TestObject streak{100, 450, 9, 80, 200};
    EXPECT_TRUE(targetIsR1(runScene({target, streak}), target));
}

// A horizontal streak above the target covers its columns, so the col spans merge and R1 is stretched to
// the streak's width; the target is still inside it.
TEST(RegionsOfInterestPruneTest, SceneStreakStretchesBoxButContainsTarget) {
    const TestObject target{245, 336, 11, 11, 200};
    const TestObject streak{150, 300, 9, 81, 200};
    EXPECT_TRUE(targetIsR1(runScene({target, streak}), target));
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

TEST(RegionsOfInterestPruneTest, SetupTest) { testRegionsOfInterestPruneSetup(); }

// An alpha above 2 (e.g. 5, exponent 4) or NaN cannot make a config, so the algorithm never runs with it:
// create() throws, and an algorithm keeps the last valid config it was given. Negative values are valid
// (bypass) and covered by NegativeAlphaBypassesCostFunction.
TEST(RegionsOfInterestPruneTest, AlphaAboveTwoIsRejected) {
    for (const float alpha :
         {5.0F, 2.01F, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
        EXPECT_THROW((void)RegionsOfInterestPruneConfig::create(kX0, kY0, alpha), fsw::invalid_argument)
            << "alpha " << alpha;
    }
    RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(kX0, kY0, 2.0F)};
    EXPECT_THROW(alg.setConfig(RegionsOfInterestPruneConfig::create(kX0, kY0, 5.0F)), fsw::invalid_argument);
    const Frame f = equalDistanceSmallAndLarge();
    const RoiCandidates out = alg.update(f.rowSums.data(),
                                         f.numRows,
                                         f.colSums.data(),
                                         f.numCols,
                                         f.rowIntensities.data(),
                                         f.colIntensities.data(),
                                         f.binGrid());
    ASSERT_EQ(out.numCandidates, 2U);
    EXPECT_EQ(out.candidates[0].row, kSmallRow) << "alpha 2 should still be in effect";
}

// setConfig() changes the reference point and alpha used by the next update().
TEST(RegionsOfInterestPruneTest, SetConfigUpdatesReferencePointAndAlpha) {
    const Frame diagonal = fiveObjectDiagonal();
    RegionsOfInterestPruneAlgorithm alg{RegionsOfInterestPruneConfig::create(0U, 0U, kIntensityOnly)};
    const auto run = [&alg](const Frame& f) {
        return alg.update(f.rowSums.data(),
                          f.numRows,
                          f.colSums.data(),
                          f.numCols,
                          f.rowIntensities.data(),
                          f.colIntensities.data(),
                          f.binGrid());
    };
    EXPECT_EQ(corners(run(diagonal)).count({90U, 90U}), 0U);
    alg.setConfig(RegionsOfInterestPruneConfig::create(99U, 99U, kIntensityOnly));
    EXPECT_EQ(corners(run(diagonal)).count({90U, 90U}), 1U);
    EXPECT_EQ(corners(run(diagonal)).count({10U, 10U}), 0U);

    const Frame sizes = equalDistanceSmallAndLarge();
    alg.setConfig(RegionsOfInterestPruneConfig::create(kX0, kY0, kIntensityOnly));
    EXPECT_EQ(run(sizes).candidates[0].row, kLargeRow);
    alg.setConfig(RegionsOfInterestPruneConfig::create(kX0, kY0, 2.0F));
    EXPECT_EQ(run(sizes).candidates[0].row, kSmallRow);
}

// ---------------------------------------------------------------------------
// Property tests on fixed frames
// ---------------------------------------------------------------------------

TEST(RegionsOfInterestPruneTest, OutputIsWellFormedOnSampleFrames) {
    std::vector<TestObject> grid;
    for (const uint32_t r : {5U, 37U, 61U, 88U}) {
        for (const uint32_t c : {3U, 44U, 59U, 97U}) {
            grid.push_back({r, c, 3, 2, (r * 7U) + c});
        }
    }
    const std::vector<Frame> frames{
        fiveObjectDiagonal(),
        makeFrame(kRows, kCols, kTile, {{20, 20, 3, 3, 100}, {26, 26, 3, 3, 100}}),
        makeFrame(kRows, kCols, kTile, grid),
        makeFrame(kRows, kCols, 7U, grid),
        makeFrame(kRows, kCols, 1U, grid),
    };
    for (const Frame& f : frames) {
        for (const float alpha : {-1.0F, -0.3F, 0.0F, 0.5F, 1.0F, 1.5F, 2.0F}) {
            propertyRegionsOfInterestPruneOutputIsWellFormed(f, kX0, kY0, alpha);
            propertyRegionsOfInterestPruneOutputIsWellFormed(f, 0U, kRows, alpha);
        }
    }
}
