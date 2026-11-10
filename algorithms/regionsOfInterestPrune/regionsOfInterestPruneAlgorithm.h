#ifndef F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H
#define F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"

#include <array>
#include <cstdint>
#include <span>

static constexpr uint32_t ROI_CANDIDATES_MAX = 16;  //!< Maximum number of candidates retained/published
static constexpr uint32_t ROW_SPANS_MAX = 4;        //!< Number of row spans kept (closest to the reference row)
static constexpr uint32_t COL_SPANS_MAX = 4;        //!< Number of col spans kept (closest to the reference col)

static constexpr uint32_t MAX_IMAGE_ROWS = 3000;  //!< [px] Full camera image height
static constexpr uint32_t MAX_IMAGE_COLS = 4096;  //!< [px] Full camera image width

static constexpr uint32_t DEFAULT_X0 = MAX_IMAGE_COLS / 2U;  //!< [px] Default reference column (image center)
static constexpr uint32_t DEFAULT_Y0 = MAX_IMAGE_ROWS / 2U;  //!< [px] Default reference row (image center)

static constexpr float MIN_ALPHA_AREA_PRUNE_SCALING = -1.0F;     //!< [-] Lowest accepted area exponent
static constexpr float MAX_ALPHA_AREA_PRUNE_SCALING = 1.0F;      //!< [-] Highest accepted area exponent
static constexpr float DEFAULT_ALPHA_AREA_PRUNE_SCALING = 0.0F;  //!< [-] Default area exponent (area ignored)

//!< Worst-case row-span x col-span cross-product size, before ranking/truncation to ROI_CANDIDATES_MAX.
static constexpr uint32_t MAX_RAW_CANDIDATES = ROW_SPANS_MAX * COL_SPANS_MAX;
static_assert(MAX_RAW_CANDIDATES <= ROI_CANDIDATES_MAX,
              "ROW_SPANS_MAX x COL_SPANS_MAX must not exceed ROI_CANDIDATES_MAX");

/*! @brief Internal bounding-box candidate (corner coordinates + pixel count). */
struct RoiCandidateEntry {
    uint32_t row{};           //!< [px] Top-left row of the bounding box
    uint32_t col{};           //!< [px] Top-left column of the bounding box
    uint32_t height{};        //!< [px] Height of the bounding box
    uint32_t width{};         //!< [px] Width of the bounding box
    uint32_t count{};         //!< [-] Estimated above-threshold pixel count
    uint64_t intensity{};     //!< [-] Estimated intensity: min(row-span intensity sum, col-span intensity sum)
    float scaledIntensity{};  //!< [-] intensity / (width * height)^alphaAreaPruneScaling
};

/*! @brief Bin grid of ROI tiles covering the image, used to drop candidates that touch no lit tile.
 *
 *  Tiles are regionSize x regionSize pixels in row-major order; tiles at the right and bottom edges
 *  can be smaller. litCounts[tileRow * numRegionCols + tileCol] is the number of above-threshold
 *  pixels in that tile. The grid must cover the image passed to update(): numRegionRows =
 *  ceil(numRows / regionSize), numRegionCols = ceil(numCols / regionSize), and litCounts holds
 *  exactly numRegionRows * numRegionCols entries. Otherwise update() treats the grid as unusable and
 *  returns no candidates.
 */
struct RoiBinGrid {
    uint32_t regionSize{};                //!< [px] Tile edge length
    uint32_t numRegionRows{};             //!< [-]  Number of tile rows
    uint32_t numRegionCols{};             //!< [-]  Number of tile columns
    std::span<const uint32_t> litCounts;  //!< [-]  Above-threshold pixel count per tile, row-major
};

struct RoiCandidates {
    uint32_t numCandidates{};                                        //!< [-] Number of valid entries
    std::array<RoiCandidateEntry, ROI_CANDIDATES_MAX> candidates{};  //!< [-] Sorted by scaledIntensity descending,
                                                                     //!< then distance to the reference
                                                                     //!< point ascending, then area
                                                                     //!< ascending
};

/*! @brief Validated configuration for the regions-of-interest pruning algorithm.
 *
 *  The reference point used for span selection and the distance tie-break is the absolute pixel
 *  position (x0, y0): x0 is the column, y0 the row, in the same coordinates as the published
 *  cornerX/cornerY. Defaults to the full-image center (DEFAULT_X0, DEFAULT_Y0).
 *
 *  alphaAreaPruneScaling is the area exponent in scaledIntensity = intensity / area^alpha: 0 ranks by
 *  intensity alone, a positive value favors smaller boxes, a negative value larger ones.
 *
 *  An instance can only exist if x0 is in [0, MAX_IMAGE_COLS], y0 is in [0, MAX_IMAGE_ROWS] and
 *  alphaAreaPruneScaling is in [MIN_ALPHA_AREA_PRUNE_SCALING, MAX_ALPHA_AREA_PRUNE_SCALING].
 *  Construct via RegionsOfInterestPruneConfig::create(...).
 */
class RegionsOfInterestPruneConfig final {
   public:
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) -- x0/y0 are a paired 2-D point.
    static RegionsOfInterestPruneConfig create(uint32_t x0, uint32_t y0, float alphaAreaPruneScaling) {
        if (!isValidX0(x0)) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: x0 must be in [0, MAX_IMAGE_COLS]");
        }
        if (!isValidY0(y0)) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: y0 must be in [0, MAX_IMAGE_ROWS]");
        }
        if (!isValidAlphaAreaPruneScaling(alphaAreaPruneScaling)) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: alphaAreaPruneScaling must be in [-1, 1]");
        }
        return {x0, y0, alphaAreaPruneScaling};
    }
    // NOLINTEND(bugprone-easily-swappable-parameters)

    static bool isValidX0(uint32_t x0) { return x0 <= MAX_IMAGE_COLS; }
    static bool isValidY0(uint32_t y0) { return y0 <= MAX_IMAGE_ROWS; }
    // NaN fails both comparisons and infinities fall outside the range, so no separate isfinite check is needed.
    static bool isValidAlphaAreaPruneScaling(float alpha) {
        return alpha >= MIN_ALPHA_AREA_PRUNE_SCALING && alpha <= MAX_ALPHA_AREA_PRUNE_SCALING;
    }

    uint32_t getX0() const { return x0; }
    uint32_t getY0() const { return y0; }
    float getAlphaAreaPruneScaling() const { return alphaAreaPruneScaling; }

   private:
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) -- private; only reached through the validated create().
    RegionsOfInterestPruneConfig(uint32_t x0, uint32_t y0, float alphaAreaPruneScaling)
        : x0(x0), y0(y0), alphaAreaPruneScaling(alphaAreaPruneScaling) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    uint32_t x0;                  //!< [px] Reference point column
    uint32_t y0;                  //!< [px] Reference point row
    float alphaAreaPruneScaling;  //!< [-] Area exponent in scaledIntensity
};

/*! @brief algorithm for the regions-of-interest pruning stage.
 *
 *  Accepts raw row/col sum arrays and returns a
 *  RoiCandidates.  Pipeline stages:
 *    1. Single pass over rowSums / colSums: find contiguous non-zero spans, accumulate per-span
 *       sums, and keep the ROW_SPANS_MAX / COL_SPANS_MAX spans whose midpoints are closest to the
 *       reference row / col.
 *    2. Cross-product of the kept spans → bounding boxes with count = min(R[k], C[l]).
 *    3. Sort by count descending (ties broken by distance to the reference point, ascending, then by
 *       window area, ascending), return top ROI_CANDIDATES_MAX entries.
 */
class RegionsOfInterestPruneAlgorithm final {
   public:
    explicit RegionsOfInterestPruneAlgorithm(const RegionsOfInterestPruneConfig& config);

    void setConfig(const RegionsOfInterestPruneConfig& config);

    RoiCandidates update(const uint16_t* rowSums,
                         uint32_t numRows,
                         const uint16_t* colSums,
                         uint32_t numCols,
                         const uint32_t* rowIntensities,
                         const uint32_t* colIntensities,
                         const RoiBinGrid& binGrid) const;

   private:
    RegionsOfInterestPruneConfig cfg;

    //!< Contiguous non-zero run in a 1-D sum array.
    struct SpanEntry {
        uint32_t start{};      //!< [px] First index of the span
        uint32_t length{};     //!< [px] Number of entries in the span
        uint32_t sum{};        //!< [-] Accumulated sum over the span
        uint64_t intensity{};  //!< [-] Accumulated intensity over the span
        float distance{};      //!< [px] |span midpoint - reference coordinate|
    };
    //!< Fixed-capacity closest-K span list: data[0..count) are valid, sorted by distance ascending.
    template <uint32_t Capacity>
    struct TopSpans {
        uint32_t count{};
        std::array<SpanEntry, Capacity> data{};
    };
    using RowTopSpans = TopSpans<ROW_SPANS_MAX>;
    using ColTopSpans = TopSpans<COL_SPANS_MAX>;
    //!< Fixed-capacity candidate list: data[0..count) are the valid entries.
    struct CandidateArray {
        uint32_t count{};
        std::array<RoiCandidateEntry, MAX_RAW_CANDIDATES> data{};
    };

    // Step 1: find contiguous non-zero spans and keep the Capacity closest to `reference` in a single pass,
    // summing each span's counts and intensities.
    template <uint32_t Capacity>
    static TopSpans<Capacity> findTopSpans(std::span<const uint16_t> s,
                                           std::span<const uint32_t> intensities,
                                           float reference);

    // Step 2: cross-product of top row/col spans → bounding-box candidates.
    static CandidateArray buildCandidates(const RowTopSpans& rowSpans, const ColTopSpans& colSpans);

    // Step 3: drop candidates that overlap no lit tile of the bin grid.
    static CandidateArray filterByLitTiles(const CandidateArray& candidates, const RoiBinGrid& binGrid);
    static bool isBinGridUsable(const RoiBinGrid& binGrid, uint32_t numRows, uint32_t numCols);
    static bool overlapsLitTile(const RoiCandidateEntry& candidate, const RoiBinGrid& binGrid);

    // Step 4: set each candidate's scaledIntensity = intensity / area^alpha.
    static void scaleIntensityByArea(CandidateArray& candidates, float alphaAreaPruneScaling);

    // Step 4: sort candidates by scaledIntensity descending (ties broken by distance to the reference point,
    // then window area), truncate, and pack the result.
    static RoiCandidates packOutput(CandidateArray candidates, float referenceRow, float referenceCol);
};

#endif
