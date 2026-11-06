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

//!< Worst-case row-span x col-span cross-product size, before ranking/truncation to ROI_CANDIDATES_MAX.
static constexpr uint32_t MAX_RAW_CANDIDATES = ROW_SPANS_MAX * COL_SPANS_MAX;
static_assert(MAX_RAW_CANDIDATES <= ROI_CANDIDATES_MAX,
              "ROW_SPANS_MAX x COL_SPANS_MAX must not exceed ROI_CANDIDATES_MAX");

/*! @brief Internal bounding-box candidate (corner coordinates + pixel count). */
struct RoiCandidateEntry {
    uint32_t row{};     //!< [px] Top-left row of the bounding box
    uint32_t col{};     //!< [px] Top-left column of the bounding box
    uint32_t height{};  //!< [px] Height of the bounding box
    uint32_t width{};   //!< [px] Width of the bounding box
    uint32_t count{};   //!< [-] Estimated above-threshold pixel count
};

struct RoiCandidates {
    uint32_t numCandidates{};                                        //!< [-] Number of valid entries
    std::array<RoiCandidateEntry, ROI_CANDIDATES_MAX> candidates{};  //!< [-] Sorted by count descending,
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
 *  An instance can only exist if x0 is in [0, MAX_IMAGE_COLS] and y0 is in [0, MAX_IMAGE_ROWS].
 *  Construct via RegionsOfInterestPruneConfig::create(...).
 */
class RegionsOfInterestPruneConfig final {
   public:
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) -- x0/y0 are a paired 2-D point.
    static RegionsOfInterestPruneConfig create(uint32_t x0, uint32_t y0) {
        if (!isValidX0(x0)) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: x0 must be in [0, MAX_IMAGE_COLS]");
        }
        if (!isValidY0(y0)) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: y0 must be in [0, MAX_IMAGE_ROWS]");
        }
        return {x0, y0};
    }
    // NOLINTEND(bugprone-easily-swappable-parameters)

    static bool isValidX0(uint32_t x0) { return x0 <= MAX_IMAGE_COLS; }
    static bool isValidY0(uint32_t y0) { return y0 <= MAX_IMAGE_ROWS; }

    uint32_t getX0() const { return x0; }
    uint32_t getY0() const { return y0; }

   private:
    RegionsOfInterestPruneConfig(uint32_t x0, uint32_t y0) : x0(x0), y0(y0) {}

    uint32_t x0;  //!< [px] Reference point column
    uint32_t y0;  //!< [px] Reference point row
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

    RoiCandidates update(const uint16_t* rowSums, uint32_t numRows, const uint16_t* colSums, uint32_t numCols) const;

   private:
    RegionsOfInterestPruneConfig cfg;

    //!< Contiguous non-zero run in a 1-D sum array.
    struct SpanEntry {
        uint32_t start{};   //!< [px] First index of the span
        uint32_t length{};  //!< [px] Number of entries in the span
        uint32_t sum{};     //!< [-] Accumulated sum over the span
        float distance{};   //!< [px] |span midpoint - reference coordinate|
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

    // Step 1: find contiguous non-zero spans and keep the Capacity closest to `reference` in a single pass.
    template <uint32_t Capacity>
    static TopSpans<Capacity> findTopSpans(std::span<const uint16_t> s, float reference);

    // Step 2: cross-product of top row/col spans → bounding-box candidates.
    static CandidateArray buildCandidates(const RowTopSpans& rowSpans, const ColTopSpans& colSpans);

    // Step 3: sort candidates by count descending (ties broken by distance to the reference point,
    // then window area), truncate, and pack the result.
    static RoiCandidates packOutput(CandidateArray candidates, float referenceRow, float referenceCol);
};

#endif
