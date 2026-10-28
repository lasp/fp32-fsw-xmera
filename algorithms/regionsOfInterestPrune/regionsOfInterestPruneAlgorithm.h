#ifndef F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H
#define F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"

#include <array>
#include <cstdint>
#include <span>

static constexpr uint32_t ROI_CANDIDATES_MAX = 16;    //!< Maximum number of candidates retained/published
static constexpr uint32_t DEFAULT_MAX_ROW_SPANS = 3;  //!< Default value for maxRowSpans
static constexpr uint32_t DEFAULT_MAX_COL_SPANS = 3;  //!< Default value for maxColSpans
static constexpr uint32_t ROW_SPANS_MAX = 4;          //!< Upper bound on maxRowSpans
static constexpr uint32_t COL_SPANS_MAX = 4;          //!< Upper bound on maxColSpans

//!< Worst-case row-span x col-span cross-product size, before ranking/truncation to ROI_CANDIDATES_MAX.
static constexpr uint32_t MAX_RAW_CANDIDATES = ROW_SPANS_MAX * COL_SPANS_MAX;
static_assert(MAX_RAW_CANDIDATES <= ROI_CANDIDATES_MAX,
              "ROW_SPANS_MAX x COL_SPANS_MAX must not exceed ROI_CANDIDATES_MAX");
static_assert(DEFAULT_MAX_ROW_SPANS <= ROW_SPANS_MAX && DEFAULT_MAX_COL_SPANS <= COL_SPANS_MAX,
              "Default span counts must lie within the valid range");

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
                                                                     //!< then distance to image center
                                                                     //!< ascending, then area ascending
};

/*! @brief Validated configuration for the regions-of-interest pruning algorithm.
 *
 *  An instance can only exist if maxRowSpans is in [1, ROW_SPANS_MAX] and maxColSpans is in
 *  [1, COL_SPANS_MAX].
 *  Construct via RegionsOfInterestPruneConfig::create(...).
 */
class RegionsOfInterestPruneConfig final {
   public:
    static RegionsOfInterestPruneConfig create(uint32_t maxRowSpans, uint32_t maxColSpans) {
        if (!isValidMaxRowSpans(maxRowSpans)) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: maxRowSpans must be in [1, ROW_SPANS_MAX]");
        }
        if (!isValidMaxColSpans(maxColSpans)) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: maxColSpans must be in [1, COL_SPANS_MAX]");
        }
        return {maxRowSpans, maxColSpans};
    }

    static bool isValidMaxRowSpans(uint32_t maxRowSpans) { return maxRowSpans > 0 && maxRowSpans <= ROW_SPANS_MAX; }
    static bool isValidMaxColSpans(uint32_t maxColSpans) { return maxColSpans > 0 && maxColSpans <= COL_SPANS_MAX; }

    uint32_t getMaxRowSpans() const { return maxRowSpans; }
    uint32_t getMaxColSpans() const { return maxColSpans; }

   private:
    RegionsOfInterestPruneConfig(uint32_t maxRowSpans, uint32_t maxColSpans)
        : maxRowSpans(maxRowSpans), maxColSpans(maxColSpans) {}

    uint32_t maxRowSpans;
    uint32_t maxColSpans;
};

/*! @brief algorithm for the regions-of-interest pruning stage.
 *
 *  Accepts raw row/col sum arrays and returns a
 *  RoiCandidates.  Pipeline stages:
 *    1. Single pass over rowSums / colSums: find contiguous non-zero spans, accumulate per-span
 *       sums, and keep a running top maxRowSpans / maxColSpans spans by sum.
 *    2. Cross-product of the kept spans → bounding boxes with count = min(R[k], C[l]).
 *    3. Sort by count descending (ties broken by distance to image center, ascending, then by
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
    };
    //!< Fixed-capacity top-K span list: data[0..count) are valid, sorted by sum descending.
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

    // Step 1: find contiguous non-zero spans and keep the top-keep by sum in a single pass.
    template <uint32_t Capacity>
    static TopSpans<Capacity> findTopSpans(std::span<const uint16_t> s, uint32_t keep);

    // Step 2: cross-product of top row/col spans → bounding-box candidates.
    static CandidateArray buildCandidates(const RowTopSpans& rowSpans, const ColTopSpans& colSpans);

    // Step 3: sort candidates by count descending (ties broken by distance to image center, then
    // window area), truncate, and pack the result.
    static RoiCandidates packOutput(CandidateArray candidates, uint32_t numRows, uint32_t numCols);
};

#endif
