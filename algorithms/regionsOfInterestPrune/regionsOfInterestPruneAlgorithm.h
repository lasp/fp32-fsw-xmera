#ifndef F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H
#define F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"

#include <array>
#include <cstdint>
#include <utility>

static constexpr uint32_t ROI_CANDIDATES_MAX = 16;    //!< Maximum number of candidates retained/published
static constexpr uint32_t DEFAULT_MAX_ROW_SPANS = 3;  //!< Default value for maxRowSpans
static constexpr uint32_t DEFAULT_MAX_COL_SPANS = 3;  //!< Default value for maxColSpans

//!< Maximum contiguous non-zero spans tracked per row/col scan; any beyond this are dropped.
static constexpr uint32_t MAX_SPANS = 128;
//!< Worst-case row-span x col-span cross-product size, before ranking/truncation to ROI_CANDIDATES_MAX.
static constexpr uint32_t MAX_RAW_CANDIDATES = MAX_SPANS * MAX_SPANS;

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
 *  An instance can only exist if both maxRowSpans and maxColSpans are > 0.
 *  Construct via RegionsOfInterestPruneConfig::create(...).
 */
class RegionsOfInterestPruneConfig final {
   public:
    static RegionsOfInterestPruneConfig create(uint32_t maxRowSpans, uint32_t maxColSpans) {
        if (!isValidMaxRowSpans(maxRowSpans)) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: maxRowSpans must be > 0");
        }
        if (!isValidMaxColSpans(maxColSpans)) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: maxColSpans must be > 0");
        }
        return {maxRowSpans, maxColSpans};
    }

    static bool isValidMaxRowSpans(uint32_t maxRowSpans) { return maxRowSpans > 0; }
    static bool isValidMaxColSpans(uint32_t maxColSpans) { return maxColSpans > 0; }

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
 *    1. Find contiguous non-zero spans in rowSums / colSums; accumulate per-span sums.
 *    2. Pre-filter to top maxRowSpans / maxColSpans spans by accumulator value.
 *    3. Cross-product of filtered spans → bounding boxes with count = min(R[k], C[l]).
 *    4. Sort by count descending (ties broken by distance to image center, ascending, then by
 *       window area, ascending), return top ROI_CANDIDATES_MAX entries.
 */
class RegionsOfInterestPruneAlgorithm final {
   public:
    explicit RegionsOfInterestPruneAlgorithm(const RegionsOfInterestPruneConfig& config);

    void setConfig(const RegionsOfInterestPruneConfig& config);

    RoiCandidates update(const uint16_t* rowSums, uint32_t numRows, const uint16_t* colSums, uint32_t numCols) const;

   private:
    RegionsOfInterestPruneConfig cfg;

    using Span = std::pair<uint32_t, uint32_t>;

    //!< Fixed-capacity span list: data[0..count) are the valid entries.
    struct SpanArray {
        uint32_t count{};
        std::array<Span, MAX_SPANS> data{};
    };
    //!< Fixed-capacity value/index list: data[0..count) are the valid entries.
    struct AccumArray {
        uint32_t count{};
        std::array<uint32_t, MAX_SPANS> data{};
    };
    //!< Fixed-capacity candidate list: data[0..count) are the valid entries.
    struct CandidateArray {
        uint32_t count{};
        std::array<RoiCandidateEntry, MAX_RAW_CANDIDATES> data{};
    };

    // Step 1: find contiguous non-zero spans and accumulate per-span sums.
    static std::pair<SpanArray, AccumArray> findSpans(const uint16_t* s, uint32_t n);

    // Step 2: return indices of the top-keep entries in vals (by descending value).
    static AccumArray topIndices(const AccumArray& vals, uint32_t keep);

    // Step 3: cross-product of filtered row/col spans → bounding-box candidates.
    static CandidateArray buildCandidates(const SpanArray& rowSpans,
                                          const AccumArray& R,
                                          const AccumArray& rowIdx,
                                          const SpanArray& colSpans,
                                          const AccumArray& C,
                                          const AccumArray& colIdx);

    // Step 4: sort candidates by count descending (ties broken by distance to image center, then
    // window area), truncate, and pack the result.
    static RoiCandidates packOutput(CandidateArray candidates, uint32_t numRows, uint32_t numCols);
};

#endif
