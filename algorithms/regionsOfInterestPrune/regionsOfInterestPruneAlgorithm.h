#ifndef F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H
#define F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <span>

static constexpr uint32_t ROW_SPANS_MAX = 2;  //!< Number of row spans kept (closest to the reference row)
static constexpr uint32_t COL_SPANS_MAX = 2;  //!< Number of col spans kept (closest to the reference col)

//!< Capacity of RoiCandidates: the full row-span x col-span cross-product, so no candidate is ever dropped for
//!< lack of room. update() returns at most ROI_CANDIDATES_OUT_MAX of them.
static constexpr uint32_t ROI_CANDIDATES_MAX = ROW_SPANS_MAX * COL_SPANS_MAX;

//!< Number of ranked candidates update() returns: the list is cut to the first ROI_CANDIDATES_OUT_MAX after ranking.
static constexpr uint32_t ROI_CANDIDATES_OUT_MAX = 2;
static_assert(ROI_CANDIDATES_OUT_MAX <= ROI_CANDIDATES_MAX, "cannot return more candidates than are formed");

//!< [-] Ghost-box removal (Step 2b) drops the worse span pairing only when the better pairing's mismatch is strictly
//!< below this fraction of the worse one; otherwise the projections are ambiguous and all boxes are kept.
static constexpr float GHOST_PAIRING_MARGIN = 0.5F;

static constexpr uint32_t MAX_IMAGE_ROWS = 3000;  //!< [px] Full camera image height
static constexpr uint32_t MAX_IMAGE_COLS = 4096;  //!< [px] Full camera image width

//!< [px] Default reference column (image center). Signed, like the Eigen::Vector2i centralPoint it initializes.
static constexpr int32_t DEFAULT_CENTRAL_POINT_X = static_cast<int32_t>(MAX_IMAGE_COLS / 2U);
//!< [px] Default reference row (image center).
static constexpr int32_t DEFAULT_CENTRAL_POINT_Y = static_cast<int32_t>(MAX_IMAGE_ROWS / 2U);

//!< alphaAreaPruneScaling: any value below 0 bypasses the cost function (rank by distance only); a value in [0, 2]
//!< uses the area exponent (alpha - 1), which lies in [-1, 1]. Values above 2 and NaN are rejected.
static constexpr float MAX_ALPHA_AREA_PRUNE_SCALING = 2.0F;      //!< [-] Highest accepted value (exponent +1)
static constexpr float DEFAULT_ALPHA_AREA_PRUNE_SCALING = 1.0F;  //!< [-] Default: exponent 0, area ignored

/*! @brief Bounding-box candidate: corner, size, lit pixel count, intensity and scaled intensity. */
struct RoiCandidateEntry {
    uint32_t row{};            //!< [px] Top-left row of the bounding box
    uint32_t col{};            //!< [px] Top-left column of the bounding box
    uint32_t height{};         //!< [px] Height of the bounding box
    uint32_t width{};          //!< [px] Width of the bounding box
    uint32_t litPixelCount{};  //!< [-] Sum of litPixelCount over the bin-grid tiles the box overlaps
    uint64_t intensity{};      //!< [-] Estimated intensity: min(row-span intensity sum, col-span intensity sum)
    float scaledIntensity{};   //!< [-] intensity / (width * height)^(alphaAreaPruneScaling - 1); 0 in bypass
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

/*! @brief Fixed-capacity candidate list: candidates[0..numCandidates) are valid.
 *
 *  Used through the whole pipeline (built, ghost-filtered, tile-filtered, scaled, ranked in place, then cut).
 *  update() returns at most ROI_CANDIDATES_OUT_MAX, with the cut slots zeroed, sorted by distance to the
 *  reference point ascending, ties broken by scaledIntensity descending (in bypass, alpha < 0, ties keep the
 *  order the boxes were formed in).
 */
struct RoiCandidates {
    uint32_t numCandidates{};                                        //!< [-] Number of valid entries
    std::array<RoiCandidateEntry, ROI_CANDIDATES_MAX> candidates{};  //!< [-] Candidate boxes
};

/*! @brief Validated configuration for the regions-of-interest pruning algorithm.
 *
 *  The reference point used for span selection and candidate ranking is the absolute pixel
 *  position centralPoint: centralPoint.x() is the column, centralPoint.y() the row, in the same coordinates
 *  as the published cornerX/cornerY. Defaults to the full-image center
 *  (DEFAULT_CENTRAL_POINT_X, DEFAULT_CENTRAL_POINT_Y).
 *
 *  alphaAreaPruneScaling sets the tie-break between boxes equally close to the reference point:
 *    - below 0: the cost function is bypassed. Boxes are ranked by distance alone, equally close boxes keep
 *      the order they were formed in (closer row span first, then closer col span), and scaledIntensity is 0.
 *    - in [0, 2]: scaledIntensity = intensity / area^(alpha - 1), the exponent alpha - 1 lying in [-1, 1].
 *      1 uses intensity alone, 2 divides by the area (favors smaller boxes), 0 multiplies by it (larger).
 *
 *  An instance can only exist if centralPoint.x() is in [0, MAX_IMAGE_COLS], centralPoint.y() is in
 *  [0, MAX_IMAGE_ROWS] and
 *  alphaAreaPruneScaling is at most MAX_ALPHA_AREA_PRUNE_SCALING and not NaN (any negative value is accepted).
 *  Construct via RegionsOfInterestPruneConfig::create(...).
 */
class RegionsOfInterestPruneConfig final {
   public:
    static RegionsOfInterestPruneConfig create(const Eigen::Vector2i& centralPoint, float alphaAreaPruneScaling) {
        if (!isValidCentralPointX(centralPoint.x())) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: centralPoint.x must be in [0, MAX_IMAGE_COLS]");
        }
        if (!isValidCentralPointY(centralPoint.y())) {
            FSW_THROW_INVALID_ARGUMENT("regionsOfInterestPrune: centralPoint.y must be in [0, MAX_IMAGE_ROWS]");
        }
        if (!isValidAlphaAreaPruneScaling(alphaAreaPruneScaling)) {
            FSW_THROW_INVALID_ARGUMENT(
                "regionsOfInterestPrune: alphaAreaPruneScaling must be <= 2 (negative bypasses the cost function)");
        }
        return {centralPoint, alphaAreaPruneScaling};
    }

    // Eigen::Vector2i is signed, so a negative coordinate is possible and rejected here.
    static bool isValidCentralPointX(int32_t x) { return x >= 0 && static_cast<uint32_t>(x) <= MAX_IMAGE_COLS; }
    static bool isValidCentralPointY(int32_t y) { return y >= 0 && static_cast<uint32_t>(y) <= MAX_IMAGE_ROWS; }
    // NaN fails the comparison and +infinity is above the maximum; any negative value, -infinity included, is a
    // valid request to bypass the cost function.
    static bool isValidAlphaAreaPruneScaling(float alpha) { return alpha <= MAX_ALPHA_AREA_PRUNE_SCALING; }

    Eigen::Vector2i getCentralPoint() const { return centralPoint; }
    float getAlphaAreaPruneScaling() const { return alphaAreaPruneScaling; }

   private:
    // Private; only reached through the validated create().
    RegionsOfInterestPruneConfig(const Eigen::Vector2i& centralPoint, float alphaAreaPruneScaling)
        : centralPoint(centralPoint), alphaAreaPruneScaling(alphaAreaPruneScaling) {}

    Eigen::Vector2i centralPoint;  //!< [px] Reference point (x() = column, y() = row)
    float alphaAreaPruneScaling;   //!< [-] Below 0: bypass; otherwise the area exponent is alpha - 1
};

/*! @brief algorithm for the regions-of-interest pruning stage.
 *
 *  Accepts the row/col above-threshold pixel sums, the row/col intensity sums and the bin grid of one image,
 *  and returns a RoiCandidates.  Pipeline stages:
 *    1. Single pass over rowSums / colSums: find contiguous non-zero spans, accumulate per-span
 *       pixel counts and intensities, and keep the ROW_SPANS_MAX / COL_SPANS_MAX spans whose
 *       midpoints are closest to the reference row / col.
 *    2. Cross-product of the kept spans → bounding boxes with intensity = min(row, col span intensity).
 *    2b. With 2 row and 2 col spans, drop the two ghost boxes of the span pairing whose pixel counts
 *       and intensities disagree (see removeGhostPairs).
 *    3. Drop boxes that overlap no lit bin-grid tile.
 *    4. scaledIntensity = intensity / area^(alpha - 1); skipped (0) when alpha < 0 (bypass).
 *    5. Sort by distance to the reference point ascending; ties by scaledIntensity descending, or, in
 *       bypass, by the order the boxes were formed in.
 *    6. Return the first ROI_CANDIDATES_OUT_MAX.
 */
class RegionsOfInterestPruneAlgorithm final {
   public:
    explicit RegionsOfInterestPruneAlgorithm(const RegionsOfInterestPruneConfig& config);

    void setConfig(const RegionsOfInterestPruneConfig& config);

    // imageSize: x() = numCols, y() = numRows. Row arrays have numRows entries, column arrays numCols.
    RoiCandidates update(const Eigen::Vector2<uint32_t>& imageSize,
                         const uint16_t* rowSums,
                         const uint32_t* rowIntensities,
                         const uint16_t* colSums,
                         const uint32_t* colIntensities,
                         const RoiBinGrid& binGrid) const;

   private:
    RegionsOfInterestPruneConfig cfg;

    //!< Contiguous non-zero run in a 1-D count array.
    struct SpanEntry {
        uint32_t start{};      //!< [px] First index of the span
        uint32_t length{};     //!< [px] Number of entries in the span
        uint64_t litPixels{};  //!< [-] Accumulated above-threshold pixel count over the span
        uint64_t intensity{};  //!< [-] Accumulated intensity over the span
        float distance{};      //!< [px] |span midpoint - reference coordinate|
    };
    //!< Fixed-capacity closest-K span list: data[0..count) are valid, sorted by distance ascending.
    template <uint32_t Capacity>
    struct ClosestSpans {
        uint32_t count{};
        std::array<SpanEntry, Capacity> data{};
    };
    using RowClosestSpans = ClosestSpans<ROW_SPANS_MAX>;
    using ColClosestSpans = ClosestSpans<COL_SPANS_MAX>;

    // Step 1: find contiguous non-zero spans and keep the Capacity closest to `reference` in a single pass,
    // summing each span's counts and intensities.
    template <uint32_t Capacity>
    static ClosestSpans<Capacity> findClosestSpans(std::span<const uint16_t> counts,
                                                   std::span<const uint32_t> intensities,
                                                   float reference);

    // Step 2: cross-product of closest row/col spans → bounding-box candidates.
    static RoiCandidates buildCandidates(const RowClosestSpans& rowSpans, const ColClosestSpans& colSpans);

    // Step 2b: with 2 row x 2 col spans, drop the ghost boxes of the span pairing that the projections reject.
    static RoiCandidates removeGhostPairs(const RoiCandidates& candidates,
                                          const RowClosestSpans& rowSpans,
                                          const ColClosestSpans& colSpans);

    // Step 3: drop candidates that overlap no lit tile of the bin grid; set litPixelCount from the covered tiles.
    static RoiCandidates filterByLitTiles(const RoiCandidates& candidates, const RoiBinGrid& binGrid);
    static bool isBinGridUsable(const RoiBinGrid& binGrid, const Eigen::Vector2<uint32_t>& imageSize);
    static uint32_t sumLitTiles(const RoiCandidateEntry& candidate, const RoiBinGrid& binGrid);

    // Step 4: set each candidate's scaledIntensity = intensity / area^(alpha - 1).
    static void scaleIntensityByArea(RoiCandidates& candidates, float alphaAreaPruneScaling);
    static bool isCostFunctionBypassed(float alphaAreaPruneScaling) { return alphaAreaPruneScaling < 0.0F; }

    // Step 5: sort candidates in place by distance to the reference point ascending; ties broken by scaledIntensity,
    // or kept in formation order when the cost function is bypassed.
    static void rankCandidates(RoiCandidates& candidates, float referenceRow, float referenceCol, bool bypassCost);
};

#endif
