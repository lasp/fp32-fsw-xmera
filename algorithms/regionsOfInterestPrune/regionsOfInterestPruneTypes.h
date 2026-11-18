#ifndef F32XMERA_REGIONS_OF_INTEREST_PRUNE_TYPES_H
#define F32XMERA_REGIONS_OF_INTEREST_PRUNE_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Capacity of RoiCandidates_c.candidates; update() fills at most ROI_CANDIDATES_OUT_MAX (2) of them. Must
   match ROI_CANDIDATES_MAX in regionsOfInterestPruneAlgorithm.h (enforced by a static_assert in the
   C shim). */
#define REGIONS_OF_INTEREST_PRUNE_ROI_CANDIDATES_MAX 4

/**
 * @brief Plain-old-data mirror of the C++ RoiCandidateEntry.
 *  - row, col       [px] top-left corner of the bounding box
 *  - height, width  [px] bounding box size
 *  - litPixelCount  [-]  sum of litPixelCount over the bin-grid tiles the box overlaps
 *  - intensity      [-]  estimated intensity, min(row-span intensity sum, col-span intensity sum)
 *  - scaledIntensity [-] intensity / (width * height)^(alphaAreaPruneScaling - 1); 0 when the cost is bypassed
 */
typedef struct {
    uint32_t row;
    uint32_t col;
    uint32_t height;
    uint32_t width;
    uint32_t litPixelCount;
    uint64_t intensity;
    float scaledIntensity;
} RoiCandidateEntry_c;

/**
 * @brief Plain-old-data mirror of the C++ RoiCandidates.
 *  - numCandidates carries the number of valid entries in candidates
 *  - candidates[i] for i < numCandidates is sorted by distance to the reference point ascending, ties broken
 *    by scaledIntensity descending (kept in formation order when the cost function is bypassed); trailing
 *    slots are zero
 */
typedef struct {
    uint32_t numCandidates;
    RoiCandidateEntry_c candidates[REGIONS_OF_INTEREST_PRUNE_ROI_CANDIDATES_MAX];
} RoiCandidates_c;

/**
 * @brief Plain-old-data mirror of the C++ RoiBinGrid.
 *  - regionSize     [px] tile edge length
 *  - numRegionRows  [-]  number of tile rows, = ceil(numRows / regionSize)
 *  - numRegionCols  [-]  number of tile columns, = ceil(numCols / regionSize)
 *  - litCounts      [-]  above-threshold pixel count per tile, row-major (numLitCounts entries)
 *  - numLitCounts   [-]  must equal numRegionRows * numRegionCols, otherwise no candidates are returned
 */
typedef struct {
    uint32_t regionSize;
    uint32_t numRegionRows;
    uint32_t numRegionCols;
    const uint32_t* litCounts;
    uint32_t numLitCounts;
} RoiBinGrid_c;

/**
 * @brief POD representation of a signed integer 2-vector (Eigen::Vector2i): x is the column, y the row [px].
 */
typedef struct {
    int32_t x;
    int32_t y;
} Vector2i_c;

/**
 * @brief Plain-old-data mirror of the C++ RegionsOfInterestPruneConfig.
 *  - centralPoint [px]: absolute reference point (x = column, y = row). Must satisfy
 *    0 <= centralPoint.x <= MAX_IMAGE_COLS (4096) and 0 <= centralPoint.y <= MAX_IMAGE_ROWS (3000)
 *    (regionsOfInterestPruneAlgorithm.h).
 *  - alphaAreaPruneScaling [-]: at most 2, not NaN. Any negative value bypasses the cost function (rank by
 *    distance only); otherwise the area exponent in scaledIntensity is alphaAreaPruneScaling - 1.
 */
typedef struct {
    Vector2i_c centralPoint;
    float alphaAreaPruneScaling;
} RegionsOfInterestPruneConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_REGIONS_OF_INTEREST_PRUNE_TYPES_H
