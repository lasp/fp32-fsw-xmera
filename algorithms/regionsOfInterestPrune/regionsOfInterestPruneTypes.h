#ifndef F32XMERA_REGIONS_OF_INTEREST_PRUNE_TYPES_H
#define F32XMERA_REGIONS_OF_INTEREST_PRUNE_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of candidates retained/published at the C boundary. Must match
   ROI_CANDIDATES_MAX in regionsOfInterestPruneAlgorithm.h (enforced by a static_assert in the
   C shim). */
#define REGIONS_OF_INTEREST_PRUNE_ROI_CANDIDATES_MAX 16

/**
 * @brief Plain-old-data mirror of the C++ RoiCandidateEntry.
 *  - row, col       [px] top-left corner of the bounding box
 *  - height, width  [px] bounding box size
 *  - count          [-]  estimated above-threshold pixel count
 */
typedef struct {
    uint32_t row;
    uint32_t col;
    uint32_t height;
    uint32_t width;
    uint32_t count;
} RoiCandidateEntry_c;

/**
 * @brief Plain-old-data mirror of the C++ RoiCandidates.
 *  - numCandidates carries the number of valid entries in candidates
 *  - candidates[i] for i < numCandidates is sorted by count descending; trailing slots are zero
 */
typedef struct {
    uint32_t numCandidates;
    RoiCandidateEntry_c candidates[REGIONS_OF_INTEREST_PRUNE_ROI_CANDIDATES_MAX];
} RoiCandidates_c;

/**
 * @brief Plain-old-data mirror of the C++ RegionsOfInterestPruneConfig.
 *  - maxRowSpans, maxColSpans: pre-filter span counts kept before the cross-product; both must
 *    be > 0.
 */
typedef struct {
    uint32_t maxRowSpans;
    uint32_t maxColSpans;
} RegionsOfInterestPruneConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_REGIONS_OF_INTEREST_PRUNE_TYPES_H
