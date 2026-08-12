#ifndef F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_C_H
#define F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_C_H

#include "regionsOfInterestPruneTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ RegionsOfInterestPruneAlgorithm instance.
 */
typedef struct RegionsOfInterestPruneAlgorithmHandle RegionsOfInterestPruneAlgorithmHandle;

/**
 * @brief Get the REGIONS_OF_INTEREST_PRUNE_ROI_CANDIDATES_MAX constant for Ada validation.
 * @return The maximum number of candidates returned at the C boundary.
 */
uint32_t RegionsOfInterestPruneAlgorithm_getMaxCandidatesCount(void);

/**
 * @brief Construct a new RegionsOfInterestPruneAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new RegionsOfInterestPruneAlgorithm (must be destroyed).
 */
RegionsOfInterestPruneAlgorithmHandle* RegionsOfInterestPruneAlgorithm_create(
    const RegionsOfInterestPruneConfig_c* config);

/**
 * @brief Destroy a previously created RegionsOfInterestPruneAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void RegionsOfInterestPruneAlgorithm_destroy(RegionsOfInterestPruneAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void RegionsOfInterestPruneAlgorithm_setConfig(RegionsOfInterestPruneAlgorithmHandle* self,
                                               const RegionsOfInterestPruneConfig_c* config);

/**
 * @brief Find and rank bounding-box candidates from the row/column above-threshold pixel sums.
 * @param self     Pointer to the instance.
 * @param rowSums  Pointer to the per-row above-threshold pixel sums (length numRows).
 * @param numRows  Number of rows in rowSums.
 * @param colSums  Pointer to the per-column above-threshold pixel sums (length numCols).
 * @param numCols  Number of columns in colSums.
 * @return The ranked candidates, sorted by estimated pixel count descending.
 */
RoiCandidates_c RegionsOfInterestPruneAlgorithm_update(const RegionsOfInterestPruneAlgorithmHandle* self,
                                                       const uint16_t* rowSums,
                                                       uint32_t numRows,
                                                       const uint16_t* colSums,
                                                       uint32_t numCols);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_REGIONS_OF_INTEREST_PRUNE_ALGORITHM_C_H
