#ifndef F32XMERA_MIMU_MAJORITY_VOTE_TYPES_H
#define F32XMERA_MIMU_MAJORITY_VOTE_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Number of IMUs.
 */
#define MIMU_COUNT_C 3

/**
 * @brief Plain-old-data mirror of the C++ MimuMajorityVoteConfig.
 *
 *  - omegaThreshold must be finite and > 0
 *  - faultPersistenceLimit must be > 0
 */
typedef struct {
    float omegaThreshold;           /*!< [rad/s] threshold to determine if a MIMU is faulted */
    uint32_t faultPersistenceLimit; /*!< [-] consecutive faults needed to trigger faultDetected */
} MimuMajorityVoteConfig_c;

/**
 * @brief Sized array of 3-vectors.
 */
typedef struct {
    Vector3f_c vec[MIMU_COUNT_C];
} Vector3fArray3_c;

/**
 * @brief POD output from the MIMU majority vote algorithm.
 *
 * Layout must match the Adamant Mimu_Majority_Vote_Output packed record:
 *   Avg_Ang_Vel_Body : Packed_F32x3  (3 floats)
 *   Fault_Detected   : Unsigned_8    (0 = false, nonzero = true)
 *   Mimu_Index_Faulted : Integer_32  (-1 if no fault)
 */
typedef struct {
    Vector3f_c avgOmega_BN_B; /*!< [rad/s] Averaged angular velocity in body frame */
    uint8_t faultDetected;    /*!< Whether a MIMU fault was detected */
    int32_t mimuIndexFaulted; /*!< Index of faulted MIMU (-1 if no fault) */
} MimuMajorityVoteOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MIMU_MAJORITY_VOTE_TYPES_H
