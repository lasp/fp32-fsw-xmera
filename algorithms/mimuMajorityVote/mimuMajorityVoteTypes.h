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
 * @brief Sized array of 3-vectors.
 */
typedef struct {
    Vector3f_c vec[MIMU_COUNT_C];
} Vector3fArray3_c;

/**
 * @brief POD result of one majority vote (gyro or accel), mirroring the C++ MimuVoteResult.
 *
 * Layout must match the Adamant Mimu_Vote_Result packed record:
 *   Average            : Packed_F32x3             (3 floats)
 *   Fault_Detected     : Unsigned_8               (0 = false, nonzero = true)
 *   Imu_Difference_Mag : array of F32             (MIMU_COUNT_C floats)
 *   Imu_Valid          : array of Unsigned_8      (MIMU_COUNT_C, 0/1 per IMU)
 */
typedef struct {
    Vector3f_c average;                   /*!< Averaged measurement (outlier-excluded once a fault persists) */
    uint8_t faultDetected;                /*!< Whether an IMU was rejected for this quantity (0/1) */
    float imuDifferenceMag[MIMU_COUNT_C]; /*!< Each IMU's difference magnitude from the 3-IMU average */
    uint8_t imuValid[MIMU_COUNT_C];       /*!< Whether each IMU is valid for this quantity (0/1) */
} MimuVoteResult_c;

/**
 * @brief POD output from the MIMU majority vote algorithm: independent gyro and accel votes.
 *
 * Layout must match the Adamant Mimu_Majority_Vote_Output packed record (gyro result, then accel).
 */
typedef struct {
    MimuVoteResult_c gyro;  /*!< [rad/s] Angular-velocity vote */
    MimuVoteResult_c accel; /*!< [m/s^2] Acceleration vote */
} MimuMajorityVoteOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MIMU_MAJORITY_VOTE_TYPES_H
