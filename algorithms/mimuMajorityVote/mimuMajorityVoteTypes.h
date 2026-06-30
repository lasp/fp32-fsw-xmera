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
 *  - omegaThreshold / accelThreshold must be finite and > 0
 *  - gyroFaultPersistenceLimit / accelFaultPersistenceLimit must be > 0
 */
typedef struct {
    float omegaThreshold;                /*!< [rad/s] gyro threshold to determine if a MIMU is faulted */
    uint32_t gyroFaultPersistenceLimit;  /*!< [-] consecutive gyro faults needed to trigger gyroFaultDetected */
    float accelThreshold;                /*!< [m/s^2] accel threshold to determine if a MIMU is faulted */
    uint32_t accelFaultPersistenceLimit; /*!< [-] consecutive accel faults needed to trigger accelFaultDetected */
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
 *   Avg_Ang_Vel_Body       : Packed_F32x3  (3 floats)
 *   Gyro_Fault_Detected    : Unsigned_8    (0 = false, nonzero = true)
 *   Gyro_Mimu_Index_Faulted: Integer_32    (-1 if no fault)
 *   Avg_Accel_Body         : Packed_F32x3  (3 floats)
 *   Accel_Fault_Detected   : Unsigned_8    (0 = false, nonzero = true)
 *   Accel_Mimu_Index_Faulted: Integer_32   (-1 if no fault)
 */
typedef struct {
    Vector3f_c avgOmega_BN_B;      /*!< [rad/s] Averaged angular velocity in body frame */
    uint8_t gyroFaultDetected;     /*!< Whether a gyro MIMU fault was detected */
    int32_t gyroMimuIndexFaulted;  /*!< Index of gyro-faulted MIMU (-1 if no fault) */
    Vector3f_c avgAccel_B;         /*!< [m/s^2] Averaged apparent acceleration in body frame */
    uint8_t accelFaultDetected;    /*!< Whether an accel MIMU fault was detected */
    int32_t accelMimuIndexFaulted; /*!< Index of accel-faulted MIMU (-1 if no fault) */
} MimuMajorityVoteOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MIMU_MAJORITY_VOTE_TYPES_H
