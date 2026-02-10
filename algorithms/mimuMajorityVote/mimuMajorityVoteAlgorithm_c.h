/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_MIMUMAJORITYVOTEALGORITHM_C_H
#define F32XIMERA_MIMUMAJORITYVOTEALGORITHM_C_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MimuMajorityVoteAlgorithm instance.
 */
typedef struct MimuMajorityVoteAlgorithm MimuMajorityVoteAlgorithm;

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3f).
 */
typedef struct {
    float data[3];
} Vector3f_c;

/**
 * @brief C-compatible input angular velocity from a single IMU.
 */
typedef struct {
    Vector3f_c angVelBody; /*!< [rad/s] Angular velocity in body frame */
} MimuInput_c;

/**
 * @brief C-compatible output from the MIMU majority vote algorithm.
 */
typedef struct {
    Vector3f_c avgAngVelBody; /*!< [rad/s] Averaged angular velocity in body frame */
    bool faultDetected;       /*!< Whether a MIMU fault was detected */
    int32_t mimuIndexFaulted; /*!< Index of faulted MIMU (-1 if no fault) */
} MimuMajorityVoteOutput_c;

/**
 * @brief Get the maximum IMU vehicle count constant for validation.
 * @return The maximum IMU count (MAX_IMU_VEH_COUNT = 4).
 */
uint32_t MimuMajorityVoteAlgorithm_getMaxImuVehCount(void);

/**
 * @brief Construct a new MimuMajorityVoteAlgorithm instance.
 * @return Pointer to a new MimuMajorityVoteAlgorithm (must be destroyed).
 */
MimuMajorityVoteAlgorithm* MimuMajorityVoteAlgorithm_create(void);

/**
 * @brief Destroy a previously created MimuMajorityVoteAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithm* self);

/**
 * @brief Run the update step.
 * @param self         Pointer to the instance.
 * @param imuInputs    Pointer to array of IMU input structs (max MAX_IMU_VEH_COUNT elements).
 * @param numberOfImus Number of valid IMU inputs in the array.
 * @return MimuMajorityVoteOutput_c  The computed output.
 */
MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithm* self,
                                                            const MimuInput_c* imuInputs,
                                                            uint32_t numberOfImus);

/**
 * @brief Set the omega threshold.
 * @param self  Pointer to the instance.
 * @param value The new omega threshold value.
 */
void MimuMajorityVoteAlgorithm_setOmegaThreshold(MimuMajorityVoteAlgorithm* self, float value);

/**
 * @brief Get the current omega threshold.
 * @param self Pointer to the instance.
 * @return float  The current omega threshold.
 */
float MimuMajorityVoteAlgorithm_getOmegaThreshold(const MimuMajorityVoteAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_MIMUMAJORITYVOTEALGORITHM_C_H
