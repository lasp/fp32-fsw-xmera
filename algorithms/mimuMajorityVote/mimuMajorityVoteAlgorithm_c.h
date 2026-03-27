#ifndef F32XIMERA_MIMUMAJORITYVOTEALGORITHM_C_H
#define F32XIMERA_MIMUMAJORITYVOTEALGORITHM_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MimuMajorityVoteAlgorithm instance.
 */
typedef struct MimuMajorityVoteAlgorithm MimuMajorityVoteAlgorithm;

/**
 * @brief Maximum number of IMU vehicles.
 */
#define MAX_IMU_VEH_COUNT_C 4

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3f).
 */
typedef struct {
    float data[3];
} Vector3f_c;

/**
 * @brief POD output from the MIMU majority vote algorithm.
 *
 * Layout must match the Adamant Mimu_Majority_Vote_Output packed record:
 *   Avg_Ang_Vel_Body : Packed_F32x3  (3 floats)
 *   Fault_Detected   : Unsigned_8    (0 = false, nonzero = true)
 *   Mimu_Index_Faulted : Integer_32  (-1 if no fault)
 */
typedef struct {
    Vector3f_c avgAngVelBody; /*!< [rad/s] Averaged angular velocity in body frame */
    uint8_t faultDetected;    /*!< Whether a MIMU fault was detected */
    int32_t mimuIndexFaulted; /*!< Index of faulted MIMU (-1 if no fault) */
} MimuMajorityVoteOutput_c;

/**
 * @brief Get the MAX_IMU_VEH_COUNT constant for Ada validation.
 * @return The maximum IMU vehicle count (MAX_IMU_VEH_COUNT_C).
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
 * @brief Run the majority vote update step.
 * @param self         Pointer to the instance.
 * @param imuInputs    Pointer to array of IMU angular velocity 3-vectors (MAX_IMU_VEH_COUNT_C elements).
 * @param numberOfImus Number of valid IMU inputs in the array.
 * @return MimuMajorityVoteOutput_c  The computed majority vote output.
 */
MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithm* self,
                                                          const Vector3f_c* imuInputs,
                                                          uint32_t numberOfImus);

/**
 * @brief Set the omega threshold for fault detection.
 * @param self  Pointer to the instance.
 * @param value The new omega threshold value [rad/s].
 */
void MimuMajorityVoteAlgorithm_setOmegaThreshold(MimuMajorityVoteAlgorithm* self, float value);

/**
 * @brief Get the current omega threshold.
 * @param self Pointer to the instance.
 * @return float  The current omega threshold [rad/s].
 */
float MimuMajorityVoteAlgorithm_getOmegaThreshold(const MimuMajorityVoteAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_MIMUMAJORITYVOTEALGORITHM_C_H
