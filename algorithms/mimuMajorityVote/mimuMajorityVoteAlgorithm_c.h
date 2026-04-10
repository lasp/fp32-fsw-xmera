#ifndef F32XMERA_MIMUMAJORITYVOTEALGORITHM_C_H
#define F32XMERA_MIMUMAJORITYVOTEALGORITHM_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MimuMajorityVoteAlgorithm instance.
 */
typedef struct MimuMajorityVoteAlgorithm MimuMajorityVoteAlgorithm;

/**
 * @brief Number of IMUs.
 */
#define MIMU_COUNT_C 3

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3f).
 */
typedef struct {
    float data[3];
} Vector3f_c;

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

/**
 * @brief Get the kMimuCount constant for Ada validation.
 * @return The IMU count (MIMU_COUNT_C).
 */
uint32_t MimuMajorityVoteAlgorithm_getMimuCount(void);

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
 * @brief Reset fault persistence counters to zero.
 * @param self Pointer to the instance.
 */
void MimuMajorityVoteAlgorithm_reset(MimuMajorityVoteAlgorithm* self);

/**
 * @brief Run the majority vote update step.
 * @param self      Pointer to the instance.
 * @param imuOmegas_BN_B IMU angular velocity 3-vectors.
 * @return MimuMajorityVoteOutput_c  The computed majority vote output.
 */
MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithm* self,
                                                          const Vector3fArray3_c* imuOmegas_BN_B);

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

/**
 * @brief Set the fault persistence limit.
 * @param self  Pointer to the instance.
 * @param value The new fault persistence limit.
 */
void MimuMajorityVoteAlgorithm_setFaultPersistenceLimit(MimuMajorityVoteAlgorithm* self, uint32_t value);

/**
 * @brief Get the current fault persistence limit.
 * @param self Pointer to the instance.
 * @return uint32_t  The current fault persistence limit.
 */
uint32_t MimuMajorityVoteAlgorithm_getFaultPersistenceLimit(const MimuMajorityVoteAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MIMUMAJORITYVOTEALGORITHM_C_H
