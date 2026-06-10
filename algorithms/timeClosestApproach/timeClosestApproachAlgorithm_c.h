#ifndef F32XMERA_TIME_CA_ALGORITHM_C_H
#define F32XMERA_TIME_CA_ALGORITHM_C_H

#include "timeClosestApproachTypes.h"
#include <stdint.h>

#define TIME_CA_MAX_FILTER_STATES 6

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ TimeClosestApproachAlgorithm instance.
 */
typedef struct TimeClosestApproachAlgorithmHandle TimeClosestApproachAlgorithmHandle;

/**
 * @brief Fixed-size 6×6 filter covariance matrix (column-major).
 */
typedef struct {
    float data[TIME_CA_MAX_FILTER_STATES * TIME_CA_MAX_FILTER_STATES]; /*!< 6×6 covariance matrix, column-major */
} FilterCovariance_c;

/**
 * @brief Get the TIME_CA_MAX_FILTER_STATES constant for Ada elaboration-time validation.
 * @return The value of TIME_CA_MAX_FILTER_STATES.
 */
uint32_t TimeClosestApproachAlgorithm_getMaxFilterStates(void);

/**
 * @brief Construct a new TimeClosestApproachAlgorithm instance.
 * @return Pointer to a new TimeClosestApproachAlgorithm handle (must be destroyed with
 *         TimeClosestApproachAlgorithm_destroy).
 */
TimeClosestApproachAlgorithmHandle* TimeClosestApproachAlgorithm_create(void);

/**
 * @brief Destroy a previously created TimeClosestApproachAlgorithm.
 * @param self Pointer to the handle to destroy.
 */
void TimeClosestApproachAlgorithm_destroy(TimeClosestApproachAlgorithmHandle* self);

/**
 * @brief Compute the time of closest approach estimate and its standard deviation.
 * @param self       Pointer to the handle (read-only).
 * @param r_BN_N     Spacecraft position in inertial coordinates (m).
 * @param v_BN_N     Spacecraft velocity in inertial coordinates (m/s).
 * @param covariance Pointer to the bounded filter covariance.
 * @return TimeClosestApproachOutput_c  The computed TCA estimate and standard deviation.
 */
TimeClosestApproachOutput_c TimeClosestApproachAlgorithm_update(const TimeClosestApproachAlgorithmHandle* self,
                                                                const Vector3d_c* r_BN_N,
                                                                const Vector3d_c* v_BN_N,
                                                                const FilterCovariance_c* covariance);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_TIME_CA_ALGORITHM_C_H
