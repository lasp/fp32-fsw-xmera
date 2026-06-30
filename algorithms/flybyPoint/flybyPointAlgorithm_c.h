#ifndef F32XMERA_FLYBY_POINT_ALGORITHM_C_H
#define F32XMERA_FLYBY_POINT_ALGORITHM_C_H

#include "flybyPointTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ FlybyPointAlgorithm instance.
 */
typedef struct FlybyPointAlgorithmHandle FlybyPointAlgorithmHandle;

/**
 * @brief POD representation of a double-precision 3-vector (Eigen::Vector3d).
 */
typedef struct {
    double data[3];
} Vector3d_c;

/**
 * @brief Construct a new FlybyPointAlgorithm instance with the supplied configuration.
 * @param config Pointer to the configuration (validated; throws on invalid input).
 * @return Pointer to a new FlybyPointAlgorithm (must be destroyed).
 */
FlybyPointAlgorithmHandle* FlybyPointAlgorithm_create(const FlybyPointConfig_c* config);

/**
 * @brief Destroy a previously created FlybyPointAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void FlybyPointAlgorithm_destroy(FlybyPointAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the new configuration (validated; throws on invalid input).
 */
void FlybyPointAlgorithm_setConfig(FlybyPointAlgorithmHandle* self, const FlybyPointConfig_c* config);

/**
 * @brief Reset the algorithm state.
 * @param self Pointer to the instance.
 */
void FlybyPointAlgorithm_reset(FlybyPointAlgorithmHandle* self);

/**
 * @brief Compute the flyby-point reference attitude guidance for the current time.
 * @param self             Pointer to the instance.
 * @param currentSimNanos  Current simulation time [ns].
 * @param r_BN_N           Spacecraft position relative to the body in inertial frame [m].
 * @param v_BN_N           Spacecraft velocity relative to the body in inertial frame [m/s].
 * @return AttGuideOutput_c  Reference attitude, rate, acceleration, and validity flags.
 */
AttGuideOutput_c FlybyPointAlgorithm_updateState(FlybyPointAlgorithmHandle* self,
                                                 uint64_t currentSimNanos,
                                                 Vector3d_c r_BN_N,
                                                 Vector3d_c v_BN_N);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_FLYBY_POINT_ALGORITHM_C_H
