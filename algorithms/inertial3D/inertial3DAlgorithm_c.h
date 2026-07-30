#ifndef F32XMERA_INERTIAL3DALGORITHM_C_H
#define F32XMERA_INERTIAL3DALGORITHM_C_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ Inertial3DAlgorithm instance.
 */
typedef struct Inertial3DAlgorithmHandle Inertial3DAlgorithmHandle;

/**
 * @brief Construct a new Inertial3DAlgorithm instance from the supplied configuration.
 * @param sigma_RN [-] MRP from inertial frame N to reference frame R.
 * @return Pointer to a new Inertial3DAlgorithm (must be destroyed). Validated; throws on invalid input.
 */
Inertial3DAlgorithmHandle* Inertial3DAlgorithm_create(Vector3f_c sigma_RN);

/**
 * @brief Destroy a previously created Inertial3DAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void Inertial3DAlgorithm_destroy(Inertial3DAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self     Pointer to the instance.
 * @param sigma_RN [-] MRP from inertial frame N to reference frame R.
 * Validated; throws on invalid input.
 */
void Inertial3DAlgorithm_setConfig(Inertial3DAlgorithmHandle* self, Vector3f_c sigma_RN);

/**
 * @brief Run the update step.
 * @param self Pointer to the instance.
 * @return Vector3f_c  The fixed reference-attitude MRP sigma_RN.
 */
Vector3f_c Inertial3DAlgorithm_update(const Inertial3DAlgorithmHandle* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_INERTIAL3DALGORITHM_C_H
