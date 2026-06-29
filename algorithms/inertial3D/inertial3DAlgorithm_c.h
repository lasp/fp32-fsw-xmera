#ifndef F32XMERA_INERTIAL3DALGORITHM_C_H
#define F32XMERA_INERTIAL3DALGORITHM_C_H

#include "inertial3DTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ Inertial3DAlgorithm instance.
 */
typedef struct Inertial3DAlgorithmHandle Inertial3DAlgorithmHandle;

/**
 * @brief Construct a new Inertial3DAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new Inertial3DAlgorithm (must be destroyed).
 */
Inertial3DAlgorithmHandle* Inertial3DAlgorithm_create(const Inertial3DConfig_c* config);

/**
 * @brief Destroy a previously created Inertial3DAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void Inertial3DAlgorithm_destroy(Inertial3DAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void Inertial3DAlgorithm_setConfig(Inertial3DAlgorithmHandle* self, const Inertial3DConfig_c* config);

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
