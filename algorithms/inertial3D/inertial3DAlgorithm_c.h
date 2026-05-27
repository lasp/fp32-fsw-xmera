#ifndef F32XMERA_INERTIAL3DALGORITHM_C_H
#define F32XMERA_INERTIAL3DALGORITHM_C_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "utilities/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ Inertial3DAlgorithm instance.
 */
typedef struct Inertial3DAlgorithmHandle Inertial3DAlgorithmHandle;

/**
 * @brief Construct a new Inertial3DAlgorithm instance.
 * @return Pointer to a new Inertial3DAlgorithm (must be destroyed).
 */
Inertial3DAlgorithmHandle* Inertial3DAlgorithm_create(void);

/**
 * @brief Destroy a previously created Inertial3DAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void Inertial3DAlgorithm_destroy(Inertial3DAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self Pointer to the instance.
 * @return AttRefMsgF32Payload  The computed reference message.
 */
AttRefMsgF32Payload Inertial3DAlgorithm_update(const Inertial3DAlgorithmHandle* self);

/**
 * @brief Set the σ_RN three-vector.
 * @param self    Pointer to the instance.
 * @param sigmaRN 3-vector in flattened POD format.
 */
void Inertial3DAlgorithm_setSigmaRN(Inertial3DAlgorithmHandle* self, Vector3f_c sigmaRN);

/**
 * @brief Get the current σ_RN three-vector.
 * @param self Pointer to the instance.
 * @return Vector3f_c  Flattened POD containing the vector.
 */
Vector3f_c Inertial3DAlgorithm_getSigmaRN(const Inertial3DAlgorithmHandle* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_INERTIAL3DALGORITHM_C_H
