#ifndef F32XMERA_MRPPDALGORITHM_C_H
#define F32XMERA_MRPPDALGORITHM_C_H

#include "mrpPDTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MrpPDAlgorithm instance.
 */
typedef struct MrpPDAlgorithmHandle MrpPDAlgorithmHandle;

/**
 * @brief Construct a new MrpPDAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new MrpPDAlgorithm (must be destroyed).
 */
MrpPDAlgorithmHandle* MrpPDAlgorithm_create(const MrpPDConfig_c* config);

/**
 * @brief Destroy a previously created MrpPDAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpPDAlgorithm_destroy(MrpPDAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void MrpPDAlgorithm_setConfig(MrpPDAlgorithmHandle* self, const MrpPDConfig_c* config);

/**
 * @brief Run the PD control update.
 * @param self        Pointer to the instance.
 * @param sigma_BR    Body-to-reference attitude MRP.
 * @param omega_BR_B  Body-to-reference angular rate in body-frame components.
 * @param domega_RN_B Reference angular acceleration in body-frame components.
 * @return Vector3f_c  The commanded control torque Lr.
 */
Vector3f_c MrpPDAlgorithm_update(const MrpPDAlgorithmHandle* self,
                                 Vector3f_c sigma_BR,
                                 Vector3f_c omega_BR_B,
                                 Vector3f_c domega_RN_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRPPDALGORITHM_C_H
