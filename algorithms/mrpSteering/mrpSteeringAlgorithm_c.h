#ifndef F32XMERA_MRP_STEERING_ALGORITHM_C_H
#define F32XMERA_MRP_STEERING_ALGORITHM_C_H

#include "mrpSteeringTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MrpSteeringAlgorithm instance.
 */
typedef struct MrpSteeringAlgorithmHandle MrpSteeringAlgorithmHandle;

/**
 * @brief Get the MRP_STEERING_MAX_NUM_RW constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t MrpSteeringAlgorithm_getMaxNumRw(void);

/**
 * @brief Construct a new MrpSteeringAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new MrpSteeringAlgorithm (must be destroyed).
 */
MrpSteeringAlgorithmHandle* MrpSteeringAlgorithm_create(const MrpSteeringConfig_c* config);

/**
 * @brief Destroy a previously created MrpSteeringAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpSteeringAlgorithm_destroy(MrpSteeringAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The integral state is preserved.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void MrpSteeringAlgorithm_setConfig(MrpSteeringAlgorithmHandle* self, const MrpSteeringConfig_c* config);

/**
 * @brief Reset the integrating runtime state (zero the integral of the rate tracking error).
 * @param self Pointer to the instance.
 */
void MrpSteeringAlgorithm_reInitialize(MrpSteeringAlgorithmHandle* self);

/**
 * @brief Compute the commanded control torque Lr for the current guidance and reaction-wheel speeds.
 * @param self         Pointer to the instance.
 * @param attGuidInput Attitude guidance input (sigma_BR, omega_BR_B, omega_RN_B, domega_RN_B).
 * @param wheelSpeeds  Current reaction-wheel speeds.
 * @return The commanded control torque Lr in body-frame components.
 */
Vector3f_c MrpSteeringAlgorithm_update(MrpSteeringAlgorithmHandle* self,
                                       const MrpSteeringInputGuidance_c* attGuidInput,
                                       const MrpSteeringRwSpeeds_c* wheelSpeeds);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_MRP_STEERING_ALGORITHM_C_H */
