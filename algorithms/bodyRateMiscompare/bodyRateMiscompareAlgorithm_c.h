#ifndef F32XMERA_BODYRATEMISCOMPAREALGORITHM_C_H
#define F32XMERA_BODYRATEMISCOMPAREALGORITHM_C_H

#include "bodyRateMiscompareTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ BodyRateMiscompareAlgorithm instance.
 */
typedef struct BodyRateMiscompareAlgorithmHandle BodyRateMiscompareAlgorithmHandle;

/**
 * @brief Construct a new BodyRateMiscompareAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new BodyRateMiscompareAlgorithm (must be destroyed).
 */
BodyRateMiscompareAlgorithmHandle* BodyRateMiscompareAlgorithm_create(const BodyRateMiscompareConfig_c* config);

/**
 * @brief Destroy a previously created BodyRateMiscompareAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void BodyRateMiscompareAlgorithm_destroy(BodyRateMiscompareAlgorithmHandle* self);

/**
 * @brief Apply a new configuration, resetting the latched fault state.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void BodyRateMiscompareAlgorithm_setConfig(BodyRateMiscompareAlgorithmHandle* self,
                                           const BodyRateMiscompareConfig_c* config);

/**
 * @brief Clear the persistence counter only; a latched fault is preserved.
 * @param self Pointer to the instance.
 */
void BodyRateMiscompareAlgorithm_reInitialize(BodyRateMiscompareAlgorithmHandle* self);

/**
 * @brief Clear the persistence counter and re-arm the latched fault from configuration.
 * @param self Pointer to the instance.
 */
void BodyRateMiscompareAlgorithm_reInitializeAll(BodyRateMiscompareAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self      Pointer to the instance.
 * @param imuOmega  IMU body rate vector.
 * @param stOmega   Star tracker body rate vector.
 * @return BodyRateMiscompareOutput_c  The computed output.
 */
BodyRateMiscompareOutput_c BodyRateMiscompareAlgorithm_update(BodyRateMiscompareAlgorithmHandle* self,
                                                              Vector3f_c imuOmega,
                                                              Vector3f_c stOmega);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_BODYRATEMISCOMPAREALGORITHM_C_H
