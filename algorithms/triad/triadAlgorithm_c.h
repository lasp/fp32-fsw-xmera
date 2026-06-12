#ifndef F32XMERA_TRIAD_ALGORITHM_C_H
#define F32XMERA_TRIAD_ALGORITHM_C_H

#include "triadTypes.h"
#include "utilities/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ TriadAlgorithm instance.
 */
typedef struct TriadAlgorithmHandle TriadAlgorithmHandle;

/**
 * @brief Construct a new TriadAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new TriadAlgorithm (must be destroyed).
 */
TriadAlgorithmHandle* TriadAlgorithm_create(const TriadConfig_c* config);

/**
 * @brief Destroy a previously created TriadAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void TriadAlgorithm_destroy(TriadAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime (validated; throws on invalid input).
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply.
 */
void TriadAlgorithm_setConfig(TriadAlgorithmHandle* self, const TriadConfig_c* config);

/**
 * @brief Compute the reference attitude that aligns the thrust axis with the requested inertial
 *        thrust direction while keeping the solar array drive axis as sun-facing as possible.
 * @param self        Pointer to the instance.
 * @param sigma_BN    Current body attitude MRP wrt inertial N.
 * @param rHat_SB_B   Unit sun direction in body-frame components.
 * @param thrustHat_B Unit thrust direction in body-frame components.
 * @return Vector3f_c  Reference attitude MRP sigma_RN wrt inertial N.
 */
Vector3f_c TriadAlgorithm_update(TriadAlgorithmHandle* self,
                                 const Vector3f_c* sigma_BN,
                                 const Vector3f_c* rHat_SB_B,
                                 const Vector3f_c* thrustHat_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_TRIAD_ALGORITHM_C_H
