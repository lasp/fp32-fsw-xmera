#ifndef F32XMERA_TRIAD_ALGORITHM_C_H
#define F32XMERA_TRIAD_ALGORITHM_C_H

#include "triadTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ TriadAlgorithm instance.
 */
typedef struct TriadAlgorithmHandle TriadAlgorithmHandle;

/**
 * @brief Construct a new TriadAlgorithm instance from the supplied configuration.
 * @param sadaHat_B      [-] solar array drive axis, unit vector in body-frame components.
 * @param thrustReqHat_N [-] requested thrust direction, unit vector in inertial-frame components.
 * @param n3Axis         [-] inertial z-axis direction (+Z or -Z) used as the fallback constraint axis
 *                           when the sun and thrust reference are aligned.
 * @return Pointer to a new TriadAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
TriadAlgorithmHandle* TriadAlgorithm_create(const Vector3f_c* sadaHat_B,
                                            const Vector3f_c* thrustReqHat_N,
                                            N3Axis_c n3Axis);

/**
 * @brief Destroy a previously created TriadAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void TriadAlgorithm_destroy(TriadAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The algorithm holds no runtime state, so
 *        nothing is carried across the swap.
 * @param self           Pointer to the instance.
 * @param sadaHat_B      [-] solar array drive axis, unit vector in body-frame components.
 * @param thrustReqHat_N [-] requested thrust direction, unit vector in inertial-frame components.
 * @param n3Axis         [-] inertial z-axis direction (+Z or -Z) used as the fallback constraint axis
 *                           when the sun and thrust reference are aligned.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void TriadAlgorithm_setConfig(TriadAlgorithmHandle* self,
                              const Vector3f_c* sadaHat_B,
                              const Vector3f_c* thrustReqHat_N,
                              N3Axis_c n3Axis);

/**
 * @brief Compute the reference attitude that aligns the thrust axis with the requested inertial
 *        thrust direction while keeping the solar array drive axis as sun-facing as possible.
 * @param self        Pointer to the instance.
 * @param rHat_SB_N   Unit sun direction in inertial-frame components.
 * @param thrustHat_B Unit thrust direction in body-frame components.
 * @return Vector3f_c  Reference attitude MRP sigma_RN wrt inertial N.
 */
Vector3f_c TriadAlgorithm_update(TriadAlgorithmHandle* self,
                                 const Vector3f_c* rHat_SB_N,
                                 const Vector3f_c* thrustHat_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_TRIAD_ALGORITHM_C_H
