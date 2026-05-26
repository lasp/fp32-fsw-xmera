#ifndef F32XIMERA_SUNSAFEPOINTALGORITHM_C_H
#define F32XIMERA_SUNSAFEPOINTALGORITHM_C_H

#include "utilities/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunSafePointAlgorithm instance.
 */
typedef struct SunSafePointAlgorithm SunSafePointAlgorithm;

/**
 * @brief C-compatible sun safe point attitude guidance output.
 */
typedef struct {
    Vector3f_c sigma_BR;   /*!< attitude error (MRPs) of B relative to R */
    Vector3f_c omega_BR_B; /*!< [rad/s] body rate error of B relative to R in B frame */
    Vector3f_c omega_RN_B; /*!< [rad/s] reference frame rate of R relative to N in B frame */
} SunSafePointOutput_c;

/**
 * @brief Construct a new SunSafePointAlgorithm instance.
 * @return Pointer to a new SunSafePointAlgorithm (must be destroyed).
 */
SunSafePointAlgorithm* SunSafePointAlgorithm_create(void);

/**
 * @brief Destroy a previously created SunSafePointAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunSafePointAlgorithm_destroy(SunSafePointAlgorithm* self);

/**
 * @brief Run the update step.
 * @param self         Pointer to the instance.
 * @param vehSunPntBdy Sun direction vector in body frame.
 * @param omega_BN_B   Inertial body angular velocity in body frame.
 * @return SunSafePointOutput_c  The computed guidance output.
 */
SunSafePointOutput_c SunSafePointAlgorithm_update(const SunSafePointAlgorithm* self,
                                                  Vector3f_c vehSunPntBdy,
                                                  Vector3f_c omega_BN_B);

/**
 * @brief Set the desired constant spin rate about the sun heading vector.
 * @param self Pointer to the instance.
 * @param rate Desired constant spin rate [rad/s].
 */
void SunSafePointAlgorithm_setSunAxisSpinRate(SunSafePointAlgorithm* self, float rate);

/**
 * @brief Get the desired constant spin rate about the sun heading vector.
 * @param self Pointer to the instance.
 * @return float  Current sun axis spin rate [rad/s].
 */
float SunSafePointAlgorithm_getSunAxisSpinRate(const SunSafePointAlgorithm* self);

/**
 * @brief Set the desired body rate vector used when no sun direction is available.
 * @param self  Pointer to the instance.
 * @param omega Desired body rate vector [rad/s].
 */
void SunSafePointAlgorithm_setOmega_RN_B(SunSafePointAlgorithm* self, Vector3f_c omega);

/**
 * @brief Get the desired body rate vector used when no sun direction is available.
 * @param self Pointer to the instance.
 * @return Vector3f_c  Current desired body rate vector [rad/s].
 */
Vector3f_c SunSafePointAlgorithm_getOmega_RN_B(const SunSafePointAlgorithm* self);

/**
 * @brief Set the commanded body unit vector to point at the sun.
 * @param self Pointer to the instance.
 * @param sHat Commanded body unit vector (norm must be within 1e-3 of 1.0).
 */
void SunSafePointAlgorithm_setSHatBdyCmd(SunSafePointAlgorithm* self, Vector3f_c sHat);

/**
 * @brief Get the commanded body unit vector to point at the sun.
 * @param self Pointer to the instance.
 * @return Vector3f_c  Current commanded body unit vector.
 */
Vector3f_c SunSafePointAlgorithm_getSHatBdyCmd(const SunSafePointAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_SUNSAFEPOINTALGORITHM_C_H
