#ifndef F32XMERA_MRPPDALGORITHM_C_H
#define F32XMERA_MRPPDALGORITHM_C_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MrpPDAlgorithm instance.
 */
typedef struct MrpPDAlgorithmHandle MrpPDAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param K                 [N*m]   proportional gain on the MRP error; must be >= 0.
 * @param P                 [N*m*s] rate-error feedback gain; must be >= 0.
 * @param knownTorquePntB_B [N*m]   known external torque, body-frame components; must be finite.
 * @param ISCPntB_B      [kg*m^2]   spacecraft inertia about point B; must be a valid inertia matrix.
 * @return true if the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 * @note The accepted value ranges are defined by MrpPDConfig::create; this predicate
 *       reports whether a candidate set would be accepted, without throwing.
 */
bool MrpPDAlgorithm_validateConfig(float K, float P, Vector3f_c knownTorquePntB_B, Matrix3f_c ISCPntB_B);

/**
 * @brief Construct a new MrpPDAlgorithm instance from the supplied configuration.
 * @param K                 [N*m]   proportional gain on the MRP error.
 * @param P                 [N*m*s] rate-error feedback gain.
 * @param knownTorquePntB_B [N*m]   known external torque, body-frame components.
 * @param ISCPntB_B      [kg*m^2]   spacecraft inertia about point B.
 * @return Pointer to a new MrpPDAlgorithm (must be destroyed). Validated; throws on invalid input.
 */
MrpPDAlgorithmHandle* MrpPDAlgorithm_create(float K, float P, Vector3f_c knownTorquePntB_B, Matrix3f_c ISCPntB_B);

/**
 * @brief Destroy a previously created MrpPDAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpPDAlgorithm_destroy(MrpPDAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self              Pointer to the instance.
 * @param K                 [N*m]   proportional gain on the MRP error.
 * @param P                 [N*m*s] rate-error feedback gain.
 * @param knownTorquePntB_B [N*m]   known external torque, body-frame components.
 * @param ISCPntB_B      [kg*m^2]   spacecraft inertia about point B.
 * Validated; throws on invalid input.
 */
void MrpPDAlgorithm_setConfig(MrpPDAlgorithmHandle* self,
                              float K,
                              float P,
                              Vector3f_c knownTorquePntB_B,
                              Matrix3f_c ISCPntB_B);

/**
 * @brief Compute the commanded control torque Lr for the current guidance errors.
 * @param self        Pointer to the instance.
 * @param sigma_BR    [-]      MRP attitude tracking error.
 * @param omega_BR_B  [rad/s]  angular rate tracking error in body-frame components.
 * @param domega_RN_B [rad/s^2] reference angular acceleration in body-frame components.
 * @return Vector3f_c [N*m] commanded control torque in body-frame components.
 */
Vector3f_c MrpPDAlgorithm_update(const MrpPDAlgorithmHandle* self,
                                 Vector3f_c sigma_BR,
                                 Vector3f_c omega_BR_B,
                                 Vector3f_c domega_RN_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRPPDALGORITHM_C_H
