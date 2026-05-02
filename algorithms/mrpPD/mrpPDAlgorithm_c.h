/* SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef XMERAF32_MRPPDALGORITHM_C_H
#define XMERAF32_MRPPDALGORITHM_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MrpPDAlgorithm instance.
 */
typedef struct MrpPDAlgorithm MrpPDAlgorithm;

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3f).
 */
typedef struct {
    float data[3];
} Vector3f_c;

/**
 * @brief POD representation of a 3x3 matrix (Eigen::Matrix3f).
 */
typedef struct {
    float data[3][3];
} Matrix3f_c;

/**
 * @brief Plain-old-data mirror of the C++ MrpPDConfig fields.
 *
 * Caller fills this struct and passes it to MrpPDAlgorithm_create or _setConfig. The C++ side
 * validates each field via MrpPDConfig::create and throws on invalid input.
 *  - K and P must be >= 0
 *  - knownTorquePntB_B must be finite
 *  - spacecraftInertia must be a valid inertia tensor (symmetric positive-definite, triangle
 *    inequality on principal moments)
 */
typedef struct {
    float K;
    float P;
    Vector3f_c knownTorquePntB_B;
    Matrix3f_c spacecraftInertia;
} MrpPDConfig_c;

/**
 * @brief Construct a new MrpPDAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new MrpPDAlgorithm (must be destroyed).
 */
MrpPDAlgorithm* MrpPDAlgorithm_create(const MrpPDConfig_c* config);

/**
 * @brief Destroy a previously created MrpPDAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpPDAlgorithm_destroy(MrpPDAlgorithm* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void MrpPDAlgorithm_setConfig(MrpPDAlgorithm* self, const MrpPDConfig_c* config);

/**
 * @brief Compute the required attitude control torque Lr.
 * @param self        Pointer to the instance.
 * @param sigma_BR    Body-to-reference MRP attitude error.
 * @param omega_BR_B  Body-to-reference angular rate error in B-frame components.
 * @param domega_RN_B Reference frame inertial angular acceleration in B-frame components.
 * @return Vector3f_c [N*m] commanded control torque about point B in B-frame components.
 */
Vector3f_c MrpPDAlgorithm_update(MrpPDAlgorithm* self,
                                 const Vector3f_c* sigma_BR,
                                 const Vector3f_c* omega_BR_B,
                                 const Vector3f_c* domega_RN_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // XMERAF32_MRPPDALGORITHM_C_H
