#ifndef F32XMERA_MRP_FEEDBACK_ALGORITHM_C_H
#define F32XMERA_MRP_FEEDBACK_ALGORITHM_C_H

#include "mrpFeedbackTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MrpFeedbackAlgorithm instance.
 */
typedef struct MrpFeedbackAlgorithmHandle MrpFeedbackAlgorithmHandle;

/**
 * @brief Get the kMaxNumRw constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t MrpFeedbackAlgorithm_getMaxNumRw(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param K                 [N*m]   proportional gain on the MRP error; must be finite and >= 0.
 * @param P                 [N*m*s] rate-error feedback gain; must be finite and >= 0.
 * @param Ki                [N*m]   integral feedback gain; must be finite and >= 0 (0 disables the integral).
 * @param integralLimit     [N*m*s] anti-windup clamp on the integral state; must be finite and >= 0.
 * @param controlLawType    [-]     control-law variant; must be NORMAL or SIMPLE_INTEGRAL.
 * @param controlPeriod     [s]     time between two algorithm update calls; must be finite and > 0.
 * @param knownTorquePntB_B [N*m]   known external torque, body-frame components; must be finite.
 * @param ISCPntB_B      [kg*m^2]   spacecraft inertia about point B; must be a valid inertia matrix.
 * @param rwConfiguration   [-]     reaction-wheel configuration, or NULL to omit the reaction-wheel momentum
 *                                  term; when supplied, numRW <= max, finite inertias, near-unit spin axes.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool MrpFeedbackAlgorithm_validateConfig(float K,
                                         float P,
                                         float Ki,
                                         float integralLimit,
                                         ControlLawType_c controlLawType,
                                         float controlPeriod,
                                         const Vector3f_c* knownTorquePntB_B,
                                         const Matrix3f_c* ISCPntB_B,
                                         const MrpFeedbackRwConfig_c* rwConfiguration);

/**
 * @brief Construct a new MrpFeedbackAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first; invalid input throws.
 * @return Pointer to a new MrpFeedbackAlgorithm (must be destroyed).
 */
MrpFeedbackAlgorithmHandle* MrpFeedbackAlgorithm_create(float K,
                                                        float P,
                                                        float Ki,
                                                        float integralLimit,
                                                        ControlLawType_c controlLawType,
                                                        float controlPeriod,
                                                        const Vector3f_c* knownTorquePntB_B,
                                                        const Matrix3f_c* ISCPntB_B,
                                                        const MrpFeedbackRwConfig_c* rwConfiguration);

/**
 * @brief Destroy a previously created MrpFeedbackAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpFeedbackAlgorithm_destroy(MrpFeedbackAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The integral state is preserved.
 * Validate the values with validateConfig first; invalid input throws.
 * @param self Pointer to the instance.
 */
void MrpFeedbackAlgorithm_setConfig(MrpFeedbackAlgorithmHandle* self,
                                    float K,
                                    float P,
                                    float Ki,
                                    float integralLimit,
                                    ControlLawType_c controlLawType,
                                    float controlPeriod,
                                    const Vector3f_c* knownTorquePntB_B,
                                    const Matrix3f_c* ISCPntB_B,
                                    const MrpFeedbackRwConfig_c* rwConfiguration);

/**
 * @brief Reset the integrating runtime state (zero the integral of the MRP tracking error).
 * @param self Pointer to the instance.
 */
void MrpFeedbackAlgorithm_reInitialize(MrpFeedbackAlgorithmHandle* self);

/**
 * @brief Compute the commanded control torque Lr and integral feedback torque Li for the current guidance and
 *        reaction-wheel speeds.
 * @param self         Pointer to the instance.
 * @param attGuidInput Attitude guidance input (sigma_BR, omega_BR_B, omega_RN_B, domega_RN_B).
 * @param wheelSpeeds  Current reaction-wheel speeds.
 * @return MrpFeedbackOutput_c Commanded control torque and integral feedback torque in body-frame components.
 */
MrpFeedbackOutput_c MrpFeedbackAlgorithm_update(MrpFeedbackAlgorithmHandle* self,
                                                const MrpFeedbackInputGuidance_c* attGuidInput,
                                                const MrpFeedbackRwSpeeds_c* wheelSpeeds);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_MRP_FEEDBACK_ALGORITHM_C_H */
