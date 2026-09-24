#ifndef F32XMERA_MRP_STEERING_ALGORITHM_C_H
#define F32XMERA_MRP_STEERING_ALGORITHM_C_H

#include "mrpSteeringTypes.h"

#include "utilities/fsw/deviceAvailability.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"
#include <stdbool.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MrpSteeringAlgorithm instance.
 */
typedef struct MrpSteeringAlgorithmHandle MrpSteeringAlgorithmHandle;

/**
 * @brief RW spin axes in body frame, three components per wheel in row major order.
 */
typedef struct {
    float data[3 * RW_EFF_CNT]; /*!< [-] three components per wheel */
} MrpSteeringRwSpinAxes_c;

/**
 * @brief Per-wheel spin-axis inertia, one entry per wheel slot.
 */
typedef struct {
    float data[RW_EFF_CNT]; /*!< [kg*m^2] one entry per wheel */
} MrpSteeringRwInertias_c;

/**
 * @brief Availability of each wheel slot, one entry per slot: 0 available, 1 unavailable.
 */
typedef struct {
    DeviceAvailability_c availability[RW_EFF_CNT]; /*!< [-] one entry per wheel */
} MrpSteeringRwAvailability_c;

/**
 * @brief Get the kMaxNumRw constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t MrpSteeringAlgorithm_getMaxNumRw(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param K1                         [rad/s] proportional gain on MRP errors; must be >= 0.
 * @param K3                         [rad/s] cubic gain in the steering saturation function; must be >= 0.
 * @param omegaMax                   [rad/s] maximum rate command of the steering law; must be > 0.
 * @param ignoreOuterLoopFeedforward [-]     whether the outer-loop feedforward term is excluded.
 * @param P                          [N*m*s] rate error feedback gain; must be >= 0.
 * @param Ki                         [N*m]   integral feedback gain on the rate error; must be >= 0.
 * @param integralLimit              [N*m]   integral limit that avoids wind-up; must be >= 0.
 * @param controlPeriod              [s]     time between two update calls; must be > 0.
 * @param knownTorquePntB_B          [N*m]   known external torque in body-frame components.
 * @param ISCPntB_B                  [kg*m^2] spacecraft inertia about point B; must be a valid inertia matrix.
 * @param rwConfiguration            [-]     reaction-wheel configuration, or NULL to omit the reaction-wheel
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool MrpSteeringAlgorithm_validateConfig(float K1,
                                         float K3,
                                         float omegaMax,
                                         bool ignoreOuterLoopFeedforward,
                                         float P,
                                         float Ki,
                                         float integralLimit,
                                         float controlPeriod,
                                         const Vector3f_c* knownTorquePntB_B,
                                         const Matrix3f_c* ISCPntB_B,
                                         const MrpSteeringRwSpinAxes_c* GsMatrix_B,
                                         const MrpSteeringRwInertias_c* JsList,
                                         const MrpSteeringRwAvailability_c* wheelAvailability);

/**
 * @brief Construct a new MrpSteeringAlgorithm instance from the supplied configuration.
 * @param K1                         [rad/s] proportional gain on MRP errors; must be >= 0.
 * @param K3                         [rad/s] cubic gain in the steering saturation function; must be >= 0.
 * @param omegaMax                   [rad/s] maximum rate command of the steering law; must be > 0.
 * @param ignoreOuterLoopFeedforward [-]     whether the outer-loop feedforward term is excluded.
 * @param P                          [N*m*s] rate error feedback gain; must be >= 0.
 * @param Ki                         [N*m]   integral feedback gain on the rate error; must be >= 0.
 * @param integralLimit              [N*m]   integral limit that avoids wind-up; must be >= 0.
 * @param controlPeriod              [s]     time between two update calls; must be > 0.
 * @param knownTorquePntB_B          [N*m]   known external torque in body-frame components.
 * @param ISCPntB_B                  [kg*m^2] spacecraft inertia about point B; must be a valid inertia matrix.
 * @param GsMatrix_B                 [-]     RW spin axes, three per wheel in row major order, or NULL to omit
 *                                           the reaction-wheel terms. The other two are then ignored.
 * @param JsList                     [kg*m^2] per-wheel spin-axis inertia.
 * @param wheelAvailability          [-]     availability of each wheel: 0 available, 1 unavailable.
 * @return Pointer to a new MrpSteeringAlgorithm (must be destroyed).
 */
MrpSteeringAlgorithmHandle* MrpSteeringAlgorithm_create(float K1,
                                                        float K3,
                                                        float omegaMax,
                                                        bool ignoreOuterLoopFeedforward,
                                                        float P,
                                                        float Ki,
                                                        float integralLimit,
                                                        float controlPeriod,
                                                        const Vector3f_c* knownTorquePntB_B,
                                                        const Matrix3f_c* ISCPntB_B,
                                                        const MrpSteeringRwSpinAxes_c* GsMatrix_B,
                                                        const MrpSteeringRwInertias_c* JsList,
                                                        const MrpSteeringRwAvailability_c* wheelAvailability);

/**
 * @brief Destroy a previously created MrpSteeringAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpSteeringAlgorithm_destroy(MrpSteeringAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The integral state is preserved.
 * @param self   Pointer to the instance.
 * @param K1                         [rad/s] proportional gain on MRP errors; must be >= 0.
 * @param K3                         [rad/s] cubic gain in the steering saturation function; must be >= 0.
 * @param omegaMax                   [rad/s] maximum rate command of the steering law; must be > 0.
 * @param ignoreOuterLoopFeedforward [-]     whether the outer-loop feedforward term is excluded.
 * @param P                          [N*m*s] rate error feedback gain; must be >= 0.
 * @param Ki                         [N*m]   integral feedback gain on the rate error; must be >= 0.
 * @param integralLimit              [N*m]   integral limit that avoids wind-up; must be >= 0.
 * @param controlPeriod              [s]     time between two update calls; must be > 0.
 * @param knownTorquePntB_B          [N*m]   known external torque in body-frame components.
 * @param ISCPntB_B                  [kg*m^2] spacecraft inertia about point B; must be a valid inertia matrix.
 * @param GsMatrix_B                 [-]     RW spin axes, three per wheel in row major order, or NULL to omit
 *                                           the reaction-wheel terms. The other two are then ignored.
 * @param JsList                     [kg*m^2] per-wheel spin-axis inertia.
 * @param wheelAvailability          [-]     availability of each wheel: 0 available, 1 unavailable.
 */
void MrpSteeringAlgorithm_setConfig(MrpSteeringAlgorithmHandle* self,
                                    float K1,
                                    float K3,
                                    float omegaMax,
                                    bool ignoreOuterLoopFeedforward,
                                    float P,
                                    float Ki,
                                    float integralLimit,
                                    float controlPeriod,
                                    const Vector3f_c* knownTorquePntB_B,
                                    const Matrix3f_c* ISCPntB_B,
                                    const MrpSteeringRwSpinAxes_c* GsMatrix_B,
                                    const MrpSteeringRwInertias_c* JsList,
                                    const MrpSteeringRwAvailability_c* wheelAvailability);

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
