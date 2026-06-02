#ifndef F32XMERA_MRPFEEDBACKALGORITHM_C_H
#define F32XMERA_MRPFEEDBACKALGORITHM_C_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWAvailabilityMsgPayload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "utilities/plainCAlgorithmDataTypes.h"
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
 * @brief C-compatible mirror of the C++ ControlLawType enum class.
 *
 * Numeric values must stay in lockstep with the C++ enum class in mrpFeedbackTypes.h.
 */
typedef enum { CONTROL_LAW_TYPE_NORMAL_C = 0, CONTROL_LAW_TYPE_SIMPLE_INTEGRAL_C = 1 } ControlLawType_c;

/**
 * @brief Plain-old-data mirror of the C++ MrpFeedbackConfig fields.
 *
 * Caller fills this struct and passes it to MrpFeedbackAlgorithm_create or _setConfig. The C++
 * side validates each field via MrpFeedbackConfig::create and throws on invalid input.
 *  - K, P, Ki, integralLimit must be >= 0
 *  - controlLawType is unconstrained
 *  - knownTorquePntB_B is unconstrained
 *  - ISCPntB_B must be a valid inertia tensor (symmetric, positive-definite, triangle inequality)
 *  - numRW must be in [0, RW_EFF_CNT]; GsMatrix_B and JsList must be finite
 */
typedef struct {
    float K;
    float P;
    float Ki;
    float integralLimit;
    ControlLawType_c controlLawType;
    Vector3f_c knownTorquePntB_B;
    Matrix3f_c ISCPntB_B;
    int32_t numRW;                    /*!< number of reaction wheels (0..RW_EFF_CNT) */
    float GsMatrix_B[3 * RW_EFF_CNT]; /*!< RW spin-axis matrix, body frame (column-per-wheel) */
    float JsList[RW_EFF_CNT];         /*!< RW spin-axis inertias */
} MrpFeedbackConfig_c;

/**
 * @brief C-compatible mirror of the C++ MrpFeedbackGuidInput (per-cycle tracking error).
 */
typedef struct {
    Vector3f_c sigma_BR;    /*!< attitude error (MRP) of B relative to R */
    Vector3f_c omega_BR_B;  /*!< [r/s] body rate error of B relative to R, B frame */
    Vector3f_c omega_RN_B;  /*!< [r/s] reference rate of R relative to N, B frame */
    Vector3f_c domega_RN_B; /*!< [r/s^2] reference acceleration of R relative to N, B frame */
} MrpFeedbackGuidInput_c;

/**
 * @brief C-compatible mirror of the C++ MrpFeedbackOutput.
 */
typedef struct {
    Vector3f_c controlTorque;     /*!< [N*m] required control torque Lr */
    Vector3f_c intFeedbackTorque; /*!< [N*m] integral feedback torque Li */
} MrpFeedbackOutput_c;

/**
 * @brief Construct a new MrpFeedbackAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new MrpFeedbackAlgorithm (must be destroyed).
 */
MrpFeedbackAlgorithmHandle* MrpFeedbackAlgorithm_create(const MrpFeedbackConfig_c* config);

/**
 * @brief Destroy a previously created MrpFeedbackAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpFeedbackAlgorithm_destroy(MrpFeedbackAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void MrpFeedbackAlgorithm_setConfig(MrpFeedbackAlgorithmHandle* self, const MrpFeedbackConfig_c* config);

/**
 * @brief Reset the algorithm: clear the integral state. The spacecraft inertia and the RW array
 *        configuration are part of the immutable config (MrpFeedbackConfig_c).
 * @param self  Pointer to the instance.
 */
void MrpFeedbackAlgorithm_reset(MrpFeedbackAlgorithmHandle* self);

/**
 * @brief Compute the required control torque Lr and integral feedback torque Li.
 * @param self               Pointer to the instance.
 * @param callTime           Time stamp for update [ns].
 * @param guid               Attitude/rate tracking error.
 * @param wheelSpeeds        Reaction-wheel speeds (used only when configured numRW > 0).
 * @param wheelAvailability  Per-wheel availability (true = available).
 * @return MrpFeedbackOutput_c  Control torque and integral feedback torque payloads.
 */
MrpFeedbackOutput_c MrpFeedbackAlgorithm_update(MrpFeedbackAlgorithmHandle* self,
                                                uint64_t callTime,
                                                const MrpFeedbackGuidInput_c* guid,
                                                const float wheelSpeeds[RW_EFF_CNT],
                                                const bool wheelAvailability[RW_EFF_CNT]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRPFEEDBACKALGORITHM_C_H
