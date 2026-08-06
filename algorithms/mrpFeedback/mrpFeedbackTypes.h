#ifndef F32XMERA_MRP_FEEDBACK_TYPES_H
#define F32XMERA_MRP_FEEDBACK_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/deviceAvailability.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C-compatible mirror of the C++ ControlLawType enum class.
 *
 * Numeric values must stay in lockstep with the C++ enum class in mrpFeedbackAlgorithm.h.
 */
typedef enum { CONTROL_LAW_TYPE_NORMAL_C = 0, CONTROL_LAW_TYPE_SIMPLE_INTEGRAL_C = 1 } ControlLawType_c;

/**
 * @brief Plain-old-data mirror of the C++ MrpFeedbackInputRwData (reaction-wheel configuration).
 *
 * numRW must not exceed RW_EFF_CNT, and each active spin axis (column of GsMatrix_B) must be a
 * unit vector; the spin axes are normalized when the configuration is built.
 */
typedef struct {
    uint32_t numRW;                                     /*!< [-] number of reaction wheels on the vehicle */
    float GsMatrix_B[3 * RW_EFF_CNT];                   /*!< [-] RW spin axes in body frame, three per wheel */
    float JsList[RW_EFF_CNT];                           /*!< [kg*m^2] per-wheel spin-axis inertia */
    DeviceAvailability_c wheelAvailability[RW_EFF_CNT]; /*!< [-] AVAILABLE / UNAVAILABLE state of each wheel */
} MrpFeedbackRwConfig_c;

/**
 * @brief Plain-old-data mirror of the C++ algorithm guidance input.
 */
typedef struct {
    Vector3f_c sigma_BR;    /*!< [-] MRP attitude tracking error */
    Vector3f_c omega_BR_B;  /*!< [rad/s] angular rate tracking error in body-frame components */
    Vector3f_c omega_RN_B;  /*!< [rad/s] reference angular rate in body-frame components */
    Vector3f_c domega_RN_B; /*!< [rad/s^2] reference angular acceleration in body-frame components */
} MrpFeedbackInputGuidance_c;

/**
 * @brief Plain-old-data carrier for the per-wheel RW speed vector.
 */
typedef struct {
    float wheelSpeeds[RW_EFF_CNT]; /*!< [r/s] reaction-wheel speeds */
} MrpFeedbackRwSpeeds_c;

/**
 * @brief Plain-old-data mirror of the C++ MrpFeedbackOutput.
 */
typedef struct {
    Vector3f_c controlTorque;          /*!< [N*m] commanded control torque Lr */
    Vector3f_c integralFeedbackTorque; /*!< [N*m] integral feedback torque Li */
} MrpFeedbackOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRP_FEEDBACK_TYPES_H
