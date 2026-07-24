#ifndef F32XMERA_MRP_FEEDBACK_TYPES_H
#define F32XMERA_MRP_FEEDBACK_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of reaction wheels handled at the C boundary. Must match RW_EFF_CNT in
   msgPayloadDef/definitions.h (enforced by a static_assert in the C shim). */
#define MRP_FEEDBACK_MAX_NUM_RW 36

/**
 * @brief C-compatible mirror of the C++ ControlLawType enum class.
 *
 * Numeric values must stay in lockstep with the C++ enum class in mrpFeedbackAlgorithm.h.
 */
typedef enum { CONTROL_LAW_TYPE_NORMAL_C = 0, CONTROL_LAW_TYPE_SIMPLE_INTEGRAL_C = 1 } ControlLawType_c;

/**
 * @brief Plain-old-data mirror of the C++ MrpFeedbackInputRwData (reaction-wheel configuration).
 *
 * numRW must not exceed MRP_FEEDBACK_MAX_NUM_RW, and each active spin axis (column of GsMatrix_B) must be a
 * unit vector; the spin axes are normalized when the configuration is built.
 */
typedef struct {
    uint32_t numRW;                                     /*!< [-] number of reaction wheels on the vehicle */
    float GsMatrix_B[3 * MRP_FEEDBACK_MAX_NUM_RW];      /*!< [-] RW spin axes in body frame, three per wheel */
    float JsList[MRP_FEEDBACK_MAX_NUM_RW];              /*!< [kg*m^2] per-wheel spin-axis inertia */
    int32_t wheelAvailability[MRP_FEEDBACK_MAX_NUM_RW]; /*!< [-] AVAILABLE / UNAVAILABLE state of each wheel */
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
    float wheelSpeeds[MRP_FEEDBACK_MAX_NUM_RW]; /*!< [r/s] reaction-wheel speeds */
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
