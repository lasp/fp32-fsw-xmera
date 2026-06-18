#ifndef F32XMERA_MRP_STEERING_TYPES_H
#define F32XMERA_MRP_STEERING_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of reaction wheels handled at the C boundary. Must match RW_EFF_CNT in
   msgPayloadDef/definitions.h (enforced by a static_assert in the C shim). */
#define MRP_STEERING_MAX_NUM_RW 36

/**
 * @brief Plain-old-data mirror of the C++ MrpSteeringControlParameters.
 */
typedef struct {
    float K1;                        /*!< [rad/s] proportional gain applied to MRP errors (>= 0) */
    float K3;                        /*!< [rad/s] cubic gain in the steering saturation function (>= 0) */
    float omegaMax;                  /*!< [rad/s] maximum rate command of steering control (> 0) */
    bool ignoreOuterLoopFeedforward; /*!< [-] whether the outer-loop feedforward term is excluded */
    float P;                         /*!< [N*m*s] rate error feedback gain (>= 0) */
    float Ki;                        /*!< [N*m] integral feedback gain on the rate error (>= 0) */
    float integralLimit;             /*!< [N*m] integral limit to avoid wind-up (>= 0) */
    float controlPeriod;             /*!< [s] time between two algorithm update calls (> 0) */
} MrpSteeringControlParameters_c;

/**
 * @brief Plain-old-data mirror of the C++ InputRwData (reaction-wheel configuration).
 *
 * numRW must not exceed MRP_STEERING_MAX_NUM_RW, and each active spin axis (column of GsMatrix_B) must be a
 * unit vector; the spin axes are normalized when the configuration is built.
 */
typedef struct {
    uint32_t numRW;                                /*!< [-] number of reaction wheels on the vehicle */
    float GsMatrix_B[3 * MRP_STEERING_MAX_NUM_RW]; /*!< [-] RW spin axes in body frame, three per wheel */
    float JsList[MRP_STEERING_MAX_NUM_RW];         /*!< [kg*m^2] per-wheel spin-axis inertia */
    FSWdeviceAvailability
        wheelAvailability[MRP_STEERING_MAX_NUM_RW]; /*!< [-] AVAILABLE / UNAVAILABLE state of each wheel */
} MrpSteeringRwConfig_c;

/**
 * @brief Plain-old-data mirror of the C++ MrpSteeringConfig.
 *
 * hasRwConfiguration selects whether rwConfiguration is used, mirroring the C++ std::optional: when false the
 * reaction-wheel momentum term is omitted and rwConfiguration is ignored.
 */
typedef struct {
    MrpSteeringControlParameters_c controlParameters; /*!< [-] steering-law gains and feedforward toggle */
    Vector3f_c knownTorquePntB_B;                     /*!< [N*m] known external torque in body-frame components */
    Matrix3f_c ISCPntB_B;                             /*!< [kg*m^2] spacecraft inertia about point B */
    bool hasRwConfiguration;                          /*!< [-] true when rwConfiguration is provided */
    MrpSteeringRwConfig_c rwConfiguration;            /*!< [-] reaction-wheel configuration (used iff above is true) */
} MrpSteeringConfig_c;

/**
 * @brief Plain-old-data mirror of the C++ algorithm guidance input.
 */
typedef struct {
    Vector3f_c sigma_BR;    /*!< [-] MRP attitude tracking error */
    Vector3f_c omega_BR_B;  /*!< [rad/s] angular rate tracking error in body-frame components */
    Vector3f_c omega_RN_B;  /*!< [rad/s] reference angular rate in body-frame components */
    Vector3f_c domega_RN_B; /*!< [rad/s^2] reference angular acceleration in body-frame components */
} MrpSteeringInputGuidance_c;

/**
 * @brief Plain-old-data carrier for the per-wheel RW speed vector.
 */
typedef struct {
    float wheelSpeeds[MRP_STEERING_MAX_NUM_RW]; /*!< [r/s] reaction-wheel speeds */
} MrpSteeringRwSpeeds_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRP_STEERING_TYPES_H
