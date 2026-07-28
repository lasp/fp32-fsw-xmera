#ifndef F32XMERA_THRUSTER_PLATFORM_REFERENCE_TYPES_H
#define F32XMERA_THRUSTER_PLATFORM_REFERENCE_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of reaction wheels handled at the C boundary. Must match RW_EFF_CNT in
   msgPayloadDef/definitions.h (enforced by a static_assert in the adapter). */
#define THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW 36

/**
 * @brief Plain-old-data mirror of the C++ ThrusterPlatformReferenceRwArrayConfiguration.
 */
typedef struct {
    uint32_t numRW; /*!< [-]    number of reaction wheels on the vehicle */
    float GsMatrix_B[3 *
                     THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW]; /*!< [-]   RW spin axes in body frame, three per wheel */
    float JsList[THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW];     /*!< [kgm2] RW spin-axis inertias */
} ThrusterPlatformReferenceRwArrayConfiguration_c;

/**
 * @brief Plain-old-data mirror of the C++ ThrusterPlatformReferenceConfig.
 *
 * Caller fills this struct and passes it to ThrusterPlatformReferenceAlgorithm_create or _setConfig.
 * The C++ side validates each field via ThrusterPlatformReferenceConfig::create and throws on invalid
 * input.
 *  - sigma_MB / r_BM_M / r_FM_F must be finite
 *  - K / Ki must be finite and non-negative
 *  - theta1Max / theta2Max must be finite (a non-positive bound disables clamping on that axis)
 *  - rwConfig.numRW must not exceed THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW; each spin axis a unit vector
 */
typedef struct {
    Vector3f_c sigma_MB;  /*!< [-]   MRP orientation of the M frame w.r.t. the B frame */
    Vector3f_c r_BM_M;    /*!< [m]   B frame origin w.r.t. M frame origin, M frame coordinates */
    Vector3f_c r_FM_F;    /*!< [m]   F frame origin w.r.t. M frame origin, F frame coordinates */
    float K;              /*!< [1/s] momentum dumping proportional gain */
    float Ki;             /*!< [-]   momentum dumping integral gain */
    float theta1Max;      /*!< [rad] absolute bound on the tip angle */
    float theta2Max;      /*!< [rad] absolute bound on the tilt angle */
    bool momentumDumping; /*!< [-]   whether reaction wheel momentum dumping is active */
    ThrusterPlatformReferenceRwArrayConfiguration_c rwConfig; /*!< [-] RW configuration used for momentum dumping */
} ThrusterPlatformReferenceConfig_c;

/**
 * @brief Plain-old-data mirror of the C++ ThrusterPlatformReferenceInputs.
 */
typedef struct {
    Vector3f_c r_CB_B; /*!< [m]   center of mass w.r.t. B origin, B frame */
    Vector3f_c r_TF_F; /*!< [m]   thrust application point w.r.t. F origin, F frame */
    Vector3f_c tHat_F; /*!< [-]   thrust unit direction, F frame */
    float thrust;      /*!< [N]   thrust magnitude */
    float wheelSpeeds[THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW]; /*!< [r/s] reaction-wheel speeds */
} ThrusterPlatformReferenceInputs_c;

/**
 * @brief Plain-old-data mirror of the C++ ThrusterPlatformReferenceOutput.
 */
typedef struct {
    float theta1;      /*!< [rad] platform tip reference angle */
    float theta2;      /*!< [rad] platform tilt reference angle */
    Vector3f_c Lreq_B; /*!< [Nm] torque to be compensated by the RWs, B frame */
    Vector3f_c r_TB_B; /*!< [m]  thrust application point w.r.t. B origin, B frame */
    Vector3f_c tHat_B; /*!< [-]  thrust unit direction, B frame */
    float thrust;      /*!< [N]  thrust magnitude */
} ThrusterPlatformReferenceOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUSTER_PLATFORM_REFERENCE_TYPES_H
