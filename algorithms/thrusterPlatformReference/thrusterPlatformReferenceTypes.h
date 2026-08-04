#ifndef F32XMERA_THRUSTER_PLATFORM_REFERENCE_TYPES_H
#define F32XMERA_THRUSTER_PLATFORM_REFERENCE_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ ThrusterPlatformReferenceRwArrayConfiguration.
 */
typedef struct {
    uint32_t numRW;                   /*!< [-]    number of reaction wheels on the vehicle */
    float GsMatrix_B[3 * RW_EFF_CNT]; /*!< [-]   RW spin axes in body frame, three per wheel */
    float JsList[RW_EFF_CNT];         /*!< [kgm2] RW spin-axis inertias */
} ThrusterPlatformReferenceRwArrayConfiguration_c;

/**
 * @brief Plain-old-data mirror of the C++ ThrusterPlatformReferenceInputs.
 */
typedef struct {
    Vector3f_c r_CB_B;             /*!< [m]   center of mass w.r.t. B origin, B frame */
    Vector3f_c r_TF_F;             /*!< [m]   thrust application point w.r.t. F origin, F frame */
    Vector3f_c tHat_F;             /*!< [-]   thrust unit direction, F frame */
    float thrust;                  /*!< [N]   thrust magnitude */
    float wheelSpeeds[RW_EFF_CNT]; /*!< [r/s] reaction-wheel speeds */
} ThrusterPlatformReferenceInputs_c;

/**
 * @brief Plain-old-data mirror of the C++ ThrusterPlatformReferenceOutput.
 */
typedef struct {
    Vector3f_c Lcomp_B; /*!< [Nm] torque to be compensated by the RWs, B frame */
    Vector3f_c r_TB_B;  /*!< [m]  thrust application point w.r.t. B origin, B frame */
    Vector3f_c tHat_B;  /*!< [-]  thrust unit direction, B frame */
    float thrust;       /*!< [N]  thrust magnitude */
} ThrusterPlatformReferenceOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUSTER_PLATFORM_REFERENCE_TYPES_H
