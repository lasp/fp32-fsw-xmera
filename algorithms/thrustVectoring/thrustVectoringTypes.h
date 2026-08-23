#ifndef F32XMERA_THRUST_VECTORING_TYPES_H
#define F32XMERA_THRUST_VECTORING_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of reaction wheels handled at the C boundary. Must match RW_EFF_CNT in
   msgPayloadDef/definitions.h (enforced by a static_assert in the adapter). */
#define THRUST_VECTORING_MAX_NUM_RW 36

/**
 * @brief Plain-old-data mirror of the C++ ThrustVectoringRwArrayConfiguration.
 */
typedef struct {
    uint32_t numRW;                                    /*!< [-]    number of reaction wheels on the vehicle */
    float GsMatrix_B[3 * THRUST_VECTORING_MAX_NUM_RW]; /*!< [-]   RW spin axes in body frame, three per wheel */
    float JsList[THRUST_VECTORING_MAX_NUM_RW];         /*!< [kgm2] RW spin-axis inertias */
} ThrustVectoringRwArrayConfiguration_c;

/**
 * @brief Plain-old-data mirror of the C++ ThrustVectoringInputs.
 */
typedef struct {
    Vector3f_c r_CB_B;                              /*!< [m]   center of mass w.r.t. B origin, B frame */
    Vector3f_c r_TF_F;                              /*!< [m]   thrust application point w.r.t. F origin, F frame */
    Vector3f_c tHat_F;                              /*!< [-]   thrust unit direction, F frame */
    float thrust;                                   /*!< [N]   thrust magnitude */
    float wheelSpeeds[THRUST_VECTORING_MAX_NUM_RW]; /*!< [r/s] reaction-wheel speeds */
} ThrustVectoringInputs_c;

/**
 * @brief Plain-old-data mirror of the C++ ThrustVectoringOutput.
 */
typedef struct {
    Vector3f_c r_TB_B; /*!< [m]  thrust application point w.r.t. B origin, B frame */
    Vector3f_c tHat_B; /*!< [-]  thrust unit direction, B frame */
    float thrust;      /*!< [N]  thrust magnitude */
} ThrustVectoringOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUST_VECTORING_TYPES_H
