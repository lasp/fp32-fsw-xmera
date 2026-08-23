#ifndef F32XMERA_THRUST_VECTORING_TYPES_H
#define F32XMERA_THRUST_VECTORING_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ ThrustVectoringInputs.
 */
typedef struct {
    Vector3f_c r_CB_B; /*!< [m]  center of mass w.r.t. B origin, B frame */
    Vector3f_c r_TF_F; /*!< [m]  thrust application point w.r.t. F origin, F frame */
    Vector3f_c tHat_F; /*!< [-]  thrust unit direction, F frame */
    float thrust;      /*!< [N]  thrust magnitude */
    Vector3f_c Lreq_B; /*!< [Nm] requested thruster torque about the center of mass, B frame */
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
