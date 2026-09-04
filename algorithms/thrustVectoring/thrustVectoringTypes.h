#ifndef F32XMERA_THRUST_VECTORING_TYPES_H
#define F32XMERA_THRUST_VECTORING_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

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
