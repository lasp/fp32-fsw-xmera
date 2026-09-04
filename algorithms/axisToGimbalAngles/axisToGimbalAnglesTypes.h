#ifndef F32XMERA_AXIS_TO_GIMBAL_ANGLES_TYPES_H
#define F32XMERA_AXIS_TO_GIMBAL_ANGLES_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ AxisToGimbalAnglesOutput.
 */
typedef struct {
    float gimbalAngle1; /*!< [rad] inclination of the thrust axis projected into the mount y-z plane */
    float gimbalAngle2; /*!< [rad] inclination of the thrust axis projected into the mount x-z plane */
} AxisToGimbalAnglesOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_AXIS_TO_GIMBAL_ANGLES_TYPES_H
