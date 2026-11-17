#ifndef F32XMERA_SUNLINE_EPHEM_TYPES_H
#define F32XMERA_SUNLINE_EPHEM_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ SunlineEphemOutput.
 */
typedef struct {
    float rHat_SB_N[3]; /*!< [-] unit Sun direction, inertial frame */
    float rHat_SB_B[3]; /*!< [-] unit Sun direction, body frame */
} SunlineEphemOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUNLINE_EPHEM_TYPES_H
