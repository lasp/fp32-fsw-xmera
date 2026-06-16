#ifndef F32XMERA_CSS_COMM_TYPES_H
#define F32XMERA_CSS_COMM_TYPES_H

#include "msgPayloadDef/definitions.h"

#include <stdint.h>

#define MAX_NUM_CHEBY_POLYS 11

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bounded array of CSS sensor values (double precision), used for both input and output.
 */
typedef struct {
    double data[MAX_NUM_CSS_SENSORS];
} CssSensorValues_c;

/**
 * @brief Plain-old-data mirror of the C++ CssCommConfig.
 *
 *  - numSensors must be in [1, MAX_NUM_CSS_SENSORS]
 *  - maxSensorValue must be finite and > 0
 *  - chebyCount must be in [1, MAX_NUM_CHEBY_POLYS]
 *  - chebyPolynomials coefficients must all be finite
 */
typedef struct {
    uint32_t numSensors;
    double maxSensorValue;
    uint32_t chebyCount;
    double chebyPolynomials[MAX_NUM_CHEBY_POLYS];
} CssCommConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CSS_COMM_TYPES_H
