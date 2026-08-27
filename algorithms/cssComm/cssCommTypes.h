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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CSS_COMM_TYPES_H
