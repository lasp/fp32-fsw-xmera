#ifndef F32XMERA_TIME_CLOSEST_APPROACH_TYPES_H
#define F32XMERA_TIME_CLOSEST_APPROACH_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POD output of the TCA algorithm.
 */
typedef struct {
    float tCA;      /*!< [s] Time of closest approach estimate */
    float sigmaTca; /*!< [s] Standard deviation of the TCA estimate */
} TimeClosestApproachOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_TIME_CLOSEST_APPROACH_TYPES_H
