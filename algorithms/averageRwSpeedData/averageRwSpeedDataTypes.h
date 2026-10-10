#ifndef F32XMERA_AVERAGE_RW_SPEED_DATA_TYPES_H
#define F32XMERA_AVERAGE_RW_SPEED_DATA_TYPES_H

#include "msgPayloadDef/definitions.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POD equivalent of RwSpeedSample.
 *
 * All wheels share the measurement time. A sample with measTime 0 is not ingested.
 */
typedef struct {
    uint64_t measTime;             /*!< [ns] measurement time of the sample */
    float wheelSpeeds[RW_EFF_CNT]; /*!< [r/s] reaction-wheel speeds */
} RwSpeedSample_c;

/**
 * @brief Plain-old-data carrier for the averaged reaction-wheel speeds returned by update().
 */
typedef struct {
    float wheelSpeeds[RW_EFF_CNT]; /*!< [r/s] averaged reaction-wheel speeds */
} AverageRwSpeeds_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_AVERAGE_RW_SPEED_DATA_TYPES_H
