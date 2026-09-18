#ifndef CSS_ARRAY_AVAILABILITY_MESSAGE_F32_H
#define CSS_ARRAY_AVAILABILITY_MESSAGE_F32_H

#include "definitions.h"
#include "utilities/fsw/deviceAvailability.h"

/*! @brief Availability state of every coarse sun sensor in the array. An unavailable sensor takes no part
 * in the sun heading estimate. */
typedef struct {
    DeviceAvailability_c cssAvailability[MAX_NUM_CSS_SENSORS];  //!< [-] availability state of each sensor
} CSSArrayAvailabilityMsgF32Payload;

#endif
