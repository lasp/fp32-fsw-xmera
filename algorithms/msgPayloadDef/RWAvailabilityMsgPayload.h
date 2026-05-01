#ifndef RW_AVAILABILITY_MSG_PAYLOAD_H
#define RW_AVAILABILITY_MSG_PAYLOAD_H

#include "definitions.h"
#include <stdint.h>

/*! @brief Reaction wheel availability states */
enum RWAvailability { AVAILABLE = 0, UNAVAILABLE = 1 };

/*! @brief RW availability message payload */
typedef struct {
    int32_t wheelAvailability[RW_EFF_CNT];  //!< [-] Availability status for each reaction wheel
} RWAvailabilityMsgPayload;

#endif
