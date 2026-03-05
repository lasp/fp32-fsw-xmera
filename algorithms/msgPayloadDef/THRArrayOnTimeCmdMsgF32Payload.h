#ifndef THR_ON_TIME_MESSAGE_F32_H
#define THR_ON_TIME_MESSAGE_F32_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    float onTimeRequest[MAX_EFF_CNT];  //!< - Control request fraction array
} THRArrayOnTimeCmdMsgF32Payload;

#endif
