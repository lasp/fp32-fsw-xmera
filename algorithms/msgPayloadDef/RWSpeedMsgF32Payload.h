/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef RW_SPEED_MESSAGE_F32_H
#define RW_SPEED_MESSAGE_F32_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for reaction wheel speeds*/
typedef struct {
    float wheelSpeeds[RW_EFF_CNT];  //!< r/s The current angular velocities of the RW wheel
    float wheelThetas[RW_EFF_CNT];  //!< rad The current angle of the RW if jitter is enabled
} RWSpeedMsgF32Payload;

#endif
