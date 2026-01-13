/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef RW_CONSTELLATION_MESSAGE_F32_H
#define RW_CONSTELLATION_MESSAGE_F32_H

#include "RWConfigElementMsgF32Payload.h"
#include "definitions.h"

/*! @brief Message used to define an array of RW FSW configurations  */
typedef struct {
    int numRW;                                                //!< [-] number of RWs
    RWConfigElementMsgF32Payload reactionWheels[RW_EFF_CNT];  //!< [-] array of the reaction wheels
} RWConstellationMsgF32Payload;

#endif
