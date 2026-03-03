#ifndef THR_ARRAY_CMD_FORCE_MESSAGE_F32_H
#define THR_ARRAY_CMD_FORCE_MESSAGE_F32_H

#include "definitions.h"

/*! @brief Message used to define a vector of thruster force commands */
typedef struct {
    float thrForce[MAX_EFF_CNT];  //!< [N] array of thruster force values
} THRArrayCmdForceMsgF32Payload;

#endif
