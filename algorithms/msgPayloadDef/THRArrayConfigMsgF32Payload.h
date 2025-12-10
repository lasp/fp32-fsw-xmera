#ifndef THR_ARRAY_MESSAGE_F32_H
#define THR_ARRAY_MESSAGE_F32_H

#include "THRConfigMsgF32Payload.h"
#include <architecture/msgPayloadDef/definitions.h>

/*! @brief FSW message definition containing the thruster cluster information */
typedef struct {
    uint32_t numThrusters;                       //!< [-] number of thrusters
    THRConfigMsgF32Payload thrusters[MAX_EFF_CNT];  //!< [-] array of thruster configuration information
} THRArrayConfigMsgF32Payload;

#endif
