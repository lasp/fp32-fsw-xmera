#ifndef CMD_FORCE_BODY_MESSAGE_F32_H
#define CMD_FORCE_BODY_MESSAGE_F32_H

/*! @brief Message used to define the vehicle control force vector in Body frame components*/
typedef struct {
    float forceRequestBody[3];  //!< [N] Control force requested
} CmdForceBodyMsgF32Payload;

#endif
