/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef CMD_TORQUE_BODY_MESSAGE_F32_H
#define CMD_TORQUE_BODY_MESSAGE_F32_H

/*! @brief Message used to define the vehicle control torque vector in Body frame components*/
typedef struct {
    float torqueRequestBody[3];  //!< [Nm] Control torque requested
} CmdTorqueBodyMsgF32Payload;

#endif
