/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef RATE_COMMAND_MESSAGE_F32_H
#define RATE_COMMAND_MESSAGE_F32_H

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    float omega_BastR_B[3];   //!< [r/s]   Desired body rate relative to R
    float omegap_BastR_B[3];  //!< [r/s^2] Body-frame derivative of omega_BastR_B
} RateCmdMsgF32Payload;

#endif
