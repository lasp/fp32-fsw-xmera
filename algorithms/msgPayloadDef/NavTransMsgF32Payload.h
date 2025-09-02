/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef NAV_TRANS_F32_MESSAGE_H
#define NAV_TRANS_F32_MESSAGE_H

/*! @brief Structure used to define the output definition for translatoin guidance*/
typedef struct {
    float timeTag;        //!< [s]   Current vehicle time-tag associated with measurements*/
    float r_BN_N[3];      //!< [m]   Current inertial spacecraft position vector in inertial frame N components
    float v_BN_N[3];      //!< [m/s] Current inertial velocity of the spacecraft in inertial frame N components
    float vehAccumDV[3];  //!< [m/s] Total accumulated delta-velocity for s/c
} NavTransMsgF32Payload;

#endif
