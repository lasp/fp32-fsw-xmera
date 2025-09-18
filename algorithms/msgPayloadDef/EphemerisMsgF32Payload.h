/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef EPHEMERIS_MESSAGE_F32_H
#define EPHEMERIS_MESSAGE_F32_H

/*! @brief Message structure used to write ephemeris states out to other modules*/
typedef struct {
    float r_BdyZero_N[3];  //!< [m] Position of orbital body
    float v_BdyZero_N[3];  //!< [m/s] Velocity of orbital body
    float sigma_BN[3];     //!< MRP attitude of the orbital body fixed frame relative to inertial
    float omega_BN_B[3];   //!< [r/s] angular velocity of the orbital body relative to inertial
    float timeTag;         //!< [s] vehicle Time-tag for state
} EphemerisMsgF32Payload;

#endif
