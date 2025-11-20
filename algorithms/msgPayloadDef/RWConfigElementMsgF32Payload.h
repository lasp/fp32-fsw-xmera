/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef RW_CONFIG_ELEMENT_MESSAGE_F32_H
#define RW_CONFIG_ELEMENT_MESSAGE_F32_H

/*! @brief Message used to define a single FSW RW configuration message */
typedef struct {
    float gsHat_B[3];  //!< [-] Spin axis unit vector of the wheel in structure
    float Js;          //!< [kgm2] Spin axis inertia of the wheel
    float uMax;        //!< [Nm]   maximum RW motor torque
} RWConfigElementMsgF32Payload;

#endif
