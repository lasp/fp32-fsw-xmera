/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef RW_CONFIG_MESSAGE_F32_H
#define RW_CONFIG_MESSAGE_F32_H

#include "architecture/msgPayloadDef/definitions.h"
#include <stdint.h>

/*! @brief RW array configuration FSW msg */
typedef struct {
    float GsMatrix_B[3 * RW_EFF_CNT];  //!< [-]    The RW spin axis matrix in body frame components
    float JsList[RW_EFF_CNT];          //!< [kgm2] The spin axis inertia for RWs
    int numRW;                         //!< [-]    The number of reaction wheels available on vehicle
    float uMax[RW_EFF_CNT];            //!< [Nm]   The maximum RW motor torque
} RWArrayConfigMsgF32Payload;

#endif
