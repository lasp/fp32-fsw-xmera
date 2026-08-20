#ifndef RW_CONFIG_MESSAGE_F32_H
#define RW_CONFIG_MESSAGE_F32_H

#include "definitions.h"
#include <stdint.h>

/*! @brief RW array configuration FSW msg */
typedef struct {
    float GsMatrix_B[3 * kMaxNumRw];  //!< [-]    The RW spin axis matrix in body frame components
    float JsList[kMaxNumRw];          //!< [kgm2] The spin axis inertia for RWs
    int numRW;                        //!< [-]    The number of reaction wheels available on vehicle
    float uMax[kMaxNumRw];            //!< [Nm]   The maximum RW motor torque
} RWArrayConfigMsgF32Payload;

#endif
