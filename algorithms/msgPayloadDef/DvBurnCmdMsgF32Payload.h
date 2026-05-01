#ifndef DV_BURN_CMD_F32_MESSAGE_H
#define DV_BURN_CMD_F32_MESSAGE_H

#include <stdint.h>

/*! @brief Input burn command structure used to configure a delta-V burn. */
typedef struct {
    float dvInrtlCmd[3];     //!< [m/s] Commanded delta-V vector in inertial coordinates
    float dvRotVecUnit[3];   //!< [-]   Unit vector defining the burn frame's rotation axis seed
    float dvRotVecMag;       //!< [r/s] Constant rotation rate of the burn frame about its 3rd axis
    uint64_t burnStartTime;  //!< [ns]  Time at which the burn starts
} DvBurnCmdMsgF32Payload;

#endif
