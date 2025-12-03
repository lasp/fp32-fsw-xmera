#ifndef ACC_PKT_DATA_MESSAGE_F32_H
#define ACC_PKT_DATA_MESSAGE_F32_H

#define MAX_ACC_BUF_PKT 120

#include "AccPktDataMsgF32Payload.h"

/*! @brief Structure used to define accelerometer package data */
typedef struct {
    AccPktDataMsgF32Payload accPkts[MAX_ACC_BUF_PKT];  //!< [-] Accelerometer buffer read in
} AccDataMsgF32Payload;

#endif
