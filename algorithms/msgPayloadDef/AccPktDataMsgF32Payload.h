#include <stdint.h>

#ifndef ACC_DATA_MESSAGE_F32_H
#define ACC_DATA_MESSAGE_F32_H

/*! @brief Structure used to define accelerometer package data */
typedef struct {
    uint64_t measTime;  //!< [Tick] Measurement time for accel
    float gyro_B[3];    //!< [r/s] Angular rate measurement from gyro
    float accel_B[3];   //!< [m/s2] Acceleration in platform frame
} AccPktDataMsgF32Payload;

#endif
