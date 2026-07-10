#ifndef mimuFaultMsg_h
#define mimuFaultMsg_h

#include "definitions.h"

#include <cstdint>

/*! @brief Structure used to define the mimu fault message */
typedef struct {
    bool gyroFaultDetected;                   //!< gyro fault detected bool
    bool gyroImuValid[kMimuCount];            //!< gyro-valid flag for each IMU
    float gyroImuDifferenceMag[kMimuCount];   //!< [rad/s] each IMU's angular-velocity difference magnitude
    bool accelFaultDetected;                  //!< accel fault detected bool
    bool accelImuValid[kMimuCount];           //!< accel-valid flag for each IMU
    float accelImuDifferenceMag[kMimuCount];  //!< [m/s^2] each IMU's acceleration difference magnitude
} MimuFaultMsgPayload;

#endif /* mimuFaultMsg_h */
