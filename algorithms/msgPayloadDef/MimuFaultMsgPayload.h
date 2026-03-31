#ifndef mimuFaultMsg_h
#define mimuFaultMsg_h

#include "definitions.h"

#include <cstdint>

/*! @brief Structure used to define the mimu fault message */
typedef struct {
    bool faultDetected;                     //!< fault detected bool
    bool validImus[kMimuCount];             //!< valid IMU flags for each IMU
    float omegaDifferencesMag[kMimuCount];  //!< [rad/s] magnitude of each IMU's difference from the 3-IMU average
} MimuFaultMsgPayload;

#endif /* mimuFaultMsg_h */
