#ifndef ASC_PLATFORM_ATTITUDE_TM_MSG_F32_PAYLOAD_H
#define ASC_PLATFORM_ATTITUDE_TM_MSG_F32_PAYLOAD_H

#include <stdbool.h>
#include <stdint.h>

/*! @brief Which camera heads (str) and gyro units (miru) contributed to the solution. */
typedef struct {
    bool miruD;
    bool miruC;
    bool miruB;
    bool miruA;
    bool strD;
    bool strC;
    bool strB;
    bool strA;
} AscSolutionSourcesF32;

/*! @brief Platform solution health information. */
typedef struct {
    uint32_t propagationCounter;        //!< [-] up to 2000 s
    uint8_t residual;                   //!< [-] 0 - 255
    AscSolutionSourcesF32 sources;      //!< [-] contributing sensors
    bool solutionValidAngularVelocity;  //!< [-] rate solution valid
    bool solutionValidAttitude;         //!< [-] attitude solution valid
} AscPlatformSolutionInfoF32;

/*! @brief micro-ASC platform attitude telemetry packet (F32). */
typedef struct {
    double dataFieldHeaderTimestamp;   //!< [s] packet header time
    uint16_t dpuApId;                  //!< [-] DPU application process ID
    uint16_t sequenceCtlCount;         //!< [-] packet sequence count
    uint16_t reportId;                 //!< [-] report ID (platform attitude)
    float platformAttitude[4];         //!< [-] q_CN, inertial-to-case quaternion (scalar-last)
    float platformAngularVelocity[4];  //!< [-] dq_CN, case-frame rate encoded as a quaternion (scalar-last)
    float linearAcceleration[3];       //!< [m/s^2] linear acceleration
    AscPlatformSolutionInfoF32 info;   //!< [-] solution health
    double solutionTimestamp;          //!< [s] solution time tag (center of image integration)
} AscPlatformAttitudeTmMsgF32Payload;

#endif
