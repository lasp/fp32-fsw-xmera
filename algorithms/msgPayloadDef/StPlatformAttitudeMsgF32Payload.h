#ifndef ST_PLATFORM_ATTITUDE_MSG_F32_PAYLOAD_H
#define ST_PLATFORM_ATTITUDE_MSG_F32_PAYLOAD_H

/*! @brief Star tracker platform attitude solution (F32). */
typedef struct {
    float platformAttitude[4];         //!< [-] q_PN, inertial-to-platform quaternion (scalar-last)
    float platformAngularVelocity[4];  //!< [-] dq_PN, platform-frame rate encoded as a quaternion (scalar-last)
    double solutionTimestamp;          //!< [s] solution time tag (center of image integration)
} StPlatformAttitudeMsgF32Payload;

#endif
