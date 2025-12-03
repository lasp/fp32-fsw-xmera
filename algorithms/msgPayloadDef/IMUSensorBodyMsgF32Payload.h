#ifndef IMU_SENSOR_BODY_MESSAGE_F32_H
#define IMU_SENSOR_BODY_MESSAGE_F32_H

/*! @brief Output structure for IMU structure in vehicle body frame*/
typedef struct {
    float DVFrameBody[3];  //!< m/s Accumulated DVs in body
    float AccelBody[3];    //!< m/s2 Apparent acceleration of the body
    float DRFrameBody[3];  //!< r  Accumulated DRs in body
    float AngVelBody[3];   //!< r/s Angular velocity in platform body
} IMUSensorBodyMsgF32Payload;

#endif
