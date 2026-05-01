#ifndef ST_SENSOR_MESSAGE_F32_H
#define ST_SENSOR_MESSAGE_F32_H

/*! @brief Star tracker sensor message (F32) */
typedef struct {
    float timeTag;         //!< [s] Time tag placed on the output state
    float qInrtl2Case[4];  //!< [-] Quaternion to go from the inertial to case
    float omega_CN_C[3];   //!< [rad/s] Inertial angular velocity in case frame
} STSensorMsgF32Payload;

#endif
