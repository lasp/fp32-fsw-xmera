#ifndef ST_ATTITUDE_MESSAGE_F32_H
#define ST_ATTITUDE_MESSAGE_F32_H

/*! @brief Output structure for ST attitude measurement in vehicle body frame*/
typedef struct {
    float timeTag;          //!< [s] Vehicle time code associated with measurement
    float MRP_BdyInrtl[3];  //!< [-] MRP estimate of inertial to body transformation
    float omega_BN_B[3];    //!< [rad/s] Platform inertial angular velocity
    float dcm_CB[9];        //!< Star Tracker mount frame in the body frame
} STAttMsgF32Payload;

#endif
