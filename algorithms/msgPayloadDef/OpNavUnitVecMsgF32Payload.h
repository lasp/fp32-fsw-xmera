#ifndef OPNAVUVEC_MESSAGE_F32_H
#define OPNAVUVEC_MESSAGE_F32_H

//!@brief Optical navigation message for unit vector measurements
/*! This message contains the output of any measurement method (primarily image processing)
 * that outputs a unit vector from the target to the camera.
 */
typedef struct {
    double timeTag;        //!< [s] Current time of validity for output
    bool valid;            //!< Quality of measurement if 1, invalid if 0
    float covar_N[3 * 3];  //!< [-] Covariance of measurement in the inertial frame
    float covar_B[3 * 3];  //!< [-] Covariance of measurement in the body frame
    float covar_C[3 * 3];  //!< [-] Covariance of measurement in the camera frame
    float rhat_BN_N[3];    //!< [-] measurement in the inertial frame
    float rhat_BN_B[3];    //!< [-] measurement in the body frame
    float rhat_BN_C[3];    //!< [-] measurement in the camera frame
} OpNavUnitVecMsgF32Payload;

#endif
