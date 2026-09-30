#ifndef OPNAVUVEC_MESSAGE_F32_H
#define OPNAVUVEC_MESSAGE_F32_H

//!@brief Optical navigation message for unit vector measurements
/*! This message contains the output of any measurement method (primarily image processing)
 * that outputs a unit vector from the target to the camera. Inertial-frame quantities only;
 * the camera- and body-frame equivalents are carried on the producer's diagnostic message.
 */
typedef struct {
    double timeTag;        //!< [s] Current time of validity for output
    bool valid;            //!< Quality of measurement if 1, invalid if 0
    float covar_N[3 * 3];  //!< [-] Covariance of measurement in the inertial frame
    float rhat_BN_N[3];    //!< [-] measurement in the inertial frame
} OpNavUnitVecMsgF32Payload;

#endif
