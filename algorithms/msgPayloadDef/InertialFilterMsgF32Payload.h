#ifndef INERTIAL_FILTER_MESSAGE_F32_H
#define INERTIAL_FILTER_MESSAGE_F32_H

#define AKF_N_STATES_F32 6

/*! @brief structure for filter-states output for the unscented kalman filter
implementation of the inertial state estimator*/
typedef struct {
    double timeTag;                                    //!< [s] Current time of validity for output
    float covar[AKF_N_STATES_F32 * AKF_N_STATES_F32];  //!< [-] Current covariance of the filter
    float state[AKF_N_STATES_F32];                     //!< [-] Current estimated state of the filter
    int numObs;                                        //!< [-] Valid observation count for this frame
} InertialFilterMsgF32Payload;

#endif
