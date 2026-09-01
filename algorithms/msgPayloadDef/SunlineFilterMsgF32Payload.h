#ifndef SUNLINE_FILTER_MESSAGE_F32_H
#define SUNLINE_FILTER_MESSAGE_F32_H

#define SKF_N_STATES 6
#define MAX_N_CSS_MEAS 32

/*! @brief structure for filter-states output from a sunline state estimator (fp32 variant). The
 weighted least squares estimator populates only timeTag, numObs, the first three entries of state,
 and postFitRes; the remaining fields exist for the Kalman filter variants that share this message. */
typedef struct {
    double timeTag;                            //!< [s] Current time of validity for output
    float covar[SKF_N_STATES * SKF_N_STATES];  //!< [-] Current covariance of the filter
    float state[SKF_N_STATES];                 //!< [-] Current estimated state of the filter
    float stateError[SKF_N_STATES];            //!< [-] Current deviation of the state from the reference state
    float postFitRes[MAX_N_CSS_MEAS];          //!< [-] PostFit Residuals
    int numObs;                                //!< [-] Valid observation count for this frame
} SunlineFilterMsgF32Payload;

#endif
