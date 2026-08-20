#ifndef FILTER_RES_MESSAGE_F32_H
#define FILTER_RES_MESSAGE_F32_H

#include "definitions.h"

/*! @brief Filter residuals message containing the post- and pre-fit residuals from the filtering
 * process for a given measurement. */
typedef struct {
    double timeTag;                                                     //!< [s] Current time of validity for output
    bool valid;                                                         //!< Quality of measurement if 1, invalid if 0
    int numberOfObservations;                                           //!< Number of observations in this message
    int sizeOfObservations;                                             //!< Size of observation vector in this message
    double observation[kMaxMeasurementVector * kMaxMeasurementNumber];  //!< Measurement values processed
    double preFits[kMaxMeasurementVector * kMaxMeasurementNumber];      //!< Measurement prefit residuals
    double postFits[kMaxMeasurementVector * kMaxMeasurementNumber];     //!< Measurement postfit residuals
} FilterResidualsMsgF32Payload;

#endif  // FILTER_RES_MESSAGE_F32_H
