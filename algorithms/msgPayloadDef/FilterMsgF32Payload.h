// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTER_MESSAGE_F32_H
#define FILTER_MESSAGE_F32_H

#include "fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h"

/*! @brief structure for filter-states output from a filter (fp32 variant) */
typedef struct {
    double timeTag;                                       //!< [s]  Current time of validity for output
    int numberOfStates;                                   //!< [-]  Number of scalar states in the state vector
    double covar[MAX_STATES_VECTOR * MAX_STATES_VECTOR];  //!< [-]  Current covariance of the filter
    float state[MAX_STATES_VECTOR];                       //!< [-]  Current estimated state of the filter
    float stateError[MAX_STATES_VECTOR];  //!< [-]  Current deviation of the state from the reference state
} FilterMsgF32Payload;

#endif
