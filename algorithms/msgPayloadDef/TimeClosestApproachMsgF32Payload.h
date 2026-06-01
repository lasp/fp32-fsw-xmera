// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef TCA_MESSAGE_F32_H
#define TCA_MESSAGE_F32_H

/*! @brief structure for on-board time of closest approach during a flyby (fp32 variant) */
typedef struct {
    double timeTag;             //!< [s] Current time of validity for output
    float timeClosestApproach;  //!< [s] predicted time of closest approach in spacecraft time
    float standardDeviation;    //!< [s] time of closest approach standard deviation
} TimeClosestApproachMsgF32Payload;

#endif
