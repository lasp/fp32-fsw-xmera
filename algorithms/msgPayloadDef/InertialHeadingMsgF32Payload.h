/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef INERTIAL_HEADING_MSG_F32_PAYLOAD_H
#define INERTIAL_HEADING_MSG_F32_PAYLOAD_H

typedef struct {
    float rHat_XN_N[3];  //!< [] unit heading vector to thing "X" in inertial frame
} InertialHeadingMsgF32Payload;

#endif
