/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef BODY_HEADING_MSG_F32_PAYLOAD_H
#define BODY_HEADING_MSG_F32_PAYLOAD_H

typedef struct {
    float rHat_XB_B[3];  //!< [] unit heading vector to thing "X" in body frame
} BodyHeadingMsgF32Payload;

#endif
