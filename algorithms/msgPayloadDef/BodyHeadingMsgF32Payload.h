#ifndef BODY_HEADING_MESSAGE_F32_H
#define BODY_HEADING_MESSAGE_F32_H

/*! @brief Body-frame heading message (FP32). */
typedef struct {
    float rHat_XB_B[3];  //!< [] unit heading vector to any thing "X" in the spacecraft, "B", body frame
} BodyHeadingMsgF32Payload;

#endif /* BODY_HEADING_MESSAGE_F32_H */
