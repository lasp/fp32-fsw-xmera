#ifndef TWO_AXIS_GIMBAL_MESSAGE_F32_H
#define TWO_AXIS_GIMBAL_MESSAGE_F32_H

/*! @brief Structure used to define the two-axis gimbal state information (FP32). */
typedef struct {
    float theta1;        //!< [rad] Sequential rotation angle about first rotation axis
    float theta2;        //!< [rad] Sequential rotation angle about second rotation axis
    int stepsCommanded;  //!< Current number of commanded gimbal steps
    int stepCount;       //!< Current gimbal step count (number of steps taken)
} TwoAxisGimbalMsgF32Payload;

#endif /* TWO_AXIS_GIMBAL_MESSAGE_F32_H */
