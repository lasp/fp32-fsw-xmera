#ifndef HINGED_RIGID_BODY_MESSAGE_F32_H
#define HINGED_RIGID_BODY_MESSAGE_F32_H

/*! @brief Structure used to define the individual Hinged Rigid Body  data message*/
typedef struct {
    float theta;     //!< [rad], panel angular displacement
    float thetaDot;  //!< [rad/s], panel angular displacement rate
} HingedRigidBodyMsgF32Payload;

#endif /* HINGED_RIGID_BODY_MESSAGE_F32_H */
