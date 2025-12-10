#ifndef FSW_THR_CONFIG_MESSAGE_F32_H
#define FSW_THR_CONFIG_MESSAGE_F32_H

/*! @brief Single Thruster configuration message */
typedef struct {
    float rThrust_B[3];     //!< [m] Location of the thruster in the spacecraft
    float tHatThrust_B[3];  //!< [-] Unit vector of the thrust direction
    float maxThrust;        //!< [N] Max thrust
} THRConfigMsgF32Payload;

#endif
