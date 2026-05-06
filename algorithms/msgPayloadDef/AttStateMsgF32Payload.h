#ifndef ATT_STATE_MESSAGE_F32_H
#define ATT_STATE_MESSAGE_F32_H

/*! @brief Structure used to define the euler set for attitude reference generation */
typedef struct {
    float state[3];  //!< [] 3D attitude orientation coordinate set. Units depend on the chosen attitude coordinate
                     //!< (e.g. radians for Euler angles, dimensionless for MRP/quaternions).
    float rate[3];   //!< [] 3D attitude rate coordinate set. May be omega (rad/s) or attitude-coordinate rates with
                     //!< appropriate units.
} AttStateMsgF32Payload;

#endif
