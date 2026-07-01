#ifndef COMOPNAV_MESSAGE_F32_H
#define COMOPNAV_MESSAGE_F32_H

//!@brief Center of mass optical navigation measurement message
/*! This message is output by the center of brightness converter module and contains the center of mass (COM) after
 * offsetting the center of brightness (COB) using the phase angle correction.
 */
typedef struct {
    uint64_t timeTag;             //!< --[ns] Current vehicle time-tag associated with measurements
    bool valid;                   //!< -- Quality of measurement
    int64_t cameraID;             //!< -- [-] ID of the camera that took the image
    float centerOfMass[2];        //!< -- [-] Center x, y of bright pixels after correction
    float centerOfBrightness[2];  //!< -- [-] Center x, y of bright pixels
    float offsetFactor;           //!< -- [-] COM/COB offset factor as a fraction of object radius
    int32_t objectPixelRadius;    //!< -- [-] radius of object in pixels
    float phaseAngle;             //!< -- [rad] angle between Sun-Object-Camera
    float sunDirection;           //!< -- [rad] Sun direction in the image
} OpNavCOMMsgF32Payload;

#endif
