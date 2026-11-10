#ifndef COB_CONVERTER_DIAGNOSTIC_MESSAGE_F32_H
#define COB_CONVERTER_DIAGNOSTIC_MESSAGE_F32_H

#include <stdint.h>

//!@brief Diagnostic telemetry from the center of brightness converter module
/*! The essential measurement -- the COM heading and its covariance in the inertial frame --
 * is published on OpNavUnitVecMsgF32Payload. This message carries the COM pixel coordinates,
 * the phase angle correction metadata, and the outlier and undistortion flags.
 */
typedef struct {
    bool comValid;                //!< -- Quality of the COM measurement
    int64_t cameraID;             //!< -- [-] ID of the camera that took the image
    float centerOfMass[2];        //!< -- [-] Center x, y of bright pixels after correction
    float offsetFactor;           //!< -- [-] COM/COB offset factor as a fraction of object radius
    int32_t objectPixelRadius;    //!< -- [-] radius of object in pixels
    float phaseAngle;             //!< -- [rad] angle between Sun-Object-Camera
    float sunDirection;           //!< -- [rad] Sun direction in the image
    bool comErrorOutlierTrigger;  //!< -- true if the COM heading error exceeded the outlier gate
    bool brownConradyCOMValid;    //!< -- true if the COM Brown-Conrady undistortion converged
    bool brownConradyCOBValid;    //!< -- true if the COB Brown-Conrady undistortion converged
} CobConverterDiagnosticMsgF32Payload;

#endif
