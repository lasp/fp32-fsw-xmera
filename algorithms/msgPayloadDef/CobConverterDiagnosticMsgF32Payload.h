#ifndef COB_CONVERTER_DIAGNOSTIC_MESSAGE_F32_H
#define COB_CONVERTER_DIAGNOSTIC_MESSAGE_F32_H

#include <stdint.h>

//!@brief Diagnostic telemetry from the center of brightness converter module
/*! The essential measurement -- the COM heading and its covariance in the inertial frame --
 * is published on OpNavUnitVecMsgF32Payload. Everything else lands here: the same COM heading
 * expressed in the camera and body frames, the uncorrected COB heading, the pixel-space
 * centers, and the phase angle correction metadata.
 */
typedef struct {
    uint64_t comTimeTag;          //!< -- [ns] Current vehicle time-tag associated with measurements
    bool comValid;                //!< -- Quality of the COM measurement
    int64_t cameraID;             //!< -- [-] ID of the camera that took the image
    float covar_C[3 * 3];         //!< -- [-] COM covariance in the camera frame
    float covar_B[3 * 3];         //!< -- [-] COM covariance in the body frame
    float rhat_BN_C[3];           //!< -- [-] COM measurement in the camera frame
    float rhat_BN_B[3];           //!< -- [-] COM measurement in the body frame
    float rhat_COB_C[3];          //!< -- [-] COB measurement in the camera frame
    float rhat_COB_N[3];          //!< -- [-] COB measurement in the inertial frame
    float centerOfMass[2];        //!< -- [-] Center x, y of bright pixels after correction
    float centerOfBrightness[2];  //!< -- [-] Center x, y of bright pixels
    float offsetFactor;           //!< -- [-] COM/COB offset factor as a fraction of object radius
    int32_t objectPixelRadius;    //!< -- [-] radius of object in pixels
    float phaseAngle;             //!< -- [rad] angle between Sun-Object-Camera
    float sunDirection;           //!< -- [rad] Sun direction in the image
    bool coberrorOutlierTrigger;  //!< -- true if the predicted COB error >= numStandardDeviations * standard deviations
} CobConverterDiagnosticMsgF32Payload;

#endif
