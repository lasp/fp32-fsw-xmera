#ifndef F32XMERA_COB_CONVERTER_TYPES_H
#define F32XMERA_COB_CONVERTER_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POD representation of a 2-vector (Eigen::Vector2f), e.g. pixel coordinates.
 */
typedef struct {
    float data[2];
} Vector2f_c;

/**
 * @brief POD representation of a double-precision 3x3 matrix (Eigen::Matrix3d), row-major.
 */
typedef struct {
    double data[3][3];
} Matrix3d_c;

/**
 * @brief Plain-old-data mirror of the C++ CalibrationCoefficients fields.
 */
typedef struct {
    float k1; /*!< [-] 1st radial distortion coefficient */
    float k2; /*!< [-] 2nd radial distortion coefficient */
    float k3; /*!< [-] 3rd radial distortion coefficient */
    float p1; /*!< [-] 1st tangential distortion coefficient */
    float p2; /*!< [-] 2nd tangential distortion coefficient */
} CalibrationCoefficients_c;

/**
 * @brief Plain-old-data mirror of the C++ CobConverterOutput and CobConverterDiagnosticOutput
 *        fields. Kept flat so the C entry point stays a single by-value return.
 */
typedef struct {
    /* Essential output: inertial frame only. */
    Matrix3f_c covar_N;    /*!< [-] COM covariance, inertial frame */
    Vector3f_c rhat_BN_N;  /*!< [-] COM unit vector, inertial frame */
    double unitVecTimeTag; /*!< [s] measurement timestamp */
    bool unitVecValid;     /*!< [-] COM unit vector validity flag */

    /* Diagnostic output. */
    Matrix3f_c covar_C;            /*!< [-] COM covariance, camera frame */
    Matrix3f_c covar_B;            /*!< [-] COM covariance, body frame */
    Vector3f_c rhat_BN_C;          /*!< [-] COM unit vector, camera frame */
    Vector3f_c rhat_BN_B;          /*!< [-] COM unit vector, body frame */
    Vector3f_c rhat_COB_C;         /*!< [-] COB unit vector, camera frame */
    Vector3f_c rhat_COB_N;         /*!< [-] COB unit vector, inertial frame */
    Vector2f_c centerOfBrightness; /*!< [px] COB pixel coordinates */
    Vector2f_c centerOfMass;       /*!< [px] COM pixel coordinates */
    float offsetFactor;            /*!< [-] phase-angle offset factor (gamma) */
    int32_t objectPixelRadius;     /*!< [px] object radius in pixels */
    float phaseAngle;              /*!< [rad] phase angle alpha_PA */
    float sunDirection;            /*!< [rad] sun direction phi in image plane */
    uint64_t comTimeTag;           /*!< [ns] measurement timestamp */
    bool comValid;                 /*!< [-] COM validity flag */
    bool comErrorOutlierTrigger;   /*!< [-] true if the COM heading error exceeded the gate */
} CobConverterOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_COB_CONVERTER_TYPES_H
