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
 * @brief C-compatible enumeration mirroring PhaseAngleCorrectionMethodAlgorithm.
 *
 * Numeric values must stay in lockstep with the C++ enum class in cobConverterAlgorithm.h.
 */
typedef enum {
    PHASE_ANGLE_CORRECTION_METHOD_NO_CORRECTION_ALG_C = 0,
    PHASE_ANGLE_CORRECTION_METHOD_BINARY_ALG_C = 1
} PhaseAngleCorrectionMethodAlgorithm_c;

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
 * @brief Plain-old-data mirror of the C++ CobConverterConfig fields.
 *
 * Caller fills this struct and passes it to CobConverterAlgorithm_create or _setConfig.
 * The C++ side validates the fields via CobConverterConfig::create and throws on invalid input.
 */
typedef struct {
    PhaseAngleCorrectionMethodAlgorithm_c phaseAngleCorrectionMethod; /*!< [-] phase-angle correction model */
    float radius;                                                     /*!< [m] object radius (must be > 0) */
    float radiusUncertainty;                           /*!< [m] object radius uncertainty (must be >= 0) */
    Matrix3f_c attitudeCovariance;                     /*!< [-] attitude error covariance, body frame */
    float numStandardDeviations;                       /*!< [-] number of sigmas for outlier gating (must be > 0) */
    float standardDeviation;                           /*!< [-] explicit COB error standard deviation, if specified */
    bool specifiedStandardDeviation;                   /*!< [-] true if standardDeviation should be used as-is */
    bool outlierDetectionEnabled;                      /*!< [-] enable COB outlier detection */
    CalibrationCoefficients_c calibrationCoefficients; /*!< [-] Brown-Conrady distortion coefficients */
    int32_t cameraId;                                  /*!< [-] camera identifier */
    float fieldOfView;                                 /*!< [rad] camera field of view */
    float resolutionX;                                 /*!< [px] horizontal resolution */
    float resolutionY;                                 /*!< [px] vertical resolution */
    Vector3f_c bodyToCameraMrp;                        /*!< [-] MRP body-to-camera */
} CobConverterConfig_c;

/**
 * @brief Plain-old-data mirror of the C++ CobConverterInput fields.
 */
typedef struct {
    bool cobValid;                          /*!< [-] validity flag */
    int32_t cobPixelsFound;                 /*!< [-] bright pixels */
    Vector2f_c cobCenterOfBrightness;       /*!< [px] COB pixel coordinates */
    uint64_t cobTimeTag;                    /*!< [ns] measurement time */
    Vector3f_c sigma_BN;                    /*!< [-] body-to-inertial MRP */
    Vector3f_c vehSunPntBdy;                /*!< [-] sun direction, body frame */
    Vector3d_c filterVehPosition;           /*!< [m] spacecraft position, inertial frame */
    Matrix3d_c filterVehPositionCovariance; /*!< [m^2] spacecraft position covariance, inertial frame */
} CobConverterInput_c;

/**
 * @brief Plain-old-data mirror of the C++ CobConverterOutput fields.
 */
typedef struct {
    Matrix3f_c covar_N;            /*!< [-] COM covariance, inertial frame */
    Matrix3f_c covar_C;            /*!< [-] COM covariance, camera frame */
    Matrix3f_c covar_B;            /*!< [-] COM covariance, body frame */
    Vector3f_c rhat_BN_N;          /*!< [-] COM unit vector, inertial frame */
    Vector3f_c rhat_BN_C;          /*!< [-] COM unit vector, camera frame */
    Vector3f_c rhat_BN_B;          /*!< [-] COM unit vector, body frame */
    double unitVecTimeTag;         /*!< [s] measurement timestamp */
    bool unitVecValid;             /*!< [-] COM unit vector validity flag */
    Vector2f_c centerOfBrightness; /*!< [px] COB pixel coordinates */
    Vector2f_c centerOfMass;       /*!< [px] COM pixel coordinates */
    float offsetFactor;            /*!< [-] phase-angle offset factor (gamma) */
    int32_t objectPixelRadius;     /*!< [px] object radius in pixels */
    float phaseAngle;              /*!< [rad] phase angle alpha_PA */
    float sunDirection;            /*!< [rad] sun direction phi in image plane */
    uint64_t comTimeTag;           /*!< [ns] measurement timestamp */
    bool comValid;                 /*!< [-] COM validity flag */
    bool coberrorOutlierTrigger;   /*!< [-] true if COB error exceeded outlier threshold */
} CobConverterOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_COB_CONVERTER_TYPES_H
