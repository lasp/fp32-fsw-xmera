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
