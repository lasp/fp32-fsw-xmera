#ifndef F32XMERA_FLYBY_POINT_TYPES_H
#define F32XMERA_FLYBY_POINT_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C-compatible mirror of the C++ FlybyPointConfig fields.
 *
 * Pass this struct to FlybyPointAlgorithm_create or FlybyPointAlgorithm_setConfig.
 * The C++ side validates each field via FlybyPointConfig::create and throws on
 * invalid input.
 */
typedef struct {
    double timeBetweenFilterData;       /*!< [s]       minimum time between filter data reads (> 0) */
    float toleranceForCollinearity;     /*!< [-]       collinearity tolerance threshold (> 0) */
    int signOfOrbitNormalFrameVector;   /*!< [-]       sign of orbit-normal reference vector (+1 or -1) */
    float maximumRateThreshold;         /*!< [deg/s]   maximum allowed predicted angular rate (> 0) */
    float maximumAccelerationThreshold; /*!< [deg/s^2] maximum allowed predicted angular acceleration (> 0) */
    float positionKnowledgeSigma;       /*!< [m]       position knowledge sigma bound (> 0) */
} FlybyPointConfig_c;

/**
 * @brief C-compatible mirror of the C++ AttGuideOutput.
 */
typedef struct {
    Vector3f_c sigma_RN;                 /*!< [-]       MRP reference attitude */
    Vector3f_c omega_RN_N;               /*!< [rad/s]   reference angular velocity in inertial frame */
    Vector3f_c domega_RN_N;              /*!< [rad/s^2] reference angular acceleration in inertial frame */
    bool collinearityTrigger;            /*!< true if r and v are collinear (collision trajectory) */
    bool maxRateTrigger;                 /*!< true if the predicted rate exceeds the maximum threshold */
    bool maxAccelerationTrigger;         /*!< true if the predicted acceleration exceeds the maximum threshold */
    bool positionKnowledgeExceedTrigger; /*!< true if the position error exceeds the a-priori sigma bound */
} AttGuideOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_FLYBY_POINT_TYPES_H
