#ifndef F32XMERA_INERTIALFILTERTYPES_H
#define F32XMERA_INERTIALFILTERTYPES_H

#include <stdbool.h>
#include <stdint.h>

#define INERTIAL_FILTER_NUM_STATES 6 /* MRP attitude(3) + body angular rate(3) */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C mirror of the validated InertialFilterConfig inputs.
 *
 * The fields correspond one-to-one to InertialFilterConfig::create(...); the shim converts this POD
 * and calls create(), which validates and throws on invalid input. Matrices are row-major.
 */
typedef struct {
    double alpha;                                                                /*!< sigma-point spread */
    double beta;                                                                 /*!< prior-knowledge tunable */
    double processNoise[INERTIAL_FILTER_NUM_STATES][INERTIAL_FILTER_NUM_STATES]; /*!< N x N process noise Q (PSD) */
    double initialState[INERTIAL_FILTER_NUM_STATES];                             /*!< initial state seed */
    double initialCovariance[INERTIAL_FILTER_NUM_STATES]
                            [INERTIAL_FILTER_NUM_STATES]; /*!< N x N initial covariance P0 (PSD) */
    double stMeasurementNoiseStd;                         /*!< star-tracker attitude meas. noise std, >=0 */
    double gyroMeasurementNoiseStd;                       /*!< gyro rate meas. noise std, >=0 */
} InertialFilterConfig_c;

/**
 * @brief C mirror of a raw star-tracker attitude reading. timeTag > 0 flags a fresh reading.
 */
typedef struct {
    double timeTag;     /*!< [s] measurement time tag */
    double sigma_BN[3]; /*!< MRP attitude of body w.r.t. inertial frame */
} StAttData_c;

/**
 * @brief C mirror of a raw gyro reading. timeTag > 0 flags a fresh reading.
 */
typedef struct {
    double timeTag; /*!< [s] measurement time tag */
    double rate[3]; /*!< body angular rate (rad/s) */
} RateData_c;

/**
 * @brief C mirror of a per-cycle residual snapshot (valid only when that measurement fired).
 */
typedef struct {
    bool valid;            /*!< true iff this measurement fired and was applied this cycle */
    double observation[3]; /*!< measured value */
    double preFit[3];      /*!< pre-fit residual */
    double postFit[3];     /*!< post-fit residual */
} InertialResiduals_c;

/**
 * @brief C mirror of the post-update filter snapshot.
 */
typedef struct {
    double state[INERTIAL_FILTER_NUM_STATES];                                  /*!< estimated state */
    double covariance[INERTIAL_FILTER_NUM_STATES][INERTIAL_FILTER_NUM_STATES]; /*!< N x N covariance (row-major) */
    InertialResiduals_c stAttResiduals;                                        /*!< star-tracker attitude residuals */
    InertialResiduals_c rateResiduals;                                         /*!< gyro rate residuals */
} InertialFilterOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_INERTIALFILTERTYPES_H
