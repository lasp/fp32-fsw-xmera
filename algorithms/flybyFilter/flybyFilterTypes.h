#ifndef F32XMERA_FLYBYFILTERTYPES_H
#define F32XMERA_FLYBYFILTERTYPES_H

#include <stdbool.h>
#include <stdint.h>

#define FLYBY_FILTER_NUM_STATES 6 /* position(3) + velocity(3) */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C mirror of the validated FlybyFilterConfig inputs (internal km / km/s units).
 *
 * The fields correspond one-to-one to FlybyFilterConfig::create(...); the shim converts this POD
 * and calls create(), which validates and throws on invalid input. Matrices are row-major.
 */
typedef struct {
    double alpha; /*!< sigma-point spread, (0,1) */
    double beta;  /*!< prior-knowledge tunable, [0,2] */
    double mu;    /*!< central-body grav. param [km^3/s^2], >0 */
    double processNoise[FLYBY_FILTER_NUM_STATES][FLYBY_FILTER_NUM_STATES]; /*!< N x N process noise Q (PSD) */
    double initialState[FLYBY_FILTER_NUM_STATES];                          /*!< [km, km/s] initial state seed */
    double initialCovariance[FLYBY_FILTER_NUM_STATES]
                            [FLYBY_FILTER_NUM_STATES]; /*!< N x N initial covariance P0 (PSD) */
    double headingMeasurementNoiseStd;                 /*!< heading meas. noise std, >=0 */
} FlybyFilterConfig_c;

/**
 * @brief C mirror of a raw optical-navigation heading reading. timeTag > 0 flags a fresh reading.
 */
typedef struct {
    double timeTag;      /*!< [s] measurement time tag */
    double rhat_BN_N[3]; /*!< heading unit vector (spacecraft -> body, inertial frame) */
} FlybyHeadingData_c;

/**
 * @brief C mirror of the per-cycle heading residual snapshot (valid only when a measurement fired).
 */
typedef struct {
    bool valid;            /*!< true iff a heading measurement fired and was applied this cycle */
    double observation[3]; /*!< measured heading unit vector */
    double preFit[3];      /*!< pre-fit residual */
    double postFit[3];     /*!< post-fit residual */
} FlybyHeadingResiduals_c;

/**
 * @brief C mirror of the post-update filter snapshot (internal km / km/s units).
 */
typedef struct {
    double state[FLYBY_FILTER_NUM_STATES];                               /*!< [km, km/s] estimated state */
    double covariance[FLYBY_FILTER_NUM_STATES][FLYBY_FILTER_NUM_STATES]; /*!< N x N covariance (row-major) */
    FlybyHeadingResiduals_c headingResiduals;                            /*!< heading residual snapshot */
} FlybyFilterOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_FLYBYFILTERTYPES_H
