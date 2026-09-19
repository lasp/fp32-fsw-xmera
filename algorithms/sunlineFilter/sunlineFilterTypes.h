#ifndef F32XMERA_SUNLINEFILTER_TYPES_H
#define F32XMERA_SUNLINEFILTER_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SUNLINE_FILTER_MAX_CSS 8    /* Maximum number of coarse sun sensors */
#define SUNLINE_FILTER_NUM_STATES 7 /* Filter state dimension: s_hat(3) + omega(3) + bias(1) */

/**
 * @brief Plain-old-data mirror of the C++ CssData update input.
 *
 * Only the first numberOfCss entries of cosValues are meaningful.
 */
typedef struct {
    double timeTag;                           /*!< [s] time tag of the CSS reading */
    double cosValues[SUNLINE_FILTER_MAX_CSS]; /*!< [-] CSS cosine measurement values */
} SunlineCssData_c;

/**
 * @brief Plain-old-data mirror of the C++ RateData update input.
 */
typedef struct {
    double timeTag; /*!< [s] time tag of the gyro reading */
    double rate[3]; /*!< [rad/s] body rate in body frame */
} SunlineRateData_c;

/**
 * @brief Plain-old-data mirror of the C++ FilterStateOutput.
 *
 * covariance is stored row-major (covariance[row][col]).
 */
typedef struct {
    double state[SUNLINE_FILTER_NUM_STATES];                                 /*!< [-] filter state estimate */
    double covariance[SUNLINE_FILTER_NUM_STATES][SUNLINE_FILTER_NUM_STATES]; /*!< [-] N x N state covariance */
} SunlineFilterStateOutput_c;

/**
 * @brief Plain-old-data mirror of the C++ CssResidualsOutput.
 *
 * valid is true only on cycles a CSS measurement fired. Only the first
 * numberOfActiveCss entries of each residual array are meaningful.
 */
typedef struct {
    bool valid;                                 /*!< [-] true iff a CSS measurement fired this cycle */
    int32_t numberOfActiveCss;                  /*!< [-] number of CSS above the sensor threshold */
    double observation[SUNLINE_FILTER_MAX_CSS]; /*!< [-] measured CSS cosine values */
    double preFit[SUNLINE_FILTER_MAX_CSS];      /*!< [-] pre-update CSS residuals */
    double postFit[SUNLINE_FILTER_MAX_CSS];     /*!< [-] post-update CSS residuals */
} SunlineCssResidualsOutput_c;

/**
 * @brief Plain-old-data mirror of the C++ RateResidualsOutput.
 *
 * valid is true only on cycles a rate measurement fired.
 */
typedef struct {
    bool valid;            /*!< [-] true iff a rate measurement fired this cycle */
    double observation[3]; /*!< [rad/s] measured body rate */
    double preFit[3];      /*!< [rad/s] pre-update rate residuals */
    double postFit[3];     /*!< [rad/s] post-update rate residuals */
} SunlineRateResidualsOutput_c;

/**
 * @brief Plain-old-data mirror of the C++ SunlineFilterOutput.
 */
typedef struct {
    SunlineFilterStateOutput_c filterState;     /*!< [-] filter state and covariance snapshot */
    SunlineCssResidualsOutput_c cssResiduals;   /*!< [-] CSS residuals for this cycle */
    SunlineRateResidualsOutput_c rateResiduals; /*!< [-] rate residuals for this cycle */
} SunlineFilterOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUNLINEFILTER_TYPES_H
