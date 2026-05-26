#ifndef F32XIMERA_SUNLINESRUKFALGORITHM_C_H
#define F32XIMERA_SUNLINESRUKFALGORITHM_C_H

#include "utilities/plainCAlgorithmDataTypes.h"
#include <stdint.h>

#define SUNLINE_SRUKF_MAX_NUM_CSS 32

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C-compatible input structure for the sunline SRuKF algorithm.
 */
typedef struct {
    double timeTag;                             /*!< [s] Time tag */
    Vector3f_c sigma_BN;                        /*!< [-] Inertial-to-body MRP */
    Vector3f_c omega_BN_B;                      /*!< [rad/s] Body rate in body frame */
    Vector3f_c vehSunPntBdy;                    /*!< [-] Sun pointing vector in body frame */
    uint32_t nCSS;                              /*!< [-] Number of coarse sun sensors */
    float cosValues[SUNLINE_SRUKF_MAX_NUM_CSS]; /*!< [-] CSS cosine measurement values */
} SunlineSRuKFInput_c;

/**
 * @brief C-compatible output structure for the sunline SRuKF algorithm.
 */
typedef struct {
    double timeTag;          /*!< [s] Time tag */
    Vector3f_c sigma_BN;     /*!< [-] Inertial-to-body MRP */
    Vector3f_c omega_BN_B;   /*!< [rad/s] Body rate in body frame */
    Vector3f_c vehSunPntBdy; /*!< [-] Sun pointing vector in body frame */
} SunlineSRuKFOutput_c;

/**
 * @brief Get the maximum number of CSS sensors.
 * @return The maximum CSS count (SUNLINE_SRUKF_MAX_NUM_CSS).
 */
uint32_t SunlineSRuKFAlgorithm_getMaxNumCss(void);

/**
 * @brief Run the sunline SRuKF update step (stateless).
 * @param input Pointer to the input structure (read-only).
 * @return SunlineSRuKFOutput_c  The computed output.
 */
SunlineSRuKFOutput_c SunlineSRuKFAlgorithm_updateState(const SunlineSRuKFInput_c* input);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_SUNLINESRUKFALGORITHM_C_H
