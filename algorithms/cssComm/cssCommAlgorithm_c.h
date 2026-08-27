#ifndef F32XMERA_CSS_COMM_ALGORITHM_C_H
#define F32XMERA_CSS_COMM_ALGORITHM_C_H

#include "cssCommTypes.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ CssCommAlgorithm instance.
 */
typedef struct CssCommAlgorithmHandle CssCommAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param numSensors       Number of active CSS sensors.
 * @param maxSensorValues  Per-sensor scale factors.
 * @param chebyPolynomials Chebyshev polynomial coefficients.
 * @return true if the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 * @note The accepted value ranges are defined by CssCommConfig::create; this predicate
 *       reports whether a candidate set would be accepted, without throwing.
 */
bool CssCommAlgorithm_validateConfig(uint32_t numSensors,
                                     double maxSensorValues[MAX_NUM_CSS_SENSORS],
                                     double chebyPolynomials[MAX_NUM_CHEBY_POLYS]);

/**
 * @brief Construct a new CssCommAlgorithm instance from the supplied configuration.
 * @param numSensors       Number of active CSS sensors.
 * @param maxSensorValues  Per-sensor scale factors.
 * @param chebyPolynomials Chebyshev polynomial coefficients.
 * @return Pointer to a new CssCommAlgorithm (must be destroyed). Validated; throws on invalid input.
 */
CssCommAlgorithmHandle* CssCommAlgorithm_create(uint32_t numSensors,
                                                double maxSensorValues[MAX_NUM_CSS_SENSORS],
                                                double chebyPolynomials[MAX_NUM_CHEBY_POLYS]);

/**
 * @brief Destroy a previously created CssCommAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void CssCommAlgorithm_destroy(CssCommAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self             Pointer to the instance.
 * @param numSensors       Number of active CSS sensors.
 * @param maxSensorValues  Per-sensor scale factors.
 * @param chebyPolynomials Chebyshev polynomial coefficients.
 * Validated; throws on invalid input.
 */
void CssCommAlgorithm_setConfig(CssCommAlgorithmHandle* self,
                                uint32_t numSensors,
                                double maxSensorValues[MAX_NUM_CSS_SENSORS],
                                double chebyPolynomials[MAX_NUM_CHEBY_POLYS]);

/**
 * @brief Run the CSS communication correction update.
 * @param self        Pointer to the instance.
 * @param inputValues Pointer to the input CSS sensor values.
 * @return CssSensorValues_c The corrected CSS sensor values.
 */
CssSensorValues_c CssCommAlgorithm_update(const CssCommAlgorithmHandle* self, const CssSensorValues_c* inputValues);

/**
 * @brief Get the MAX_NUM_CSS_SENSORS constant for Ada validation.
 * @return The value of MAX_NUM_CSS_SENSORS.
 */
uint32_t CssCommAlgorithm_getMaxNumCssSensors(void);

/**
 * @brief Get the MAX_NUM_CHEBY_POLYS constant for Ada validation.
 * @return The value of MAX_NUM_CHEBY_POLYS.
 */
uint32_t CssCommAlgorithm_getMaxNumChebyPolys(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CSS_COMM_ALGORITHM_C_H
