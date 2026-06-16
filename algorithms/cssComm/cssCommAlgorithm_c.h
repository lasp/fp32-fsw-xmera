#ifndef F32XMERA_CSS_COMM_ALGORITHM_C_H
#define F32XMERA_CSS_COMM_ALGORITHM_C_H

#include "cssCommTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ CssCommAlgorithm instance.
 */
typedef struct CssCommAlgorithmHandle CssCommAlgorithmHandle;

/**
 * @brief Construct a new CssCommAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new CssCommAlgorithm (must be destroyed).
 */
CssCommAlgorithmHandle* CssCommAlgorithm_create(const CssCommConfig_c* config);

/**
 * @brief Destroy a previously created CssCommAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void CssCommAlgorithm_destroy(CssCommAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void CssCommAlgorithm_setConfig(CssCommAlgorithmHandle* self, const CssCommConfig_c* config);

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
