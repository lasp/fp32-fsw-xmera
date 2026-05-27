#ifndef F32XMERA_CSS_COMM_ALGORITHM_C_H
#define F32XMERA_CSS_COMM_ALGORITHM_C_H

#include "cssCommTypes.h"
#include "msgPayloadDef/definitions.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ CssCommAlgorithm instance.
 */
typedef struct CssCommAlgorithmHandle CssCommAlgorithmHandle;

/**
 * @brief Bounded array of CSS sensor values (double precision).
 */
typedef struct {
    double data[MAX_NUM_CSS_SENSORS];
} CssSensorValues_c;

/**
 * @brief Bounded array of Chebyshev polynomial coefficients.
 */
typedef struct {
    double data[MAX_NUM_CHEBY_POLYS];
} ChebyPolynomials_c;

/**
 * @brief Construct a new CssCommAlgorithm instance.
 * @return Pointer to a new CssCommAlgorithm (must be destroyed).
 */
CssCommAlgorithmHandle* CssCommAlgorithm_create(void);

/**
 * @brief Destroy a previously created CssCommAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void CssCommAlgorithm_destroy(CssCommAlgorithmHandle* self);

/**
 * @brief Run the CSS communication correction update.
 * @param self        Pointer to the instance.
 * @param inputValues Pointer to the input CSS sensor values.
 * @return CssSensorValues_c The corrected CSS sensor values.
 */
CssSensorValues_c CssCommAlgorithm_update(const CssCommAlgorithmHandle* self, const CssSensorValues_c* inputValues);

/**
 * @brief Set the number of CSS sensors.
 * @param self            Pointer to the instance.
 * @param numberOfSensors The number of CSS sensors to process.
 */
void CssCommAlgorithm_setNumSensors(CssCommAlgorithmHandle* self, uint32_t numberOfSensors);

/**
 * @brief Get the number of CSS sensors.
 * @param self Pointer to the instance.
 * @return uint32_t The number of CSS sensors.
 */
uint32_t CssCommAlgorithm_getNumSensors(const CssCommAlgorithmHandle* self);

/**
 * @brief Set the maximum sensor value (scale factor).
 * @param self     Pointer to the instance.
 * @param maxValue The maximum sensor value.
 */
void CssCommAlgorithm_setMaxSensorValue(CssCommAlgorithmHandle* self, double maxValue);

/**
 * @brief Get the maximum sensor value (scale factor).
 * @param self Pointer to the instance.
 * @return double The maximum sensor value.
 */
double CssCommAlgorithm_getMaxSensorValue(const CssCommAlgorithmHandle* self);

/**
 * @brief Set the Chebyshev polynomial count.
 * @param self  Pointer to the instance.
 * @param count The number of Chebyshev polynomials.
 */
void CssCommAlgorithm_setChebyCount(CssCommAlgorithmHandle* self, uint32_t count);

/**
 * @brief Get the Chebyshev polynomial count.
 * @param self Pointer to the instance.
 * @return uint32_t The number of Chebyshev polynomials.
 */
uint32_t CssCommAlgorithm_getChebyCount(const CssCommAlgorithmHandle* self);

/**
 * @brief Set the Chebyshev polynomial coefficients.
 * @param self        Pointer to the instance.
 * @param polynomials Pointer to the polynomial coefficients.
 */
void CssCommAlgorithm_setChebyPolynomials(CssCommAlgorithmHandle* self, const ChebyPolynomials_c* polynomials);

/**
 * @brief Get the Chebyshev polynomial coefficients.
 * @param self Pointer to the instance.
 * @return ChebyPolynomials_c The polynomial coefficients.
 */
ChebyPolynomials_c CssCommAlgorithm_getChebyPolynomials(const CssCommAlgorithmHandle* self);

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
