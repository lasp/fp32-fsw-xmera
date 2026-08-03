#ifndef F32XMERA_COB_CONVERTER_ALGORITHM_C_H
#define F32XMERA_COB_CONVERTER_ALGORITHM_C_H

#include "cobConverterTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ CobConverterAlgorithm instance.
 */
typedef struct CobConverterAlgorithmHandle CobConverterAlgorithmHandle;

/**
 * @brief Construct a new CobConverterAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new CobConverterAlgorithm instance (must be destroyed).
 */
CobConverterAlgorithmHandle* CobConverterAlgorithm_create(const CobConverterConfig_c* config);

/**
 * @brief Destroy a previously created CobConverterAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void CobConverterAlgorithm_destroy(CobConverterAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void CobConverterAlgorithm_setConfig(CobConverterAlgorithmHandle* self, const CobConverterConfig_c* config);

/**
 * @brief Run the update step: convert pixel-based COB into unit vectors and outputs.
 * @param self  Pointer to the instance.
 * @param input Pointer to the algorithm input payload.
 * @return CobConverterOutput_c Populated output (zeroed if input->cobValid is false or
 *         input->cobPixelsFound is zero).
 */
CobConverterOutput_c CobConverterAlgorithm_updateState(CobConverterAlgorithmHandle* self,
                                                       const CobConverterInput_c* input);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_COB_CONVERTER_ALGORITHM_C_H
