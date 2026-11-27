#ifndef F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_C_H
#define F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_C_H

#include "centerOfBrightnessTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ CenterOfBrightnessAlgorithm instance.
 */
typedef struct CenterOfBrightnessAlgorithmHandle CenterOfBrightnessAlgorithmHandle;

/**
 * @brief Tell if create/setConfig accepts a configuration.
 * @param relativeBrightnessIncreaseThreshold [-] minimum relative brightness increase that sets valid.
 *                                                It must be >= 0. The function accepts +inf.
 * @param numberOfPointsBrightnessAverage     [-] number of steps in the brightness rolling average. It must be more
 * than 0.
 * @return true when the configuration is correct. This function does not throw exceptions. Thus you can use it
 *         to prevent an incorrect configuration in create/setConfig, which throw exceptions.
 */
bool CenterOfBrightnessAlgorithm_validateConfig(float relativeBrightnessIncreaseThreshold,
                                                int32_t numberOfPointsBrightnessAverage);

/**
 * @brief Make a new CenterOfBrightnessAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first. If a value is incorrect, this function throws an exception.
 * Refer to validateConfig for the parameter limits.
 * @return Pointer to a new CenterOfBrightnessAlgorithm handle. Call CenterOfBrightnessAlgorithm_destroy to delete it.
 */
CenterOfBrightnessAlgorithmHandle* CenterOfBrightnessAlgorithm_create(float relativeBrightnessIncreaseThreshold,
                                                                      int32_t numberOfPointsBrightnessAverage);

/**
 * @brief Replace the algorithm's configuration at runtime. The brightness history does not change.
 * Validate the values with validateConfig first. If a value is incorrect, this function throws an exception.
 * Refer to validateConfig for the parameter limits.
 * @param self Pointer to the handle.
 */
void CenterOfBrightnessAlgorithm_setConfig(CenterOfBrightnessAlgorithmHandle* self,
                                           float relativeBrightnessIncreaseThreshold,
                                           int32_t numberOfPointsBrightnessAverage);

/**
 * @brief Delete a CenterOfBrightnessAlgorithm instance that CenterOfBrightnessAlgorithm_create made.
 * @param self Pointer to the handle to delete.
 */
void CenterOfBrightnessAlgorithm_destroy(CenterOfBrightnessAlgorithmHandle* self);

/**
 * @brief Clear the algorithm's runtime state (the brightness history).
 * @param self Pointer to the handle.
 */
void CenterOfBrightnessAlgorithm_reInitialize(CenterOfBrightnessAlgorithmHandle* self);

/**
 * @brief Calculate the center of brightness of the non-zero pixels inside a region of interest.
 * The function clips the region to the image bounds. If the image view is incorrect (null pixels, rows or
 * cols <= 0, strideBytes < cols) or the region is fully outside the image, the function gives the default result
 * (zero, valid is false) and does not change the brightness history. This function does not throw exceptions.
 * @param self  Pointer to the handle.
 * @param image Single-channel 8-bit image. Non-zero pixels are bright. The caller keeps control of the
 *              pixel buffer, which must hold rows * strideBytes bytes.
 * @param roi   Region of interest in full-image pixel coordinates.
 * @return Center of brightness result.
 */
CenterOfBrightnessResult_c CenterOfBrightnessAlgorithm_update(CenterOfBrightnessAlgorithmHandle* self,
                                                              ImageView_c image,
                                                              RegionOfInterest_c roi);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_C_H
