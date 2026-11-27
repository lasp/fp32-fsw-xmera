/* The compiler compiles this file as C, not C++. This shows that the shim header contains only C code
 * and that C code can call the API. */
#include "../centerOfBrightnessAlgorithm_c.h"

CenterOfBrightnessResult_c centerOfBrightnessRunFromC(const uint8_t* pixels, int32_t rows, int32_t cols);

CenterOfBrightnessResult_c centerOfBrightnessRunFromC(const uint8_t* pixels, int32_t rows, int32_t cols) {
    const ImageView_c image = {.pixels = pixels, .rows = rows, .cols = cols, .strideBytes = cols};
    const RegionOfInterest_c roi = {.centerX = cols / 2, .centerY = rows / 2, .width = cols, .height = rows};

    CenterOfBrightnessAlgorithmHandle* handle = CenterOfBrightnessAlgorithm_create(0.0F, 5);
    const CenterOfBrightnessResult_c result = CenterOfBrightnessAlgorithm_update(handle, image, roi);
    CenterOfBrightnessAlgorithm_destroy(handle);
    return result;
}
