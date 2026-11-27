#ifndef F32XMERA_CENTER_OF_BRIGHTNESS_TYPES_H
#define F32XMERA_CENTER_OF_BRIGHTNESS_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read-only view of a single-channel 8-bit image in row-major order.
 *
 * Pixel (row r, column c) is at byte offset r * strideBytes + c from pixels.
 * The caller keeps control of the pixel buffer.
 */
typedef struct {
    const uint8_t* pixels; /*!< [-] address of the first pixel (row 0, column 0) */
    int32_t rows;          /*!< [px] image height */
    int32_t cols;          /*!< [px] image width */
    int32_t strideBytes;   /*!< [B] bytes from the start of one row to the start of the next. It must be >= cols. */
} ImageView_c;

/**
 * @brief Region of interest: a rectangle in full-image pixel coordinates.
 *
 * The region has columns [centerX - width / 2, centerX - width / 2 + width) and
 * rows [centerY - height / 2, centerY - height / 2 + height). The divisions are integer divisions.
 */
typedef struct {
    int32_t centerX; /*!< [px] center column */
    int32_t centerY; /*!< [px] center row */
    int32_t width;   /*!< [px] region width */
    int32_t height;  /*!< [px] region height */
} RegionOfInterest_c;

/**
 * @brief Plain-old-data (POD) equivalent of the C++ CenterOfBrightnessResult.
 */
typedef struct {
    float centerOfBrightness[2];    /*!< [px] centroid, full-image coordinates: [0] column (x), [1] row (y) */
    int32_t pixelsFound;            /*!< [-] number of bright pixels. It is 0 when valid is false. */
    float rollingAverageBrightness; /*!< [-] rolling average of the bright pixel count */
    bool valid;                     /*!< [-] true when the brightness increase is equal to or more than the threshold */
    bool noPixelTrigger;            /*!< [-] diagnostic flag: true when update() found no bright pixels */
    bool notExceedingBrightnessIncreaseTrigger; /*!< [-] diagnostic flag for a brightness increase less than the
                                                   threshold */
} CenterOfBrightnessResult_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CENTER_OF_BRIGHTNESS_TYPES_H
