#ifndef BILINEARINTERPOLATION_H_
#define BILINEARINTERPOLATION_H_

#include <math.h>
#include <optional>

/*! @brief Bilinear interpolation function to solve for the value of an unknown function of two variables f(x,y)
 *         at the point (x,y).
 * @param x1 Data point x1
 * @param x2 Data point x2
 * @param y1 Data point y1
 * @param y2 Data point y2
 * @param z11 Function value at point (x1, y1)
 * @param z12 Function value at point (x1, y2)
 * @param z21 Function value at point (x2, y1)
 * @param z22 Function value at point (x2, y2)
 * @param x Function x coordinate for interpolation
 * @param y Function y coordinate for interpolation
 * @return std::optional<float>
 */
inline std::optional<float> bilinearInterpolation(float x1,
                                                  float x2,
                                                  float y1,
                                                  float y2,
                                                  float z11,
                                                  float z12,
                                                  float z21,
                                                  float z22,
                                                  float x,
                                                  float y) {
    if (isnan(x1) || isnan(x2) || isnan(y1) || isnan(y2) || isnan(z11) || isnan(z12) || isnan(z21) || isnan(z22) ||
        isnan(x) || isnan(y)) {
        return std::nullopt;
    }
    if (isinf(x1) || isinf(x2) || isinf(y1) || isinf(y2) || isinf(z11) || isinf(z12) || isinf(z21) || isinf(z22) ||
        isinf(x) || isinf(y)) {
        return std::nullopt;
    }

    // No interpolation check
    if (x1 == x2 && y1 == y2 && z11 == z12 && z12 == z21 && z21 == z22) {
        return z11;
    }

    // Linear interpolation along x check
    if (y1 == y2 && z11 == z12 && z21 == z22) {
        return z11 * (x2 - x) / (x2 - x1) + z22 * (x - x1) / (x2 - x1);
    }

    // Linear interpolation along y check
    if (x1 == x2 && z12 == z22 && z11 == z21) {
        return z11 * (y2 - y) / (y2 - y1) + z22 * (y - y1) / (y2 - y1);
    }

    // Bilinear interpolation otherwise
    if (x1 < x && x < x2 && y1 < y && y < y2 && x1 != x2 && y1 != y2) {
        return 1.0F / ((x2 - x1) * (y2 - y1)) *
               (z11 * (x2 - x) * (y2 - y) + z21 * (x - x1) * (y2 - y) + z12 * (x2 - x) * (y - y1) +
                z22 * (x - x1) * (y - y1));
    }

    return std::nullopt;
}

#endif
