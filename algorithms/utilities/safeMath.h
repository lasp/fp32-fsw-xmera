#ifndef SAFE_MATH_H_
#define SAFE_MATH_H_

#include <math.h>
#include <limits>
#include <numbers>

/*! @brief Safe wrapper around tanf that clamps the input to
 *         [-π/2 + ε, π/2 - ε] to avoid the singularities
 *         at ±π/2. Inputs outside this range are railed to ±cot(ε).
 *  @param x Input value in radians
 *  @return tanf(clamp(x, -π/2 + ε, π/2 - ε))
 */
inline float safeTanf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    constexpr float eps = std::numeric_limits<float>::epsilon();
    constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
    if (x < -halfPi + eps) {
        return -cosf(eps) / sinf(eps);
    }
    if (x > halfPi - eps) {
        return cosf(eps) / sinf(eps);
    }
    return tanf(x);
}

/*! @brief Safe wrapper around tan that clamps the input to
 *         [-π/2 + ε, π/2 - ε] to avoid the singularities
 *         at ±π/2. Inputs outside this range are railed to ±cot(ε).
 *  @param x Input value in radians
 *  @return tan(clamp(x, -π/2 + ε, π/2 - ε))
 */
inline double safeTan(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    constexpr double eps = std::numeric_limits<double>::epsilon();
    constexpr double halfPi = std::numbers::pi / 2.0;
    if (x < -halfPi + eps) {
        return -cos(eps) / sin(eps);
    }
    if (x > halfPi - eps) {
        return cos(eps) / sin(eps);
    }
    return tan(x);
}

/*! @brief Safe wrapper around atanf. Since atan always returns a value in
 *         (-π/2, π/2) for all real inputs, the output guards are defensive
 *         and never trigger in practice.
 *  @param x Input value
 *  @return atanf(x)
 */
inline float safeAtanf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
    float const atanx = atanf(x);
    if (atanx > halfPi) {
        return halfPi;
    }
    if (atanx < -halfPi) {
        return -halfPi;
    }
    return atanx;
}

/*! @brief Safe wrapper around atan. Since atan always returns a value in
 *         (-π/2, π/2) for all real inputs, the output guards are defensive
 *         and never trigger in practice.
 *  @param x Input value
 *  @return atan(x)
 */
inline double safeAtan(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    constexpr double halfPi = std::numbers::pi / 2.0;
    double const atanx = atan(x);
    if (atanx > halfPi) {
        return halfPi;
    }
    if (atanx < -halfPi) {
        return -halfPi;
    }
    return atanx;
}

/*! @brief Safe wrapper around tanhf. Since tanh always returns a value in
 *         (-1, 1) for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value
 *  @return tanhf(x)
 */
inline float safeTanHf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    float const tanhx = tanhf(x);
    if (tanhx < -1.0F) {
        return -1.0F;
    }
    if (tanhx > 1.0F) {
        return 1.0F;
    }
    return tanhx;
}

/*! @brief Safe wrapper around tanh. Since tanh always returns a value in
 *         (-1, 1) for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value
 *  @return tanh(x)
 */
inline double safeTanH(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    double const tanhx = tanh(x);
    if (tanhx < -1.0) {
        return -1.0;
    }
    if (tanhx > 1.0) {
        return 1.0;
    }
    return tanhx;
}

/*! @brief Safe wrapper around atanhf that clamps the input to
 *         [-(1 - ε), (1 - ε)]. The domain of atanh is the open interval
 *         (-1, 1); passing ±1 produces ±infinity, so the bound is kept
 *         strictly inside the domain.
 *  @param x Input value
 *  @return atanhf(clamp(x, -(1 - ε), (1 - ε)))
 */
inline float safeAtanHf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    constexpr float eps = std::numeric_limits<float>::epsilon();
    if (x < -(1.0F - eps)) {
        return atanhf(-(1.0F - eps));
    }
    if (x > (1.0F - eps)) {
        return atanhf(1.0F - eps);
    }
    return atanhf(x);
}

/*! @brief Safe wrapper around atanh that clamps the input to
 *         [-(1 - ε), (1 - ε)]. The domain of atanh is the open interval
 *         (-1, 1); passing ±1 produces ±infinity, so the bound is kept
 *         strictly inside the domain.
 *  @param x Input value
 *  @return atanh(clamp(x, -(1 - ε), (1 - ε)))
 */
inline double safeAtanH(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    constexpr double eps = std::numeric_limits<double>::epsilon();
    if (x < -(1.0 - eps)) {
        return atanh(-(1.0 - eps));
    }
    if (x > (1.0 - eps)) {
        return atanh(1.0 - eps);
    }
    return atanh(x);
}

/*! @brief Safe wrapper around coshf that caps the output to [1, 1/ε]
 *         to prevent overflow for large |x|.
 *  @param x Input value
 *  @return min(coshf(x), 1/ε)
 */
inline float safeCosHf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 1.0F;
    }
    constexpr float eps = std::numeric_limits<float>::epsilon();
    float const coshx = coshf(x);
    if (coshx > 1.0F / eps) {
        return 1.0F / eps;
    }
    return coshx;
}

/*! @brief Safe wrapper around cosh that caps the output to [1, 1/ε]
 *         to prevent overflow for large |x|.
 *  @param x Input value
 *  @return min(cosh(x), 1/ε)
 */
inline double safeCosH(double const x) {
    if (isnan(x) || isinf(x)) {
        return 1.0;
    }
    constexpr double eps = std::numeric_limits<double>::epsilon();
    double const coshx = cosh(x);
    if (coshx > 1.0 / eps) {
        return 1.0 / eps;
    }
    return coshx;
}

/*! @brief Safe wrapper around sinhf that caps the output to [-1/ε, 1/ε]
 *         to prevent overflow for large |x|.
 *  @param x Input value
 *  @return clamp(sinhf(x), -1/ε, 1/ε)
 */
inline float safeSinHf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    constexpr float eps = std::numeric_limits<float>::epsilon();
    float const sinhx = sinhf(x);
    if (sinhx < -1.0F / eps) {
        return -1.0F / eps;
    }
    if (sinhx > 1.0F / eps) {
        return 1.0F / eps;
    }
    return sinhx;
}

/*! @brief Safe wrapper around sinh that caps the output to [-1/ε, 1/ε]
 *         to prevent overflow for large |x|.
 *  @param x Input value
 *  @return clamp(sinh(x), -1/ε, 1/ε)
 */
inline double safeSinH(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    constexpr double eps = std::numeric_limits<double>::epsilon();
    double const sinhx = sinh(x);
    if (sinhx < -1.0 / eps) {
        return -1.0 / eps;
    }
    if (sinhx > 1.0 / eps) {
        return 1.0 / eps;
    }
    return sinhx;
}

/*! @brief Safe wrapper around cosf. Since cos always returns a value in
 *         [-1, 1] for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value in radians
 *  @return cosf(x)
 */
inline float safeCosf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    float const cosx = cosf(x);
    if (cosx < -1.0F) {
        return -1.0F;
    }
    if (cosx > 1.0F) {
        return 1.0F;
    }
    return cosx;
}

/*! @brief Safe wrapper around cos. Since cos always returns a value in
 *         [-1, 1] for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value in radians
 *  @return cos(x)
 */
inline double safeCos(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    double const cosx = cos(x);
    if (cosx < -1.0) {
        return -1.0;
    }
    if (cosx > 1.0) {
        return 1.0;
    }
    return cosx;
}

/*! @brief Safe wrapper around sinf. Since sin always returns a value in
 *         [-1, 1] for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value in radians
 *  @return sinf(x)
 */
inline float safeSinf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    float const sinx = sinf(x);
    if (sinx < -1.0F) {
        return -1.0F;
    }
    if (sinx > 1.0F) {
        return 1.0F;
    }
    return sinx;
}

/*! @brief Safe wrapper around sin. Since sin always returns a value in
 *         [-1, 1] for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value in radians
 *  @return sin(x)
 */
inline double safeSin(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    double const sinx = sin(x);
    if (sinx < -1.0) {
        return -1.0;
    }
    if (sinx > 1.0) {
        return 1.0;
    }
    return sinx;
}

/*! @brief Safe wrapper around acosf that clamps the input to [-1, 1], the
 *         closed domain of acos.
 *  @param x Input value
 *  @return acosf(clamp(x, -1, 1))
 */
inline float safeAcosf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    if (x < -1.0F) {
        return std::numbers::pi_v<float>;
    }
    if (x > 1.0F) {
        return 0.0F;
    }
    return acosf(x);
}

/*! @brief Safe wrapper around acos that clamps the input to [-1, 1], the
 *         closed domain of acos.
 *  @param x Input value
 *  @return acos(clamp(x, -1, 1))
 */
inline double safeAcos(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    if (x < -1.0) {
        return std::numbers::pi;
    }
    if (x > 1.0) {
        return 0.0;
    }
    return acos(x);
}

/*! @brief Safe wrapper around asinf that clamps the input to [-1, 1], the
 *         closed domain of asin.
 *  @param x Input value
 *  @return asinf(clamp(x, -1, 1))
 */
inline float safeAsinf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    if (x < -1.0F) {
        return -std::numbers::pi_v<float> / 2.0F;
    }
    if (x > 1.0F) {
        return std::numbers::pi_v<float> / 2.0F;
    }
    return asinf(x);
}

/*! @brief Safe wrapper around asin that clamps the input to [-1, 1], the
 *         closed domain of asin.
 *  @param x Input value
 *  @return asin(clamp(x, -1, 1))
 */
inline double safeAsin(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    if (x < -1.0) {
        return -std::numbers::pi / 2.0;
    }
    if (x > 1.0) {
        return std::numbers::pi / 2.0;
    }
    return asin(x);
}

/*! @brief Safe wrapper around sqrtf that clamps negative inputs to zero, the
 *         lower bound of the domain of sqrt.
 *  @param x Input value
 *  @return sqrtf(max(x, 0))
 */
inline float safeSqrtf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    if (x < 0.0F) {
        return 0.0F;
    }
    return sqrtf(x);
}

/*! @brief Safe wrapper around sqrt that clamps negative inputs to zero, the
 *         lower bound of the domain of sqrt.
 *  @param x Input value
 *  @return sqrt(max(x, 0))
 */
inline double safeSqrt(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    if (x < 0.0) {
        return 0.0;
    }
    return sqrt(x);
}

/*! @brief Safe wrapper around atan2f that returns 0 when both arguments are
 *         zero, the only input pair for which atan2 is undefined.
 *  @param y Numerator argument (opposite side)
 *  @param x Denominator argument (adjacent side)
 *  @return atan2f(y, x), or 0 when both y and x are zero
 */
inline float safeAtan2f(float const y, float const x) {
    if (isnan(y) || isnan(x) || isinf(x) || isinf(y)) {
        return 0.0F;
    }
    if (y == 0.0F && x == 0.0F) {
        return 0.0F;
    }
    return atan2f(y, x);
}

/*! @brief Safe wrapper around atan2 that returns 0 when both arguments are
 *         zero, the only input pair for which atan2 is undefined.
 *  @param y Numerator argument (opposite side)
 *  @param x Denominator argument (adjacent side)
 *  @return atan2(y, x), or 0 when both y and x are zero
 */
inline double safeAtan2(double const y, double const x) {
    if (isnan(y) || isnan(x) || isinf(x) || isinf(y)) {
        return 0.0;
    }
    if (y == 0.0 && x == 0.0) {
        return 0.0;
    }
    return atan2(y, x);
}

#endif
