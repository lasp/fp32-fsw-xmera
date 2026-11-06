#ifndef SAFE_MATH_H_
#define SAFE_MATH_H_

#include <math.h>
#include <limits>
#include <numbers>

/*! @brief Safe wrapper around tanf. Preserves the periodicity and sign of
 *         tangent and saturates the output to ±1/ε (2^23 ≈ 8.39e6) near the
 *         singularities at π/2 + kπ. Near a singularity the sign follows the
 *         stored value of x: the float nearest π/2 lies above π/2, so it
 *         returns -1/ε.
 *         Non-finite inputs return zero.
 *  @param x Input angle in radians
 *  @return tanf(x) clamped to [-1/ε, 1/ε], or 0.0F if x is NaN or infinite
 */
inline float safeTanf(float const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0F;
    }
    constexpr float eps = std::numeric_limits<float>::epsilon();
    constexpr float maxTan = 1.0F / eps;
    float const tanx = tanf(x);
    if (tanx < -maxTan) {
        return -maxTan;
    }
    if (tanx > maxTan) {
        return maxTan;
    }
    return tanx;
}

/*! @brief Safe wrapper around tan. Preserves the periodicity and sign of
 *         tangent and saturates the output to ±1/ε (2^52 ≈ 4.5e15) near the
 *         singularities at π/2 + kπ. Near a singularity the sign follows the
 *         stored value of x: the double nearest π/2 lies below π/2, so it
 *         returns +1/ε.
 *         Non-finite inputs return zero.
 *  @param x Input angle in radians
 *  @return tan(x) clamped to [-1/ε, 1/ε], or 0.0 if x is NaN or infinite
 */
inline double safeTan(double const x) {
    if (isnan(x) || isinf(x)) {
        return 0.0;
    }
    constexpr double eps = std::numeric_limits<double>::epsilon();
    constexpr double maxTan = 1.0 / eps;
    double const tanx = tan(x);
    if (tanx < -maxTan) {
        return -maxTan;
    }
    if (tanx > maxTan) {
        return maxTan;
    }
    return tanx;
}

/*! @brief Safe wrapper around atanf. atan is finite for every non-NaN
 *         input, including ±Inf which maps to ±π/2.
 *         NaN inputs return zero.
 *  @param x Input value (any real, unitless ratio)
 *  @return atanf(x) in [-π/2, π/2], or 0.0F if x is NaN
 */
inline float safeAtanf(float const x) {
    if (isnan(x)) {
        return 0.0F;
    }
    return atanf(x);
}

/*! @brief Safe wrapper around atan. atan is finite for every non-NaN
 *         input, including ±Inf which maps to ±π/2.
 *         NaN inputs return zero.
 *  @param x Input value (any real, unitless ratio)
 *  @return atan(x) in [-π/2, π/2], or 0.0 if x is NaN
 */
inline double safeAtan(double const x) {
    if (isnan(x)) {
        return 0.0;
    }
    return atan(x);
}

/*! @brief Safe wrapper around tanhf. tanh is finite for every non-NaN
 *         input, including ±Inf which maps to ±1. The output guards are
 *         defensive and never trigger in practice.
 *         NaN inputs return zero.
 *  @param x Input value (any real)
 *  @return tanhf(x) in [-1, 1], or 0.0F if x is NaN
 */
inline float safeTanHf(float const x) {
    if (isnan(x)) {
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

/*! @brief Safe wrapper around tanh. tanh is finite for every non-NaN
 *         input, including ±Inf which maps to ±1. The output guards are
 *         defensive and never trigger in practice.
 *         NaN inputs return zero.
 *  @param x Input value (any real)
 *  @return tanh(x) in [-1, 1], or 0.0 if x is NaN
 */
inline double safeTanH(double const x) {
    if (isnan(x)) {
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
 *         NaN and ±Inf inputs return zero. Finite inputs outside the bound,
 *         including the largest value below 1 (1 - ε/2), return
 *         atanhf(±(1 - ε)) ≈ ±8.32.
 *  @param x Input value (any real)
 *  @return atanhf(clamp(x, -(1 - ε), (1 - ε))), or 0.0F if x is NaN or ±Inf
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
 *         NaN and ±Inf inputs return zero. Finite inputs outside the bound,
 *         including the largest value below 1 (1 - ε/2), return
 *         atanh(±(1 - ε)) ≈ ±18.37.
 *  @param x Input value (any real)
 *  @return atanh(clamp(x, -(1 - ε), (1 - ε))), or 0.0 if x is NaN or ±Inf
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

/*! @brief Safe wrapper around coshf. The output is capped at 1/ε, which is
 *         reached for |x| > ~16.6, well before coshf overflows.
 *         ±Inf inputs return the cap. NaN inputs return 1 (cosh(0)).
 *  @param x Input value (any real)
 *  @return min(coshf(x), 1/ε) in [1, 1/ε], or 1.0F if x is NaN
 */
inline float safeCosHf(float const x) {
    if (isnan(x)) {
        return 1.0F;
    }
    constexpr float eps = std::numeric_limits<float>::epsilon();
    float const coshx = coshf(x);
    if (coshx > 1.0F / eps) {
        return 1.0F / eps;
    }
    return coshx;
}

/*! @brief Safe wrapper around cosh. The output is capped at 1/ε, which is
 *         reached for |x| > ~36.7, well before cosh overflows.
 *         ±Inf inputs return the cap. NaN inputs return 1 (cosh(0)).
 *  @param x Input value (any real)
 *  @return min(cosh(x), 1/ε) in [1, 1/ε], or 1.0 if x is NaN
 */
inline double safeCosH(double const x) {
    if (isnan(x)) {
        return 1.0;
    }
    constexpr double eps = std::numeric_limits<double>::epsilon();
    double const coshx = cosh(x);
    if (coshx > 1.0 / eps) {
        return 1.0 / eps;
    }
    return coshx;
}

/*! @brief Safe wrapper around sinhf. The output is capped at ±1/ε, which is
 *         reached for |x| > ~16.6, well before sinhf overflows.
 *         NaN and ±Inf inputs return zero.
 *  @param x Input value (any real)
 *  @return clamp(sinhf(x), -1/ε, 1/ε), or 0.0F if x is NaN or ±Inf
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

/*! @brief Safe wrapper around sinh. The output is capped at ±1/ε, which is
 *         reached for |x| > ~36.7, well before sinh overflows.
 *         NaN and ±Inf inputs return zero.
 *  @param x Input value (any real)
 *  @return clamp(sinh(x), -1/ε, 1/ε), or 0.0 if x is NaN or ±Inf
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

/*! @brief Safe wrapper around cosf. cosf always returns a value in [-1, 1],
 *         so the output guards are defensive and never trigger in practice.
 *         NaN and ±Inf inputs return zero.
 *  @param x Input value in radians
 *  @return cosf(x) in [-1, 1], or 0.0F if x is NaN or ±Inf
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

/*! @brief Safe wrapper around cos. cos always returns a value in [-1, 1],
 *         so the output guards are defensive and never trigger in practice.
 *         NaN and ±Inf inputs return zero.
 *  @param x Input value in radians
 *  @return cos(x) in [-1, 1], or 0.0 if x is NaN or ±Inf
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

/*! @brief Safe wrapper around sinf. sinf always returns a value in [-1, 1],
 *         so the output guards are defensive and never trigger in practice.
 *         NaN and ±Inf inputs return zero.
 *  @param x Input value in radians
 *  @return sinf(x) in [-1, 1], or 0.0F if x is NaN or ±Inf
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

/*! @brief Safe wrapper around sin. sin always returns a value in [-1, 1],
 *         so the output guards are defensive and never trigger in practice.
 *         NaN and ±Inf inputs return zero.
 *  @param x Input value in radians
 *  @return sin(x) in [-1, 1], or 0.0 if x is NaN or ±Inf
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
