#ifndef SAFE_MATH_FLOAT_H_
#define SAFE_MATH_FLOAT_H_

#include <float.h>
#include <math.h>
#include <numbers>

constexpr float kMaxFloat = 1e9;

// ============================================================================
// Float versions
// ============================================================================

/*! @brief Safe wrapper around tanf that clamps the input to
 *         [-π/2 + FLT_EPSILON, π/2 - FLT_EPSILON] to avoid the singularities
 *         at ±π/2. Inputs outside this range are railed to ±cot(FLT_EPSILON).
 *  @param x Input value in radians
 *  @return tanf(clamp(x, -π/2 + FLT_EPSILON, π/2 - FLT_EPSILON))
 */
inline float safeTanf(float const x) {
    if (x < -std::numbers::pi_v<float> / 2 + FLT_EPSILON) {
        return -cosf(FLT_EPSILON) / sinf(FLT_EPSILON);
    }
    if (x > std::numbers::pi_v<float> / 2 - FLT_EPSILON) {
        return cosf(FLT_EPSILON) / sinf(FLT_EPSILON);
    }
    return tanf(x);
};

/*! @brief Safe wrapper around atanf. Since atanf always returns a value in
 *         (-π/2, π/2) for all real inputs, the output guards are defensive
 *         and never trigger in practice.
 *  @param x Input value
 *  @return atanf(x)
 */
inline float safeAtanf(float const x) {
    auto const atanx = atanf(x);
    if (atanx > std::numbers::pi_v<float> / 2) {
        return std::numbers::pi_v<float> / 2;
    }
    if (atanx < -std::numbers::pi_v<float> / 2) {
        return -std::numbers::pi_v<float> / 2;
    }
    return atanx;
};

/*! @brief Safe wrapper around tanhf. Since tanhf always returns a value in
 *         (-1, 1) for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value
 *  @return tanhf(x)
 */
inline float safeTanHf(float const x) {
    auto const tanhx = tanhf(x);
    if (tanhx < -1.0F) {
        return -1.0F;
    }
    if (tanhx > 1.0F) {
        return 1.0F;
    }
    return tanhx;
};

/*! @brief Safe wrapper around atanhf that clamps the input to
 *         [-(1 - FLT_EPSILON), (1 - FLT_EPSILON)]. The domain of atanhf is
 *         the open interval (-1, 1); passing ±1 produces ±infinity, so the
 *         bound is kept strictly inside the domain.
 *  @param x Input value
 *  @return atanhf(clamp(x, -(1 - FLT_EPSILON), (1 - FLT_EPSILON)))
 */
inline float safeAtanHf(float const x) {
    if (x < -(1.0F - FLT_EPSILON)) {
        return atanhf(-(1.0F - FLT_EPSILON));
    }
    if (x > (1.0F - FLT_EPSILON)) {
        return atanhf(1.0F - FLT_EPSILON);
    }
    return atanhf(x);
};

/*! @brief Safe wrapper around coshf that caps the output to [1, 1/FLT_EPSILON]
 *         to prevent overflow for large |x|.
 *  @param x Input value
 *  @return min(coshf(x), 1/FLT_EPSILON)
 */
inline float safeCosHf(float const x) {
    auto const coshx = coshf(x);
    if (coshx > 1.0F / FLT_EPSILON) {
        return 1.0F / FLT_EPSILON;
    }
    return coshx;
};

/*! @brief Safe wrapper around sinhf that caps the output to [-1/FLT_EPSILON,
 *         1/FLT_EPSILON] to prevent overflow for large |x|.
 *  @param x Input value
 *  @return clamp(sinhf(x), -1/FLT_EPSILON, 1/FLT_EPSILON)
 */
inline float safeSinHf(float const x) {
    auto const sinhx = sinhf(x);
    if (sinhx < -1.0F / FLT_EPSILON) {
        return -1.0F / FLT_EPSILON;
    }
    if (sinhx > 1.0F / FLT_EPSILON) {
        return 1.0F / FLT_EPSILON;
    }
    return sinhx;
};

/*! @brief Safe wrapper around cosf. Since cosf always returns a value in
 *         [-1, 1] for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value in radians
 *  @return cosf(x)
 */
inline float safeCosf(float const x) {
    auto const cosx = cosf(x);
    if (cosx < -1.0F) {
        return -1.0F;
    }
    if (cosx > 1.0F) {
        return 1.0F;
    }
    return cosx;
};

/*! @brief Safe wrapper around sinf. Since sinf always returns a value in
 *         [-1, 1] for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value in radians
 *  @return sinf(x)
 */
inline float safeSinf(float const x) {
    auto const sinx = sinf(x);
    if (sinx < -1.0F) {
        return -1.0F;
    }
    if (sinx > 1.0F) {
        return 1.0F;
    }
    return sinx;
};

/*! @brief Safe wrapper around acosf that clamps the input to [-1, 1], the
 *         closed domain of acosf.
 *  @param x Input value
 *  @return acosf(clamp(x, -1, 1))
 */
inline float safeAcosf(float const x) {
    if (x < -1.0F) {
        return std::numbers::pi_v<float>;
    }
    if (x > 1.0F) {
        return 0.0F;
    }
    return acosf(x);
};

/*! @brief Safe wrapper around asinf that clamps the input to [-1, 1], the
 *         closed domain of asinf.
 *  @param x Input value
 *  @return asinf(clamp(x, -1, 1))
 */
inline float safeAsinf(float const x) {
    if (x < -1.0F) {
        return -std::numbers::pi_v<float> / 2;
    }
    if (x > 1.0F) {
        return std::numbers::pi_v<float> / 2;
    }
    return asinf(x);
};

/*! @brief Safe wrapper around sqrtf that clamps negative inputs to zero, the
 *         lower bound of the domain of sqrtf.
 *  @param x Input value
 *  @return sqrtf(max(x, 0))
 */
inline float safeSqrtf(float const x) {
    if (x < 0.0F) {
        return 0.0F;
    }
    return sqrtf(x);
};

/*! @brief Safe wrapper around atan2f that returns 0 when both arguments are
 *         zero, the only input pair for which atan2f is undefined.
 *  @param y Numerator argument (opposite side)
 *  @param x Denominator argument (adjacent side)
 *  @return atan2f(y, x), or 0.0F when both y and x are zero
 */
inline float safeAtan2f(float const y, float const x) {
    if (y == 0.0F && x == 0.0F) {
        return 0.0F;
    }
    return atan2f(y, x);
};

// ============================================================================
// Double versions
// ============================================================================

/*! @brief Safe wrapper around tan that clamps the input to
 *         [-π/2 + DBL_EPSILON, π/2 - DBL_EPSILON] to avoid the singularities
 *         at ±π/2. Inputs outside this range are railed to ±cot(DBL_EPSILON).
 *  @param x Input value in radians
 *  @return tan(clamp(x, -π/2 + DBL_EPSILON, π/2 - DBL_EPSILON))
 */
inline double safeTan(double const x) {
    if (x < -std::numbers::pi / 2 + DBL_EPSILON) {
        return -cos(DBL_EPSILON) / sin(DBL_EPSILON);
    }
    if (x > std::numbers::pi / 2 - DBL_EPSILON) {
        return cos(DBL_EPSILON) / sin(DBL_EPSILON);
    }
    return tan(x);
};

/*! @brief Safe wrapper around atan. Since atan always returns a value in
 *         (-π/2, π/2) for all real inputs, the output guards are defensive
 *         and never trigger in practice.
 *  @param x Input value
 *  @return atan(x)
 */
inline double safeAtan(double const x) {
    auto const atanx = atan(x);
    if (atanx > std::numbers::pi / 2) {
        return std::numbers::pi / 2;
    }
    if (atanx < -std::numbers::pi / 2) {
        return -std::numbers::pi / 2;
    }
    return atanx;
};

/*! @brief Safe wrapper around tanh. Since tanh always returns a value in
 *         (-1, 1) for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value
 *  @return tanh(x)
 */
inline double safeTanH(double const x) {
    auto const tanhx = tanh(x);
    if (tanhx < -1.0) {
        return -1.0;
    }
    if (tanhx > 1.0) {
        return 1.0;
    }
    return tanhx;
};

/*! @brief Safe wrapper around atanh that clamps the input to
 *         [-(1 - DBL_EPSILON), (1 - DBL_EPSILON)]. The domain of atanh is
 *         the open interval (-1, 1); passing ±1 produces ±infinity, so the
 *         bound is kept strictly inside the domain.
 *  @param x Input value
 *  @return atanh(clamp(x, -(1 - DBL_EPSILON), (1 - DBL_EPSILON)))
 */
inline double safeAtanH(double const x) {
    if (x < -(1.0 - DBL_EPSILON)) {
        return atanh(-(1.0 - DBL_EPSILON));
    }
    if (x > (1.0 - DBL_EPSILON)) {
        return atanh(1.0 - DBL_EPSILON);
    }
    return atanh(x);
};

/*! @brief Safe wrapper around cosh that caps the output to [1, 1/DBL_EPSILON]
 *         to prevent overflow for large |x|.
 *  @param x Input value
 *  @return min(cosh(x), 1/DBL_EPSILON)
 */
inline double safeCosH(double const x) {
    auto const coshx = cosh(x);
    if (coshx > 1.0 / DBL_EPSILON) {
        return 1.0 / DBL_EPSILON;
    }
    return coshx;
};

/*! @brief Safe wrapper around sinh that caps the output to [-1/DBL_EPSILON,
 *         1/DBL_EPSILON] to prevent overflow for large |x|.
 *  @param x Input value
 *  @return clamp(sinh(x), -1/DBL_EPSILON, 1/DBL_EPSILON)
 */
inline double safeSinH(double const x) {
    auto const sinhx = sinh(x);
    if (sinhx < -1.0 / DBL_EPSILON) {
        return -1.0 / DBL_EPSILON;
    }
    if (sinhx > 1.0 / DBL_EPSILON) {
        return 1.0 / DBL_EPSILON;
    }
    return sinhx;
};

/*! @brief Safe wrapper around cos. Since cos always returns a value in
 *         [-1, 1] for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value in radians
 *  @return cos(x)
 */
inline double safeCos(double const x) {
    auto const cosx = cos(x);
    if (cosx < -1.0) {
        return -1.0;
    }
    if (cosx > 1.0) {
        return 1.0;
    }
    return cosx;
};

/*! @brief Safe wrapper around sin. Since sin always returns a value in
 *         [-1, 1] for all real inputs, the output guards are defensive and
 *         never trigger in practice.
 *  @param x Input value in radians
 *  @return sin(x)
 */
inline double safeSin(double const x) {
    auto const sinx = sin(x);
    if (sinx < -1.0) {
        return -1.0;
    }
    if (sinx > 1.0) {
        return 1.0;
    }
    return sinx;
};

/*! @brief Safe wrapper around acos that clamps the input to [-1, 1], the
 *         closed domain of acos.
 *  @param x Input value
 *  @return acos(clamp(x, -1, 1))
 */
inline double safeAcos(double const x) {
    if (x < -1.0) {
        return std::numbers::pi;
    }
    if (x > 1.0) {
        return 0.0;
    }
    return acos(x);
};

/*! @brief Safe wrapper around asin that clamps the input to [-1, 1], the
 *         closed domain of asin.
 *  @param x Input value
 *  @return asin(clamp(x, -1, 1))
 */
inline double safeAsin(double const x) {
    if (x < -1.0) {
        return -std::numbers::pi / 2;
    }
    if (x > 1.0) {
        return std::numbers::pi / 2;
    }
    return asin(x);
};

/*! @brief Safe wrapper around sqrt that clamps negative inputs to zero, the
 *         lower bound of the domain of sqrt.
 *  @param x Input value
 *  @return sqrt(max(x, 0))
 */
inline double safeSqrt(double const x) {
    if (x < 0.0) {
        return 0.0;
    }
    return sqrt(x);
};

/*! @brief Safe wrapper around atan2 that returns 0 when both arguments are
 *         zero, the only input pair for which atan2 is undefined.
 *  @param y Numerator argument (opposite side)
 *  @param x Denominator argument (adjacent side)
 *  @return atan2(y, x), or 0.0 when both y and x are zero
 */
inline double safeAtan2(double const y, double const x) {
    if (y == 0.0 && x == 0.0) {
        return 0.0;
    }
    return atan2(y, x);
};

#endif
