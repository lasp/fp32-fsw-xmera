#ifndef SAFE_MATH_FLOAT_H_
#define SAFE_MATH_FLOAT_H_

#include <float.h>
#include <math.h>

// ============================================================================
// Float versions
// ============================================================================

/*! @brief Safe wrapper around tanf that clamps the input to [-1, 1] rad to
 *         stay well clear of the singularities at ±π/2.
 *  @param x Input value in radians
 *  @return tanf(clamp(x, -1, 1))
 */
inline float safeTanf(float const x){
    if (x < -1.0F) {
        return tanf(-1.0F);
    }
    if (x > 1.0F) {
        return tanf(1.0F);
    }
    return tanf(x);
};

/*! @brief Safe wrapper around atanf that clamps the input to [-1, 1].
 *  @param x Input value
 *  @return atanf(clamp(x, -1, 1))
 */
inline float safeAtanf(float const x){
    if (x < -1.0F) {
        return atanf(-1.0F);
    }
    if (x > 1.0F) {
        return atanf(1.0F);
    }
    return atanf(x);
};

/*! @brief Safe wrapper around tanhf that clamps the input to [-1, 1].
 *  @param x Input value
 *  @return tanhf(clamp(x, -1, 1))
 */
inline float safeTanHf(float const x){
    if (x < -1.0F) {
        return tanhf(-1.0F);
    }
    if (x > 1.0F) {
        return tanhf(1.0F);
    }
    return tanhf(x);
};

/*! @brief Safe wrapper around atanhf that clamps the input to
 *         [-(1 - FLT_EPSILON), (1 - FLT_EPSILON)]. The domain of atanhf is
 *         the open interval (-1, 1); passing ±1 produces ±infinity, so the
 *         bound is kept strictly inside the domain.
 *  @param x Input value
 *  @return atanhf(clamp(x, -(1 - FLT_EPSILON), (1 - FLT_EPSILON)))
 */
inline float safeAtanHf(float const x){
    if (x < -(1.0F - FLT_EPSILON)) {
        return atanhf(-(1.0F - FLT_EPSILON));
    }
    if (x > (1.0F - FLT_EPSILON)) {
        return atanhf(1.0F - FLT_EPSILON);
    }
    return atanhf(x);
};

/*! @brief Safe wrapper around coshf that clamps the input to [-1, 1].
 *  @param x Input value
 *  @return coshf(clamp(x, -1, 1))
 */
inline float safeCosHf(float const x){
    if (x < -1.0F) {
        return coshf(-1.0F);
    }
    if (x > 1.0F) {
        return coshf(1.0F);
    }
    return coshf(x);
};

/*! @brief Safe wrapper around sinhf that clamps the input to [-1, 1].
 *  @param x Input value
 *  @return sinhf(clamp(x, -1, 1))
 */
inline float safeSinHf(float const x){
    if (x < -1.0F) {
        return sinhf(-1.0F);
    }
    if (x > 1.0F) {
        return sinhf(1.0F);
    }
    return sinhf(x);
};

/*! @brief Safe wrapper around cosf that clamps the input to [-1, 1] rad.
 *  @param x Input value in radians
 *  @return cosf(clamp(x, -1, 1))
 */
inline float safeCosf(float const x){
    if (x < -1.0F) {
        return cosf(-1.0F);
    }
    if (x > 1.0F) {
        return cosf(1.0F);
    }
    return cosf(x);
};

/*! @brief Safe wrapper around sinf that clamps the input to [-1, 1] rad.
 *  @param x Input value in radians
 *  @return sinf(clamp(x, -1, 1))
 */
inline float safeSinf(float const x){
    if (x < -1.0F) {
        return sinf(-1.0F);
    }
    if (x > 1.0F) {
        return sinf(1.0F);
    }
    return sinf(x);
};

/*! @brief Safe wrapper around acosf that clamps the input to [-1, 1], the
 *         closed domain of acosf.
 *  @param x Input value
 *  @return acosf(clamp(x, -1, 1))
 */
inline float safeAcosf(float const x){
    if (x < -1.0F) {
        return acosf(-1.0F);
    }
    if (x > 1.0F) {
        return acosf(1.0F);
    }
    return acosf(x);
};

/*! @brief Safe wrapper around asinf that clamps the input to [-1, 1], the
 *         closed domain of asinf.
 *  @param x Input value
 *  @return asinf(clamp(x, -1, 1))
 */
inline float safeAsinf(float const x){
    if (x < -1.0F) {
        return asinf(-1.0F);
    }
    if (x > 1.0F) {
        return asinf(1.0F);
    }
    return asinf(x);
};

/*! @brief Safe wrapper around sqrtf that clamps negative inputs to zero, the
 *         lower bound of the domain of sqrtf.
 *  @param x Input value
 *  @return sqrtf(max(x, 0))
 */
inline float safeSqrtf(float const x){
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
inline float safeAtan2f(float const y, float const x){
    if (y == 0.0F && x == 0.0F) {
        return 0.0F;
    }
    return atan2f(y, x);
};

// ============================================================================
// Double versions
// ============================================================================

/*! @brief Safe wrapper around tan that clamps the input to [-1, 1] rad to
 *         stay well clear of the singularities at ±π/2.
 *  @param x Input value in radians
 *  @return tan(clamp(x, -1, 1))
 */
inline double safeTan(double const x){
    if (x < -1.0) {
        return tan(-1.0);
    }
    if (x > 1.0) {
        return tan(1.0);
    }
    return tan(x);
};

/*! @brief Safe wrapper around atan that clamps the input to [-1, 1].
 *  @param x Input value
 *  @return atan(clamp(x, -1, 1))
 */
inline double safeAtan(double const x){
    if (x < -1.0) {
        return atan(-1.0);
    }
    if (x > 1.0) {
        return atan(1.0);
    }
    return atan(x);
};

/*! @brief Safe wrapper around tanh that clamps the input to [-1, 1].
 *  @param x Input value
 *  @return tanh(clamp(x, -1, 1))
 */
inline double safeTanH(double const x){
    if (x < -1.0) {
        return tanh(-1.0);
    }
    if (x > 1.0) {
        return tanh(1.0);
    }
    return tanh(x);
};

/*! @brief Safe wrapper around atanh that clamps the input to
 *         [-(1 - DBL_EPSILON), (1 - DBL_EPSILON)]. The domain of atanh is
 *         the open interval (-1, 1); passing ±1 produces ±infinity, so the
 *         bound is kept strictly inside the domain.
 *  @param x Input value
 *  @return atanh(clamp(x, -(1 - DBL_EPSILON), (1 - DBL_EPSILON)))
 */
inline double safeAtanH(double const x){
    if (x < -(1.0 - DBL_EPSILON)) {
        return atanh(-(1.0 - DBL_EPSILON));
    }
    if (x > (1.0 - DBL_EPSILON)) {
        return atanh(1.0 - DBL_EPSILON);
    }
    return atanh(x);
};

/*! @brief Safe wrapper around cosh that clamps the input to [-1, 1].
 *  @param x Input value
 *  @return cosh(clamp(x, -1, 1))
 */
inline double safeCosH(double const x){
    if (x < -1.0) {
        return cosh(-1.0);
    }
    if (x > 1.0) {
        return cosh(1.0);
    }
    return cosh(x);
};

/*! @brief Safe wrapper around sinh that clamps the input to [-1, 1].
 *  @param x Input value
 *  @return sinh(clamp(x, -1, 1))
 */
inline double safeSinH(double const x){
    if (x < -1.0) {
        return sinh(-1.0);
    }
    if (x > 1.0) {
        return sinh(1.0);
    }
    return sinh(x);
};

/*! @brief Safe wrapper around cos that clamps the input to [-1, 1] rad.
 *  @param x Input value in radians
 *  @return cos(clamp(x, -1, 1))
 */
inline double safeCos(double const x){
    if (x < -1.0) {
        return cos(-1.0);
    }
    if (x > 1.0) {
        return cos(1.0);
    }
    return cos(x);
};

/*! @brief Safe wrapper around sin that clamps the input to [-1, 1] rad.
 *  @param x Input value in radians
 *  @return sin(clamp(x, -1, 1))
 */
inline double safeSin(double const x){
    if (x < -1.0) {
        return sin(-1.0);
    }
    if (x > 1.0) {
        return sin(1.0);
    }
    return sin(x);
};

/*! @brief Safe wrapper around acos that clamps the input to [-1, 1], the
 *         closed domain of acos.
 *  @param x Input value
 *  @return acos(clamp(x, -1, 1))
 */
inline double safeAcos(double const x){
    if (x < -1.0) {
        return acos(-1.0);
    }
    if (x > 1.0) {
        return acos(1.0);
    }
    return acos(x);
};

/*! @brief Safe wrapper around asin that clamps the input to [-1, 1], the
 *         closed domain of asin.
 *  @param x Input value
 *  @return asin(clamp(x, -1, 1))
 */
inline double safeAsin(double const x){
    if (x < -1.0) {
        return asin(-1.0);
    }
    if (x > 1.0) {
        return asin(1.0);
    }
    return asin(x);
};

/*! @brief Safe wrapper around sqrt that clamps negative inputs to zero, the
 *         lower bound of the domain of sqrt.
 *  @param x Input value
 *  @return sqrt(max(x, 0))
 */
inline double safeSqrt(double const x){
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
inline double safeAtan2(double const y, double const x){
    if (y == 0.0 && x == 0.0) {
        return 0.0;
    }
    return atan2(y, x);
};

#endif
