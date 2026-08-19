#ifndef CHEBYSHEV_UTILITIES_H
#define CHEBYSHEV_UTILITIES_H

#include "freestandingIsFinite.hpp"

#include <array>
#include <cstddef>

/**
 * @brief Calculate Chebyshev Polynomial
 *
 * @tparam T Floating-point type (float or double).
 * @tparam N Capacity of the coefficients array.
 * @param coefficients Series coefficients.
 * @param numberOfCoefficients How many leading coefficients to use. Must be <= N.
 * @param evaluationPoint Point to evaluate the series.
 * @return The evaluated series value.
 */
template <typename T, std::size_t N>
inline T calculateChebyValue(const std::array<T, N>& coefficients,
                             const unsigned int numberOfCoefficients,
                             const T evaluationPoint) {
    // Sum of zero terms is 0.
    if (numberOfCoefficients == 0U) {
        return static_cast<T>(0.0);
    }
    // Non-finite evaluationPoint: substitute 0.0 rather than propagate NaN/Inf.
    const T safePoint = fsw::is_finite(evaluationPoint) ? evaluationPoint : static_cast<T>(0.0);

    auto chebyPrev = static_cast<T>(1.0);                    // T_0(x) = 1
    auto chebyNow = safePoint;                               // T_1(x) = x
    const auto valueMult = static_cast<T>(2.0) * safePoint;  // 2x, reused every recurrence step

    auto estValue = coefficients.at(0) * chebyPrev;  // c0 * T_0
    if (numberOfCoefficients > 1) {
        estValue += coefficients.at(1) * chebyNow;  // c1 * T_1
        for (unsigned int i = 2; i < numberOfCoefficients; ++i) {
            const auto chebyLocalPrev = chebyNow;           // save T_{i-1} before it's overwritten
            chebyNow = (valueMult * chebyNow) - chebyPrev;  // T_i = 2x*T_{i-1} - T_{i-2}
            chebyPrev = chebyLocalPrev;                     // slide the window: T_{i-2} <- T_{i-1}
            estValue += coefficients.at(i) * chebyNow;      // ci * T_i
        }
    }
    return estValue;
}

#endif
