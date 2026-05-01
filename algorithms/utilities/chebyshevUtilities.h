#ifndef _CHEBYSHEV_UTILITIES_FP32_H_
#define _CHEBYSHEV_UTILITIES_FP32_H_

#include <array>
#include <cstddef>

/*! Calculate Chebyshev Polynomial */
template <typename T, std::size_t N>
inline T calculateChebyValue(const std::array<T, N>& coefficients,
                             const unsigned int numberOfCoefficients,
                             const T evaluationPoint) {
    auto chebyPrev = static_cast<T>(1.0);
    auto chebyNow = evaluationPoint;
    const auto valueMult = static_cast<T>(2.0) * evaluationPoint;

    auto estValue = coefficients.at(0) * chebyPrev;
    if (numberOfCoefficients > 1) {
        estValue += coefficients.at(1) * chebyNow;
        for (unsigned int i = 2; i < numberOfCoefficients; ++i) {
            const auto chebyLocalPrev = chebyNow;
            chebyNow = (valueMult * chebyNow) - chebyPrev;
            chebyPrev = chebyLocalPrev;
            estValue += coefficients.at(i) * chebyNow;
        }
    }
    return estValue;
}

#endif
