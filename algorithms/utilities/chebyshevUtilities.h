/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

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
