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

#ifndef _EPHEMERIS_UTILITIES_FP32_H_
#define _EPHEMERIS_UTILITIES_FP32_H_

#include "msgPayloadDef/definitions.h"
#include <array>

/*! Calculate Chebychev Polynominal */
inline double calculateChebyValue(const std::array<double, MAX_OE_COEFF>& coefficients,
                                  const unsigned int numberOfCoefficients,
                                  const double evaluationPoint) {
    double chebyPrev = 1.0;
    double chebyNow = evaluationPoint;
    const double valueMult = 2 * evaluationPoint;

    double estValue = coefficients.at(0) * chebyPrev;
    if (numberOfCoefficients > 1) {
        estValue += coefficients.at(1) * chebyNow;
        for (auto i = 2; i < numberOfCoefficients; ++i) {
            const double chebyLocalPrev = chebyNow;
            chebyNow = (valueMult * chebyNow) - chebyPrev;
            chebyPrev = chebyLocalPrev;
            estValue += coefficients.at(i) * chebyNow;
        }
    }
    return estValue;
};

inline float calculateChebyValueF32(const std::array<float, MAX_OE_COEFF>& coefficients,
                                    const unsigned int numberOfCoefficients,
                                    const float evaluationPoint) {
    float chebyPrev = 1.0;
    float chebyNow = evaluationPoint;
    const float valueMult = 2.0 * evaluationPoint;

    float estValue = coefficients.at(0) * chebyPrev;
    if (numberOfCoefficients <= 1) {
        return estValue;
    }
    estValue += coefficients.at(1) * chebyNow;
    for (int i = 2; i < numberOfCoefficients; ++i) {
        const float chebyLocalPrev = chebyNow;
        chebyNow = (valueMult * chebyNow) - chebyPrev;
        chebyPrev = chebyLocalPrev;
        estValue += coefficients.at(i) * chebyNow;
    }

    return estValue;
}

#endif
