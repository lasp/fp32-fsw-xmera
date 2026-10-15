#ifndef TEST_CSSCOMM_H
#define TEST_CSSCOMM_H

#include "cssCommAlgorithm.h"
#include "utilities/fsw/chebyshevUtilities.h"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

// Build a per-sensor max-value array with every entry set to the same value.
inline std::array<double, kMaxNumCssSensors> uniformMaxValues(double value) {
    std::array<double, kMaxNumCssSensors> values{};
    values.fill(value);
    return values;
}

// Reference computation that independently reimplements the cssComm algorithm
inline std::array<double, kMaxNumCssSensors> referenceUpdate(
    const std::array<double, kMaxNumCssSensors>& maxSensorValues,
    const std::array<double, kMaxNumChebyPolys>& chebyPolynomials,
    const std::array<double, kMaxNumCssSensors>& inputValues) {
    uint32_t i, j;
    double ChebyDiffFactor, ChebyPrev, ChebyNow, ChebyLocalPrev, ValueMult,
        ChebyPoint; /* Parameters used for the Chebyshev Recursion Forumula */

    std::array<double, kMaxNumCssSensors> output{};

    for (i = 0; i < kMaxNumCssSensors; i++) {
        output[i] = inputValues[i] / maxSensorValues[i]; /* Scale Sensor Data */

        /* The Chebyshev fit is only defined on [-1, 1]; outside it, evaluate at the nearest limit */
        ChebyPoint = output[i];
        if (ChebyPoint > 1.0) {
            ChebyPoint = 1.0;
        } else if (ChebyPoint < -1.0) {
            ChebyPoint = -1.0;
        }

        /* Seed the polynomial computations */
        ValueMult = 2.0 * ChebyPoint;
        ChebyPrev = 1.0;
        ChebyNow = ChebyPoint;
        ChebyDiffFactor = ChebyPrev * chebyPolynomials[0]; /* first-order term */
        ChebyDiffFactor += ChebyNow * chebyPolynomials[1]; /* second-order term */

        /* Loop over remaining polynomials and add in values */
        for (j = 2; j < kMaxNumChebyPolys; j = j + 1) {
            ChebyLocalPrev = ChebyNow;
            ChebyNow = ValueMult * ChebyNow - ChebyPrev;
            ChebyPrev = ChebyLocalPrev;
            ChebyDiffFactor += chebyPolynomials[j] * ChebyNow;
        }

        output[i] = output[i] + ChebyDiffFactor;

        if (output[i] > 1.0) {
            output[i] = 1.0;
        } else if (output[i] < 0.0) {
            output[i] = 0.0;
        }
    }

    return output;
}

inline void regressionTestCssComm(std::vector<double> maxSensorValues,
                                  std::vector<double> chebyCoeffs,
                                  std::vector<double> sensorInputRatios) {
    std::array<double, kMaxNumChebyPolys> polynomials{};
    for (std::size_t i = 0; i < chebyCoeffs.size() && i < kMaxNumChebyPolys; ++i) {
        polynomials[i] = chebyCoeffs[i];
    }

    std::array<double, kMaxNumCssSensors> maxValues{};
    for (std::size_t i = 0; i < maxSensorValues.size() && i < kMaxNumCssSensors; ++i) {
        maxValues[i] = maxSensorValues[i];
    }

    CssCommAlgorithm alg{CssCommConfig::create(maxValues, polynomials)};

    std::array<double, kMaxNumCssSensors> inputValues{};
    for (std::size_t i = 0; i < sensorInputRatios.size() && i < kMaxNumCssSensors; ++i) {
        inputValues[i] = sensorInputRatios[i] * maxValues[i];
    }

    std::array<double, kMaxNumCssSensors> output{};
    EXPECT_NO_THROW(output = alg.update(inputValues));

    auto reference = referenceUpdate(maxValues, polynomials, inputValues);

    for (uint32_t i = 0; i < kMaxNumCssSensors; ++i) {
        EXPECT_NEAR(output[i], reference[i], 1e-12);
        EXPECT_TRUE(std::isfinite(output[i]));
        EXPECT_GE(output[i], 0.0);
        EXPECT_LE(output[i], 1.0);
    }
}

#endif  // TEST_CSSCOMM_H
