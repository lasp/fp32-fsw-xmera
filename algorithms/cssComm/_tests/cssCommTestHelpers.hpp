#ifndef TEST_CSSCOMM_H
#define TEST_CSSCOMM_H

#include "cssCommAlgorithm.h"
#include "utilities/fsw/chebyshevUtilities.h"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

// Build a per-sensor max-value array with every entry set to the same value.
inline std::array<double, MAX_NUM_CSS_SENSORS> uniformMaxValues(double value) {
    std::array<double, MAX_NUM_CSS_SENSORS> values{};
    values.fill(value);
    return values;
}

// Reference computation that independently reimplements the cssComm algorithm
inline std::array<double, MAX_NUM_CSS_SENSORS> referenceUpdate(
    uint32_t numSensors,
    const std::array<double, MAX_NUM_CSS_SENSORS>& maxSensorValues,
    uint32_t chebyCount,
    const std::array<double, kMaxNumChebyPolys>& chebyPolynomials,
    const std::array<double, MAX_NUM_CSS_SENSORS>& inputValues) {
    uint32_t i, j;
    double ChebyDiffFactor, ChebyPrev, ChebyNow, ChebyLocalPrev,
        ValueMult; /* Parameters used for the Chebyshev Recursion Forumula */

    std::array<double, MAX_NUM_CSS_SENSORS> output{};

    for (i = 0; i < numSensors; i++) {
        output[i] = inputValues[i] / maxSensorValues[i]; /* Scale Sensor Data */

        /* Seed the polynomial computations */
        ValueMult = 2.0 * output[i];
        ChebyPrev = 1.0;
        ChebyNow = output[i];
        ChebyDiffFactor = 0.0;
        ChebyDiffFactor =
            chebyCount > 0 ? ChebyPrev * chebyPolynomials[0] : ChebyDiffFactor; /* if only first order correction */
        ChebyDiffFactor = chebyCount > 1 ? ChebyNow * chebyPolynomials[1] + ChebyDiffFactor
                                         : ChebyDiffFactor; /* if higher order (> first) corrections */

        /* Loop over remaining polynomials and add in values */
        for (j = 2; j < chebyCount; j = j + 1) {
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

inline void regressionTestCssComm(uint32_t numSensors,
                                  std::vector<double> maxSensorValues,
                                  uint32_t chebyCount,
                                  std::vector<double> chebyCoeffs,
                                  std::vector<double> sensorInputRatios) {
    std::array<double, kMaxNumChebyPolys> polynomials{};
    for (std::size_t i = 0; i < chebyCoeffs.size() && i < kMaxNumChebyPolys; ++i) {
        polynomials[i] = chebyCoeffs[i];
    }

    std::array<double, MAX_NUM_CSS_SENSORS> maxValues{};
    for (std::size_t i = 0; i < maxSensorValues.size() && i < MAX_NUM_CSS_SENSORS; ++i) {
        maxValues[i] = maxSensorValues[i];
    }

    CssCommAlgorithm alg{CssCommConfig::create(numSensors, maxValues, chebyCount, polynomials)};

    std::array<double, MAX_NUM_CSS_SENSORS> inputValues{};
    for (std::size_t i = 0; i < sensorInputRatios.size() && i < MAX_NUM_CSS_SENSORS; ++i) {
        inputValues[i] = sensorInputRatios[i] * maxValues[i];
    }

    std::array<double, MAX_NUM_CSS_SENSORS> output{};
    EXPECT_NO_THROW(output = alg.update(inputValues));

    auto reference = referenceUpdate(numSensors, maxValues, chebyCount, polynomials, inputValues);

    for (uint32_t i = 0; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_NEAR(output[i], reference[i], 1e-12);
        EXPECT_TRUE(std::isfinite(output[i]));
        EXPECT_GE(output[i], 0.0);
        EXPECT_LE(output[i], 1.0);
    }
}

#endif  // TEST_CSSCOMM_H
