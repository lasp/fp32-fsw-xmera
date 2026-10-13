#ifndef TEST_GIMBALTOMOTORANGLESSPLINE_H
#define TEST_GIMBALTOMOTORANGLESSPLINE_H

#include "gimbalToMotorAnglesSplineAlgorithm.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

constexpr float kReferenceDefaultMotorAngle = 103.2242F * std::numbers::pi_v<float> / 180.0F;
constexpr float degToRad = std::numbers::pi_v<float> / 180.0F;

/*! @brief One row of a fixture file: gimbal angles and the corresponding motor angles. */
struct GimbalMotorSample {
    float tip;     //!< [rad] Gimbal tip angle
    float tilt;    //!< [rad] Gimbal tilt angle
    float theta1;  //!< [rad] Motor 1 angle
    float theta2;  //!< [rad] Motor 2 angle
};

/*! This method returns the full path of a fixture file.
 @return std::string
 @param fileName [-] File name relative to the fixture directory
*/
inline std::string fixturePath(const std::string& fileName) {
    // TABLE_DATA_DIR is the directory holding the fixture files, supplied by CMake at configure time
    return std::string{TABLE_DATA_DIR} + "/" + fileName;
}

/*! This method reads a CSV file that holds a single line of comma-separated values into a fixed-size array.
 @return std::array<float, numElements>
 @param fileName [-] File name relative to the fixture directory
*/
template <std::size_t numElements>
std::array<float, numElements> readCsvLine(const std::string& fileName) {
    std::ifstream file(fixturePath(fileName));
    EXPECT_TRUE(file.is_open()) << "could not open " << fixturePath(fileName);

    // Read at most numElements values so that a longer than expected file cannot overrun the array
    std::array<float, numElements> dataArray{};
    std::string valueText;
    std::size_t numValuesRead = 0;
    while (numValuesRead < numElements && std::getline(file, valueText, ',')) {
        dataArray[numValuesRead] = static_cast<float>(std::stod(valueText));
        ++numValuesRead;
    }
    EXPECT_EQ(numValuesRead, numElements) << fileName << " holds " << numValuesRead << " values";

    return dataArray;
}

/*! This method reads a fixture file with one "tip, tilt, theta1, theta2" row per line. Lines that start with '#'
 are comments.
 @return std::vector<GimbalMotorSample>
 @param fileName [-] File name relative to the fixture directory
*/
inline std::vector<GimbalMotorSample> readSamples(const std::string& fileName) {
    std::ifstream file(fixturePath(fileName));
    EXPECT_TRUE(file.is_open()) << "could not open " << fixturePath(fileName);

    std::vector<GimbalMotorSample> samples;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::array<double, 4> values{};
        std::stringstream stream(line);
        std::string valueText;
        for (double& value : values) {
            std::getline(stream, valueText, ',');
            value = std::stod(valueText);
        }
        samples.push_back({static_cast<float>(values[0]),
                           static_cast<float>(values[1]),
                           static_cast<float>(values[2]),
                           static_cast<float>(values[3])});
    }
    EXPECT_FALSE(samples.empty()) << fileName << " holds no samples";

    return samples;
}

/*! This method reads the fitted rhombus fixture.
 @return GimbalRhombusDomain
*/
inline GimbalRhombusDomain readDomain() {
    const auto values = readCsvLine<4>("splineDomain.csv");
    return {.centerTipAngle = values[0],
            .centerTiltAngle = values[1],
            .halfDiagonalTip = values[2],
            .halfDiagonalTilt = values[3]};
}

/*! This method reads the fitted spline coefficient fixture.
 @return SplineCoefficients
*/
inline SplineCoefficients readCoefficients() { return readCsvLine<NUM_SPLINE_COEFFS>("splineCoefficients.csv"); }

/*! This method returns the configuration built from the fitted fixtures.
 @return GimbalToMotorAnglesSplineConfig
*/
inline GimbalToMotorAnglesSplineConfig makeFittedConfig() {
    return GimbalToMotorAnglesSplineConfig::create(readDomain(), readCoefficients());
}

/*! This method maps normalized rhombus coordinates to gimbal angles.
 @return std::array<float, 2> [rad] Gimbal tip and tilt angles
 @param domain [rad] Addressable gimbal rhombus
 @param u [-] First normalized rhombus coordinate
 @param v [-] Second normalized rhombus coordinate
*/
inline std::array<float, 2> gimbalAnglesFromUv(const GimbalRhombusDomain& domain, const float u, const float v) {
    return {domain.centerTipAngle + (domain.halfDiagonalTip * (u + v) / 2.0F),
            domain.centerTiltAngle + (domain.halfDiagonalTilt * (u - v) / 2.0F)};
}

/*! This method returns the smallest and largest spline coefficient. B-spline basis weights are non-negative and sum
 to one, so every surface value lies between them (convex-hull property).
 @return std::array<float, 2> [rad]
 @param coefficients [rad] Spline control coefficients
*/
inline std::array<float, 2> coefficientBounds(const SplineCoefficients& coefficients) {
    const auto [minValue, maxValue] = std::ranges::minmax(coefficients);
    return {minValue, maxValue};
}

constexpr float kEps = std::numeric_limits<float>::epsilon();
constexpr float kIntervalWidth = 2.0F / static_cast<float>(NUM_SPLINE_INTERVALS);  // [-] knot spacing in u and v

/*! This method returns an upper bound on |dS/du| and |dS/dv|. The derivative of a cubic B-spline is a quadratic
 B-spline whose coefficients are the coefficient differences divided by the knot spacing.
 @return float [rad] Slope bound per unit of u or v
 @param coefficients [rad] Spline control coefficients
*/
inline float surfaceSlopeBound(const SplineCoefficients& coefficients) {
    float maxDifference = 0.0F;
    for (int i = 0; i < NUM_SPLINE_COEFFS_PER_AXIS; ++i) {
        for (int j = 0; j < NUM_SPLINE_COEFFS_PER_AXIS; ++j) {
            const float c = coefficients[(i * NUM_SPLINE_COEFFS_PER_AXIS) + j];
            if (i + 1 < NUM_SPLINE_COEFFS_PER_AXIS) {
                maxDifference =
                    std::max(maxDifference, std::fabs(coefficients[((i + 1) * NUM_SPLINE_COEFFS_PER_AXIS) + j] - c));
            }
            if (j + 1 < NUM_SPLINE_COEFFS_PER_AXIS) {
                maxDifference =
                    std::max(maxDifference, std::fabs(coefficients[(i * NUM_SPLINE_COEFFS_PER_AXIS) + j + 1] - c));
            }
        }
    }
    return maxDifference / kIntervalWidth;
}

/*! This method returns the fp32 evaluation tolerance against a float64 reference evaluation of the same surface.
 - Coefficient rounding to float: <= eps/2 * max|c| (the weights are non-negative and sum to one).
 - Weights (~10 operations) and the 4 x 4 weighted sum (~8 operations deep): ~20 eps * max|c|.
 - u and v: ~5 operations on once-rounded inputs, ~9 eps each, times the slope bound, for both u and v.
 The sum is multiplied by 3 for margin.
 @return float [rad]
 @param coefficients [rad] Spline control coefficients
*/
inline float evaluationTolerance(const SplineCoefficients& coefficients) {
    const auto [minCoeff, maxCoeff] = coefficientBounds(coefficients);
    const float maxAbsCoeff = std::max(std::fabs(minCoeff), std::fabs(maxCoeff));
    const float valueError = 20.5F * kEps * maxAbsCoeff;
    const float coordinateError = 2.0F * 9.0F * kEps * surfaceSlopeBound(coefficients);
    return 3.0F * (valueError + coordinateError);
}

#endif  // TEST_GIMBALTOMOTORANGLESSPLINE_H
