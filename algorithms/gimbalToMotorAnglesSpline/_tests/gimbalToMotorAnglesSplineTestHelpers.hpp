#ifndef TEST_GIMBALTOMOTORANGLESSPLINE_H
#define TEST_GIMBALTOMOTORANGLESSPLINE_H

#include "gimbalToMotorAnglesSplineAlgorithm.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

constexpr float degToRad = std::numbers::pi_v<float> / 180.0F;
// [rad/step] Step angle of every test configuration. The tests compare outputs in steps, so its value matters little.
constexpr float kTestStepAngle = 0.01F;
// [rad] Default motor angles of every test configuration. They differ from each other so that a swap is caught.
constexpr float kTestDefaultMotor1Angle = -0.8F;
constexpr float kTestDefaultMotor2Angle = 1.2F;

/*! @brief One row of a fixture file: gimbal angles and the corresponding motor steps. */
struct GimbalMotorSample {
    float tip;     //!< [rad] Gimbal tip angle
    float tilt;    //!< [rad] Gimbal tilt angle
    float steps1;  //!< [steps] Motor 1 steps
    float steps2;  //!< [steps] Motor 2 steps
};

/*! @brief Motor steps of an algorithm output. */
struct MotorSteps {
    float motor1;  //!< [steps] Motor 1 steps
    float motor2;  //!< [steps] Motor 2 steps
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

/*! This method reads a fixture file with one "tip, tilt, steps1, steps2" row per line. Lines that start with '#'
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

/*! This method reads the fitted motor 1 spline coefficient fixture.
 @return SplineCoefficients [steps]
*/
inline SplineCoefficients readMotor1Coefficients() {
    return readCsvLine<NUM_SPLINE_COEFFS>("splineCoefficientsMotor1.csv");
}

/*! This method reads the fitted motor 2 spline coefficient fixture.
 @return SplineCoefficients [steps]
*/
inline SplineCoefficients readMotor2Coefficients() {
    return readCsvLine<NUM_SPLINE_COEFFS>("splineCoefficientsMotor2.csv");
}

/*! This method returns the configuration built from the fitted fixtures and the test step angle.
 @return GimbalToMotorAnglesSplineConfig
*/
inline GimbalToMotorAnglesSplineConfig makeFittedConfig() {
    return GimbalToMotorAnglesSplineConfig::create(kTestStepAngle,
                                                   kTestDefaultMotor1Angle,
                                                   kTestDefaultMotor2Angle,
                                                   readDomain(),
                                                   readMotor1Coefficients(),
                                                   readMotor2Coefficients());
}

/*! This method converts an algorithm output back to motor steps. The algorithm multiplies the steps by the step angle
 and this method divides by it, so the steps differ from the surface value by kAngleConversionRelativeError at most.
 @return MotorSteps [steps]
 @param output [rad] Algorithm output
 @param stepAngle [rad/step] Step angle of the configuration
*/
inline MotorSteps motorSteps(const GimbalToMotorAnglesSplineOutput& output, const float stepAngle = kTestStepAngle) {
    return {.motor1 = output.motorAngle1 / stepAngle, .motor2 = output.motorAngle2 / stepAngle};
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
 @return std::array<float, 2> [steps]
 @param coefficients [steps] Spline control coefficients
*/
inline std::array<float, 2> coefficientBounds(const SplineCoefficients& coefficients) {
    const auto [minValue, maxValue] = std::ranges::minmax(coefficients);
    return {minValue, maxValue};
}

constexpr float kEps = std::numeric_limits<float>::epsilon();
// [-] Relative fp32 rounding error of a surface value, as a fraction of the largest |coefficient|. Rounding the
// coefficients to float gives eps/2. The weights (~10 operations) and the 4 x 4 weighted sum (~8 operations deep)
// give ~20 eps, because the weights are non-negative and sum to one.
constexpr float kSurfaceValueRelativeError = 20.5F * kEps;
// [-] Relative fp32 rounding error of the motor steps that motorSteps recovers from an output: the multiplication by
// the step angle in the algorithm and the division by it in the test, eps/2 each.
constexpr float kAngleConversionRelativeError = kEps;
constexpr float kIntervalWidth = 2.0F / static_cast<float>(NUM_SPLINE_INTERVALS);  // [-] knot spacing in u and v

// [steps] Scale of the linear test surfaces: motor 1 = kLinearSurfaceScale * v and motor 2 = kLinearSurfaceScale * u
constexpr float kLinearSurfaceScale = 1000.0F;

/*! This method returns the Greville abscissa of a coefficient index: the average of the three inner knots of its
 basis function. A cubic B-spline whose coefficients are the Greville abscissae is exactly the coordinate itself.
 @return float [-]
 @param index [-] Coefficient index along one coordinate, 0 to NUM_SPLINE_COEFFS_PER_AXIS - 1
*/
inline float grevilleAbscissa(const int index) { return -1.0F + (kIntervalWidth * static_cast<float>(index - 1)); }

/*! @brief Coordinate that a linear test surface changes along. */
enum class SurfaceAxis { U, V };

/*! This method returns the coefficients of the linear surface S(u, v) = kLinearSurfaceScale * u or
 kLinearSurfaceScale * v. Coefficient c[i][j] is the scaled Greville abscissa of i along u, or of j along v.
 @return SplineCoefficients [steps]
 @param axis Coordinate that the surface changes along
*/
inline SplineCoefficients linearSurfaceCoefficients(const SurfaceAxis axis) {
    SplineCoefficients coefficients{};
    for (int i = 0; i < NUM_SPLINE_COEFFS_PER_AXIS; ++i) {
        for (int j = 0; j < NUM_SPLINE_COEFFS_PER_AXIS; ++j) {
            const int index = (axis == SurfaceAxis::U) ? i : j;
            coefficients[(i * NUM_SPLINE_COEFFS_PER_AXIS) + j] = kLinearSurfaceScale * grevilleAbscissa(index);
        }
    }
    return coefficients;
}

/*! This method returns a valid rhombus that does not depend on the fit fixtures.
 @return GimbalRhombusDomain [rad]
*/
inline GimbalRhombusDomain linearTestDomain() {
    return {.centerTipAngle = 0.01F, .centerTiltAngle = 0.0F, .halfDiagonalTip = 0.3F, .halfDiagonalTilt = 0.45F};
}

/*! This method returns a configuration with known outputs: motor 1 = kLinearSurfaceScale * v steps and motor 2 =
 kLinearSurfaceScale * u steps inside the rhombus. The two motors have different surfaces, so a swap of the motors or
 of u and v is caught.
 @return GimbalToMotorAnglesSplineConfig
*/
inline GimbalToMotorAnglesSplineConfig makeLinearConfig() {
    return GimbalToMotorAnglesSplineConfig::create(kTestStepAngle,
                                                   kTestDefaultMotor1Angle,
                                                   kTestDefaultMotor2Angle,
                                                   linearTestDomain(),
                                                   linearSurfaceCoefficients(SurfaceAxis::V),
                                                   linearSurfaceCoefficients(SurfaceAxis::U));
}

// [steps] Tolerance of the linear surfaces: the evaluation error plus the angle conversion error. Their coefficients
// are at most kLinearSurfaceScale * (1 + h) in magnitude, and their slope is kLinearSurfaceScale, so the rounding of u
// and v (~9 eps each) adds about as much as the value error. The sum is multiplied by 3 for margin, as in
// evaluationTolerance.
constexpr float kLinearSurfaceTolerance = 3.0F * (1.0F + kIntervalWidth) * kLinearSurfaceScale *
                                          ((2.0F * kSurfaceValueRelativeError) + kAngleConversionRelativeError);

/*! This method returns an upper bound on |dS/du| and |dS/dv|. The derivative of a cubic B-spline is a quadratic
 B-spline whose coefficients are the coefficient differences divided by the knot spacing.
 @return float [steps] Slope bound per unit of u or v
 @param coefficients [steps] Spline control coefficients
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

/*! This method returns the fp32 evaluation tolerance of a surface value against a float64 reference evaluation of the
 same surface.
 - Coefficient rounding to float: <= eps/2 * max|c| (the weights are non-negative and sum to one).
 - Weights (~10 operations) and the 4 x 4 weighted sum (~8 operations deep): ~20 eps * max|c|.
 - u and v: ~5 operations on once-rounded inputs, ~9 eps each, times the slope bound, for both u and v.
 The sum is multiplied by 3 for margin.
 @return float [steps]
 @param coefficients [steps] Spline control coefficients
*/
inline float evaluationTolerance(const SplineCoefficients& coefficients) {
    const auto [minCoeff, maxCoeff] = coefficientBounds(coefficients);
    const float maxAbsCoeff = std::max(std::fabs(minCoeff), std::fabs(maxCoeff));
    const float valueError = kSurfaceValueRelativeError * maxAbsCoeff;
    const float coordinateError = 2.0F * 9.0F * kEps * surfaceSlopeBound(coefficients);
    return 3.0F * (valueError + coordinateError);
}

/*! This method returns the tolerance of the motor steps that motorSteps recovers from an output against a float64
 reference evaluation of the same surface: the fp32 evaluation tolerance plus the angle conversion error.
 @return float [steps]
 @param coefficients [steps] Spline control coefficients
*/
inline float stepTolerance(const SplineCoefficients& coefficients) {
    const auto [minCoeff, maxCoeff] = coefficientBounds(coefficients);
    const float maxAbsCoeff = std::max(std::fabs(minCoeff), std::fabs(maxCoeff));
    return evaluationTolerance(coefficients) + (kAngleConversionRelativeError * maxAbsCoeff);
}

/*! This method returns the value at x of the cubic B-spline basis function with the given index, by the Cox-de Boor
 recursion in float64. The knots are uniform, t_k = -1 + h (k - 3), so the knot vector extends three knots past each
 end of [-1, 1]. This evaluation does not share any code with the algorithm.
 @return double [-]
 @param index [-] Basis function index, 0 to NUM_SPLINE_COEFFS_PER_AXIS - 1
 @param degree [-] Polynomial degree of the basis function
 @param x [-] Normalized coordinate
*/
inline double referenceBasis(const int index, const int degree, const double x) {
    const double h = 2.0 / static_cast<double>(NUM_SPLINE_INTERVALS);
    const auto knot = [h](const int k) { return -1.0 + (h * static_cast<double>(k - 3)); };
    if (degree == 0) {
        return (knot(index) <= x && x < knot(index + 1)) ? 1.0 : 0.0;
    }
    const double left = (x - knot(index)) / (knot(index + degree) - knot(index));
    const double right = (knot(index + degree + 1) - x) / (knot(index + degree + 1) - knot(index + 1));
    return (left * referenceBasis(index, degree - 1, x)) + (right * referenceBasis(index + 1, degree - 1, x));
}

/*! @brief Values of all the cubic basis functions along u and along v at one point, in float64. */
struct ReferenceBasisValues {
    std::array<double, NUM_SPLINE_COEFFS_PER_AXIS> alongU;  //!< [-] B_i(u) for every coefficient index i
    std::array<double, NUM_SPLINE_COEFFS_PER_AXIS> alongV;  //!< [-] B_j(v) for every coefficient index j
};

/*! This method returns the values of all the cubic basis functions at (u, v), by the Cox-de Boor recursion.
 @return ReferenceBasisValues [-]
 @param u [-] First argument of the surface
 @param v [-] Second argument of the surface
*/
inline ReferenceBasisValues referenceBasisValues(const double u, const double v) {
    ReferenceBasisValues basis{};
    for (int k = 0; k < NUM_SPLINE_COEFFS_PER_AXIS; ++k) {
        basis.alongU[k] = referenceBasis(k, 3, u);
        basis.alongV[k] = referenceBasis(k, 3, v);
    }
    return basis;
}

/*! This method returns the surface sum_i sum_j c_ij B_i(u) B_j(v) in float64 from precomputed basis values.
 @return double [steps]
 @param coefficients [steps] Spline control coefficients
 @param basis [-] Basis values at the point
*/
inline double referenceSurface(const SplineCoefficients& coefficients, const ReferenceBasisValues& basis) {
    double value = 0.0;
    for (int i = 0; i < NUM_SPLINE_COEFFS_PER_AXIS; ++i) {
        double rowValue = 0.0;
        for (int j = 0; j < NUM_SPLINE_COEFFS_PER_AXIS; ++j) {
            rowValue += static_cast<double>(coefficients[(i * NUM_SPLINE_COEFFS_PER_AXIS) + j]) * basis.alongV[j];
        }
        value += basis.alongU[i] * rowValue;
    }
    return value;
}

/*! This method returns the surface S(u, v) = sum_i sum_j c_ij B_i(u) B_j(v) in float64, where i is along u and j is
 along v.
 @return double [steps]
 @param coefficients [steps] Spline control coefficients
 @param u [-] First argument of the surface
 @param v [-] Second argument of the surface
*/
inline double referenceSurface(const SplineCoefficients& coefficients, const double u, const double v) {
    return referenceSurface(coefficients, referenceBasisValues(u, v));
}

/*! This method maps gimbal angles to normalized rhombus coordinates in float64, clamped to [-1, 1].
 @return std::array<double, 2> [-] u and v
 @param domain [rad] Addressable gimbal rhombus
 @param tip [rad] Gimbal tip angle
 @param tilt [rad] Gimbal tilt angle
*/
inline std::array<double, 2> referenceUv(const GimbalRhombusDomain& domain, const float tip, const float tilt) {
    const double x = (static_cast<double>(tip) - static_cast<double>(domain.centerTipAngle)) /
                     static_cast<double>(domain.halfDiagonalTip);
    const double y = (static_cast<double>(tilt) - static_cast<double>(domain.centerTiltAngle)) /
                     static_cast<double>(domain.halfDiagonalTilt);
    return {std::clamp(x + y, -1.0, 1.0), std::clamp(x - y, -1.0, 1.0)};
}

/*! This method returns the sum of the squared errors between the surfaces and the measured table, over both motors.
 This is the quantity the offline least-squares fit minimizes. The surfaces are evaluated with the float64 reference,
 not with the algorithm, because the fp32 evaluation errors of the algorithm change the sum by more than a small
 coefficient change does. PropertyMotorStepsAreSurfacesAtUv checks that the algorithm matches the reference.
 The basis values at the table points do not depend on the coefficients, so the caller computes them once.
 @return double [steps^2]
 @param motor1Coefficients [steps] Motor 1 spline control coefficients
 @param motor2Coefficients [steps] Motor 2 spline control coefficients
 @param samples Measured table entries inside the rhombus
 @param basis [-] Basis values at the table entries, in the same order
*/
inline double tableSumSquaredError(const SplineCoefficients& motor1Coefficients,
                                   const SplineCoefficients& motor2Coefficients,
                                   const std::vector<GimbalMotorSample>& samples,
                                   const std::vector<ReferenceBasisValues>& basis) {
    double sumSquaredError = 0.0;
    for (std::size_t k = 0; k < samples.size(); ++k) {
        const double error1 = referenceSurface(motor1Coefficients, basis[k]) - static_cast<double>(samples[k].steps1);
        const double error2 = referenceSurface(motor2Coefficients, basis[k]) - static_cast<double>(samples[k].steps2);
        sumSquaredError += (error1 * error1) + (error2 * error2);
    }
    return sumSquaredError;
}

// [steps] Smallest coefficient perturbation used by the least-squares property tests. The design matrix of the fit
// has full rank, so any non-zero perturbation raises the table error, by the squared norm of the perturbation surface
// at the table points. Reading the coefficients and the table points as float moves the minimum slightly, which adds
// a term that is linear in the perturbation. The corner coefficients touch only a few table points with small weights,
// but a 10 step change of one of them still raises the error by much more than the linear term. A change of 0.1 step
// can fall below the linear term.
constexpr float kMinCoefficientPerturbation = 10.0F;
// [steps] Largest coefficient perturbation; the fitted coefficients are much larger, so this stays a slight change
constexpr float kMaxCoefficientPerturbation = 50.0F;

// ---------------------------------------------------------------------------
// Properties shared by the unit tests and the fuzz tests
// ---------------------------------------------------------------------------

/*! This method returns the configuration built from the fitted fixtures. The files are read once.
 @return const GimbalToMotorAnglesSplineConfig&
*/
inline const GimbalToMotorAnglesSplineConfig& fittedConfig() {
    static const GimbalToMotorAnglesSplineConfig config = makeFittedConfig();
    return config;
}

/*! This method returns the measured table entries inside the rhombus. The file is read once.
 @return const std::vector<GimbalMotorSample>&
*/
inline const std::vector<GimbalMotorSample>& tableSamples() {
    static const std::vector<GimbalMotorSample> samples = readSamples("tablePointsInDomain.csv");
    return samples;
}

/*! This method returns the basis values at the measured table entries, in the fitted rhombus. They are computed once.
 @return const std::vector<ReferenceBasisValues>&
*/
inline const std::vector<ReferenceBasisValues>& tableBasisValues() {
    static const std::vector<ReferenceBasisValues> basis = [] {
        std::vector<ReferenceBasisValues> values;
        values.reserve(tableSamples().size());
        for (const auto& sample : tableSamples()) {
            const auto [u, v] = referenceUv(fittedConfig().getDomain(), sample.tip, sample.tilt);
            values.push_back(referenceBasisValues(u, v));
        }
        return values;
    }();
    return basis;
}

/*! This method returns the table error of coefficients in the fitted rhombus.
 @return double [steps^2]
 @param motor1Coefficients [steps] Motor 1 spline control coefficients
 @param motor2Coefficients [steps] Motor 2 spline control coefficients
*/
inline double tableError(const SplineCoefficients& motor1Coefficients, const SplineCoefficients& motor2Coefficients) {
    return tableSumSquaredError(motor1Coefficients, motor2Coefficients, tableSamples(), tableBasisValues());
}

/*! This method returns the table error of the fitted coefficients. It is computed once.
 @return double [steps^2]
*/
inline double fittedTableError() {
    static const double error =
        tableError(fittedConfig().getMotor1Coefficients(), fittedConfig().getMotor2Coefficients());
    return error;
}

/*! This method checks that the motor steps of an output lie in the convex hull of the coefficients of their own
 surface, with the fp32 evaluation slack and the angle conversion error.
 @param output [rad] Algorithm output
 @param stepAngle [rad/step] Step angle of the configuration
 @param motor1Coefficients [steps] Motor 1 spline control coefficients
 @param motor2Coefficients [steps] Motor 2 spline control coefficients
*/
inline void expectWithinCoefficientBounds(const GimbalToMotorAnglesSplineOutput& output,
                                          const float stepAngle,
                                          const SplineCoefficients& motor1Coefficients,
                                          const SplineCoefficients& motor2Coefficients) {
    const MotorSteps steps = motorSteps(output, stepAngle);
    for (const auto& [motorStep, coefficients] :
         {std::pair{steps.motor1, &motor1Coefficients}, std::pair{steps.motor2, &motor2Coefficients}}) {
        const auto [minCoeff, maxCoeff] = coefficientBounds(*coefficients);
        const float slack = (kSurfaceValueRelativeError + kAngleConversionRelativeError) *
                            std::max(std::fabs(minCoeff), std::fabs(maxCoeff));
        EXPECT_GE(motorStep, minCoeff - slack);
        EXPECT_LE(motorStep, maxCoeff + slack);
    }
}

/*! Any float input, including NaN, infinities, and huge values, gives a finite output: either the initial output of
 the algorithm or motor steps inside the coefficient convex hull of each motor.
 @param gimbalAngle1 [rad] Gimbal tip angle
 @param gimbalAngle2 [rad] Gimbal tilt angle
*/
inline void propertyOutputIsFiniteForAnyInput(const float gimbalAngle1, const float gimbalAngle2) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    GimbalToMotorAnglesSplineAlgorithm fresh(fittedConfig());
    const auto initial = fresh.update(nan, nan);
    GimbalToMotorAnglesSplineAlgorithm alg(fittedConfig());
    const auto output = alg.update(gimbalAngle1, gimbalAngle2);

    EXPECT_TRUE(std::isfinite(output.motorAngle1)) << "tip " << gimbalAngle1 << " tilt " << gimbalAngle2;
    EXPECT_TRUE(std::isfinite(output.motorAngle2)) << "tip " << gimbalAngle1 << " tilt " << gimbalAngle2;
    // Checks if gimbalAngle1 and gimbalAngle2 fall outside the rhombus or are invalid
    const bool isInitial = output.motorAngle1 == initial.motorAngle1 && output.motorAngle2 == initial.motorAngle2;
    if (!isInitial) {
        expectWithinCoefficientBounds(output,
                                      fittedConfig().getStepAngle(),
                                      fittedConfig().getMotor1Coefficients(),
                                      fittedConfig().getMotor2Coefficients());
    }
}

/*! A request outside the rhombus returns the previous valid output. An outside point that is not past the rhombus by
 more than 0.1% is skipped, because the fp32 mapping can move it onto the edge.
 @param validU [-] u of the point in the rhombus
 @param validV [-] v of the point in the rhombus
 @param outsideU [-] u of the point outside the rhombus
 @param outsideV [-] v of the point outside the rhombus
*/
inline void propertyOutsideDomainHoldsPreviousOutput(const float validU,
                                                     const float validV,
                                                     const float outsideU,
                                                     const float outsideV) {
    if (std::fabs(outsideU) <= 1.001F && std::fabs(outsideV) <= 1.001F) {
        return;
    }
    const GimbalRhombusDomain& domain = fittedConfig().getDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(fittedConfig());
    const auto valid = gimbalAnglesFromUv(domain, validU, validV);
    const auto validOutput = alg.update(valid[0], valid[1]);
    const auto outside = gimbalAnglesFromUv(domain, outsideU, outsideV);
    const auto heldOutput = alg.update(outside[0], outside[1]);

    EXPECT_EQ(heldOutput.motorAngle1, validOutput.motorAngle1) << "u " << outsideU << " v " << outsideV;
    EXPECT_EQ(heldOutput.motorAngle2, validOutput.motorAngle2) << "u " << outsideU << " v " << outsideV;
}

/*! Nearby requests inside the rhombus give nearby outputs. For each motor, the change of the steps is bounded by the
 slope bound of its surface times the distance, plus the step tolerance of both points.
 @param u [-] u of the first point
 @param v [-] v of the first point
 @param du [-] Offset in u of the second point
 @param dv [-] Offset in v of the second point
*/
inline void propertySurfaceIsLipschitz(const float u, const float v, const float du, const float dv) {
    const float u2 = std::clamp(u + du, -1.0F, 1.0F);
    const float v2 = std::clamp(v + dv, -1.0F, 1.0F);
    const GimbalRhombusDomain& domain = fittedConfig().getDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(fittedConfig());
    const auto first = gimbalAnglesFromUv(domain, u, v);
    const auto second = gimbalAnglesFromUv(domain, u2, v2);
    const MotorSteps firstSteps = motorSteps(alg.update(first[0], first[1]), fittedConfig().getStepAngle());
    const MotorSteps secondSteps = motorSteps(alg.update(second[0], second[1]), fittedConfig().getStepAngle());

    // The mapped offsets equal the requested ones to within ~10 eps; 10% covers that
    const auto tolerance = [&](const SplineCoefficients& coefficients) {
        return (surfaceSlopeBound(coefficients) * 1.1F * (std::fabs(u2 - u) + std::fabs(v2 - v))) +
               (2.0F * stepTolerance(coefficients));
    };
    EXPECT_NEAR(firstSteps.motor1, secondSteps.motor1, tolerance(fittedConfig().getMotor1Coefficients()))
        << "u " << u << " v " << v;
    EXPECT_NEAR(firstSteps.motor2, secondSteps.motor2, tolerance(fittedConfig().getMotor2Coefficients()))
        << "u " << u << " v " << v;
}

/*! Any valid configuration gives outputs inside the coefficient convex hull of each motor for requests inside its
 rhombus.
 @param centerTip [rad] Gimbal tip angle at the rhombus center
 @param centerTilt [rad] Gimbal tilt angle at the rhombus center
 @param halfTip [rad] Half of the rhombus diagonal along the tip axis
 @param halfTilt [rad] Half of the rhombus diagonal along the tilt axis
 @param motor1CoefficientValues [steps] NUM_SPLINE_COEFFS motor 1 spline control coefficients
 @param motor2CoefficientValues [steps] NUM_SPLINE_COEFFS motor 2 spline control coefficients
 @param u [-] u of the request
 @param v [-] v of the request
*/
inline void propertyOutputWithinCoefficientBounds(const float centerTip,
                                                  const float centerTilt,
                                                  const float halfTip,
                                                  const float halfTilt,
                                                  const std::vector<float>& motor1CoefficientValues,
                                                  const std::vector<float>& motor2CoefficientValues,
                                                  const float u,
                                                  const float v) {
    SplineCoefficients motor1Coefficients{};
    SplineCoefficients motor2Coefficients{};
    std::ranges::copy(motor1CoefficientValues, motor1Coefficients.begin());
    std::ranges::copy(motor2CoefficientValues, motor2Coefficients.begin());
    const GimbalRhombusDomain domain{.centerTipAngle = centerTip,
                                     .centerTiltAngle = centerTilt,
                                     .halfDiagonalTip = halfTip,
                                     .halfDiagonalTilt = halfTilt};
    GimbalToMotorAnglesSplineAlgorithm alg(GimbalToMotorAnglesSplineConfig::create(kTestStepAngle,
                                                                                   kTestDefaultMotor1Angle,
                                                                                   kTestDefaultMotor2Angle,
                                                                                   domain,
                                                                                   motor1Coefficients,
                                                                                   motor2Coefficients));
    const auto angles = gimbalAnglesFromUv(domain, u, v);
    const auto output = alg.update(angles[0], angles[1]);

    expectWithinCoefficientBounds(output, kTestStepAngle, motor1Coefficients, motor2Coefficients);
}

/*! The fitted coefficients minimize the sum of the squared errors to the measured table, so changing any set of
 coefficients of either motor gives a larger error. A zero entry leaves that coefficient unchanged.
 @param motor1Perturbation [steps] NUM_SPLINE_COEFFS changes of the fitted motor 1 coefficients
 @param motor2Perturbation [steps] NUM_SPLINE_COEFFS changes of the fitted motor 2 coefficients
*/
inline void propertyPerturbedCoefficientsIncreaseTableError(const std::vector<float>& motor1Perturbation,
                                                            const std::vector<float>& motor2Perturbation) {
    const auto isZero = [](float value) { return value == 0.0F; };
    if (std::ranges::all_of(motor1Perturbation, isZero) && std::ranges::all_of(motor2Perturbation, isZero)) {
        return;
    }
    SplineCoefficients motor1Perturbed = fittedConfig().getMotor1Coefficients();
    SplineCoefficients motor2Perturbed = fittedConfig().getMotor2Coefficients();
    for (int k = 0; k < NUM_SPLINE_COEFFS; ++k) {
        motor1Perturbed[k] += motor1Perturbation[k];
        motor2Perturbed[k] += motor2Perturbation[k];
    }

    EXPECT_GT(tableError(motor1Perturbed, motor2Perturbed), fittedTableError());
}

#endif  // TEST_GIMBALTOMOTORANGLESSPLINE_H
