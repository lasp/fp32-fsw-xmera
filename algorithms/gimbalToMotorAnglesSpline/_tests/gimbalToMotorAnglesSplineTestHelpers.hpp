#ifndef TEST_GIMBALTOMOTORANGLESSPLINE_H
#define TEST_GIMBALTOMOTORANGLESSPLINE_H

#include "gimbalToMotorAnglesSplineAlgorithm.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <utility>

constexpr float degToRad = std::numbers::pi_v<float> / 180.0F;
constexpr uint32_t kTestStepsPerRevolution = 628U;

/*! @brief Motor steps of an algorithm output. */
struct MotorSteps {
    float motor1;  //!< [steps] Motor 1 steps
    float motor2;  //!< [steps] Motor 2 steps
};

/*! This method converts an algorithm output back to motor steps. The algorithm multiplies the steps by the step angle
 2*pi / stepsPerRevolution and this method divides by the same fp32 step angle, so the steps differ from the surface
 value by kAngleConversionRelativeError at most.
 @return MotorSteps [steps]
 @param output [rad] Algorithm output
 @param stepsPerRevolution [steps] Motor steps per revolution of the configuration
*/
inline MotorSteps motorSteps(const GimbalToMotorAnglesSplineOutput& output,
                             const uint32_t stepsPerRevolution = kTestStepsPerRevolution) {
    const float stepAngle = 2.0F * std::numbers::pi_v<float> / static_cast<float>(stepsPerRevolution);  // [rad/step]
    return {.motor1 = output.motorAngle1 / stepAngle, .motor2 = output.motorAngle2 / stepAngle};
}

/*! This method maps normalized rhombus coordinates to gimbal angles.
 @return std::array<float, 2> [rad] Gimbal tip and tilt angles
 @param domain [rad] Addressable gimbal rhombus
 @param u [-] First normalized rhombus coordinate
 @param v [-] Second normalized rhombus coordinate
*/
inline std::array<float, 2> gimbalAnglesFromUv(const GimbalRhombusDomain& domain, const float u, const float v) {
    return {domain.halfDiagonalTip * (u + v) / 2.0F, domain.halfDiagonalTilt * (u - v) / 2.0F};
}

/*! This method returns the gimbal angles at a radius and a direction about zero.
 @return std::array<float, 2> [rad] Gimbal tip and tilt angles
 @param radius [rad] Distance from zero gimbal angles
 @param direction [rad] Angle from the tip axis toward the tilt axis
*/
inline std::array<float, 2> gimbalAnglesOnCircle(const double radius, const double direction) {
    return {static_cast<float>(radius * std::cos(direction)), static_cast<float>(radius * std::sin(direction))};
}

/*! This method returns gimbal angles in the circle of radius thetaMax about zero, at a fraction of thetaMax from zero
 and in a direction. The module assumes that every request is in this circle. The test rhombi have thetaMax well
 inside the radius of their largest circle, much more than the fp32 rounding of u and v, so the algorithm always
 accepts these angles.
 @return std::array<float, 2> [rad] Gimbal tip and tilt angles
 @param domain [rad] Addressable gimbal rhombus with the radius thetaMax of its circle
 @param radiusFraction [-] Distance from zero as a fraction of thetaMax, in [0, 1]
 @param direction [rad] Angle from the tip axis toward the tilt axis
*/
inline std::array<float, 2> gimbalAnglesInCircle(const GimbalRhombusDomain& domain,
                                                 const float radiusFraction,
                                                 const float direction) {
    return gimbalAnglesOnCircle(static_cast<double>(radiusFraction) * static_cast<double>(domain.thetaMax),
                                static_cast<double>(direction));
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
            coefficients.at((i * NUM_SPLINE_COEFFS_PER_AXIS) + j) = kLinearSurfaceScale * grevilleAbscissa(index);
        }
    }
    return coefficients;
}

/*! This method returns the rhombus of the linear test configuration. The radius of its largest circle is about
 0.2496 rad, so thetaMax = 0.24 rad is inside it by about 0.01 rad.
 @return GimbalRhombusDomain [rad]
*/
inline GimbalRhombusDomain linearTestDomain() {
    return {.halfDiagonalTip = 0.3F, .halfDiagonalTilt = 0.45F, .thetaMax = 0.24F};
}

/*! This method returns a configuration with known outputs: motor 1 = kLinearSurfaceScale * v steps and motor 2 =
 kLinearSurfaceScale * u steps inside the rhombus. The two motors have different surfaces, so a swap of the motors or
 of u and v is caught.
 @return GimbalToMotorAnglesSplineConfig
*/
inline GimbalToMotorAnglesSplineConfig makeLinearConfig() {
    return GimbalToMotorAnglesSplineConfig::create(kTestStepsPerRevolution,
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
// [steps] Tolerance of the linear surfaces evaluated directly at exact u and v: the value error only, multiplied by 3
// for margin, as in surfaceValueTolerance
constexpr float kLinearSurfaceEvaluationTolerance =
    3.0F * (1.0F + kIntervalWidth) * kLinearSurfaceScale * kSurfaceValueRelativeError;

// [steps] Largest magnitude of the rough test coefficients, half of the largest magnitude that the configuration
// accepts
constexpr float kRoughCoefficientMagnitude = 50000.0F;

/*! This method returns a value in [-1, 1) from an integer, by an integer hash. The value is a multiple of 2^-23, thus
 it is exact in fp32, and the same on every platform. std::uniform_real_distribution is not used, because its output
 depends on the standard library.
 @return float [-]
 @param key [-] Integer to hash
*/
inline float hashToUnit(uint32_t key) {
    key ^= key >> 16U;
    key *= 0x7feb352dU;
    key ^= key >> 15U;
    key *= 0x846ca68bU;
    key ^= key >> 16U;
    constexpr int32_t half = 1 << 23;
    return static_cast<float>(static_cast<int32_t>(key >> 8U) - half) / static_cast<float>(half);
}

/*! This method returns made-up coefficients that change irregularly from one coefficient to the next, in
 [-kRoughCoefficientMagnitude, kRoughCoefficientMagnitude). A linear or constant surface is the same polynomial on
 every interval, thus it cannot show an error where one interval meets the next. These coefficients make a different
 cubic on every interval.
 @return SplineCoefficients [steps]
 @param seed [-] Seed that selects the coefficient set
*/
inline SplineCoefficients roughSurfaceCoefficients(const uint32_t seed) {
    SplineCoefficients coefficients{};
    for (std::size_t k = 0; k < coefficients.size(); ++k) {
        coefficients.at(k) = kRoughCoefficientMagnitude *
                             hashToUnit((seed * static_cast<uint32_t>(NUM_SPLINE_COEFFS)) + static_cast<uint32_t>(k));
    }
    return coefficients;
}

/*! This method returns the rhombus of the rough test configuration. It is wider along tip than along tilt, while the
 linear test rhombus is wider along tilt, so a swap of the half-diagonals is caught. The radius of its largest circle
 is about 0.1536 rad, so thetaMax = 0.15 rad is inside it by about 0.0036 rad.
 @return GimbalRhombusDomain [rad]
*/
inline GimbalRhombusDomain roughTestDomain() {
    return {.halfDiagonalTip = 0.24F, .halfDiagonalTilt = 0.2F, .thetaMax = 0.15F};
}

/*! This method returns the rough test configuration: made-up coefficients with a different seed for each motor, in
 the rough test rhombus. The tests use it where the linear configuration is too smooth, and in place of a fitted
 configuration. The Python tests check the fitted configuration.
 @return GimbalToMotorAnglesSplineConfig
*/
inline GimbalToMotorAnglesSplineConfig makeRoughConfig() {
    return GimbalToMotorAnglesSplineConfig::create(
        kTestStepsPerRevolution, roughTestDomain(), roughSurfaceCoefficients(1U), roughSurfaceCoefficients(2U));
}

/*! This method returns an upper bound on |dS/du| and |dS/dv|. The derivative of a cubic B-spline is a quadratic
 B-spline whose coefficients are the coefficient differences divided by the knot spacing.
 @return float [steps] Slope bound per unit of u or v
 @param coefficients [steps] Spline control coefficients
*/
inline float surfaceSlopeBound(const SplineCoefficients& coefficients) {
    float maxDifference = 0.0F;
    for (int i = 0; i < NUM_SPLINE_COEFFS_PER_AXIS; ++i) {
        for (int j = 0; j < NUM_SPLINE_COEFFS_PER_AXIS; ++j) {
            const float c = coefficients.at((i * NUM_SPLINE_COEFFS_PER_AXIS) + j);
            if (i + 1 < NUM_SPLINE_COEFFS_PER_AXIS) {
                maxDifference =
                    std::max(maxDifference, std::fabs(coefficients.at(((i + 1) * NUM_SPLINE_COEFFS_PER_AXIS) + j) - c));
            }
            if (j + 1 < NUM_SPLINE_COEFFS_PER_AXIS) {
                maxDifference =
                    std::max(maxDifference, std::fabs(coefficients.at((i * NUM_SPLINE_COEFFS_PER_AXIS) + j + 1) - c));
            }
        }
    }
    return maxDifference / kIntervalWidth;
}

/*! This method returns the fp32 evaluation tolerance of a surface value at exact u and v against a float64 reference
 evaluation of the same surface, for evaluateSurface called directly.
 - Coefficient rounding to float: <= eps/2 * max|c| (the weights are non-negative and sum to one).
 - Weights (~10 operations) and the 4 x 4 weighted sum (~8 operations deep): ~20 eps * max|c|.
 The sum is multiplied by 3 for margin.
 @return float [steps]
 @param coefficients [steps] Spline control coefficients
*/
inline float surfaceValueTolerance(const SplineCoefficients& coefficients) {
    const auto [minCoeff, maxCoeff] = coefficientBounds(coefficients);
    return 3.0F * kSurfaceValueRelativeError * std::max(std::fabs(minCoeff), std::fabs(maxCoeff));
}

/*! This method returns the fp32 evaluation tolerance of a surface value against a float64 reference evaluation of the
 same surface, when the algorithm computes u and v from gimbal angles.
 - Coefficient rounding to float: <= eps/2 * max|c| (the weights are non-negative and sum to one).
 - Weights (~10 operations) and the 4 x 4 weighted sum (~8 operations deep): ~20 eps * max|c|.
 - u and v: ~5 operations on once-rounded inputs, ~9 eps each, times the slope bound, for both u and v.
 The sum is multiplied by 3 for margin.
 @return float [steps]
 @param coefficients [steps] Spline control coefficients
*/
inline float evaluationTolerance(const SplineCoefficients& coefficients) {
    const float coordinateError = 2.0F * 9.0F * kEps * surfaceSlopeBound(coefficients);
    return surfaceValueTolerance(coefficients) + (3.0F * coordinateError);
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

/*! This method returns the surface S(u, v) = sum_i sum_j c_ij B_i(u) B_j(v) in float64, where i is along u and j is
 along v.
 @return double [steps]
 @param coefficients [steps] Spline control coefficients
 @param u [-] First argument of the surface
 @param v [-] Second argument of the surface
*/
inline double referenceSurface(const SplineCoefficients& coefficients, const double u, const double v) {
    std::array<double, NUM_SPLINE_COEFFS_PER_AXIS> alongV{};  // [-] B_j(v) for every coefficient index j
    for (int j = 0; j < NUM_SPLINE_COEFFS_PER_AXIS; ++j) {
        alongV.at(j) = referenceBasis(j, 3, v);
    }
    double value = 0.0;
    for (int i = 0; i < NUM_SPLINE_COEFFS_PER_AXIS; ++i) {
        double rowValue = 0.0;
        for (int j = 0; j < NUM_SPLINE_COEFFS_PER_AXIS; ++j) {
            rowValue += static_cast<double>(coefficients.at((i * NUM_SPLINE_COEFFS_PER_AXIS) + j)) * alongV.at(j);
        }
        value += referenceBasis(i, 3, u) * rowValue;
    }
    return value;
}

/*! This method maps gimbal angles to normalized rhombus coordinates in float64, clamped to [-1, 1].
 @return std::array<double, 2> [-] u and v
 @param domain [rad] Addressable gimbal rhombus
 @param tip [rad] Gimbal tip angle
 @param tilt [rad] Gimbal tilt angle
*/
inline std::array<double, 2> referenceUv(const GimbalRhombusDomain& domain, const float tip, const float tilt) {
    const double x = static_cast<double>(tip) / static_cast<double>(domain.halfDiagonalTip);
    const double y = static_cast<double>(tilt) / static_cast<double>(domain.halfDiagonalTilt);
    return {std::clamp(x + y, -1.0, 1.0), std::clamp(x - y, -1.0, 1.0)};
}

// ---------------------------------------------------------------------------
// Properties shared by the unit tests and the fuzz tests
// ---------------------------------------------------------------------------

/*! This method returns the rough test configuration. It is made once.
 @return const GimbalToMotorAnglesSplineConfig&
*/
inline const GimbalToMotorAnglesSplineConfig& roughConfig() {
    static const GimbalToMotorAnglesSplineConfig config = makeRoughConfig();
    return config;
}

/*! This method checks that a surface value lies in the convex hull of the coefficients of its surface, with the given
 slack. The errors of u and v do not enter the slack, because every point of [-1, 1] x [-1, 1] lies inside the hull.
 @param steps [steps] Surface value
 @param coefficients [steps] Spline control coefficients of the surface
 @param slack [steps] Allowed distance outside the hull
*/
inline void expectWithinCoefficientBounds(const float steps,
                                          const SplineCoefficients& coefficients,
                                          const float slack) {
    const auto [minCoeff, maxCoeff] = coefficientBounds(coefficients);
    EXPECT_GE(steps, minCoeff - slack);
    EXPECT_LE(steps, maxCoeff + slack);
}

/*! This method checks that the motor steps of an output lie in the convex hull of the coefficients of their own
 surface, with the fp32 surface value tolerance and the angle conversion error.
 @param output [rad] Algorithm output
 @param stepsPerRevolution [steps] Motor steps per revolution of the configuration
 @param motor1Coefficients [steps] Motor 1 spline control coefficients
 @param motor2Coefficients [steps] Motor 2 spline control coefficients
*/
inline void expectWithinCoefficientBounds(const GimbalToMotorAnglesSplineOutput& output,
                                          const uint32_t stepsPerRevolution,
                                          const SplineCoefficients& motor1Coefficients,
                                          const SplineCoefficients& motor2Coefficients) {
    const MotorSteps steps = motorSteps(output, stepsPerRevolution);
    for (const auto& [motorStep, coefficients] :
         {std::pair{steps.motor1, &motor1Coefficients}, std::pair{steps.motor2, &motor2Coefficients}}) {
        const auto [minCoeff, maxCoeff] = coefficientBounds(*coefficients);
        const float slack = surfaceValueTolerance(*coefficients) +
                            (kAngleConversionRelativeError * std::max(std::fabs(minCoeff), std::fabs(maxCoeff)));
        expectWithinCoefficientBounds(motorStep, *coefficients, slack);
    }
}

/*! Any float input, including NaN, infinities, and huge values, gives a finite output: either the initial output of
 the algorithm or motor steps inside the coefficient convex hull of each motor.
 @param gimbalAngle1 [rad] Gimbal tip angle
 @param gimbalAngle2 [rad] Gimbal tilt angle
*/
inline void propertyOutputIsFiniteForAnyInput(const float gimbalAngle1, const float gimbalAngle2) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    GimbalToMotorAnglesSplineAlgorithm fresh(roughConfig());
    const auto initial = fresh.update(nan, nan);
    GimbalToMotorAnglesSplineAlgorithm alg(roughConfig());
    const auto output = alg.update(gimbalAngle1, gimbalAngle2);

    EXPECT_TRUE(std::isfinite(output.motorAngle1)) << "tip " << gimbalAngle1 << " tilt " << gimbalAngle2;
    EXPECT_TRUE(std::isfinite(output.motorAngle2)) << "tip " << gimbalAngle1 << " tilt " << gimbalAngle2;
    // Checks if gimbalAngle1 and gimbalAngle2 fall outside the rhombus or are invalid
    const bool isInitial = output.motorAngle1 == initial.motorAngle1 && output.motorAngle2 == initial.motorAngle2;
    if (!isInitial) {
        expectWithinCoefficientBounds(output,
                                      roughConfig().getStepsPerRevolution(),
                                      roughConfig().getMotor1Coefficients(),
                                      roughConfig().getMotor2Coefficients());
    }
}

/*! A request outside the rhombus returns the previous valid output. An outside point that is not past the rhombus by
 more than 0.1% is skipped, because the fp32 mapping can move it onto the edge.
 @param validRadiusFraction [-] Distance of the valid point from zero as a fraction of thetaMax, in [0, 1]
 @param validDirection [rad] Direction of the valid point
 @param outsideU [-] u of the point outside the rhombus
 @param outsideV [-] v of the point outside the rhombus
*/
inline void propertyOutsideDomainHoldsPreviousOutput(const float validRadiusFraction,
                                                     const float validDirection,
                                                     const float outsideU,
                                                     const float outsideV) {
    if (std::fabs(outsideU) <= 1.001F && std::fabs(outsideV) <= 1.001F) {
        return;
    }
    const GimbalRhombusDomain& domain = roughConfig().getDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(roughConfig());
    const auto valid = gimbalAnglesInCircle(domain, validRadiusFraction, validDirection);
    const auto validOutput = alg.update(valid.at(0), valid.at(1));
    const auto outside = gimbalAnglesFromUv(domain, outsideU, outsideV);
    const auto heldOutput = alg.update(outside.at(0), outside.at(1));

    EXPECT_EQ(heldOutput.motorAngle1, validOutput.motorAngle1) << "u " << outsideU << " v " << outsideV;
    EXPECT_EQ(heldOutput.motorAngle2, validOutput.motorAngle2) << "u " << outsideU << " v " << outsideV;
}

/*! With the coefficients set to the scaled Greville abscissae, the surfaces are exactly kLinearSurfaceScale * u and
 kLinearSurfaceScale * v, to within the fp32 evaluation error, at any point of [-1, 1] x [-1, 1], edges included. This
 pins the basis weights, the interval search, and the coefficient index order (i along u, j along v).
 @param u [-] u of the point, in [-1, 1]
 @param v [-] v of the point, in [-1, 1]
*/
inline void propertyReproducesLinearSurface(const float u, const float v) {
    static const SplineCoefficients alongU = linearSurfaceCoefficients(SurfaceAxis::U);
    static const SplineCoefficients alongV = linearSurfaceCoefficients(SurfaceAxis::V);
    EXPECT_NEAR(gimbal_to_motor_angles_spline_detail::evaluateSurface(alongU, u, v),
                kLinearSurfaceScale * u,
                kLinearSurfaceEvaluationTolerance)
        << "u " << u << " v " << v;
    EXPECT_NEAR(gimbal_to_motor_angles_spline_detail::evaluateSurface(alongV, u, v),
                kLinearSurfaceScale * v,
                kLinearSurfaceEvaluationTolerance)
        << "u " << u << " v " << v;
}

/*! Nearby points give nearby values of the rough test surfaces. For each motor, the values at (u, v) and at the second
 point differ by not more than the slope bound of its surface times |du| + |dv|, plus the value tolerance of both
 points. The surfaces are evaluated directly, at any point of [-1, 1] x [-1, 1], and the second point is kept in this
 square.
 @param u [-] u of the first point, in [-1, 1]
 @param v [-] v of the first point, in [-1, 1]
 @param du [-] Offset in u of the second point
 @param dv [-] Offset in v of the second point
*/
inline void propertySurfaceIsLipschitz(const float u, const float v, const float du, const float dv) {
    const float u2 = std::clamp(u + du, -1.0F, 1.0F);
    const float v2 = std::clamp(v + dv, -1.0F, 1.0F);
    for (const SplineCoefficients* coefficients :
         {&roughConfig().getMotor1Coefficients(), &roughConfig().getMotor2Coefficients()}) {
        const float first = gimbal_to_motor_angles_spline_detail::evaluateSurface(*coefficients, u, v);
        const float second = gimbal_to_motor_angles_spline_detail::evaluateSurface(*coefficients, u2, v2);
        const float tolerance = (surfaceSlopeBound(*coefficients) * (std::fabs(u2 - u) + std::fabs(v2 - v))) +
                                (2.0F * surfaceValueTolerance(*coefficients));
        EXPECT_NEAR(first, second, tolerance) << "u " << u << " v " << v << " u2 " << u2 << " v2 " << v2;
    }
}

/*! Any coefficients give a surface value between their smallest and largest coefficient (B-spline convex-hull
 property), with the fp32 evaluation error, at any point of [-1, 1] x [-1, 1].
 @param coefficients [steps] Spline control coefficients
 @param u [-] u of the point, in [-1, 1]
 @param v [-] v of the point, in [-1, 1]
*/
inline void propertySurfaceWithinCoefficientBounds(const SplineCoefficients& coefficients,
                                                   const float u,
                                                   const float v) {
    expectWithinCoefficientBounds(gimbal_to_motor_angles_spline_detail::evaluateSurface(coefficients, u, v),
                                  coefficients,
                                  surfaceValueTolerance(coefficients));
}

#endif  // TEST_GIMBALTOMOTORANGLESSPLINE_H
