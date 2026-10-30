#include "../gimbalToMotorAnglesSplineAlgorithm.h"
#include "gimbalToMotorAnglesSplineTestHelpers.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// Regression tests against the offline fit
// ---------------------------------------------------------------------------

// The fp32 motor steps match the scipy (float64) evaluation of the same coefficients
TEST(GimbalToMotorAnglesSplineTest, MatchesScipyReferenceEvaluation) {
    const float tolerance1 = stepTolerance(readMotor1Coefficients());
    const float tolerance2 = stepTolerance(readMotor2Coefficients());
    const auto samples = readSamples("splineReference.csv");

    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    for (const auto& sample : samples) {
        const auto result = alg.update(sample.tip, sample.tilt);
        const MotorSteps steps = motorSteps(result);
        EXPECT_NEAR(steps.motor1, sample.steps1, tolerance1) << "tip " << sample.tip << " tilt " << sample.tilt;
        EXPECT_NEAR(steps.motor2, sample.steps2, tolerance2) << "tip " << sample.tip << " tilt " << sample.tilt;
    }
}

// The motor steps match every measured table entry inside the addressable rhombus within the fit error. The fit error
// is the largest float64 surface error at the table entries, the value that the fit tool reports, and the tolerance
// adds the step tolerance of the two motors.
TEST(GimbalToMotorAnglesSplineTest, MatchesMeasuredTableInsideDomain) {
    const SplineCoefficients& motor1Coefficients = fittedConfig().getMotor1Coefficients();
    const SplineCoefficients& motor2Coefficients = fittedConfig().getMotor2Coefficients();
    const auto& samples = tableSamples();
    double maxFitError = 0.0;  // [steps]
    for (std::size_t k = 0; k < samples.size(); ++k) {
        maxFitError =
            std::max({maxFitError,
                      std::fabs(referenceSurface(motor1Coefficients, tableBasisValues()[k]) - samples[k].steps1),
                      std::fabs(referenceSurface(motor2Coefficients, tableBasisValues()[k]) - samples[k].steps2)});
    }
    const float fitTolerance = static_cast<float>(maxFitError) +
                               std::max(stepTolerance(motor1Coefficients), stepTolerance(motor2Coefficients));

    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    for (const auto& sample : samples) {
        alg.reInitialize();
        const MotorSteps steps = motorSteps(alg.update(sample.tip, sample.tilt));
        EXPECT_NEAR(steps.motor1, sample.steps1, fitTolerance) << "tip " << sample.tip << " tilt " << sample.tilt;
        EXPECT_NEAR(steps.motor2, sample.steps2, fitTolerance) << "tip " << sample.tip << " tilt " << sample.tilt;
    }
}

// Requests that are not addressable hold the previous valid output. Before the first valid request, that is the
// configured default motor angles.
TEST(GimbalToMotorAnglesSplineTest, FirstCallOutsideDomainReturnsDefault) {
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    const auto result = alg.update(25.0F * degToRad, 0.0F);

    EXPECT_EQ(result.motorAngle1, kTestDefaultMotor1Angle);
    EXPECT_EQ(result.motorAngle2, kTestDefaultMotor2Angle);
}

TEST(GimbalToMotorAnglesSplineTest, OutsideDomainHoldsPreviousValidOutput) {
    const GimbalRhombusDomain domain = linearTestDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(makeLinearConfig());
    const auto inside = gimbalAnglesFromUv(domain, 0.25F, -0.5F);
    const auto valid = alg.update(inside[0], inside[1]);
    ASSERT_NEAR(motorSteps(valid).motor1, kLinearSurfaceScale * -0.5F, kLinearSurfaceTolerance);
    ASSERT_NEAR(motorSteps(valid).motor2, kLinearSurfaceScale * 0.25F, kLinearSurfaceTolerance);

    for (const auto& [u, v] : {std::pair{1.5F, 0.0F}, std::pair{0.0F, -1.5F}, std::pair{3.0F, 3.0F}}) {
        const auto outside = gimbalAnglesFromUv(domain, u, v);
        const auto held = alg.update(outside[0], outside[1]);
        EXPECT_EQ(held.motorAngle1, valid.motorAngle1) << "u " << u << " v " << v;
        EXPECT_EQ(held.motorAngle2, valid.motorAngle2) << "u " << u << " v " << v;
    }
}

TEST(GimbalToMotorAnglesSplineTest, NonFiniteInputHoldsPreviousValidOutput) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    const GimbalRhombusDomain domain = linearTestDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(makeLinearConfig());
    const auto inside = gimbalAnglesFromUv(domain, -0.4F, 0.6F);
    const auto valid = alg.update(inside[0], inside[1]);
    ASSERT_NEAR(motorSteps(valid).motor1, kLinearSurfaceScale * 0.6F, kLinearSurfaceTolerance);
    ASSERT_NEAR(motorSteps(valid).motor2, kLinearSurfaceScale * -0.4F, kLinearSurfaceTolerance);

    for (const auto& [tip, tilt] : {std::pair{nan, 0.0F},
                                    std::pair{0.0F, nan},
                                    std::pair{inf, 0.0F},
                                    std::pair{0.0F, -inf},
                                    std::pair{nan, nan}}) {
        const auto held = alg.update(tip, tilt);
        EXPECT_EQ(held.motorAngle1, valid.motorAngle1);
        EXPECT_EQ(held.motorAngle2, valid.motorAngle2);
    }
}

// Finite inputs can overflow the normalized coordinates to +inf and -inf, whose sum is NaN
TEST(GimbalToMotorAnglesSplineTest, OverflowingFiniteInputHoldsPreviousValidOutput) {
    constexpr float big = std::numeric_limits<float>::max();
    const GimbalRhombusDomain domain = linearTestDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(makeLinearConfig());
    const auto inside = gimbalAnglesFromUv(domain, 0.1F, 0.2F);
    const auto valid = alg.update(inside[0], inside[1]);
    ASSERT_NEAR(motorSteps(valid).motor1, kLinearSurfaceScale * 0.2F, kLinearSurfaceTolerance);
    ASSERT_NEAR(motorSteps(valid).motor2, kLinearSurfaceScale * 0.1F, kLinearSurfaceTolerance);

    const auto held = alg.update(big, -big);
    EXPECT_EQ(held.motorAngle1, valid.motorAngle1);
    EXPECT_EQ(held.motorAngle2, valid.motorAngle2);
}

// The rhombus edge is addressable and a point just past it is not
TEST(GimbalToMotorAnglesSplineTest, RhombusEdgeIsAddressableAndJustOutsideIsNot) {
    const GimbalRhombusDomain domain = linearTestDomain();
    for (const auto& [u, v] :
         {std::pair{1.0F, 0.3F}, std::pair{-1.0F, -0.2F}, std::pair{0.4F, 1.0F}, std::pair{-0.7F, -1.0F}}) {
        GimbalToMotorAnglesSplineAlgorithm alg(makeLinearConfig());
        const auto onEdge = gimbalAnglesFromUv(domain, u, v);
        const auto edgeResult = alg.update(onEdge[0], onEdge[1]);
        EXPECT_NEAR(motorSteps(edgeResult).motor1, kLinearSurfaceScale * v, kLinearSurfaceTolerance)
            << "u " << u << " v " << v;
        EXPECT_NEAR(motorSteps(edgeResult).motor2, kLinearSurfaceScale * u, kLinearSurfaceTolerance)
            << "u " << u << " v " << v;

        const float outside = 1.0F + 1e-3F;
        const auto past =
            gimbalAnglesFromUv(domain, std::fabs(u) == 1.0F ? u * outside : u, std::fabs(v) == 1.0F ? v * outside : v);
        const auto pastResult = alg.update(past[0], past[1]);
        EXPECT_EQ(pastResult.motorAngle1, edgeResult.motorAngle1) << "u " << u << " v " << v;
        EXPECT_EQ(pastResult.motorAngle2, edgeResult.motorAngle2) << "u " << u << " v " << v;
    }
}

// After reInitialize(), a request outside the rhombus gives the same output as the first request of a new algorithm
TEST(GimbalToMotorAnglesSplineTest, ReInitializeRestoresInitialOutput) {
    const GimbalRhombusDomain domain = linearTestDomain();
    const auto inside = gimbalAnglesFromUv(domain, 0.5F, 0.5F);
    const auto outside = gimbalAnglesFromUv(domain, 2.0F, 0.0F);
    GimbalToMotorAnglesSplineAlgorithm fresh(makeLinearConfig());
    const auto initial = fresh.update(outside[0], outside[1]);

    GimbalToMotorAnglesSplineAlgorithm alg(makeLinearConfig());
    const auto valid = alg.update(inside[0], inside[1]);
    ASSERT_NE(valid.motorAngle1, initial.motorAngle1);
    ASSERT_NE(valid.motorAngle2, initial.motorAngle2);
    alg.reInitialize();
    const auto result = alg.update(outside[0], outside[1]);

    EXPECT_EQ(result.motorAngle1, initial.motorAngle1);
    EXPECT_EQ(result.motorAngle2, initial.motorAngle2);
}

// setConfig keeps the previous valid output, and the next reInitialize() uses the new default motor angles
TEST(GimbalToMotorAnglesSplineTest, ReInitializeUsesReconfiguredDefaults) {
    constexpr float newDefault1 = 1.255F;   // [rad]
    constexpr float newDefault2 = -2.755F;  // [rad]
    const GimbalRhombusDomain domain = linearTestDomain();
    const auto outside = gimbalAnglesFromUv(domain, 2.0F, 0.0F);
    GimbalToMotorAnglesSplineAlgorithm alg(makeLinearConfig());
    alg.setConfig(GimbalToMotorAnglesSplineConfig::create(kTestStepAngle,
                                                          newDefault1,
                                                          newDefault2,
                                                          domain,
                                                          linearSurfaceCoefficients(SurfaceAxis::V),
                                                          linearSurfaceCoefficients(SurfaceAxis::U)));

    const auto held = alg.update(outside[0], outside[1]);
    EXPECT_EQ(held.motorAngle1, kTestDefaultMotor1Angle);
    EXPECT_EQ(held.motorAngle2, kTestDefaultMotor2Angle);

    alg.reInitialize();
    const auto result = alg.update(outside[0], outside[1]);
    EXPECT_EQ(result.motorAngle1, newDefault1);
    EXPECT_EQ(result.motorAngle2, newDefault2);
}

// ---------------------------------------------------------------------------
// Reconfiguration and setup tests (validation + round trip)
// ---------------------------------------------------------------------------

TEST(GimbalToMotorAnglesSplineTest, SetConfigReplacesSurfaceAndKeepsPreviousOutput) {
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    const auto fitted = alg.update(3.0F * degToRad, 2.0F * degToRad);

    // A different constant for each motor, so that a swap of the two coefficient sets is caught
    SplineCoefficients motor1Constant{};
    SplineCoefficients motor2Constant{};
    motor1Constant.fill(1250.0F);
    motor2Constant.fill(-2500.0F);
    alg.setConfig(GimbalToMotorAnglesSplineConfig::create(kTestStepAngle,
                                                          kTestDefaultMotor1Angle,
                                                          kTestDefaultMotor2Angle,
                                                          readDomain(),
                                                          motor1Constant,
                                                          motor2Constant));

    // The previous valid output survives the reconfiguration
    const auto held = alg.update(0.0F, 80.0F * degToRad);
    EXPECT_FLOAT_EQ(held.motorAngle1, fitted.motorAngle1);
    EXPECT_FLOAT_EQ(held.motorAngle2, fitted.motorAngle2);

    // The B-spline weights sum to one, so a constant coefficient set gives a constant surface
    const auto result = alg.update(3.0F * degToRad, 2.0F * degToRad);
    EXPECT_NEAR(motorSteps(result).motor1, 1250.0F, stepTolerance(motor1Constant));
    EXPECT_NEAR(motorSteps(result).motor2, -2500.0F, stepTolerance(motor2Constant));
}

TEST(GimbalToMotorAnglesSplineTest, ConfigRoundTrip) {
    const GimbalRhombusDomain domain = readDomain();
    const SplineCoefficients motor1Coefficients = readMotor1Coefficients();
    const SplineCoefficients motor2Coefficients = readMotor2Coefficients();
    const auto config = GimbalToMotorAnglesSplineConfig::create(kTestStepAngle,
                                                                kTestDefaultMotor1Angle,
                                                                kTestDefaultMotor2Angle,
                                                                domain,
                                                                motor1Coefficients,
                                                                motor2Coefficients);

    EXPECT_EQ(config.getStepAngle(), kTestStepAngle);
    EXPECT_EQ(config.getDefaultMotor1Angle(), kTestDefaultMotor1Angle);
    EXPECT_EQ(config.getDefaultMotor2Angle(), kTestDefaultMotor2Angle);
    EXPECT_EQ(config.getDomain().centerTipAngle, domain.centerTipAngle);
    EXPECT_EQ(config.getDomain().centerTiltAngle, domain.centerTiltAngle);
    EXPECT_EQ(config.getDomain().halfDiagonalTip, domain.halfDiagonalTip);
    EXPECT_EQ(config.getDomain().halfDiagonalTilt, domain.halfDiagonalTilt);
    EXPECT_EQ(config.getMotor1Coefficients(), motor1Coefficients);
    EXPECT_EQ(config.getMotor2Coefficients(), motor2Coefficients);
}

TEST(GimbalToMotorAnglesSplineTest, ConfigRejectsInvalidStepAngle) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    constexpr float minStepAngle = GimbalToMotorAnglesSplineConfig::kMinStepAngle;
    constexpr float maxStepAngle = 2.0F * std::numbers::pi_v<float>;
    const GimbalRhombusDomain domain = readDomain();
    const SplineCoefficients motor1Coefficients = readMotor1Coefficients();
    const SplineCoefficients motor2Coefficients = readMotor2Coefficients();

    // The two limits are accepted
    for (const float stepAngle : {minStepAngle, kTestStepAngle, maxStepAngle}) {
        EXPECT_TRUE(GimbalToMotorAnglesSplineConfig::isValidStepAngle(stepAngle)) << "step angle " << stepAngle;
        EXPECT_NO_THROW(GimbalToMotorAnglesSplineConfig::create(stepAngle,
                                                                kTestDefaultMotor1Angle,
                                                                kTestDefaultMotor2Angle,
                                                                domain,
                                                                motor1Coefficients,
                                                                motor2Coefficients));
    }

    // One fp32 step past each limit, zero, negative, and non-finite values are not
    for (const float stepAngle : {std::nextafter(minStepAngle, 0.0F),
                                  std::nextafter(maxStepAngle, inf),
                                  0.0F,
                                  -kTestStepAngle,
                                  nan,
                                  inf,
                                  -inf}) {
        EXPECT_FALSE(GimbalToMotorAnglesSplineConfig::isValidStepAngle(stepAngle)) << "step angle " << stepAngle;
        EXPECT_THROW(GimbalToMotorAnglesSplineConfig::create(stepAngle,
                                                             kTestDefaultMotor1Angle,
                                                             kTestDefaultMotor2Angle,
                                                             domain,
                                                             motor1Coefficients,
                                                             motor2Coefficients),
                     fsw::invalid_argument)
            << "step angle " << stepAngle;
    }
}

// Each default motor angle must be in [-pi, pi]. The two motors are checked separately, with the other default valid.
TEST(GimbalToMotorAnglesSplineTest, ConfigRejectsInvalidDefaultMotorAngles) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    constexpr float pi = std::numbers::pi_v<float>;
    const GimbalRhombusDomain domain = linearTestDomain();
    const SplineCoefficients motor1Coefficients = linearSurfaceCoefficients(SurfaceAxis::V);
    const SplineCoefficients motor2Coefficients = linearSurfaceCoefficients(SurfaceAxis::U);
    const auto create = [&](const float default1, const float default2) {
        return GimbalToMotorAnglesSplineConfig::create(
            kTestStepAngle, default1, default2, domain, motor1Coefficients, motor2Coefficients);
    };

    // The two limits are accepted
    for (const float value : {0.0F, -pi, pi, kTestDefaultMotor1Angle}) {
        EXPECT_TRUE(GimbalToMotorAnglesSplineConfig::isValidDefaultMotorAngle(value)) << "value " << value;
        EXPECT_NO_THROW(create(value, kTestDefaultMotor2Angle)) << "value " << value;
        EXPECT_NO_THROW(create(kTestDefaultMotor1Angle, value)) << "value " << value;
    }
    // One fp32 step past each limit and non-finite values are not
    for (const float value : {std::nextafter(pi, inf), std::nextafter(-pi, -inf), nan, inf, -inf}) {
        EXPECT_FALSE(GimbalToMotorAnglesSplineConfig::isValidDefaultMotorAngle(value)) << "value " << value;
        EXPECT_THROW(create(value, kTestDefaultMotor2Angle), fsw::invalid_argument) << "value " << value;
        EXPECT_THROW(create(kTestDefaultMotor1Angle, value), fsw::invalid_argument) << "value " << value;
    }
}

TEST(GimbalToMotorAnglesSplineTest, ConfigRejectsInvalidDomain) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
    const SplineCoefficients motor1Coefficients = readMotor1Coefficients();
    const SplineCoefficients motor2Coefficients = readMotor2Coefficients();
    const auto create = [&](const GimbalRhombusDomain& domain) {
        return GimbalToMotorAnglesSplineConfig::create(kTestStepAngle,
                                                       kTestDefaultMotor1Angle,
                                                       kTestDefaultMotor2Angle,
                                                       domain,
                                                       motor1Coefficients,
                                                       motor2Coefficients);
    };
    // Fields: center tip, center tilt, tip half-diagonal, tilt half-diagonal [rad]
    const auto domainOf = [](float tip, float tilt, float halfTip, float halfTilt) {
        return GimbalRhombusDomain{
            .centerTipAngle = tip, .centerTiltAngle = tilt, .halfDiagonalTip = halfTip, .halfDiagonalTilt = halfTilt};
    };
    EXPECT_NO_THROW(create(domainOf(0.01F, 0.0F, 0.3F, 0.45F)));

    // The rhombus may reach, but not pass, the +-90 degree gimbal limits. The smallest positive half-diagonal is
    // accepted.
    const float smallest = std::numeric_limits<float>::denorm_min();
    for (const auto& [halfTip, halfTilt] : {std::pair{halfPi, halfPi},
                                            std::pair{halfPi, 0.45F},
                                            std::pair{0.3F, halfPi},
                                            std::pair{smallest, 0.45F},
                                            std::pair{0.3F, smallest}}) {
        EXPECT_TRUE(GimbalToMotorAnglesSplineConfig::isValidDomain(domainOf(0.0F, 0.0F, halfTip, halfTilt)));
        EXPECT_NO_THROW(create(domainOf(0.0F, 0.0F, halfTip, halfTilt)));
    }

    // A vertex exactly at the limit with a center away from zero is accepted, and one fp32 step past it is not. The
    // values are in [1, 2) on the same fp32 grid, so 0.5 + (pi/2 - 0.5) rounds to exactly pi/2.
    const float limitHalfDiagonal = halfPi - 0.5F;
    const float pastLimitHalfDiagonal = std::nextafter(limitHalfDiagonal, inf);
    ASSERT_EQ(0.5F + limitHalfDiagonal, halfPi);
    ASSERT_GT(0.5F + pastLimitHalfDiagonal, halfPi);
    for (const float center : {0.5F, -0.5F}) {
        EXPECT_TRUE(GimbalToMotorAnglesSplineConfig::isValidDomain(domainOf(center, 0.0F, limitHalfDiagonal, 0.45F)));
        EXPECT_NO_THROW(create(domainOf(center, 0.0F, limitHalfDiagonal, 0.45F)));
        EXPECT_TRUE(GimbalToMotorAnglesSplineConfig::isValidDomain(domainOf(0.01F, center, 0.3F, limitHalfDiagonal)));
        EXPECT_NO_THROW(create(domainOf(0.01F, center, 0.3F, limitHalfDiagonal)));
    }

    // Non-finite values, zero or negative half-diagonals, and a rhombus past +-pi/2 on either axis (from a centered
    // half-diagonal one fp32 step past pi/2, or from an off-center rhombus) are not
    const float pastLimit = std::nextafter(halfPi, inf);
    const std::array invalidDomains{
        domainOf(nan, 0.0F, 0.3F, 0.45F),
        domainOf(0.01F, nan, 0.3F, 0.45F),
        domainOf(inf, 0.0F, 0.3F, 0.45F),
        domainOf(0.01F, inf, 0.3F, 0.45F),
        domainOf(0.01F, 0.0F, nan, 0.45F),
        domainOf(0.01F, 0.0F, 0.3F, nan),
        domainOf(0.01F, 0.0F, inf, 0.45F),
        domainOf(0.01F, 0.0F, 0.3F, inf),
        domainOf(0.01F, 0.0F, 0.0F, 0.45F),
        domainOf(0.01F, 0.0F, 0.3F, 0.0F),
        domainOf(0.01F, 0.0F, -0.3F, 0.45F),
        domainOf(0.01F, 0.0F, 0.3F, -0.45F),
        domainOf(0.0F, 0.0F, pastLimit, 0.45F),
        domainOf(0.0F, 0.0F, 0.3F, pastLimit),
        domainOf(0.2F, 0.0F, halfPi - 0.1F, 0.45F),
        domainOf(-0.2F, 0.0F, halfPi - 0.1F, 0.45F),
        domainOf(0.01F, 0.2F, 0.3F, halfPi - 0.1F),
        domainOf(0.01F, -0.2F, 0.3F, halfPi - 0.1F),
        domainOf(0.5F, 0.0F, pastLimitHalfDiagonal, 0.45F),
        domainOf(-0.5F, 0.0F, pastLimitHalfDiagonal, 0.45F),
        domainOf(0.01F, 0.5F, 0.3F, pastLimitHalfDiagonal),
        domainOf(0.01F, -0.5F, 0.3F, pastLimitHalfDiagonal),
    };
    for (const auto& domain : invalidDomains) {
        EXPECT_FALSE(GimbalToMotorAnglesSplineConfig::isValidDomain(domain));
        EXPECT_THROW(create(domain), fsw::invalid_argument);
    }
}

// Every coefficient of each motor must be in [-kMaxStepsPerRev, kMaxStepsPerRev]. The two motors are checked
// separately, with the other motor's coefficients valid.
TEST(GimbalToMotorAnglesSplineTest, ConfigRejectsInvalidCoefficients) {
    constexpr float inf = std::numeric_limits<float>::infinity();
    constexpr auto maxSteps = static_cast<float>(GimbalToMotorAnglesSplineConfig::kMaxStepsPerRev);
    const GimbalRhombusDomain domain = readDomain();
    const auto withValue = [](SplineCoefficients coefficients, const std::size_t index, const float value) {
        coefficients[index] = value;
        return coefficients;
    };

    for (const int motor : {1, 2}) {
        for (const std::size_t index :
             {std::size_t{0}, std::size_t{NUM_SPLINE_COEFFS / 2}, std::size_t{NUM_SPLINE_COEFFS - 1}}) {
            const auto createWith = [&](const float value) {
                const SplineCoefficients motor1Coefficients =
                    motor == 1 ? withValue(readMotor1Coefficients(), index, value) : readMotor1Coefficients();
                const SplineCoefficients motor2Coefficients =
                    motor == 2 ? withValue(readMotor2Coefficients(), index, value) : readMotor2Coefficients();
                EXPECT_EQ(GimbalToMotorAnglesSplineConfig::isValidCoefficients(motor == 1 ? motor1Coefficients
                                                                                          : motor2Coefficients),
                          std::fabs(value) <= maxSteps)
                    << "motor " << motor << " index " << index << " value " << value;
                return GimbalToMotorAnglesSplineConfig::create(kTestStepAngle,
                                                               kTestDefaultMotor1Angle,
                                                               kTestDefaultMotor2Angle,
                                                               domain,
                                                               motor1Coefficients,
                                                               motor2Coefficients);
            };

            // The two limits are accepted
            for (const float value : {maxSteps, -maxSteps}) {
                EXPECT_NO_THROW(createWith(value)) << "motor " << motor << " index " << index << " value " << value;
            }
            // One fp32 step past each limit and non-finite values are not
            for (const float value : {std::nextafter(maxSteps, inf),
                                      std::nextafter(-maxSteps, -inf),
                                      std::numeric_limits<float>::quiet_NaN(),
                                      inf,
                                      -inf}) {
                EXPECT_THROW(createWith(value), fsw::invalid_argument)
                    << "motor " << motor << " index " << index << " value " << value;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// With the coefficients set to the scaled Greville abscissae, motor 1 is exactly kLinearSurfaceScale * v and motor 2
// is exactly kLinearSurfaceScale * u, to within the fp32 evaluation error. This pins the basis weights, the coefficient
// index order, and which coefficient set each motor uses.
TEST(GimbalToMotorAnglesSplineTest, PropertyReproducesLinearSurface) {
    const GimbalRhombusDomain domain = linearTestDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(makeLinearConfig());

    for (float u = -1.0F; u <= 1.0F; u += 0.0625F) {
        for (float v = -1.0F; v <= 1.0F; v += 0.0625F) {
            const auto angles = gimbalAnglesFromUv(domain, u, v);
            const auto result = alg.update(angles[0], angles[1]);
            EXPECT_NEAR(motorSteps(result).motor1, kLinearSurfaceScale * v, kLinearSurfaceTolerance)
                << "u " << u << " v " << v;
            EXPECT_NEAR(motorSteps(result).motor2, kLinearSurfaceScale * u, kLinearSurfaceTolerance)
                << "u " << u << " v " << v;
        }
    }
}

// Motor 1 is S1(u, v) and motor 2 is S2(u, v), where S1 and S2 are evaluated here from their
// definition in float64
TEST(GimbalToMotorAnglesSplineTest, PropertyMotorStepsAreSurfacesAtUv) {
    const SplineCoefficients motor1Coefficients = readMotor1Coefficients();
    const SplineCoefficients motor2Coefficients = readMotor2Coefficients();
    const float tolerance1 = stepTolerance(motor1Coefficients);
    const float tolerance2 = stepTolerance(motor2Coefficients);
    const GimbalRhombusDomain domain = readDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());

    double maxSwapDifference1 = 0.0;
    double maxSwapDifference2 = 0.0;
    double maxMotorDifference = 0.0;
    for (float u = -1.0F; u <= 1.0F; u += 0.025F) {
        for (float v = -1.0F; v <= 1.0F; v += 0.025F) {
            const auto angles = gimbalAnglesFromUv(domain, u, v);
            const auto [uRef, vRef] = referenceUv(domain, angles[0], angles[1]);
            const double surface1 = referenceSurface(motor1Coefficients, uRef, vRef);
            const double surface2 = referenceSurface(motor2Coefficients, uRef, vRef);
            maxSwapDifference1 =
                std::max(maxSwapDifference1, std::fabs(surface1 - referenceSurface(motor1Coefficients, vRef, uRef)));
            maxSwapDifference2 =
                std::max(maxSwapDifference2, std::fabs(surface2 - referenceSurface(motor2Coefficients, vRef, uRef)));
            maxMotorDifference = std::max(maxMotorDifference, std::fabs(surface1 - surface2));

            const auto result = alg.update(angles[0], angles[1]);
            EXPECT_NEAR(motorSteps(result).motor1, surface1, tolerance1) << "u " << u << " v " << v;
            EXPECT_NEAR(motorSteps(result).motor2, surface2, tolerance2) << "u " << u << " v " << v;
        }
    }
    // The fitted surfaces are not symmetric in u and v and differ from each other, so a swap of u and v in either
    // motor, or a swap of the two coefficient sets, would fail the checks above
    EXPECT_GT(maxSwapDifference1, 1000.0 * tolerance1);
    EXPECT_GT(maxSwapDifference2, 1000.0 * tolerance2);
    EXPECT_GT(maxMotorDifference, 1000.0 * std::max(tolerance1, tolerance2));
}

// Every output of each motor lies between the smallest and largest coefficient of its surface (B-spline convex-hull
// property), with the fp32 evaluation error
TEST(GimbalToMotorAnglesSplineTest, PropertyOutputWithinCoefficientBounds) {
    const GimbalRhombusDomain domain = readDomain();
    const SplineCoefficients motor1Coefficients = readMotor1Coefficients();
    const SplineCoefficients motor2Coefficients = readMotor2Coefficients();
    const std::vector<float> motor1Values(motor1Coefficients.begin(), motor1Coefficients.end());
    const std::vector<float> motor2Values(motor2Coefficients.begin(), motor2Coefficients.end());

    for (float u = -1.0F; u <= 1.0F; u += 0.025F) {
        for (float v = -1.0F; v <= 1.0F; v += 0.025F) {
            propertyOutputWithinCoefficientBounds(domain.centerTipAngle,
                                                  domain.centerTiltAngle,
                                                  domain.halfDiagonalTip,
                                                  domain.halfDiagonalTilt,
                                                  motor1Values,
                                                  motor2Values,
                                                  u,
                                                  v);
        }
    }
}

// Any float input gives a finite output: the initial output or motor steps inside the coefficient convex hull
TEST(GimbalToMotorAnglesSplineTest, PropertyOutputIsFiniteForAnyInput) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    constexpr float big = std::numeric_limits<float>::max();
    constexpr float tiny = std::numeric_limits<float>::denorm_min();
    for (const float tip : {nan, inf, -inf, big, -big, tiny, 0.0F, 0.1F, -0.3F}) {
        for (const float tilt : {nan, inf, -inf, big, -big, tiny, 0.0F, 0.2F, -0.4F}) {
            propertyOutputIsFiniteForAnyInput(tip, tilt);
        }
    }
}

// A request outside the rhombus returns the previous valid output
TEST(GimbalToMotorAnglesSplineTest, PropertyOutsideDomainHoldsPreviousOutput) {
    for (const auto& [u, v] :
         {std::pair{1.01F, 0.0F}, std::pair{0.0F, -1.01F}, std::pair{-3.0F, 2.0F}, std::pair{50.0F, -50.0F}}) {
        propertyOutsideDomainHoldsPreviousOutput(0.3F, -0.6F, u, v);
        propertyOutsideDomainHoldsPreviousOutput(-1.0F, 1.0F, u, v);
    }
}

// The fitted coefficients minimize the sum of the squared errors to the measured table, so changing the coefficients
// of either motor gives a larger error
TEST(GimbalToMotorAnglesSplineTest, PropertyFittedCoefficientsMinimizeTableError) {
    // All coefficients, with alternating signs. The cubic B-spline smooths the alternating pattern strongly, so the
    // surface changes much less than the coefficients do.
    std::vector<float> checkerboard(NUM_SPLINE_COEFFS);
    std::vector<float> none(NUM_SPLINE_COEFFS, 0.0F);
    for (int i = 0; i < NUM_SPLINE_COEFFS_PER_AXIS; ++i) {
        for (int j = 0; j < NUM_SPLINE_COEFFS_PER_AXIS; ++j) {
            const float sign = ((i + j) % 2 == 0) ? 1.0F : -1.0F;
            checkerboard[(i * NUM_SPLINE_COEFFS_PER_AXIS) + j] = sign * kMinCoefficientPerturbation;
        }
    }
    propertyPerturbedCoefficientsIncreaseTableError(checkerboard, none);
    propertyPerturbedCoefficientsIncreaseTableError(none, checkerboard);
    propertyPerturbedCoefficientsIncreaseTableError(checkerboard, checkerboard);
}

// The surfaces have no jump where the evaluation moves from one spline interval to the next: the points 1e-4 before
// and after each knot, along u and along v, meet the Lipschitz bound
TEST(GimbalToMotorAnglesSplineTest, PropertySurfaceIsContinuousAcrossIntervalEdges) {
    constexpr float delta = 1e-4F;  // [-] offset in u or v on each side of the knot
    for (int k = 1; k < NUM_SPLINE_INTERVALS; ++k) {
        const float knot = -1.0F + (kIntervalWidth * static_cast<float>(k));
        for (const float other : {-0.83F, -0.21F, 0.37F, 0.91F}) {
            propertySurfaceIsLipschitz(knot - delta, other, 2.0F * delta, 0.0F);
            propertySurfaceIsLipschitz(other, knot - delta, 0.0F, 2.0F * delta);
        }
    }
}
