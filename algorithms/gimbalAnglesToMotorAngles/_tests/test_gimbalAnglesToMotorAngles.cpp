#include "../gimbalAnglesToMotorAnglesAlgorithm.h"
#include "gimbalAnglesToMotorAnglesTestHelpers.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace spline = gimbal_angles_to_motor_angles_detail;

// ---------------------------------------------------------------------------
// Acceptance and hold tests
// ---------------------------------------------------------------------------

// Requests that are not addressable hold the previous valid output. Before the first valid request, that is the
// motor angles of the configured surfaces at zero gimbal angles.
TEST(GimbalAnglesToMotorAnglesTest, FirstCallOutsideDomainReturnsZeroGimbalMotorAngles) {
    const GimbalAnglesToMotorAnglesConfig config = makeRoughConfig();
    GimbalAnglesToMotorAnglesAlgorithm alg(config);
    const auto result = alg.update(25.0F * degToRad, 0.0F);

    // The same fp32 evaluation as a valid request at zero gimbal angles
    GimbalAnglesToMotorAnglesAlgorithm reference(config);
    const auto atZero = reference.update(0.0F, 0.0F);
    EXPECT_EQ(result.motorAngle1, atZero.motorAngle1);
    EXPECT_EQ(result.motorAngle2, atZero.motorAngle2);

    // And the surfaces at u = v = 0
    const SplineCoefficients& motor1Coefficients = config.getMotor1Coefficients();
    const SplineCoefficients& motor2Coefficients = config.getMotor2Coefficients();
    EXPECT_NEAR(
        motorSteps(result).motor1, referenceSurface(motor1Coefficients, 0.0, 0.0), stepTolerance(motor1Coefficients));
    EXPECT_NEAR(
        motorSteps(result).motor2, referenceSurface(motor2Coefficients, 0.0, 0.0), stepTolerance(motor2Coefficients));
}

TEST(GimbalAnglesToMotorAnglesTest, OutsideDomainHoldsPreviousValidOutput) {
    const GimbalRhombusDomain domain = linearTestDomain();
    GimbalAnglesToMotorAnglesAlgorithm alg(makeLinearConfig());
    const auto inside = gimbalAnglesFromUv(domain, 0.25F, -0.5F);
    const auto valid = alg.update(inside.at(0), inside.at(1));
    ASSERT_NEAR(motorSteps(valid).motor1, kLinearSurfaceScale * -0.5F, kLinearSurfaceTolerance);
    ASSERT_NEAR(motorSteps(valid).motor2, kLinearSurfaceScale * 0.25F, kLinearSurfaceTolerance);

    for (const auto& [u, v] : {std::pair{1.5F, 0.0F}, std::pair{0.0F, -1.5F}, std::pair{3.0F, 3.0F}}) {
        const auto outside = gimbalAnglesFromUv(domain, u, v);
        const auto held = alg.update(outside.at(0), outside.at(1));
        EXPECT_EQ(held.motorAngle1, valid.motorAngle1) << "u " << u << " v " << v;
        EXPECT_EQ(held.motorAngle2, valid.motorAngle2) << "u " << u << " v " << v;
    }
}

TEST(GimbalAnglesToMotorAnglesTest, NonFiniteInputHoldsPreviousValidOutput) {
    const GimbalRhombusDomain domain = linearTestDomain();
    GimbalAnglesToMotorAnglesAlgorithm alg(makeLinearConfig());
    const auto inside = gimbalAnglesFromUv(domain, -0.4F, 0.6F);
    const auto valid = alg.update(inside.at(0), inside.at(1));
    ASSERT_NEAR(motorSteps(valid).motor1, kLinearSurfaceScale * 0.6F, kLinearSurfaceTolerance);
    ASSERT_NEAR(motorSteps(valid).motor2, kLinearSurfaceScale * -0.4F, kLinearSurfaceTolerance);

    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    for (const auto& [tip, tilt] : {
             std::pair{nan, 0.0F},
             std::pair{0.0F, nan},
             std::pair{inf, 0.0F},
             std::pair{0.0F, -inf},
             std::pair{nan, nan},
         }) {
        const auto held = alg.update(tip, tilt);
        EXPECT_EQ(held.motorAngle1, valid.motorAngle1);
        EXPECT_EQ(held.motorAngle2, valid.motorAngle2);
    }
}

// Finite inputs can overflow the normalized coordinates to +inf and -inf, whose sum is NaN
TEST(GimbalAnglesToMotorAnglesTest, OverflowingFiniteInputHoldsPreviousValidOutput) {
    const GimbalRhombusDomain domain = linearTestDomain();
    GimbalAnglesToMotorAnglesAlgorithm alg(makeLinearConfig());
    const auto inside = gimbalAnglesFromUv(domain, 0.1F, 0.2F);
    const auto valid = alg.update(inside.at(0), inside.at(1));
    ASSERT_NEAR(motorSteps(valid).motor1, kLinearSurfaceScale * 0.2F, kLinearSurfaceTolerance);
    ASSERT_NEAR(motorSteps(valid).motor2, kLinearSurfaceScale * 0.1F, kLinearSurfaceTolerance);

    constexpr float big = std::numeric_limits<float>::max();
    const auto held = alg.update(big, -big);
    EXPECT_EQ(held.motorAngle1, valid.motorAngle1);
    EXPECT_EQ(held.motorAngle2, valid.motorAngle2);
}

// The module assumes that every request is in the circle of radius thetaMax. Requests all around this circle are
// accepted and give the motor steps of the float64 reference surfaces, for the rough and for the linear test
// configuration. With the rough coefficients, this also checks the algorithm against the reference on every interval
// that the circle crosses.
TEST(GimbalAnglesToMotorAnglesTest, CircleIsAddressable) {
    constexpr int numDirections = 720;
    for (const bool rough : {true, false}) {
        const GimbalAnglesToMotorAnglesConfig config = rough ? makeRoughConfig() : makeLinearConfig();
        const GimbalRhombusDomain& domain = config.getDomain();
        const SplineCoefficients& motor1Coefficients = config.getMotor1Coefficients();
        const SplineCoefficients& motor2Coefficients = config.getMotor2Coefficients();
        GimbalAnglesToMotorAnglesAlgorithm alg(config);

        for (int k = 0; k < numDirections; ++k) {
            const double direction = 2.0 * std::numbers::pi * k / numDirections;
            const auto angles = gimbalAnglesOnCircle(static_cast<double>(domain.thetaMax), direction);
            const auto [u, v] = referenceUv(domain, angles.at(0), angles.at(1));
            alg.reInitialize();
            const MotorSteps steps = motorSteps(alg.update(angles.at(0), angles.at(1)));
            EXPECT_NEAR(steps.motor1, referenceSurface(motor1Coefficients, u, v), stepTolerance(motor1Coefficients))
                << "rough " << rough << " direction " << direction;
            EXPECT_NEAR(steps.motor2, referenceSurface(motor2Coefficients, u, v), stepTolerance(motor2Coefficients))
                << "rough " << rough << " direction " << direction;
        }
    }
}

// The rhombus edge, not the circle of radius thetaMax, decides which requests are accepted. At each of the four edges,
// a request with |u| or |v| 1e-3 inside the edge is accepted and gives the motor steps of the float64 reference
// surfaces, although it is outside the circle. The request 1e-3 past the same edge is not accepted and holds the motor
// angles of the request inside. The offsets are much larger than the fp32 rounding of u and v.
TEST(GimbalAnglesToMotorAnglesTest, RhombusEdgeIsTheAcceptanceBoundary) {
    constexpr float inside = 1.0F - 1e-3F;
    constexpr float outside = 1.0F + 1e-3F;
    for (const bool rough : {true, false}) {
        const GimbalAnglesToMotorAnglesConfig config = rough ? makeRoughConfig() : makeLinearConfig();
        const GimbalRhombusDomain& domain = config.getDomain();
        const SplineCoefficients& motor1Coefficients = config.getMotor1Coefficients();
        const SplineCoefficients& motor2Coefficients = config.getMotor2Coefficients();
        GimbalAnglesToMotorAnglesAlgorithm alg(config);

        // Fields: u and v inside the edge, u and v past it
        for (const auto& [insideUv, outsideUv] : {
                 std::pair{std::array{inside, 0.3F}, std::array{outside, 0.3F}},
                 std::pair{std::array{-inside, -0.2F}, std::array{-outside, -0.2F}},
                 std::pair{std::array{0.4F, inside}, std::array{0.4F, outside}},
                 std::pair{std::array{-0.7F, -inside}, std::array{-0.7F, -outside}},
             }) {
            const auto near = gimbalAnglesFromUv(domain, insideUv.at(0), insideUv.at(1));
            EXPECT_GT(std::hypot(near.at(0), near.at(1)), domain.thetaMax) << "rough " << rough;
            const auto [u, v] = referenceUv(domain, near.at(0), near.at(1));
            const auto accepted = alg.update(near.at(0), near.at(1));
            EXPECT_NEAR(motorSteps(accepted).motor1,
                        referenceSurface(motor1Coefficients, u, v),
                        stepTolerance(motor1Coefficients))
                << "rough " << rough << " u " << u << " v " << v;
            EXPECT_NEAR(motorSteps(accepted).motor2,
                        referenceSurface(motor2Coefficients, u, v),
                        stepTolerance(motor2Coefficients))
                << "rough " << rough << " u " << u << " v " << v;

            const auto past = gimbalAnglesFromUv(domain, outsideUv.at(0), outsideUv.at(1));
            const auto held = alg.update(past.at(0), past.at(1));
            EXPECT_EQ(held.motorAngle1, accepted.motorAngle1) << "rough " << rough << " u " << outsideUv.at(0);
            EXPECT_EQ(held.motorAngle2, accepted.motorAngle2) << "rough " << rough << " v " << outsideUv.at(1);
        }
    }
}

// After reInitialize(), a request outside the rhombus gives the same output as the first request of a new algorithm
TEST(GimbalAnglesToMotorAnglesTest, ReInitializeRestoresInitialOutput) {
    const GimbalRhombusDomain domain = linearTestDomain();
    const auto inside = gimbalAnglesFromUv(domain, 0.5F, 0.5F);
    const auto outside = gimbalAnglesFromUv(domain, 2.0F, 0.0F);
    GimbalAnglesToMotorAnglesAlgorithm fresh(makeLinearConfig());
    const auto initial = fresh.update(outside.at(0), outside.at(1));

    GimbalAnglesToMotorAnglesAlgorithm alg(makeLinearConfig());
    const auto valid = alg.update(inside.at(0), inside.at(1));
    ASSERT_NE(valid.motorAngle1, initial.motorAngle1);
    ASSERT_NE(valid.motorAngle2, initial.motorAngle2);

    alg.reInitialize();
    const auto result = alg.update(outside.at(0), outside.at(1));
    EXPECT_EQ(result.motorAngle1, initial.motorAngle1);
    EXPECT_EQ(result.motorAngle2, initial.motorAngle2);
}

// Without a previous valid output, a rejected request gives the motor angles of the current configuration at zero
// gimbal angles. That fallback is not stored as the previous valid output, so a later setConfig() takes effect at once.
TEST(GimbalAnglesToMotorAnglesTest, FallbackUsesZeroGimbalMotorAnglesOfCurrentConfig) {
    // [steps] Offsets added to the linear surfaces, so that the new surfaces are 1000 v + 250 and 1000 u - 400. The
    // B-spline weights sum to one, so a constant added to every coefficient is added to the surface.
    constexpr float offset1 = 250.0F;
    constexpr float offset2 = -400.0F;
    const auto withOffset = [](SplineCoefficients coefficients, const float offset) {
        for (float& coefficient : coefficients) {
            coefficient += offset;
        }
        return coefficients;
    };
    const SplineCoefficients motor1Coefficients = withOffset(linearSurfaceCoefficients(SurfaceAxis::V), offset1);
    const SplineCoefficients motor2Coefficients = withOffset(linearSurfaceCoefficients(SurfaceAxis::U), offset2);
    const float tolerance1 = stepTolerance(motor1Coefficients);
    const float tolerance2 = stepTolerance(motor2Coefficients);

    const GimbalRhombusDomain domain = linearTestDomain();
    const auto inside = gimbalAnglesFromUv(domain, 0.5F, 0.5F);
    const auto outside = gimbalAnglesFromUv(domain, 2.0F, 0.0F);
    GimbalAnglesToMotorAnglesAlgorithm alg(makeLinearConfig());

    // The linear surfaces are zero at zero gimbal angles
    const auto beforeSetConfig = alg.update(outside.at(0), outside.at(1));
    ASSERT_NEAR(motorSteps(beforeSetConfig).motor1, 0.0F, kLinearSurfaceTolerance);
    ASSERT_NEAR(motorSteps(beforeSetConfig).motor2, 0.0F, kLinearSurfaceTolerance);

    alg.setConfig(GimbalAnglesToMotorAnglesConfig::create(
        kTestStepsPerRevolution, domain, motor1Coefficients, motor2Coefficients));

    // No valid output yet, so the zero gimbal motor angles of the new configuration apply at once
    const auto beforeValid = alg.update(outside.at(0), outside.at(1));
    EXPECT_NEAR(motorSteps(beforeValid).motor1, offset1, tolerance1);
    EXPECT_NEAR(motorSteps(beforeValid).motor2, offset2, tolerance2);

    // A valid output is held instead of the zero gimbal motor angles
    const auto valid = alg.update(inside.at(0), inside.at(1));
    ASSERT_NEAR(motorSteps(valid).motor1, (kLinearSurfaceScale * 0.5F) + offset1, tolerance1);
    ASSERT_NEAR(motorSteps(valid).motor2, (kLinearSurfaceScale * 0.5F) + offset2, tolerance2);
    const auto held = alg.update(outside.at(0), outside.at(1));
    EXPECT_EQ(held.motorAngle1, valid.motorAngle1);
    EXPECT_EQ(held.motorAngle2, valid.motorAngle2);

    // reInitialize() clears the valid output, so the zero gimbal motor angles apply again
    alg.reInitialize();
    const auto result = alg.update(outside.at(0), outside.at(1));
    EXPECT_EQ(result.motorAngle1, beforeValid.motorAngle1);
    EXPECT_EQ(result.motorAngle2, beforeValid.motorAngle2);
}

// ---------------------------------------------------------------------------
// Reconfiguration and setup tests (validation + round trip)
// ---------------------------------------------------------------------------

TEST(GimbalAnglesToMotorAnglesTest, SetConfigReplacesSurfaceAndKeepsPreviousOutput) {
    GimbalAnglesToMotorAnglesAlgorithm alg(makeRoughConfig());
    const auto previous = alg.update(3.0F * degToRad, 2.0F * degToRad);

    // A different constant for each motor, so that a swap of the two coefficient sets is caught
    SplineCoefficients motor1Constant{};
    SplineCoefficients motor2Constant{};
    motor1Constant.fill(1250.0F);
    motor2Constant.fill(-2500.0F);
    alg.setConfig(GimbalAnglesToMotorAnglesConfig::create(
        kTestStepsPerRevolution, roughTestDomain(), motor1Constant, motor2Constant));

    // The previous valid output survives the reconfiguration
    const auto held = alg.update(0.0F, 80.0F * degToRad);
    EXPECT_FLOAT_EQ(held.motorAngle1, previous.motorAngle1);
    EXPECT_FLOAT_EQ(held.motorAngle2, previous.motorAngle2);

    // The B-spline weights sum to one, so a constant coefficient set gives a constant surface
    const auto result = alg.update(3.0F * degToRad, 2.0F * degToRad);
    EXPECT_NEAR(motorSteps(result).motor1, 1250.0F, stepTolerance(motor1Constant));
    EXPECT_NEAR(motorSteps(result).motor2, -2500.0F, stepTolerance(motor2Constant));
}

TEST(GimbalAnglesToMotorAnglesTest, ConfigRoundTrip) {
    const GimbalRhombusDomain domain = roughTestDomain();
    const SplineCoefficients motor1Coefficients = roughSurfaceCoefficients(1U);
    const SplineCoefficients motor2Coefficients = roughSurfaceCoefficients(2U);
    const auto config = GimbalAnglesToMotorAnglesConfig::create(
        kTestStepsPerRevolution, domain, motor1Coefficients, motor2Coefficients);

    EXPECT_EQ(config.getStepsPerRevolution(), kTestStepsPerRevolution);
    EXPECT_EQ(config.getDomain().halfDiagonalTip, domain.halfDiagonalTip);
    EXPECT_EQ(config.getDomain().halfDiagonalTilt, domain.halfDiagonalTilt);
    EXPECT_EQ(config.getDomain().thetaMax, domain.thetaMax);
    EXPECT_EQ(config.getMotor1Coefficients(), motor1Coefficients);
    EXPECT_EQ(config.getMotor2Coefficients(), motor2Coefficients);
}

TEST(GimbalAnglesToMotorAnglesTest, ConfigRejectsInvalidStepsPerRevolution) {
    constexpr uint32_t maxStepsPerRevolution = kMaxStepsPerRev;
    const GimbalRhombusDomain domain = roughTestDomain();
    const SplineCoefficients motor1Coefficients = roughSurfaceCoefficients(1U);
    const SplineCoefficients motor2Coefficients = roughSurfaceCoefficients(2U);

    // The two limits are accepted
    for (const uint32_t stepsPerRevolution : {1U, kTestStepsPerRevolution, maxStepsPerRevolution}) {
        EXPECT_TRUE(GimbalAnglesToMotorAnglesConfig::isValidStepsPerRevolution(stepsPerRevolution))
            << "steps per revolution " << stepsPerRevolution;
        EXPECT_NO_THROW(GimbalAnglesToMotorAnglesConfig::create(
            stepsPerRevolution, domain, motor1Coefficients, motor2Coefficients));
    }

    // Zero and one step past the upper limit are not
    for (const uint32_t stepsPerRevolution : {0U, maxStepsPerRevolution + 1U}) {
        EXPECT_FALSE(GimbalAnglesToMotorAnglesConfig::isValidStepsPerRevolution(stepsPerRevolution))
            << "steps per revolution " << stepsPerRevolution;
        EXPECT_THROW(
            GimbalAnglesToMotorAnglesConfig::create(stepsPerRevolution, domain, motor1Coefficients, motor2Coefficients),
            fsw::invalid_argument)
            << "steps per revolution " << stepsPerRevolution;
    }
}

TEST(GimbalAnglesToMotorAnglesTest, ConfigRejectsInvalidDomain) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
    const SplineCoefficients motor1Coefficients = linearSurfaceCoefficients(SurfaceAxis::V);
    const SplineCoefficients motor2Coefficients = linearSurfaceCoefficients(SurfaceAxis::U);
    const auto create = [&](const GimbalRhombusDomain& domain) {
        return GimbalAnglesToMotorAnglesConfig::create(
            kTestStepsPerRevolution, domain, motor1Coefficients, motor2Coefficients);
    };
    // Fields: tip half-diagonal, tilt half-diagonal, thetaMax [rad]
    const auto domainOf = [](float halfTip, float halfTilt, float thetaMax) {
        return GimbalRhombusDomain{.halfDiagonalTip = halfTip, .halfDiagonalTilt = halfTilt, .thetaMax = thetaMax};
    };
    // A rhombus whose thetaMax is exactly its fp32 inscribed radius
    const auto atLimit = [](float halfTip, float halfTilt) {
        GimbalRhombusDomain domain{.halfDiagonalTip = halfTip, .halfDiagonalTilt = halfTilt, .thetaMax = 0.0F};
        domain.thetaMax = GimbalAnglesToMotorAnglesConfig::inscribedRadius(domain);
        return domain;
    };

    // The rhombus may reach, but not pass, the +-90 degree gimbal limits. thetaMax may be as large as the inscribed
    // radius, and as small as the smallest positive fp32 value.
    constexpr float smallest = std::numeric_limits<float>::denorm_min();
    for (const auto& domain : {
             linearTestDomain(),
             atLimit(0.3F, 0.45F),
             atLimit(halfPi, halfPi),
             atLimit(halfPi, 0.45F),
             atLimit(0.3F, halfPi),
             domainOf(0.3F, 0.45F, smallest),
         }) {
        EXPECT_TRUE(GimbalAnglesToMotorAnglesConfig::isValidDomain(domain))
            << domain.halfDiagonalTip << " " << domain.halfDiagonalTilt << " " << domain.thetaMax;
        EXPECT_NO_THROW(create(domain));
    }

    // Non-finite values, zero or negative values, a half-diagonal one fp32 step past pi/2, and a thetaMax one fp32
    // step past the inscribed radius are not
    const float pastLimit = std::nextafter(halfPi, inf);
    const float thetaMax = linearTestDomain().thetaMax;
    const auto pastInscribed = [&](float halfTip, float halfTilt) {
        GimbalRhombusDomain domain = atLimit(halfTip, halfTilt);
        domain.thetaMax = std::nextafter(domain.thetaMax, inf);
        return domain;
    };
    const std::array invalidDomains{
        domainOf(nan, 0.45F, thetaMax),
        domainOf(0.3F, nan, thetaMax),
        domainOf(0.3F, 0.45F, nan),
        domainOf(inf, 0.45F, thetaMax),
        domainOf(0.3F, inf, thetaMax),
        domainOf(0.3F, 0.45F, inf),
        domainOf(0.0F, 0.45F, thetaMax),
        domainOf(0.3F, 0.0F, thetaMax),
        domainOf(0.3F, 0.45F, 0.0F),
        domainOf(-0.3F, 0.45F, thetaMax),
        domainOf(0.3F, -0.45F, thetaMax),
        domainOf(0.3F, 0.45F, -thetaMax),
        domainOf(pastLimit, 0.45F, thetaMax),
        domainOf(0.3F, pastLimit, thetaMax),
        pastInscribed(0.3F, 0.45F),
        pastInscribed(halfPi, halfPi),
    };
    for (const auto& domain : invalidDomains) {
        EXPECT_FALSE(GimbalAnglesToMotorAnglesConfig::isValidDomain(domain))
            << domain.halfDiagonalTip << " " << domain.halfDiagonalTilt << " " << domain.thetaMax;
        EXPECT_THROW(create(domain), fsw::invalid_argument);
    }
}

// Every coefficient of each motor must be in [-kMaxStepsPerRev, kMaxStepsPerRev]. The two motors are checked
// separately, with the other motor's coefficients valid.
TEST(GimbalAnglesToMotorAnglesTest, ConfigRejectsInvalidCoefficients) {
    constexpr float inf = std::numeric_limits<float>::infinity();
    constexpr auto maxSteps = static_cast<float>(kMaxStepsPerRev);
    const GimbalRhombusDomain domain = roughTestDomain();
    const auto withValue = [](SplineCoefficients coefficients, const std::size_t index, const float value) {
        coefficients.at(index) = value;
        return coefficients;
    };

    for (const int motor : {1, 2}) {
        for (const std::size_t index :
             {std::size_t{0}, std::size_t{NUM_SPLINE_COEFFS / 2}, std::size_t{NUM_SPLINE_COEFFS - 1}}) {
            const auto createWith = [&](const float value) {
                const SplineCoefficients motor1Coefficients =
                    motor == 1 ? withValue(roughSurfaceCoefficients(1U), index, value) : roughSurfaceCoefficients(1U);
                const SplineCoefficients motor2Coefficients =
                    motor == 2 ? withValue(roughSurfaceCoefficients(2U), index, value) : roughSurfaceCoefficients(2U);
                EXPECT_EQ(GimbalAnglesToMotorAnglesConfig::isValidCoefficients(motor == 1 ? motor1Coefficients
                                                                                          : motor2Coefficients),
                          std::fabs(value) <= maxSteps)
                    << "motor " << motor << " index " << index << " value " << value;
                return GimbalAnglesToMotorAnglesConfig::create(
                    kTestStepsPerRevolution, domain, motor1Coefficients, motor2Coefficients);
            };

            // The two limits are accepted
            for (const float value : {maxSteps, -maxSteps}) {
                EXPECT_NO_THROW(createWith(value)) << "motor " << motor << " index " << index << " value " << value;
            }
            // One fp32 step past each limit and non-finite values are not
            for (const float value : {
                     std::nextafter(maxSteps, inf),
                     std::nextafter(-maxSteps, -inf),
                     std::numeric_limits<float>::quiet_NaN(),
                     inf,
                     -inf,
                 }) {
                EXPECT_THROW(createWith(value), fsw::invalid_argument)
                    << "motor " << motor << " index " << index << " value " << value;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// The four weights of an interval are not negative and sum to one. At the ends of the interval they are the values of
// the basis functions at the knots: (1/6, 4/6, 1/6, 0) at t = 0 and (0, 1/6, 4/6, 1/6) at t = 1. Thus the weights at
// the end of one interval are the weights at the start of the next one, moved by one coefficient.
TEST(GimbalAnglesToMotorAnglesTest, CubicWeightsArePartitionOfUnity) {
    constexpr int numSteps = 64;
    for (int k = 0; k <= numSteps; ++k) {
        const float t = static_cast<float>(k) / static_cast<float>(numSteps);
        float sum = 0.0F;
        for (const float weight : spline::cubicBSplineWeights(t)) {
            EXPECT_GE(weight, 0.0F) << "t " << t;
            sum += weight;
        }
        EXPECT_NEAR(sum, 1.0F, 8.0F * kEps) << "t " << t;
    }

    const std::array<float, 4> atStart = spline::cubicBSplineWeights(0.0F);
    const std::array<float, 4> atEnd = spline::cubicBSplineWeights(1.0F);
    constexpr std::array<float, 4> expectedStart{1.0F / 6.0F, 4.0F / 6.0F, 1.0F / 6.0F, 0.0F};
    constexpr std::array<float, 4> expectedEnd{0.0F, 1.0F / 6.0F, 4.0F / 6.0F, 1.0F / 6.0F};
    for (std::size_t p = 0; p < 4; ++p) {
        EXPECT_FLOAT_EQ(atStart.at(p), expectedStart.at(p)) << "p " << p;
        EXPECT_FLOAT_EQ(atEnd.at(p), expectedEnd.at(p)) << "p " << p;
    }
}

// splineInterval finds the interval that holds a coordinate and the local coordinate in it, so that
// -1 + h (interval + t) is the coordinate. The ends are clamped: x = -1 is the start of the first interval and x = 1 is
// the end of the last one.
TEST(GimbalAnglesToMotorAnglesTest, SplineIntervalFindsLocalCoordinate) {
    constexpr int numSteps = 600;
    for (int k = 0; k <= numSteps; ++k) {
        const float x = -1.0F + (2.0F * static_cast<float>(k) / static_cast<float>(numSteps));
        const auto [interval, t] = spline::splineInterval(x);
        EXPECT_LT(interval, NUM_SPLINE_INTERVALS) << "x " << x;
        EXPECT_GE(t, 0.0F) << "x " << x;
        EXPECT_LE(t, 1.0F + (4.0F * kEps)) << "x " << x;
        EXPECT_NEAR(-1.0F + (kIntervalWidth * (static_cast<float>(interval) + t)), x, 8.0F * kEps) << "x " << x;
    }

    const auto first = spline::splineInterval(-1.0F);
    EXPECT_EQ(first.interval, 0U);
    EXPECT_EQ(first.t, 0.0F);
    const auto last = spline::splineInterval(1.0F);
    EXPECT_EQ(last.interval, NUM_SPLINE_INTERVALS - 1U);
    EXPECT_NEAR(last.t, 1.0F, 4.0F * kEps);
}

// With the coefficients set to the scaled Greville abscissae, the surfaces are exactly kLinearSurfaceScale * u and
// kLinearSurfaceScale * v, to within the fp32 evaluation error, everywhere in [-1, 1] x [-1, 1], edges included. The
// grid step is exact in fp32, so the grid reaches both edges. The fuzz test covers the points between the grid points.
TEST(GimbalAnglesToMotorAnglesTest, PropertyReproducesLinearSurface) {
    for (float u = -1.0F; u <= 1.0F; u += 0.0625F) {
        for (float v = -1.0F; v <= 1.0F; v += 0.0625F) {
            propertyReproducesLinearSurface(u, v);
        }
    }
}

// Each rough test surface lies between the smallest and largest coefficient of its surface (B-spline convex-hull
// property), with the fp32 evaluation error, everywhere in [-1, 1] x [-1, 1]
TEST(GimbalAnglesToMotorAnglesTest, PropertySurfaceWithinCoefficientBounds) {
    constexpr int numSteps = 80;
    for (const SplineCoefficients& coefficients : {roughSurfaceCoefficients(1U), roughSurfaceCoefficients(2U)}) {
        for (int i = 0; i <= numSteps; ++i) {
            for (int j = 0; j <= numSteps; ++j) {
                propertySurfaceWithinCoefficientBounds(coefficients,
                                                       -1.0F + (2.0F * static_cast<float>(i) / numSteps),
                                                       -1.0F + (2.0F * static_cast<float>(j) / numSteps));
            }
        }
    }
}

// Any float input gives a finite output: the initial output or motor steps inside the coefficient convex hull
TEST(GimbalAnglesToMotorAnglesTest, PropertyOutputIsFiniteForAnyInput) {
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

// A request outside the rhombus returns the previous valid output. The valid requests are in the circle, one of them on
// it.
TEST(GimbalAnglesToMotorAnglesTest, PropertyOutsideDomainHoldsPreviousOutput) {
    for (const auto& [u, v] :
         {std::pair{1.01F, 0.0F}, std::pair{0.0F, -1.01F}, std::pair{-3.0F, 2.0F}, std::pair{50.0F, -50.0F}}) {
        propertyOutsideDomainHoldsPreviousOutput(0.6F, 1.0F, u, v);
        propertyOutsideDomainHoldsPreviousOutput(1.0F, 2.4F, u, v);
    }
}

// The surfaces have no jump where the evaluation moves from one spline interval to the next: the points 1e-4 before
// and after each knot, along u and along v, meet the Lipschitz bound, across the whole square including its edges
TEST(GimbalAnglesToMotorAnglesTest, PropertySurfaceIsContinuousAcrossIntervalEdges) {
    for (int k = 1; k < NUM_SPLINE_INTERVALS; ++k) {
        const float knot = -1.0F + (kIntervalWidth * static_cast<float>(k));
        for (const float other : {-1.0F, -0.83F, -0.21F, 0.37F, 0.91F, 1.0F}) {
            constexpr float delta = 1e-4F;
            propertySurfaceIsLipschitz(knot - delta, other, 2.0F * delta, 0.0F);
            propertySurfaceIsLipschitz(other, knot - delta, 0.0F, 2.0F * delta);
        }
    }
}
