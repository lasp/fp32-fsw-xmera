#include "../gimbalToMotorAnglesSplineAlgorithm.h"
#include "gimbalToMotorAnglesSplineTestHelpers.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <numbers>

namespace {

constexpr float kEps = std::numeric_limits<float>::epsilon();
constexpr float kIntervalWidth = 2.0F / static_cast<float>(NUM_SPLINE_INTERVALS);  // [-] knot spacing in u and v

/*! This method returns an upper bound on |dS/du| and |dS/dv|. The derivative of a cubic B-spline is a quadratic
 B-spline whose coefficients are the coefficient differences divided by the knot spacing.
 @return float [rad] Slope bound per unit of u or v
 @param coefficients [rad] Spline control coefficients
*/
float surfaceSlopeBound(const SplineCoefficients& coefficients) {
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
float evaluationTolerance(const SplineCoefficients& coefficients) {
    const auto [minCoeff, maxCoeff] = coefficientBounds(coefficients);
    const float maxAbsCoeff = std::max(std::fabs(minCoeff), std::fabs(maxCoeff));
    const float valueError = 20.5F * kEps * maxAbsCoeff;
    const float coordinateError = 2.0F * 9.0F * kEps * surfaceSlopeBound(coefficients);
    return 3.0F * (valueError + coordinateError);
}

}  // namespace

// ---------------------------------------------------------------------------
// Regression tests against the offline fit
// ---------------------------------------------------------------------------

// The fp32 surface matches the scipy (FITPACK, float64) evaluation of the same coefficients
TEST(GimbalToMotorAnglesSplineTest, MatchesScipyReferenceEvaluation) {
    const SplineCoefficients coefficients = readCoefficients();
    const float tolerance = evaluationTolerance(coefficients);
    const auto samples = readSamples("splineReference.csv");

    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    for (const auto& sample : samples) {
        const auto result = alg.update(sample.tip, sample.tilt);
        EXPECT_NEAR(result.motorAngle1, sample.theta1, tolerance) << "tip " << sample.tip << " tilt " << sample.tilt;
        EXPECT_NEAR(result.motorAngle2, sample.theta2, tolerance) << "tip " << sample.tip << " tilt " << sample.tilt;
    }
}

// The surface matches every measured table entry inside the addressable rhombus within the fit error. The fit tool
// reports a max fit error of 0.0428 deg for the committed coefficients; 0.045 deg adds the fp32 evaluation error.
TEST(GimbalToMotorAnglesSplineTest, MatchesMeasuredTableInsideDomain) {
    const float fitTolerance = 0.045F * degToRad;
    const auto samples = readSamples("tablePointsInDomain.csv");

    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    for (const auto& sample : samples) {
        alg.reInitialize();
        const auto result = alg.update(sample.tip, sample.tilt);
        EXPECT_NEAR(result.motorAngle1, sample.theta1, fitTolerance) << "tip " << sample.tip << " tilt " << sample.tilt;
        EXPECT_NEAR(result.motorAngle2, sample.theta2, fitTolerance) << "tip " << sample.tip << " tilt " << sample.tilt;
    }
}

// ---------------------------------------------------------------------------
// Requests that are not addressable hold the previous valid output
// ---------------------------------------------------------------------------

TEST(GimbalToMotorAnglesSplineTest, FirstCallOutsideDomainReturnsDefault) {
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    const auto result = alg.update(25.0F * degToRad, 0.0F);

    EXPECT_FLOAT_EQ(result.motorAngle1, kReferenceDefaultMotorAngle);
    EXPECT_FLOAT_EQ(result.motorAngle2, kReferenceDefaultMotorAngle);
}

TEST(GimbalToMotorAnglesSplineTest, OutsideDomainHoldsPreviousValidOutput) {
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    const auto valid = alg.update(2.0F * degToRad, -3.0F * degToRad);
    const auto held = alg.update(0.0F, 40.0F * degToRad);

    EXPECT_FLOAT_EQ(held.motorAngle1, valid.motorAngle1);
    EXPECT_FLOAT_EQ(held.motorAngle2, valid.motorAngle2);
}

TEST(GimbalToMotorAnglesSplineTest, NonFiniteInputHoldsPreviousValidOutput) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    const auto valid = alg.update(-4.0F * degToRad, 5.0F * degToRad);

    for (const auto& [tip, tilt] : {std::pair{nan, 0.0F},
                                    std::pair{0.0F, nan},
                                    std::pair{inf, 0.0F},
                                    std::pair{0.0F, -inf},
                                    std::pair{nan, nan}}) {
        const auto held = alg.update(tip, tilt);
        EXPECT_FLOAT_EQ(held.motorAngle1, valid.motorAngle1);
        EXPECT_FLOAT_EQ(held.motorAngle2, valid.motorAngle2);
    }
}

// Finite inputs can overflow the normalized coordinates to +inf and -inf, whose sum is NaN
TEST(GimbalToMotorAnglesSplineTest, OverflowingFiniteInputHoldsPreviousValidOutput) {
    constexpr float big = std::numeric_limits<float>::max();
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    const auto valid = alg.update(1.0F * degToRad, 1.0F * degToRad);
    const auto held = alg.update(big, -big);

    EXPECT_FLOAT_EQ(held.motorAngle1, valid.motorAngle1);
    EXPECT_FLOAT_EQ(held.motorAngle2, valid.motorAngle2);
}

// The rhombus edge is addressable and a point just past it is not
TEST(GimbalToMotorAnglesSplineTest, RhombusEdgeIsAddressableAndJustOutsideIsNot) {
    const GimbalRhombusDomain domain = readDomain();
    for (const auto& [u, v] :
         {std::pair{1.0F, 0.3F}, std::pair{-1.0F, -0.2F}, std::pair{0.4F, 1.0F}, std::pair{-0.7F, -1.0F}}) {
        GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
        const auto onEdge = gimbalAnglesFromUv(domain, u, v);
        const auto edgeResult = alg.update(onEdge[0], onEdge[1]);
        EXPECT_NE(edgeResult.motorAngle1, kReferenceDefaultMotorAngle) << "u " << u << " v " << v;

        alg.reInitialize();
        const float outside = 1.0F + 1e-3F;
        const auto past =
            gimbalAnglesFromUv(domain, std::fabs(u) == 1.0F ? u * outside : u, std::fabs(v) == 1.0F ? v * outside : v);
        const auto pastResult = alg.update(past[0], past[1]);
        EXPECT_FLOAT_EQ(pastResult.motorAngle1, kReferenceDefaultMotorAngle) << "u " << u << " v " << v;
        EXPECT_FLOAT_EQ(pastResult.motorAngle2, kReferenceDefaultMotorAngle) << "u " << u << " v " << v;
    }
}

TEST(GimbalToMotorAnglesSplineTest, ReInitializeRestoresDefaultOutput) {
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    alg.update(0.0F, 0.0F);
    alg.reInitialize();
    const auto result = alg.update(0.0F, 90.0F * degToRad);

    EXPECT_FLOAT_EQ(result.motorAngle1, kReferenceDefaultMotorAngle);
    EXPECT_FLOAT_EQ(result.motorAngle2, kReferenceDefaultMotorAngle);
}

// ---------------------------------------------------------------------------
// Reconfiguration and setup tests (validation + round trip)
// ---------------------------------------------------------------------------

TEST(GimbalToMotorAnglesSplineTest, SetConfigReplacesSurfaceAndKeepsPreviousOutput) {
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());
    const auto fitted = alg.update(3.0F * degToRad, 2.0F * degToRad);

    SplineCoefficients constant{};
    constant.fill(1.25F);
    alg.setConfig(GimbalToMotorAnglesSplineConfig::create(readDomain(), constant));

    // The previous valid output survives the reconfiguration
    const auto held = alg.update(0.0F, 80.0F * degToRad);
    EXPECT_FLOAT_EQ(held.motorAngle1, fitted.motorAngle1);
    EXPECT_FLOAT_EQ(held.motorAngle2, fitted.motorAngle2);

    // The B-spline weights sum to one, so a constant coefficient set gives a constant surface
    const auto result = alg.update(3.0F * degToRad, 2.0F * degToRad);
    EXPECT_NEAR(result.motorAngle1, 1.25F, 1.25F * 20.0F * kEps);
    EXPECT_NEAR(result.motorAngle2, 1.25F, 1.25F * 20.0F * kEps);
}

TEST(GimbalToMotorAnglesSplineTest, ConfigRoundTrip) {
    const GimbalRhombusDomain domain = readDomain();
    const SplineCoefficients coefficients = readCoefficients();
    const auto config = GimbalToMotorAnglesSplineConfig::create(domain, coefficients);

    EXPECT_EQ(config.getDomain().centerTipAngle, domain.centerTipAngle);
    EXPECT_EQ(config.getDomain().centerTiltAngle, domain.centerTiltAngle);
    EXPECT_EQ(config.getDomain().halfDiagonalTip, domain.halfDiagonalTip);
    EXPECT_EQ(config.getDomain().halfDiagonalTilt, domain.halfDiagonalTilt);
    EXPECT_EQ(config.getCoefficients(), coefficients);
}

TEST(GimbalToMotorAnglesSplineTest, ConfigRejectsInvalidDomain) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
    const SplineCoefficients coefficients = readCoefficients();
    // Fields: center tip, center tilt, tip half-diagonal, tilt half-diagonal [rad]
    const auto domainOf = [](float tip, float tilt, float halfTip, float halfTilt) {
        return GimbalRhombusDomain{
            .centerTipAngle = tip, .centerTiltAngle = tilt, .halfDiagonalTip = halfTip, .halfDiagonalTilt = halfTilt};
    };
    EXPECT_NO_THROW(GimbalToMotorAnglesSplineConfig::create(domainOf(0.01F, 0.0F, 0.3F, 0.45F), coefficients));

    // The rhombus may reach, but not pass, the +-90 degree gimbal limits
    EXPECT_NO_THROW(GimbalToMotorAnglesSplineConfig::create(domainOf(0.0F, 0.0F, halfPi, halfPi), coefficients));

    const std::array invalidDomains{
        domainOf(nan, 0.0F, 0.3F, 0.45F),
        domainOf(0.01F, inf, 0.3F, 0.45F),
        domainOf(0.01F, 0.0F, nan, 0.45F),
        domainOf(0.01F, 0.0F, 0.3F, inf),
        domainOf(0.01F, 0.0F, 0.0F, 0.45F),
        domainOf(0.01F, 0.0F, 0.3F, 0.0F),
        domainOf(0.01F, 0.0F, -0.3F, 0.45F),
        domainOf(0.01F, 0.0F, 0.3F, -0.45F),
        domainOf(0.2F, 0.0F, halfPi - 0.1F, 0.45F),
        domainOf(-0.2F, 0.0F, halfPi - 0.1F, 0.45F),
        domainOf(0.01F, 0.2F, 0.3F, halfPi - 0.1F),
        domainOf(0.01F, -0.2F, 0.3F, halfPi - 0.1F),
    };
    for (const auto& domain : invalidDomains) {
        EXPECT_FALSE(GimbalToMotorAnglesSplineConfig::isValidDomain(domain));
        EXPECT_THROW(GimbalToMotorAnglesSplineConfig::create(domain, coefficients), fsw::invalid_argument);
    }
}

TEST(GimbalToMotorAnglesSplineTest, ConfigRejectsNonFiniteCoefficients) {
    const GimbalRhombusDomain domain = readDomain();
    for (const float bad : {std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity()}) {
        for (const std::size_t index :
             {std::size_t{0}, std::size_t{NUM_SPLINE_COEFFS / 2}, std::size_t{NUM_SPLINE_COEFFS - 1}}) {
            SplineCoefficients coefficients = readCoefficients();
            coefficients[index] = bad;
            EXPECT_FALSE(GimbalToMotorAnglesSplineConfig::isValidCoefficients(coefficients));
            EXPECT_THROW(GimbalToMotorAnglesSplineConfig::create(domain, coefficients), fsw::invalid_argument);
        }
    }
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// With the coefficients of each row set to the Greville abscissae, the surface is exactly S(u, v) = v, so
// motor 1 = v and motor 2 = u. This pins the basis weights and the coefficient index order.
TEST(GimbalToMotorAnglesSplineTest, PropertyReproducesLinearSurface) {
    SplineCoefficients greville{};
    for (int i = 0; i < NUM_SPLINE_COEFFS_PER_AXIS; ++i) {
        for (int j = 0; j < NUM_SPLINE_COEFFS_PER_AXIS; ++j) {
            greville[(i * NUM_SPLINE_COEFFS_PER_AXIS) + j] = -1.0F + (kIntervalWidth * static_cast<float>(j - 1));
        }
    }
    const GimbalRhombusDomain domain = readDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(GimbalToMotorAnglesSplineConfig::create(domain, greville));

    for (float u = -1.0F; u <= 1.0F; u += 0.0625F) {
        for (float v = -1.0F; v <= 1.0F; v += 0.0625F) {
            const auto angles = gimbalAnglesFromUv(domain, u, v);
            const auto result = alg.update(angles[0], angles[1]);
            EXPECT_NEAR(result.motorAngle1, v, 40.0F * kEps) << "u " << u << " v " << v;
            EXPECT_NEAR(result.motorAngle2, u, 40.0F * kEps) << "u " << u << " v " << v;
        }
    }
}

// Every output lies between the smallest and largest coefficient (B-spline convex-hull property)
TEST(GimbalToMotorAnglesSplineTest, PropertyOutputWithinCoefficientBounds) {
    const SplineCoefficients coefficients = readCoefficients();
    const auto [minCoeff, maxCoeff] = coefficientBounds(coefficients);
    const float slack = 20.5F * kEps * std::max(std::fabs(minCoeff), std::fabs(maxCoeff));
    const GimbalRhombusDomain domain = readDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());

    for (float u = -1.0F; u <= 1.0F; u += 0.025F) {
        for (float v = -1.0F; v <= 1.0F; v += 0.025F) {
            const auto angles = gimbalAnglesFromUv(domain, u, v);
            const auto result = alg.update(angles[0], angles[1]);
            EXPECT_GE(result.motorAngle1, minCoeff - slack);
            EXPECT_LE(result.motorAngle1, maxCoeff + slack);
            EXPECT_GE(result.motorAngle2, minCoeff - slack);
            EXPECT_LE(result.motorAngle2, maxCoeff + slack);
        }
    }
}

// Motor 2 is the mirror of motor 1 about the tilt center. With a zero tilt center, negating the tilt swaps u and v
// exactly, so the two outputs swap exactly.
TEST(GimbalToMotorAnglesSplineTest, PropertyMotorsAreMirrorImagesAboutTiltCenter) {
    const GimbalRhombusDomain domain = readDomain();
    ASSERT_EQ(domain.centerTiltAngle, 0.0F);
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());

    for (float tip = -17.0F; tip <= 17.0F; tip += 0.7F) {
        for (float tilt = -25.0F; tilt <= 25.0F; tilt += 0.9F) {
            alg.reInitialize();
            const auto result = alg.update(tip * degToRad, tilt * degToRad);
            alg.reInitialize();
            const auto mirrored = alg.update(tip * degToRad, -tilt * degToRad);
            EXPECT_EQ(result.motorAngle1, mirrored.motorAngle2) << "tip " << tip << " tilt " << tilt;
            EXPECT_EQ(result.motorAngle2, mirrored.motorAngle1) << "tip " << tip << " tilt " << tilt;
        }
    }
}

// The surface has no jump where the evaluation moves from one spline interval to the next
TEST(GimbalToMotorAnglesSplineTest, PropertySurfaceIsContinuousAcrossIntervalEdges) {
    const SplineCoefficients coefficients = readCoefficients();
    const float slope = surfaceSlopeBound(coefficients);
    const float delta = 1e-4F;  // [-] offset in u or v on each side of the knot
    // The mapped offset is delta to within ~10 eps, so the allowed change is the slope bound over 2 delta (+10%)
    // plus the evaluation error of both points.
    const float tolerance = (slope * 2.2F * delta) + (2.0F * evaluationTolerance(coefficients));
    const GimbalRhombusDomain domain = readDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(makeFittedConfig());

    for (int k = 1; k < NUM_SPLINE_INTERVALS; ++k) {
        const float knot = -1.0F + (kIntervalWidth * static_cast<float>(k));
        for (const float other : {-0.83F, -0.21F, 0.37F, 0.91F}) {
            const auto belowU = gimbalAnglesFromUv(domain, knot - delta, other);
            const auto aboveU = gimbalAnglesFromUv(domain, knot + delta, other);
            const auto belowV = gimbalAnglesFromUv(domain, other, knot - delta);
            const auto aboveV = gimbalAnglesFromUv(domain, other, knot + delta);
            const auto lowU = alg.update(belowU[0], belowU[1]);
            const auto highU = alg.update(aboveU[0], aboveU[1]);
            const auto lowV = alg.update(belowV[0], belowV[1]);
            const auto highV = alg.update(aboveV[0], aboveV[1]);
            EXPECT_NEAR(lowU.motorAngle1, highU.motorAngle1, tolerance) << "knot " << knot << " v " << other;
            EXPECT_NEAR(lowU.motorAngle2, highU.motorAngle2, tolerance) << "knot " << knot << " v " << other;
            EXPECT_NEAR(lowV.motorAngle1, highV.motorAngle1, tolerance) << "knot " << knot << " u " << other;
            EXPECT_NEAR(lowV.motorAngle2, highV.motorAngle2, tolerance) << "knot " << knot << " u " << other;
        }
    }
}
