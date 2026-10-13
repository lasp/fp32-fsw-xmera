#include "gimbalToMotorAnglesSplineTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

#include <cmath>

namespace {

/*! This method returns the configuration built from the fitted fixtures. The files are read once.
 @return const GimbalToMotorAnglesSplineConfig&
*/
const GimbalToMotorAnglesSplineConfig& fittedConfig() {
    static const GimbalToMotorAnglesSplineConfig config = makeFittedConfig();
    return config;
}

/*! This method checks that the output lies in the coefficient convex hull, with the fp32 evaluation slack.
 @param output [rad] Algorithm output
 @param coefficients [rad] Spline control coefficients
*/
void expectWithinCoefficientBounds(const GimbalToMotorAnglesSplineOutput& output,
                                   const SplineCoefficients& coefficients) {
    const auto [minCoeff, maxCoeff] = coefficientBounds(coefficients);
    const float slack = 20.5F * kEps * std::max(std::fabs(minCoeff), std::fabs(maxCoeff));
    EXPECT_GE(output.motorAngle1, minCoeff - slack);
    EXPECT_LE(output.motorAngle1, maxCoeff + slack);
    EXPECT_GE(output.motorAngle2, minCoeff - slack);
    EXPECT_LE(output.motorAngle2, maxCoeff + slack);
}

}  // namespace

// ---------------------------------------------------------------------------
// Property fuzz
// ---------------------------------------------------------------------------

// Any float input, including NaN, infinities, and huge values, gives a finite output: either the default motor angles
// or a surface value inside the coefficient convex hull
void propertyOutputIsFiniteForAnyInput(float gimbalAngle1, float gimbalAngle2) {
    GimbalToMotorAnglesSplineAlgorithm alg(fittedConfig());
    const auto output = alg.update(gimbalAngle1, gimbalAngle2);

    EXPECT_TRUE(std::isfinite(output.motorAngle1));
    EXPECT_TRUE(std::isfinite(output.motorAngle2));
    const bool isDefault =
        output.motorAngle1 == kReferenceDefaultMotorAngle && output.motorAngle2 == kReferenceDefaultMotorAngle;
    if (!isDefault) {
        expectWithinCoefficientBounds(output, fittedConfig().getCoefficients());
    }
}
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyOutputIsFiniteForAnyInput)
    .WithDomains(fuzztest::Arbitrary<float>(),   // gimbalAngle1 [rad]
                 fuzztest::Arbitrary<float>());  // gimbalAngle2 [rad]

// A request outside the rhombus returns the previous valid output
void propertyOutsideDomainHoldsPreviousOutput(float validU, float validV, float outsideU, float outsideV) {
    if (std::fabs(outsideU) <= 1.001F && std::fabs(outsideV) <= 1.001F) {
        return;
    }
    const GimbalRhombusDomain& domain = fittedConfig().getDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(fittedConfig());
    const auto valid = gimbalAnglesFromUv(domain, validU, validV);
    const auto validOutput = alg.update(valid[0], valid[1]);
    const auto outside = gimbalAnglesFromUv(domain, outsideU, outsideV);
    const auto heldOutput = alg.update(outside[0], outside[1]);

    EXPECT_EQ(heldOutput.motorAngle1, validOutput.motorAngle1);
    EXPECT_EQ(heldOutput.motorAngle2, validOutput.motorAngle2);
}
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyOutsideDomainHoldsPreviousOutput)
    .WithDomains(fuzztest::InRange(-1.0F, 1.0F),     // validU [-]
                 fuzztest::InRange(-1.0F, 1.0F),     // validV [-]
                 fuzztest::InRange(-50.0F, 50.0F),   // outsideU [-]
                 fuzztest::InRange(-50.0F, 50.0F));  // outsideV [-]

// Nearby requests inside the rhombus give nearby outputs; the change is bounded by the surface slope bound
void propertySurfaceIsLipschitz(float u, float v, float du, float dv) {
    const SplineCoefficients& coefficients = fittedConfig().getCoefficients();
    const float slope = surfaceSlopeBound(coefficients);
    const float u2 = std::clamp(u + du, -1.0F, 1.0F);
    const float v2 = std::clamp(v + dv, -1.0F, 1.0F);
    const GimbalRhombusDomain& domain = fittedConfig().getDomain();
    GimbalToMotorAnglesSplineAlgorithm alg(fittedConfig());
    const auto first = gimbalAnglesFromUv(domain, u, v);
    const auto second = gimbalAnglesFromUv(domain, u2, v2);
    const auto firstOutput = alg.update(first[0], first[1]);
    const auto secondOutput = alg.update(second[0], second[1]);

    // The mapped offsets equal the requested ones to within ~10 eps; 10% covers that
    const float tolerance =
        (slope * 1.1F * (std::fabs(u2 - u) + std::fabs(v2 - v))) + (2.0F * evaluationTolerance(coefficients));
    EXPECT_NEAR(firstOutput.motorAngle1, secondOutput.motorAngle1, tolerance);
    EXPECT_NEAR(firstOutput.motorAngle2, secondOutput.motorAngle2, tolerance);
}
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertySurfaceIsLipschitz)
    .WithDomains(fuzztest::InRange(-1.0F, 1.0F),     // u [-]
                 fuzztest::InRange(-1.0F, 1.0F),     // v [-]
                 fuzztest::InRange(-0.01F, 0.01F),   // du [-]
                 fuzztest::InRange(-0.01F, 0.01F));  // dv [-]

// Any valid configuration gives outputs inside its coefficient convex hull for requests inside its rhombus
void propertyRandomConfigOutputWithinCoefficientBounds(float centerTip,
                                                       float centerTilt,
                                                       float halfTip,
                                                       float halfTilt,
                                                       const SplineCoefficients& coefficients,
                                                       float u,
                                                       float v) {
    const GimbalRhombusDomain domain{.centerTipAngle = centerTip,
                                     .centerTiltAngle = centerTilt,
                                     .halfDiagonalTip = halfTip,
                                     .halfDiagonalTilt = halfTilt};
    GimbalToMotorAnglesSplineAlgorithm alg(GimbalToMotorAnglesSplineConfig::create(domain, coefficients));
    const auto angles = gimbalAnglesFromUv(domain, u, v);
    const auto output = alg.update(angles[0], angles[1]);

    expectWithinCoefficientBounds(output, coefficients);
}
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyRandomConfigOutputWithinCoefficientBounds)
    .WithDomains(fuzztest::InRange(-0.5F, 0.5F),                                          // centerTip [rad]
                 fuzztest::InRange(-0.5F, 0.5F),                                          // centerTilt [rad]
                 fuzztest::InRange(0.01F, 1.0F),                                          // halfTip [rad]
                 fuzztest::InRange(0.01F, 1.0F),                                          // halfTilt [rad]
                 fuzztest::ArrayOf<NUM_SPLINE_COEFFS>(fuzztest::InRange(-10.0F, 10.0F)),  // coefficients [rad]
                 fuzztest::InRange(-1.0F, 1.0F),                                          // u [-]
                 fuzztest::InRange(-1.0F, 1.0F));                                         // v [-]
