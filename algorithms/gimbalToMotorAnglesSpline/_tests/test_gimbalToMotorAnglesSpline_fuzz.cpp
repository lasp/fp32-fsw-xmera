#include "gimbalToMotorAnglesSplineTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

namespace {

// [steps] Largest coefficient magnitude that the configuration accepts
constexpr auto kMaxCoefficient = static_cast<float>(kMaxStepsPerRev);
// [rad] Upper end of the directions of the requests in the circle
constexpr float kFullTurn = 2.0F * std::numbers::pi_v<float>;

}  // namespace

// ---------------------------------------------------------------------------
// Property fuzz
// ---------------------------------------------------------------------------

// Any float input, including NaN, infinities, and huge values, gives a finite output: either the initial output of
// the algorithm or motor steps inside the coefficient convex hull of each motor
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyOutputIsFiniteForAnyInput)
    .WithDomains(fuzztest::Arbitrary<float>(),   // gimbalAngle1 [rad]
                 fuzztest::Arbitrary<float>());  // gimbalAngle2 [rad]

// A request outside the rhombus returns the previous valid output. The valid request is in the circle of radius
// thetaMax.
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyOutsideDomainHoldsPreviousOutput)
    .WithDomains(fuzztest::InRange(0.0F, 1.0F),       // validRadiusFraction [-]
                 fuzztest::InRange(0.0F, kFullTurn),  // validDirection [rad]
                 fuzztest::InRange(-50.0F, 50.0F),    // outsideU [-]
                 fuzztest::InRange(-50.0F, 50.0F));   // outsideV [-]

// The linear test coefficients reproduce the surfaces kLinearSurfaceScale * u and kLinearSurfaceScale * v at any
// point of [-1, 1] x [-1, 1], edges included
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyReproducesLinearSurface)
    .WithDomains(fuzztest::InRange(-1.0F, 1.0F),   // u [-]
                 fuzztest::InRange(-1.0F, 1.0F));  // v [-]

// Nearby points give nearby values of the rough test surfaces; the change is bounded by the slope bound of each surface
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertySurfaceIsLipschitz)
    .WithDomains(fuzztest::InRange(-1.0F, 1.0F),     // u [-]
                 fuzztest::InRange(-1.0F, 1.0F),     // v [-]
                 fuzztest::InRange(-0.01F, 0.01F),   // du [-]
                 fuzztest::InRange(-0.01F, 0.01F));  // dv [-]

// Any coefficients give surface values inside their convex hull at any point of [-1, 1] x [-1, 1]. The coefficients
// cover the whole range that the configuration accepts.
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertySurfaceWithinCoefficientBounds)
    .WithDomains(fuzztest::ArrayOf<NUM_SPLINE_COEFFS>(fuzztest::InRange(-kMaxCoefficient,
                                                                        kMaxCoefficient)),  // coefficients [steps]
                 fuzztest::InRange(-1.0F, 1.0F),                                            // u [-]
                 fuzztest::InRange(-1.0F, 1.0F));                                           // v [-]
