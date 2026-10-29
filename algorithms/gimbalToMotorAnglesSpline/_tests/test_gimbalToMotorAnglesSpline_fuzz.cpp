#include "gimbalToMotorAnglesSplineTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

namespace {

// [steps] Largest coefficient magnitude that the configuration accepts
constexpr auto kMaxCoefficient = static_cast<float>(GimbalToMotorAnglesSplineConfig::kMaxStepsPerRev);

/*! This method returns a coefficient perturbation domain: a magnitude between the smallest and largest perturbation,
 with either sign.
 @return fuzztest::Domain<float> [steps]
*/
fuzztest::Domain<float> coefficientPerturbation() {
    return fuzztest::OneOf(fuzztest::InRange(-kMaxCoefficientPerturbation, -kMinCoefficientPerturbation),
                           fuzztest::InRange(kMinCoefficientPerturbation, kMaxCoefficientPerturbation));
}

}  // namespace

// ---------------------------------------------------------------------------
// Property fuzz
// ---------------------------------------------------------------------------

// Any float input, including NaN, infinities, and huge values, gives a finite output: either the initial output of
// the algorithm or motor steps inside the coefficient convex hull of each motor
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyOutputIsFiniteForAnyInput)
    .WithDomains(fuzztest::Arbitrary<float>(),   // gimbalAngle1 [rad]
                 fuzztest::Arbitrary<float>());  // gimbalAngle2 [rad]

// A request outside the rhombus returns the previous valid output
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyOutsideDomainHoldsPreviousOutput)
    .WithDomains(fuzztest::InRange(-1.0F, 1.0F),     // validU [-]
                 fuzztest::InRange(-1.0F, 1.0F),     // validV [-]
                 fuzztest::InRange(-50.0F, 50.0F),   // outsideU [-]
                 fuzztest::InRange(-50.0F, 50.0F));  // outsideV [-]

// Nearby requests inside the rhombus give nearby outputs; the change is bounded by the slope bound of each surface
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertySurfaceIsLipschitz)
    .WithDomains(fuzztest::InRange(-1.0F, 1.0F),     // u [-]
                 fuzztest::InRange(-1.0F, 1.0F),     // v [-]
                 fuzztest::InRange(-0.01F, 0.01F),   // du [-]
                 fuzztest::InRange(-0.01F, 0.01F));  // dv [-]

// Any valid configuration gives outputs inside the coefficient convex hull of each motor for requests inside its
// rhombus. The coefficients cover the whole range that the configuration accepts.
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyOutputWithinCoefficientBounds)
    .WithDomains(fuzztest::InRange(-0.5F, 0.5F),  // centerTip [rad]
                 fuzztest::InRange(-0.5F, 0.5F),  // centerTilt [rad]
                 fuzztest::InRange(0.01F, 1.0F),  // halfTip [rad]
                 fuzztest::InRange(0.01F, 1.0F),  // halfTilt [rad]
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxCoefficient, kMaxCoefficient))
                     .WithSize(NUM_SPLINE_COEFFS),  // motor 1 coefficients [steps]
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxCoefficient, kMaxCoefficient))
                     .WithSize(NUM_SPLINE_COEFFS),  // motor 2 coefficients [steps]
                 fuzztest::InRange(-1.0F, 1.0F),    // u [-]
                 fuzztest::InRange(-1.0F, 1.0F));   // v [-]

// The fitted coefficients minimize the sum of the squared errors to the measured table, so changing any set of
// coefficients of either motor gives a larger error. A zero entry leaves that coefficient unchanged. Each perturbation
// is a fixed-size vector, not ArrayOf<NUM_SPLINE_COEFFS>: ArrayOf copies its element domain once per entry, and
// hundreds of nested OneOf domains stall the compiler.
FUZZ_TEST(GimbalToMotorAnglesSplineAlgorithmFuzz, propertyPerturbedCoefficientsIncreaseTableError)
    .WithDomains(fuzztest::VectorOf(fuzztest::OneOf(fuzztest::Just(0.0F), coefficientPerturbation()))
                     .WithSize(NUM_SPLINE_COEFFS),  // motor 1 perturbation [steps]
                 fuzztest::VectorOf(fuzztest::OneOf(fuzztest::Just(0.0F), coefficientPerturbation()))
                     .WithSize(NUM_SPLINE_COEFFS));  // motor 2 perturbation [steps]
