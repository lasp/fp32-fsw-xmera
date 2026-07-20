// Property-based fuzz tests for SunlineFilterAlgorithm. Each FUZZ_TEST drives one
// of the property helpers in sunlineFilterTestHelpers.hpp over randomized inputs;
// the helpers guard unusable samples with an early return so the harness drops
// them silently.

#include "sunlineFilterTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

#include <Eigen/Core>
#include <limits>

namespace {

using filtering::sunlineFilter::MaxCss;
using filtering::sunlineFilter::Vector7;

// A measurement value that is either bounded-finite (the normal case) or one of the non-finite
// specials. It deliberately excludes absurd-magnitude finite values (e.g. 1e308): the filter only
// guarantees that *non-finite* inputs are skipped, not that arbitrarily large finite measurements
// avoid overflow.
inline auto boundedOrNonFinite() {
    return fuzztest::OneOf(fuzztest::InRange(-2.0, 2.0),
                           fuzztest::ElementOf<double>({std::numeric_limits<double>::quiet_NaN(),
                                                        std::numeric_limits<double>::infinity(),
                                                        -std::numeric_limits<double>::infinity()}));
}

// Thin free-function wrappers so FUZZ_TEST references an unqualified name (the property
// helpers themselves live in namespace filtering::sunlineFilter).
inline void fuzzUpdateKeepsStateValidAndBounded(Eigen::Vector3d sHatRaw,
                                                Eigen::Vector3d omega,
                                                double bias,
                                                Vector7 covDiag,
                                                Eigen::Vector<double, MaxCss> cssCos,
                                                Eigen::Vector3d rate,
                                                double dt) {
    filtering::sunlineFilter::propertyUpdateKeepsStateValidAndBounded(sHatRaw, omega, bias, covDiag, cssCos, rate, dt);
}

inline void fuzzArbitraryMeasurementsPreserveState(Eigen::Vector3d sHatRaw,
                                                   Eigen::Vector3d omega,
                                                   double bias,
                                                   Vector7 covDiag,
                                                   Eigen::Vector<double, MaxCss> cssCos,
                                                   Eigen::Vector3d rate,
                                                   double dt) {
    filtering::sunlineFilter::propertyArbitraryMeasurementsPreserveState(
        sHatRaw, omega, bias, covDiag, cssCos, rate, dt);
}

inline void fuzzRateMeasurementDoesNotIncreaseCovariance(Eigen::Vector3d sHatRaw,
                                                         Eigen::Vector3d omega,
                                                         double bias,
                                                         Vector7 covDiag,
                                                         Eigen::Vector3d rate) {
    filtering::sunlineFilter::propertyRateMeasurementDoesNotIncreaseCovariance(sHatRaw, omega, bias, covDiag, rate);
}

}  // namespace

// Heading direction, body rate, bias, initial-covariance diagonal, CSS cos-values, gyro rate, and
// the propagation time are all finite and bounded. The helper builds a valid config and checks the
// post-update estimate stays finite, unit-norm, bias-bounded, and symmetric.
FUZZ_TEST(SunlineFilterPropertyFuzz, fuzzUpdateKeepsStateValidAndBounded)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 fuzztest::InRange(0.0, 3.0),
                 xmera::fuzz::EigenVectorOf<double, 7>(fuzztest::InRange(-1.0, 1.0)),
                 xmera::fuzz::EigenVectorOf<double, MaxCss>(fuzztest::InRange(-1.0, 1.0)),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 fuzztest::InRange(0.0, 100.0));

// Same finite config inputs, but the CSS cos-values and gyro rate are fully arbitrary (including
// NaN / Inf). A non-finite measurement must be skipped, leaving the estimate finite and unit-norm.
FUZZ_TEST(SunlineFilterPropertyFuzz, fuzzArbitraryMeasurementsPreserveState)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 fuzztest::InRange(0.0, 3.0),
                 xmera::fuzz::EigenVectorOf<double, 7>(fuzztest::InRange(-1.0, 1.0)),
                 xmera::fuzz::EigenVectorOf<double, MaxCss>(boundedOrNonFinite()),
                 xmera::fuzz::EigenVectorOf<double, 3>(boundedOrNonFinite()),
                 fuzztest::InRange(0.0, 100.0));

// A rate measurement folded in with no time propagation never increases the covariance trace.
FUZZ_TEST(SunlineFilterPropertyFuzz, fuzzRateMeasurementDoesNotIncreaseCovariance)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 fuzztest::InRange(0.0, 3.0),
                 xmera::fuzz::EigenVectorOf<double, 7>(fuzztest::InRange(-1.0, 1.0)),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0));
