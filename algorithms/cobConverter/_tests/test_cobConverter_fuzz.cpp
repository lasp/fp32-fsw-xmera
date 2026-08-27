#include "cobConverterTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

#include <vector>

namespace {

// attitudeCovariance: real attitude-knowledge covariance is dense and correlated, not diagonal, so
// build a genuine symmetric PSD matrix via A^T*A + eps*I (A spans +/-pi rad, matching the old
// isotropic sigma bound); the eps*I term keeps it strictly positive-definite.
auto attitudeCovarianceDomain() {
    return fuzztest::Map(
        [](const std::vector<float>& elems) -> Eigen::Matrix3f {
            const Eigen::Matrix3f A = Eigen::Map<const Eigen::Matrix3f>(elems.data());
            return (A.transpose() * A) + (1.0e-6F * Eigen::Matrix3f::Identity());
        },
        fuzztest::VectorOf(fuzztest::InRange(-3.14F, 3.14F)).WithSize(9));
}

// numStandardDeviations is the n-sigma outlier-gating threshold
auto numStandardDeviationsDomain() { return fuzztest::InRange(0.0F, 100.0F); }
// standardDeviation is the fixed 1-sigma pixel-space error bound used directly as the
// outlier-gating sigma when specifiedStandardDeviation=true.
auto StandardDeviationsDomain() { return fuzztest::InRange(0.0F, 1000.0F); }
// Brown-Conrady distortion coefficients (normalized image-plane units)
auto calibrationCoefficientsDomain() {
    return fuzztest::StructOf<CalibrationCoefficients>(fuzztest::InRange(-10.0F, 10.0F),  // k1
                                                       fuzztest::InRange(-10.0F, 10.0F),  // k2
                                                       fuzztest::InRange(-10.0F, 10.0F),  // k3
                                                       fuzztest::InRange(-10.0F, 10.0F),  // p1
                                                       fuzztest::InRange(-10.0F, 10.0F)   // p2
    );
}

// bodyToCameraMrp/sigma_BN (reused below): arbitrary, not-necessarily-near-identity orientations.
// +/-1/sqrt(3) per component caps the worst-case MRP norm at 1, the standard "short rotation"
// convention (angle <= 180 deg) that every physical rotation has a representative in. A much wider
// range (e.g. +/-1e6) would instead concentrate samples near ~360 deg (practically identity, via
// the shadow set), since norm = tan(angle/4) saturates once a component exceeds ~10-100.
const float kMrpComponentBound = 1.0F / safeSqrtf(3.0F);
auto arbitraryMrpDomain() { return xmera::fuzz::Vector3fInRange(-kMrpComponentBound, kMrpComponentBound); }

// vehSunPntBdy: sun direction, a body-frame unit vector. Parameterizing the unit sphere by
// (z, azimuth) gives an exactly unit-length vector without normalizing near zero (which could
// hit float32 subnormals and diverge from the double reference).
auto arbitraryUnitVectorDomain() {
    constexpr float kPi = 3.14159265358979323846F;

    return fuzztest::Map(
        [](float z, float azimuth) -> Eigen::Vector3f {
            const float radial = safeSqrtf(1.0F - z * z);
            return Eigen::Vector3f{
                radial * std::cos(azimuth),
                radial * std::sin(azimuth),
                z,
            };
        },
        fuzztest::InRange(-1.0F, 1.0F),
        fuzztest::InRange(-kPi, kPi));
}

// cobPixelsFound: bright-pixel count, never negative. 0 is included deliberately (with
// cobValid=true) to exercise the "no pixels found" branch without a divide-by-zero -- both
// scaleFactor's argument and safeSqrtf(0) are well-defined at 0. 1e6 covers a detection filling
// most of the widest fuzzed resolution (8192x8192) without being physically absurd.
auto cobPixelsFoundDomain() { return fuzztest::InRange(0, 1000000); }

// cobCenterOfBrightness: a pixel coordinate, not validated against resolutionX/resolutionY (no
// such check exists in CobConverterConfig), so it ranges over [0, 8192] independent of the
// per-sample resolution -- covers on-image detections at any resolution plus some off-image values.
auto cobCenterOfBrightnessDomain() { return xmera::fuzz::EigenVectorOf<float, 2>(fuzztest::InRange(0.0F, 8192.0F)); }

// filterVehPosition: spacecraft position relative to the target. Its norm is a divisor throughout
// the algorithm, and sampling near the origin risks 0/0 = NaN on both the algorithm and the double
// reference identically -- which EXPECT_NEAR/EXPECT_LE can never accept even though both sides
// agree. Bound each component's magnitude away from zero instead, guaranteeing norm >= 1e3: 1e3 to
// 1e8 m covers close proximity (~1 km) to interplanetary approach/cruise (~100,000 km).
auto nonZeroAxisDomain(double minAbs, double maxAbs) {
    return fuzztest::OneOf(fuzztest::InRange(-maxAbs, -minAbs), fuzztest::InRange(minAbs, maxAbs));
}
auto filterVehPositionDomain() { return xmera::fuzz::EigenVectorOf<double, 3>(nonZeroAxisDomain(1.0, 1.0e11)); }

// filterVehPositionCovariance: nav-filter position uncertainty for filterVehPosition, propagated
// into the COM covariance. Real OD covariance is anisotropic and correlated, not diagonal, so build
// a genuine symmetric PSD matrix via A^T*A + eps*I (same construction as timeClosestApproach's fuzz
// test), with A spanning the same magnitude range as filterVehPosition's 1-sigma uncertainty; the
// eps*I term keeps it strictly positive-definite.
auto filterVehPositionCovarianceDomain() {
    return fuzztest::Map(
        [](const std::vector<double>& elems) -> Eigen::Matrix3d {
            const Eigen::Matrix3d A = Eigen::Map<const Eigen::Matrix3d>(elems.data());
            return (A.transpose() * A) + (1.0e-6 * Eigen::Matrix3d::Identity());
        },
        fuzztest::VectorOf(fuzztest::InRange(-1.0e5, 1.0e5)).WithSize(9));
}

}  // namespace

FUZZ_TEST(CobConverterFuzz, testCobConverter)
    .WithDomains(
        fuzztest::OneOf(fuzztest::Just(PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg),
                        fuzztest::Just(PhaseAngleCorrectionMethodAlgorithm::BinaryAlg)),  // phaseAngleCorrectionMethod
        fuzztest::InRange(1.0F, 1.0e6F),                                                  // radius [m]
        fuzztest::InRange(0.0F, 1.0e6F),                                                  // radiusUncertainty [m]
        attitudeCovarianceDomain(),                                                       // attitudeCovariance
        numStandardDeviationsDomain(),                                                    // numStandardDeviations
        StandardDeviationsDomain(),                                                       // standardDeviation
        fuzztest::OneOf(fuzztest::Just(true),
                        fuzztest::Just(false)),  // specifiedStandardDeviation
        fuzztest::OneOf(fuzztest::Just(true),
                        fuzztest::Just(false)),  // outlierDetectionEnabled
        calibrationCoefficientsDomain(),         // calibrationCoefficients
        fuzztest::Arbitrary<int>(),              // cameraId (unconstrained: no isValidCameraId check)
        fuzztest::InRange(0.175F, 2.967F),       // fieldOfViewX [rad]: ~10 deg (narrow) to ~170 deg (wide-angle)
        fuzztest::InRange(0.175F, 2.967F),       // fieldOfViewY [rad]: independent of fieldOfViewX
        fuzztest::InRange(32.0F, 8192.0F),       // resolutionX [px]: small nav camera to large science imager
        fuzztest::InRange(32.0F, 8192.0F),       // resolutionY [px]: independent of resolutionX (see below)
        arbitraryMrpDomain(),                    // bodyToCameraMrp
        fuzztest::OneOf(fuzztest::Just(true),
                        fuzztest::Just(false)),  // cobValid
        cobPixelsFoundDomain(),                  // cobPixelsFound
        cobCenterOfBrightnessDomain(),           // cobCenterOfBrightness
        fuzztest::Arbitrary<uint64_t>(),         // cobTimeTag
        arbitraryMrpDomain(),                    // sigma_BN
        arbitraryUnitVectorDomain(),             // vehSunPntBdy
        filterVehPositionDomain(),               // filterVehPosition
        filterVehPositionCovarianceDomain());    // filterVehPositionCovariance
