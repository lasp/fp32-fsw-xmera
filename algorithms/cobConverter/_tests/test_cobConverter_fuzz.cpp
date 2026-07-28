#include "cobConverterTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

#include <vector>

namespace {

// attitudeCovariance is the filter's attitude-knowledge covariance, in rad^2.
// this builds a genuine dense symmetric PSD matrix via A^T*A + eps*I -- the same construction
// used below for filterVehPositionCovarianceDomain
// A's elements span the same [0, pi]
// rad magnitude as the previous isotropic sigma bound (perfect knowledge up to full/no knowledge);
// squaring via A^T*A keeps the result in variance units (rad^2), and the small +eps*I term
// guarantees strict positive-definiteness (never exactly singular).
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

// bodyToCameraMrp is a fixed camera-mounting orientation, so it should range over arbitrary
// rotations rather than stay near identity. Bounding each component to +/-1/sqrt(3) keeps the
// worst-case (corner) MRP norm at exactly 1, so every sample lands in the standard "short
// rotation" MRP convention (norm <= 1, angle <= 180 deg) -- every physical rotation has such a
// representative, so this loses no orientations, unlike a wide unbounded-looking component range
// (e.g. +/-1e6), which would instead concentrate almost all samples near a ~360 deg rotation
// (practically identity, via the shadow set) since norm = tan(angle/4) saturates once any
// component exceeds ~10-100.
// Also reused for sigma_BN below: the spacecraft body-to-inertial attitude is likewise an
// arbitrary (not necessarily near-identity) orientation, so the same "any physical rotation, no
// shadow-set switch needed" reasoning applies.
const float kMrpComponentBound = 1.0F / safeSqrtf(3.0F);
auto arbitraryMrpDomain() { return xmera::fuzz::Vector3fInRange(-kMrpComponentBound, kMrpComponentBound); }

constexpr bool kCobValid = true;
// cobPixelsFound is a count of bright pixels detected by upstream image processing, so it can
// never be negative. 0 is included deliberately: combined with cobValid=true it exercises the
// "no pixels found" branch in updateState (input.cobPixelsFound != 0), which still produces a
// zeroed/invalid output rather than a divide-by-zero -- scaleFactor's argument is 0/kSphereSolidAngle,
// and safeSqrtf(0) is well-defined. The upper bound (1e6) covers a detection filling a large
// fraction of even the widest fuzzed resolution (8192x8192) without being physically absurd.
auto cobPixelsFoundDomain() { return fuzztest::InRange(0, 1000000); }
// cobCenterOfBrightness is a pixel coordinate reported by upstream image processing. It's not
// validated against resolutionX/resolutionY here (no such check exists in CobConverterConfig), so
// this ranges over the same [0, 8192] span as the widest resolution fuzzed above rather than being
// coupled to the per-sample resolutionX/resolutionY -- covers on-image detections at any supported
// resolution plus some off-image/edge values, without requiring a dependent domain.
auto cobCenterOfBrightnessDomain() { return xmera::fuzz::EigenVectorOf<float, 2>(fuzztest::InRange(0.0F, 8192.0F)); }
// filterVehPosition is the spacecraft's inertial position relative to the target body. Its norm
// (spacecraftRange) is a divisor throughout computePhaseAngleCorrection/computeCameraFrameUncertainty
// (Rc, constants_deltaR, ...), so sampling each component independently over a range spanning zero
// risks landing near the origin -- and at exactly (0,0,0), stableNorm()/stableNormalized() produce
// 0/0 = NaN on both the algorithm and the double reference identically, which EXPECT_NEAR/EXPECT_LE
// can never accept even though both sides agree. Bound each component's magnitude away from zero
// instead (never in (-1e3, 1e3)) so the vector's norm is always >= 1e3, no normalization needed to
// guarantee it: 1e3 to 1e8 m covers close proximity operations (~1 km) out to interplanetary
// approach/cruise (~100,000 km) for a small-body encounter.
auto nonZeroAxisDomain(double minAbs, double maxAbs) {
    return fuzztest::OneOf(fuzztest::InRange(-maxAbs, -minAbs), fuzztest::InRange(minAbs, maxAbs));
}
auto filterVehPositionDomain() { return xmera::fuzz::EigenVectorOf<double, 3>(nonZeroAxisDomain(1.0e2, 1.0e11)); }
// filterVehPositionCovariance is the nav filter's position-uncertainty covariance for
// filterVehPosition, in m^2, propagated through computeCameraFrameUncertainty's partials
// (deltaBinaryDeltaR * positionCovar * deltaBinaryDeltaR^T) to build the COM covariance. Real OD
// covariance is anisotropic and correlated (line-of-sight uncertainty typically dwarfs cross-track,
// with nonzero off-diagonal terms), not diagonal, so this builds a genuine dense symmetric PSD
// matrix via A^T*A + eps*I -- the same construction timeClosestApproach's fuzz test uses for its
// 6x6 covariance (timeClosestApproach/_tests/test_timeClosestApproach_fuzz.cpp). A's elements span
// the same 1-sigma position-uncertainty range as filterVehPosition's magnitude bound above
// (sub-meter radar/lidar-aided proximity ops up to ~1e5 m during early approach); squaring via
// A^T*A keeps the result in variance units (m^2), and the small +eps*I term guarantees strict
// positive-definiteness (never exactly singular) without perturbing physically meaningful
// magnitudes.
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
        fuzztest::InRange(1.0F, 1.0e6F),  // radius [m]: ~1 m (small NEO) to ~500 km (Ceres-scale)
        fuzztest::InRange(0.0F, 1.0e6F),  // radiusUncertainty [m]: perfectly known to Ceres-scale error
        attitudeCovarianceDomain(),       // attitudeCovariance
        numStandardDeviationsDomain(),    // numStandardDeviations
        StandardDeviationsDomain(),       // standardDeviation
        fuzztest::OneOf(fuzztest::Just(true),
                        fuzztest::Just(false)),  // specifiedStandardDeviation
        fuzztest::OneOf(fuzztest::Just(true),
                        fuzztest::Just(false)),  // outlierDetectionEnabled
        calibrationCoefficientsDomain(),         // calibrationCoefficients
        fuzztest::Arbitrary<int>(),              // cameraId (unconstrained: no isValidCameraId check)
        fuzztest::InRange(0.175F, 1.5533F),      // fieldOfViewX [rad]: ~10 deg (narrow) to 89 deg (wide-angle)
        fuzztest::InRange(0.175F, 1.5533F),      // fieldOfViewY [rad]: independent of fieldOfViewX
        fuzztest::InRange(32.0F, 8192.0F),       // resolutionX [px]: small nav camera to large science imager
        fuzztest::InRange(32.0F, 8192.0F),       // resolutionY [px]: independent of resolutionX (see below)
        arbitraryMrpDomain(),                    // bodyToCameraMrp
        fuzztest::OneOf(fuzztest::Just(true),
                        fuzztest::Just(false)),  // cobValid
        cobPixelsFoundDomain(),                  // cobPixelsFound
        cobCenterOfBrightnessDomain(),           // cobCenterOfBrightness
        fuzztest::Arbitrary<uint64_t>(),         // cobTimeTag
        arbitraryMrpDomain(),                    // sigma_BN
        arbitraryMrpDomain(),                    // vehSunPntBdy
        filterVehPositionDomain(),               // filterVehPosition
        filterVehPositionCovarianceDomain());    // filterVehPositionCovariance
