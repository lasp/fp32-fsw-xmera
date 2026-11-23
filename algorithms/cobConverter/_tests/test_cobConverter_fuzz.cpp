#include "cobConverterTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

#include <algorithm>
#include <cmath>
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
// outlierRadiusMultiple is the number of body radii in the outlier gate.
auto outlierRadiusMultipleDomain() { return fuzztest::InRange(0.5F, 10.0F); }
// Brown-Conrady distortion coefficients (normalized image-plane units). Ranges bracket typical
// OpenCV/MATLAB calibrations with margin, rather than arbitrary values that put the polynomial far
// outside any real lens.
auto calibrationCoefficientsDomain() {
    return fuzztest::StructOf<CalibrationCoefficients>(
        fuzztest::InRange(-1.0F, 1.0F),      // k1: dominant radial term; real lenses ~-0.5 (barrel) to +0.3
        fuzztest::InRange(-1.0F, 1.0F),      // k2: higher-order radial; typically |k2| <~ 1, often opposite k1
        fuzztest::InRange(-1.0F, 1.0F),      // k3: highest-order radial; small, often fixed at 0 in calibration
        fuzztest::InRange(-0.001F, 0.001F),  // p1: tangential (lens decentering); real ~1e-4..1e-3
        fuzztest::InRange(-0.001F, 0.001F)   // p2: tangential; +/-1e-3 keeps the tangential fold
                                             //     (|x_u| ~ 1/(6|p|) ~ 167)
    );
}

// Pincushion-only coefficients: k >= 0 keeps the radial slope >= 1, so the distortion never folds.
auto pincushionCoefficientsDomain() {
    return fuzztest::StructOf<CalibrationCoefficients>(fuzztest::InRange(0.0F, 1.0F),       // k1
                                                       fuzztest::InRange(0.0F, 1.0F),       // k2
                                                       fuzztest::InRange(0.0F, 1.0F),       // k3
                                                       fuzztest::InRange(-0.001F, 0.001F),  // p1
                                                       fuzztest::InRange(-0.001F, 0.001F)   // p2
    );
}

// bodyToCameraMrp/sigma_BN (reused below): arbitrary, not-necessarily-near-identity orientations.
// +/-1/sqrt(3) per component caps the worst-case MRP norm at 1, the standard "short rotation"
// convention (angle <= 180 deg) that every physical rotation has a representative in. A much wider
// range (e.g. +/-1e6) would instead concentrate samples near ~360 deg (practically identity, via
// the shadow set), since norm = tan(angle/4) saturates once a component exceeds ~10-100.
const float kMrpComponentBound = 1.0F / safeSqrtf(3.0F);
auto arbitraryMrpDomain() { return xmera::fuzz::Vector3fInRange(-kMrpComponentBound, kMrpComponentBound); }

// rHat_SB_B: sun direction, a body-frame unit vector. Parameterizing the unit sphere by
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

// cobPixelsFound: bright-pixel count. Non-positive values are included to exercise the rejection
// gate. 1e6 covers a large detection (~1000x1000 px) without being physically absurd.
auto cobPixelsFoundDomain() { return fuzztest::InRange(-1000000, 1000000); }

// cobCenterOfBrightness is fuzzed as a fraction of the image in [0, 1]^2 and scaled by the
// per-sample resolution (see testCobConverterOnSensor), so every detection lies on the sensor:
// 0 <= u <= resolutionX and 0 <= v <= resolutionY.
auto cobImageFractionDomain() { return xmera::fuzz::EigenVectorOf<float, 2>(fuzztest::InRange(0.0F, 1.0F)); }

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

// testCobConverter with the COB given as a fraction of the image, mapped to an on-sensor pixel.
void testCobConverterOnSensor(float radius,
                              float radiusUncertainty,
                              const Eigen::Matrix3f& attitudeCovariance,
                              float numStandardDeviations,
                              float outlierRadiusMultiple,
                              bool outlierDetectionEnabled,
                              const CalibrationCoefficients& calibrationCoefficients,
                              int cameraId,
                              float fieldOfViewX,
                              float fieldOfViewY,
                              float resolutionX,
                              float resolutionY,
                              const Eigen::Vector3f& bodyToCameraMrp,
                              bool cobValid,
                              int32_t cobPixelsFound,
                              const Eigen::Vector2f& cobImageFraction,
                              uint64_t cobTimeTag,
                              const Eigen::Vector3f& sigma_BN,
                              const Eigen::Vector3f& rHat_SB_B,
                              const Eigen::Vector3d& filterVehPosition,
                              const Eigen::Matrix3d& filterVehPositionCovariance) {
    const Eigen::Vector2f cobCenterOfBrightness =
        cobImageFraction.cwiseProduct(Eigen::Vector2f{resolutionX, resolutionY});
    testCobConverter(radius,
                     radiusUncertainty,
                     attitudeCovariance,
                     numStandardDeviations,
                     outlierRadiusMultiple,
                     outlierDetectionEnabled,
                     calibrationCoefficients,
                     cameraId,
                     fieldOfViewX,
                     fieldOfViewY,
                     resolutionX,
                     resolutionY,
                     bodyToCameraMrp,
                     cobValid,
                     cobPixelsFound,
                     cobCenterOfBrightness,
                     cobTimeTag,
                     sigma_BN,
                     rHat_SB_B,
                     filterVehPosition,
                     filterVehPositionCovariance);
}

}  // namespace

FUZZ_TEST(CobConverterFuzz, testCobConverterOnSensor)
    .WithDomains(fuzztest::OneOf(fuzztest::Just(0.0F),
                                 fuzztest::InRange(1.0F, 1.0e6F)),  // radius [m]; 0 disables the correction
                 fuzztest::InRange(0.0F, 1.0e6F),                   // radiusUncertainty [m]
                 attitudeCovarianceDomain(),                        // attitudeCovariance
                 numStandardDeviationsDomain(),                     // numStandardDeviations
                 outlierRadiusMultipleDomain(),                     // outlierRadiusMultiple
                 fuzztest::OneOf(fuzztest::Just(true),
                                 fuzztest::Just(false)),  // outlierDetectionEnabled
                 calibrationCoefficientsDomain(),         // calibrationCoefficients
                 fuzztest::Arbitrary<int>(),              // cameraId (unconstrained: no isValidCameraId check)
                 fuzztest::InRange(CobConverterConfig::kMinFieldOfView,
                                   CobConverterConfig::kMaxFieldOfView),  // fieldOfViewX [rad]: 0.1 to 90 deg
                 fuzztest::InRange(CobConverterConfig::kMinFieldOfView,
                                   CobConverterConfig::kMaxFieldOfView),  // fieldOfViewY [rad]: independent of X
                 fuzztest::InRange(CobConverterConfig::kMinResolution,
                                   CobConverterConfig::kMaxResolution),  // resolutionX [px]: 1 to 10000
                 fuzztest::InRange(CobConverterConfig::kMinResolution,
                                   CobConverterConfig::kMaxResolution),  // resolutionY [px]: independent of X
                 arbitraryMrpDomain(),                                   // bodyToCameraMrp
                 fuzztest::OneOf(fuzztest::Just(true),
                                 fuzztest::Just(false)),  // cobValid
                 cobPixelsFoundDomain(),                  // cobPixelsFound
                 cobImageFractionDomain(),                // cobImageFraction -> on-sensor cobCenterOfBrightness
                 fuzztest::Arbitrary<uint64_t>(),         // cobTimeTag
                 arbitraryMrpDomain(),                    // sigma_BN
                 arbitraryUnitVectorDomain(),             // rHat_SB_B
                 filterVehPositionDomain(),               // filterVehPosition
                 filterVehPositionCovarianceDomain());    // filterVehPositionCovariance

namespace {

// Round trip: the double-precision forward model applied to the solution must reproduce the input.
// Tolerance scales with the magnitudes the fp32 solver combines, floored at 1.
constexpr double kRoundTripRtol = 1e-4;
void expectUndistortRoundTrip(float xDistorted,
                              float yDistorted,
                              const CalibrationCoefficients& coefficients,
                              const UndistortedCoordinate& result) {
    const double xu = static_cast<double>(result.xUndistorted);
    const double yu = static_cast<double>(result.yUndistorted);
    const double xd = static_cast<double>(xDistorted);
    const double yd = static_cast<double>(yDistorted);
    const Eigen::Vector3d redistorted =
        cobConverterReference::applyBrownConrady(Eigen::Vector3d{xu, yu, 1.0}, coefficients);

    const double r2 = (xu * xu) + (yu * yu);
    const double kPolynomial = 1.0 + (static_cast<double>(coefficients.k1) * r2) +
                               (static_cast<double>(coefficients.k2) * r2 * r2) +
                               (static_cast<double>(coefficients.k3) * r2 * r2 * r2);
    const double noiseScale =
        std::max({1.0, std::abs(xd), std::abs(yd), std::abs(xu * kPolynomial), std::abs(yu * kPolynomial)});

    EXPECT_TRUE(std::isfinite(redistorted(0)) && std::isfinite(redistorted(1)));
    EXPECT_LE(std::abs(redistorted(0) - xd), kRoundTripRtol * noiseScale);
    EXPECT_LE(std::abs(redistorted(1) - yd), kRoundTripRtol * noiseScale);
}

// Robustness: any float (NaN, Inf, extremes, subnormals) for every input, including folding coefficients where the
// recovers-input tests can't apply: finite in -> finite out, and valid -> the result re-distorts to the input.
void fuzzUndistortArbitraryInputs(float xDistorted,
                                  float yDistorted,
                                  float k1,
                                  float k2,
                                  float k3,
                                  float p1,
                                  float p2) {
    const CalibrationCoefficients coefficients{.k1 = k1, .k2 = k2, .k3 = k3, .p1 = p1, .p2 = p2};
    const UndistortedCoordinate result =
        CobConverterAlgorithm::undistortNormalizedCoordinate(xDistorted, yDistorted, coefficients);
    if (!fsw::is_finite(xDistorted) || !fsw::is_finite(yDistorted)) {
        EXPECT_FALSE(result.valid);
        return;
    }
    // Finite inputs always yield finite outputs.
    EXPECT_TRUE(fsw::is_finite(result.xUndistorted));
    EXPECT_TRUE(fsw::is_finite(result.yUndistorted));
    if (result.valid) {
        expectUndistortRoundTrip(xDistorted, yDistorted, coefficients, result);
    } else {
        // Not converged: the initial guess is returned.
        EXPECT_EQ(result.xUndistorted, xDistorted);
        EXPECT_EQ(result.yUndistorted, yDistorted);
    }
}

// Zero coefficients return the input exactly. Bounded so r^6 stays finite in fp32 (|x|, |y| < ~1.87e6).
void fuzzUndistortZeroCoefficientsIsIdentity(float xDistorted, float yDistorted) {
    const UndistortedCoordinate result =
        CobConverterAlgorithm::undistortNormalizedCoordinate(xDistorted, yDistorted, CalibrationCoefficients{});
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.xUndistorted, xDistorted);
    EXPECT_EQ(result.yUndistorted, yDistorted);
}

// Distort (xu, yu), undistort it, and expect (xu, yu) back if valid, else the distorted input. The domain must keep
// the distortion fold-free, |F(a) - F(b)| >= foldFreeMargin |a - b|, so (xu, yu) is the only preimage.
void expectUndistortRecoversInput(float xu,
                                  float yu,
                                  const CalibrationCoefficients& coefficients,
                                  double foldFreeMargin,
                                  bool expectConverged) {
    const auto x = static_cast<double>(xu);
    const auto y = static_cast<double>(yu);
    const Eigen::Vector3d distorted =
        cobConverterReference::applyBrownConrady(Eigen::Vector3d{x, y, 1.0}, coefficients);
    const auto xDistorted = static_cast<float>(distorted(0));
    const auto yDistorted = static_cast<float>(distorted(1));
    ASSERT_TRUE(fsw::is_finite(xDistorted) && fsw::is_finite(yDistorted));

    const UndistortedCoordinate result =
        CobConverterAlgorithm::undistortNormalizedCoordinate(xDistorted, yDistorted, coefficients);
    if (expectConverged) {
        EXPECT_TRUE(result.valid);
    }
    EXPECT_TRUE(fsw::is_finite(result.xUndistorted) && fsw::is_finite(result.yUndistorted));
    if (!result.valid) {
        EXPECT_EQ(result.xUndistorted, xDistorted);
        EXPECT_EQ(result.yUndistorted, yDistorted);
        return;
    }

    const double r2 = (x * x) + (y * y);
    const double kPolynomial = 1.0 + (static_cast<double>(coefficients.k1) * r2) +
                               (static_cast<double>(coefficients.k2) * r2 * r2) +
                               (static_cast<double>(coefficients.k3) * r2 * r2 * r2);
    const double scale = std::max({1.0,
                                   std::abs(static_cast<double>(xDistorted)),
                                   std::abs(static_cast<double>(yDistorted)),
                                   std::abs(x * kPolynomial),
                                   std::abs(y * kPolynomial)});
    const double tolerance = 2.0 * kRoundTripRtol * scale / foldFreeMargin;
    EXPECT_LE(std::abs(static_cast<double>(result.xUndistorted) - x), tolerance);
    EXPECT_LE(std::abs(static_cast<double>(result.yUndistorted) - y), tolerance);
}

// |x|, |y| <= 0.3 (r^2 <= 0.18) with |k| <= 1: radial slope >= 1 - 3(0.18) - 5(0.18)^2 - 7(0.18)^3 = 0.257, minus
// tangential 8(|p1| + |p2|) r <= 0.007.
void fuzzUndistortRecoversInputNarrowFov(float xu, float yu, const CalibrationCoefficients& coefficients) {
    expectUndistortRecoversInput(xu, yu, coefficients, 0.25, false);
}

// |x|, |y| <= 12 (r <= 17) with k >= 0: radial slope >= 1, minus tangential 8(|p1| + |p2|) r <= 0.27.
void fuzzUndistortRecoversInputPincushion(float xu, float yu, const CalibrationCoefficients& coefficients) {
    expectUndistortRecoversInput(xu, yu, coefficients, 0.7, false);
}

// Coefficients as distortion at the image corner R = sqrt(2) tan(halfFieldOfView): k1 = a/R^2, k2 = b/R^4, k3 = c/R^6,
// p = q/R. Fold-free margin >= 1 - 3|a| - 5|b| - 7|c| - 8(|q1| + |q2|) >= 0.4 at any field of view, and must converge.
void fuzzUndistortRecoversInputFovScaled(float halfFieldOfView,
                                         float xFraction,
                                         float yFraction,
                                         float a,
                                         float b,
                                         float c,
                                         float q1,
                                         float q2) {
    const double halfWidth = std::tan(static_cast<double>(halfFieldOfView));
    const double cornerRadius = std::sqrt(2.0) * halfWidth;
    const double r2 = cornerRadius * cornerRadius;
    const CalibrationCoefficients coefficients{.k1 = static_cast<float>(static_cast<double>(a) / r2),
                                               .k2 = static_cast<float>(static_cast<double>(b) / (r2 * r2)),
                                               .k3 = static_cast<float>(static_cast<double>(c) / (r2 * r2 * r2)),
                                               .p1 = static_cast<float>(static_cast<double>(q1) / cornerRadius),
                                               .p2 = static_cast<float>(static_cast<double>(q2) / cornerRadius)};
    expectUndistortRecoversInput(static_cast<float>(static_cast<double>(xFraction) * halfWidth),
                                 static_cast<float>(static_cast<double>(yFraction) * halfWidth),
                                 coefficients,
                                 0.4,
                                 true);
}

}  // namespace

FUZZ_TEST(CobConverterUndistortFuzz, fuzzUndistortArbitraryInputs)
    .WithDomains(fuzztest::Arbitrary<float>(),   // xDistorted
                 fuzztest::Arbitrary<float>(),   // yDistorted
                 fuzztest::Arbitrary<float>(),   // k1
                 fuzztest::Arbitrary<float>(),   // k2
                 fuzztest::Arbitrary<float>(),   // k3
                 fuzztest::Arbitrary<float>(),   // p1
                 fuzztest::Arbitrary<float>());  // p2

FUZZ_TEST(CobConverterUndistortFuzz, fuzzUndistortZeroCoefficientsIsIdentity)
    .WithDomains(fuzztest::InRange(-1.0e6F, 1.0e6F),   // xDistorted [-]
                 fuzztest::InRange(-1.0e6F, 1.0e6F));  // yDistorted [-]

FUZZ_TEST(CobConverterUndistortFuzz, fuzzUndistortRecoversInputNarrowFov)
    .WithDomains(fuzztest::InRange(-0.3F, 0.3F),    // xu [-]: ~+/-17 deg off boresight
                 fuzztest::InRange(-0.3F, 0.3F),    // yu [-]
                 calibrationCoefficientsDomain());  // barrel and pincushion

FUZZ_TEST(CobConverterUndistortFuzz, fuzzUndistortRecoversInputPincushion)
    .WithDomains(fuzztest::InRange(-12.0F, 12.0F),  // xu [-]: image edge of a 170 deg FOV (tan 85 deg ~ 11.4)
                 fuzztest::InRange(-12.0F, 12.0F),  // yu [-]
                 pincushionCoefficientsDomain());

FUZZ_TEST(CobConverterUndistortFuzz, fuzzUndistortRecoversInputFovScaled)
    .WithDomains(fuzztest::InRange(0.0873F, 1.4835F),  // halfFieldOfView [rad]: 5 to 85 deg
                 fuzztest::InRange(-1.0F, 1.0F),       // xu as a fraction of the image half-width
                 fuzztest::InRange(-1.0F, 1.0F),       // yu as a fraction of the image half-height
                 fuzztest::InRange(-0.1F, 0.1F),       // a = k1 R^2
                 fuzztest::InRange(-0.03F, 0.03F),     // b = k2 R^4
                 fuzztest::InRange(-0.01F, 0.01F),     // c = k3 R^6
                 fuzztest::InRange(-0.005F, 0.005F),   // q1 = p1 R
                 fuzztest::InRange(-0.005F, 0.005F));  // q2 = p2 R
