#include "cobConverterTestHelpers.hpp"
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>
#include <string>

TEST(CobConverterTest, RegressionTest) {
    // create a config
    constexpr float attSigma = 0.001F;
    Eigen::Matrix3f attitudeCovariance = Eigen::Matrix3f::Zero();
    attitudeCovariance(0, 0) = attSigma * attSigma;
    attitudeCovariance(1, 1) = (0.9F * attSigma) * (0.9F * attSigma);
    attitudeCovariance(2, 2) = (0.95F * attSigma) * (0.95F * attSigma);

    const CalibrationCoefficients coefficients{};  // no distortion
    // Distinct X/Y values so the regression test actually exercises independent fields of view,
    // instead of masking a bug where X and Y are accidentally swapped or still coupled.
    const float fieldOfViewX = static_cast<float>(20.0 * std::numbers::pi / 180.0);
    const float fieldOfViewY = static_cast<float>(15.0 * std::numbers::pi / 180.0);

    // Camera boresight points along -body-x, "up" along body-y (target-pointing geometry).
    Eigen::Matrix3d dcm_CB;
    dcm_CB << 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0;
    const Eigen::Vector3f bodyToCameraMrp = dcmToMrp(dcm_CB).cast<float>();

    // create an input
    const Eigen::Vector3d r_BdyZero_N{-500.0e3, -300.0e3, 0.0};
    const Eigen::Vector3d v_BdyZero_N{8.0e3, 0.0, 0.0};
    const Eigen::Vector3d h1 = r_BdyZero_N.stableNormalized();
    Eigen::Vector3d h3 = h1.cross(v_BdyZero_N.stableNormalized());
    h3.stableNormalize();
    const Eigen::Vector3d h2 = h3.cross(h1).stableNormalized();
    Eigen::Matrix3d dcm_BN;
    dcm_BN.row(0) = h1.transpose();
    dcm_BN.row(1) = h2.transpose();
    dcm_BN.row(2) = h3.transpose();
    const Eigen::Vector3f sigma_BN = dcmToMrp(dcm_BN).cast<float>();

    const Eigen::Vector3d sunUnit_N = Eigen::Vector3d{-1.0, -1.0, 0.0}.stableNormalized();
    const Eigen::Vector3f vehSunPntBdy = (dcm_BN * sunUnit_N).cast<float>();

    // run the regression test testCobConverter(...) defined in cobConverterTestHelpers
    testCobConverter(/*radius=*/25.0e3F,
                     /*radiusUncertainty=*/8.0e3F,
                     attitudeCovariance,
                     /*numStandardDeviations=*/3.0F,
                     /*standardDeviation=*/100.0F,
                     /*specifiedStandardDeviation=*/true,
                     /*outlierDetectionEnabled=*/true,
                     coefficients,
                     /*cameraId=*/0,
                     fieldOfViewX,
                     fieldOfViewY,
                     /*resolutionX=*/512.0F,
                     /*resolutionY=*/512.0F,
                     bodyToCameraMrp,
                     /*cobValid=*/true,
                     /*cobPixelsFound=*/75,
                     /*cobCenterOfBrightness=*/Eigen::Vector2f{152.0F, 251.0F},
                     /*cobTimeTag=*/12345U,
                     sigma_BN,
                     vehSunPntBdy,
                     /*filterVehPosition=*/r_BdyZero_N,
                     /*filterVehPositionCovariance=*/Eigen::Matrix3d::Identity() * 50.0e3);
}

// Same geometry as RegressionTest, but with non-zero Brown-Conrady coefficients: with zero
// coefficients the forward and inverse models are both the identity, so only this test can tell
// whether the reference mirrors the algorithm's undistortion.
TEST(CobConverterTest, BrownConradyCalibrationTest) {
    constexpr float attSigma = 0.001F;
    Eigen::Matrix3f attitudeCovariance = Eigen::Matrix3f::Zero();
    attitudeCovariance(0, 0) = attSigma * attSigma;
    attitudeCovariance(1, 1) = (0.9F * attSigma) * (0.9F * attSigma);
    attitudeCovariance(2, 2) = (0.95F * attSigma) * (0.95F * attSigma);

    const float fieldOfViewX = static_cast<float>(20.0 * std::numbers::pi / 180.0);
    const float fieldOfViewY = static_cast<float>(15.0 * std::numbers::pi / 180.0);

    Eigen::Matrix3d dcm_CB;
    dcm_CB << 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0;
    const Eigen::Vector3f bodyToCameraMrp = dcmToMrp(dcm_CB).cast<float>();

    const Eigen::Vector3d r_BdyZero_N{-500.0e3, -300.0e3, 0.0};
    const Eigen::Vector3d v_BdyZero_N{8.0e3, 0.0, 0.0};
    const Eigen::Vector3d h1 = r_BdyZero_N.normalized();
    Eigen::Vector3d h3 = h1.cross(v_BdyZero_N.normalized());
    h3.normalize();
    const Eigen::Vector3d h2 = h3.cross(h1).normalized();
    Eigen::Matrix3d dcm_BN;
    dcm_BN.row(0) = h1.transpose();
    dcm_BN.row(1) = h2.transpose();
    dcm_BN.row(2) = h3.transpose();
    const Eigen::Vector3f sigma_BN = dcmToMrp(dcm_BN).cast<float>();

    const Eigen::Vector3d sunUnit_N = Eigen::Vector3d{-1.0, -1.0, 0.0}.normalized();
    const Eigen::Vector3f vehSunPntBdy = (dcm_BN * sunUnit_N).cast<float>();

    // (k1, k2, k3, p1, p2), matching test_cobConverter.py::test_brown_conrady_calibration.
    struct CoefficientCase {
        const char* label;
        CalibrationCoefficients coefficients;
    };
    const std::array<CoefficientCase, 4> coefficientSets{{
        {"barrel", {.k1 = -1.0F, .k2 = -2.0F, .k3 = -5.0F, .p1 = 0.0F, .p2 = 0.0F}},
        {"pincushion", {.k1 = 1.0F, .k2 = 2.0F, .k3 = 5.0F, .p1 = 0.0F, .p2 = 0.0F}},
        {"tangential", {.k1 = 0.0F, .k2 = 0.0F, .k3 = 0.0F, .p1 = 0.5F, .p2 = 0.3F}},
        {"combined", {.k1 = -0.5F, .k2 = -1.0F, .k3 = -2.0F, .p1 = 0.2F, .p2 = -0.1F}},
    }};
    // Off-center COB points, both inside the 3 * 100 px outlier gate around the image center.
    const std::array<Eigen::Vector2f, 2> cobCenters{Eigen::Vector2f{152.0F, 251.0F}, Eigen::Vector2f{400.0F, 350.0F}};

    for (const auto& [label, coefficients] : coefficientSets) {
        for (const Eigen::Vector2f& cobCenterOfBrightness : cobCenters) {
            SCOPED_TRACE(std::string(label) + " at COB (" + std::to_string(cobCenterOfBrightness(0)) + ", " +
                         std::to_string(cobCenterOfBrightness(1)) + ")");
            testCobConverter(/*radius=*/25.0e3F,
                             /*radiusUncertainty=*/8.0e3F,
                             attitudeCovariance,
                             /*numStandardDeviations=*/3.0F,
                             /*standardDeviation=*/100.0F,
                             /*specifiedStandardDeviation=*/true,
                             /*outlierDetectionEnabled=*/true,
                             coefficients,
                             /*cameraId=*/0,
                             fieldOfViewX,
                             fieldOfViewY,
                             /*resolutionX=*/512.0F,
                             /*resolutionY=*/512.0F,
                             bodyToCameraMrp,
                             /*cobValid=*/true,
                             /*cobPixelsFound=*/75,
                             cobCenterOfBrightness,
                             /*cobTimeTag=*/12345U,
                             sigma_BN,
                             vehSunPntBdy,
                             /*filterVehPosition=*/r_BdyZero_N,
                             /*filterVehPositionCovariance=*/Eigen::Matrix3d::Identity() * 50.0e3);
        }
    }
}

// RegressionTest geometry with specifiedStandardDeviation=false, so comOutlierDetection() derives sigma from the
// heading and filter covariances (the only test covering this branch).
TEST(CobConverterTest, OutlierDetectionDerivedSigmaTest) {
    constexpr float attSigma = 0.001F;
    Eigen::Matrix3f attitudeCovariance = Eigen::Matrix3f::Zero();
    attitudeCovariance(0, 0) = attSigma * attSigma;
    attitudeCovariance(1, 1) = (0.9F * attSigma) * (0.9F * attSigma);
    attitudeCovariance(2, 2) = (0.95F * attSigma) * (0.95F * attSigma);

    const CalibrationCoefficients coefficients{};  // no distortion
    const float fieldOfViewX = static_cast<float>(20.0 * std::numbers::pi / 180.0);
    const float fieldOfViewY = static_cast<float>(15.0 * std::numbers::pi / 180.0);

    Eigen::Matrix3d dcm_CB;
    dcm_CB << 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0;
    const Eigen::Vector3f bodyToCameraMrp = dcmToMrp(dcm_CB).cast<float>();

    const Eigen::Vector3d r_BdyZero_N{-500.0e3, -300.0e3, 0.0};
    const Eigen::Vector3d v_BdyZero_N{8.0e3, 0.0, 0.0};
    const Eigen::Vector3d h1 = r_BdyZero_N.stableNormalized();
    Eigen::Vector3d h3 = h1.cross(v_BdyZero_N.stableNormalized());
    h3.stableNormalize();
    const Eigen::Vector3d h2 = h3.cross(h1).stableNormalized();
    Eigen::Matrix3d dcm_BN;
    dcm_BN.row(0) = h1.transpose();
    dcm_BN.row(1) = h2.transpose();
    dcm_BN.row(2) = h3.transpose();
    const Eigen::Vector3f sigma_BN = dcmToMrp(dcm_BN).cast<float>();

    const Eigen::Vector3d sunUnit_N = Eigen::Vector3d{-1.0, -1.0, 0.0}.stableNormalized();
    const Eigen::Vector3f vehSunPntBdy = (dcm_BN * sunUnit_N).cast<float>();

    testCobConverter(/*radius=*/25.0e3F,
                     /*radiusUncertainty=*/8.0e3F,
                     attitudeCovariance,
                     /*numStandardDeviations=*/3.0F,
                     /*standardDeviation=*/100.0F,
                     /*specifiedStandardDeviation=*/false,
                     /*outlierDetectionEnabled=*/true,
                     coefficients,
                     /*cameraId=*/0,
                     fieldOfViewX,
                     fieldOfViewY,
                     /*resolutionX=*/512.0F,
                     /*resolutionY=*/512.0F,
                     bodyToCameraMrp,
                     /*cobValid=*/true,
                     /*cobPixelsFound=*/75,
                     /*cobCenterOfBrightness=*/Eigen::Vector2f{152.0F, 251.0F},
                     /*cobTimeTag=*/12345U,
                     sigma_BN,
                     vehSunPntBdy,
                     /*filterVehPosition=*/r_BdyZero_N,
                     /*filterVehPositionCovariance=*/Eigen::Matrix3d::Identity() * 50.0e3);
}

// cobPixelsFound ("[--] bright pixels", cobConverterAlgorithm.h) is the count of bright pixels in
// the detected blob. It's tempting to read that as a brightness/signal-strength measurement, where
// more counts would average down noise and REDUCE uncertainty. But computeCameraFrameUncertainty
// scales covariance by sqrt(cobPixelsFound / kSphereSolidAngle): the algorithm actually treats it as
// a SIZE measurement (a bigger blob spans more of the image, so its centroid is less certain), which
// INCREASES uncertainty as the count grows. This property test locks in that size-like behavior.
TEST(CobConverterTest, PixelsFoundIncreaseIsSizeIncreaseTest) {
    const Eigen::Matrix3f zeroCovariance = Eigen::Matrix3f::Zero();
    const CalibrationCoefficients coefficients{};
    const Eigen::Vector3f zeroMrp = Eigen::Vector3f::Zero();

    const CobConverterConfig cfg = CobConverterConfig::create(/*radius=*/25.0e3F,
                                                              /*radiusUncertainty=*/0.0F,
                                                              zeroCovariance,
                                                              /*numStandardDeviations=*/3.0F,
                                                              /*standardDeviation=*/100.0F,
                                                              /*specifiedStandardDeviation=*/true,
                                                              /*outlierDetectionEnabled=*/false,
                                                              coefficients,
                                                              /*cameraId=*/0,
                                                              /*fieldOfViewX=*/0.35F,
                                                              /*fieldOfViewY=*/0.30F,
                                                              /*resolutionX=*/512.0F,
                                                              /*resolutionY=*/512.0F,
                                                              zeroMrp);
    const CobConverterAlgorithm alg(cfg);

    // Same attitude/filter geometry throughout -- only cobPixelsFound differs.
    const VehicleAttitude attitude{.sigma_BN = Eigen::Vector3f::Zero(),
                                   .vehSunPntBdy = Eigen::Vector3f{1.0F, 0.0F, 0.0F}};
    const FilterState filter{.filterVehPosition = Eigen::Vector3d{-500.0e3, -300.0e3, 0.0},
                             .filterVehPositionCovariance = Eigen::Matrix3d::Identity() * 50.0e3};
    const auto makeCob = [](int32_t pixelsFound) {
        return CobMeasurement{.cobValid = true,
                              .cobPixelsFound = pixelsFound,
                              .cobCenterOfBrightness = Eigen::Vector2f{152.0F, 251.0F},
                              .cobTimeTag = 12345U};
    };

    const CobConverterUpdateResult fewPixels = alg.updateState(makeCob(10), attitude, filter);
    const CobConverterUpdateResult manyPixels = alg.updateState(makeCob(1000), attitude, filter);

    // A bigger detected blob (more pixels found) should widen, not shrink, the COM/COB position
    // uncertainty -- confirming this input feeds the algorithm as a size term.
    EXPECT_GT(manyPixels.output.covar_N(0, 0), fewPixels.output.covar_N(0, 0));

    // A non-positive count is rejected like an invalid COB.
    for (const int32_t pixelsFound : {0, -1, -75}) {
        const CobConverterUpdateResult rejected = alg.updateState(makeCob(pixelsFound), attitude, filter);
        EXPECT_FALSE(rejected.output.unitVecValid) << pixelsFound;
        EXPECT_FALSE(rejected.diagnostic.comValid) << pixelsFound;
        EXPECT_EQ(rejected.output.covar_N, Eigen::Matrix3f::Zero()) << pixelsFound;
    }
}

// A non-converged COM undistortion falls back to the initial guess (the pinhole heading), so it must
// not be published as valid; the heading is still populated for debugging.
TEST(CobConverterTest, BrownConradyNonConvergenceInvalidatesHeadingTest) {
    const Eigen::Matrix3f zeroCovariance = Eigen::Matrix3f::Zero();
    const Eigen::Vector3f zeroMrp = Eigen::Vector3f::Zero();
    const auto makeAlgorithm = [&](const CalibrationCoefficients& coefficients) {
        return CobConverterAlgorithm(CobConverterConfig::create(/*radius=*/25.0e3F,
                                                                /*radiusUncertainty=*/0.0F,
                                                                zeroCovariance,
                                                                /*numStandardDeviations=*/3.0F,
                                                                /*standardDeviation=*/100.0F,
                                                                /*specifiedStandardDeviation=*/true,
                                                                /*outlierDetectionEnabled=*/false,
                                                                coefficients,
                                                                /*cameraId=*/0,
                                                                /*fieldOfViewX=*/0.35F,
                                                                /*fieldOfViewY=*/0.30F,
                                                                /*resolutionX=*/512.0F,
                                                                /*resolutionY=*/512.0F,
                                                                zeroMrp));
    };

    // Sun along the position vector: zero phase angle, so the COM coincides with the COB.
    const Eigen::Vector3d position{-500.0e3, -300.0e3, 0.0};
    const VehicleAttitude attitude{.sigma_BN = Eigen::Vector3f::Zero(),
                                   .vehSunPntBdy = position.normalized().cast<float>()};
    const FilterState filter{.filterVehPosition = position,
                             .filterVehPositionCovariance = Eigen::Matrix3d::Identity() * 50.0e3};
    // Normalized x_d ~= 0.17. With k1 = 1000 the fixed point has k1 * r^2 ~= 2.4 > 1, so the
    // iteration falls into a 2-cycle and never converges (a deliberately unphysical lens).
    const CobMeasurement cob{.cobValid = true,
                             .cobPixelsFound = 75,
                             .cobCenterOfBrightness = Eigen::Vector2f{500.0F, 256.0F},
                             .cobTimeTag = 12345U};

    const CobConverterUpdateResult nonConverged =
        makeAlgorithm(CalibrationCoefficients{.k1 = 1000.0F}).updateState(cob, attitude, filter);
    ASSERT_FALSE(nonConverged.diagnostic.brownConradyCOMValid);
    EXPECT_FALSE(nonConverged.diagnostic.brownConradyCOBValid);
    EXPECT_FALSE(nonConverged.output.unitVecValid);
    EXPECT_TRUE(nonConverged.diagnostic.comValid);
    EXPECT_TRUE(nonConverged.output.rhat_BN_N.allFinite());
    EXPECT_GT(nonConverged.output.rhat_BN_N.norm(), 0.0F);

    // Control: the same measurement with an ideal pinhole camera is published as valid.
    const CobConverterUpdateResult pinhole =
        makeAlgorithm(CalibrationCoefficients{}).updateState(cob, attitude, filter);
    ASSERT_TRUE(pinhole.diagnostic.brownConradyCOMValid);
    EXPECT_TRUE(pinhole.diagnostic.brownConradyCOBValid);
    EXPECT_TRUE(pinhole.output.unitVecValid);
    EXPECT_EQ(nonConverged.output.rhat_BN_N, pinhole.output.rhat_BN_N);
}

// Realistic lenses converge within a few iterations over their full field of view (measured max in parentheses).
TEST(CobConverterTest, BrownConradyRealisticLensConvergesTest) {
    const auto expectConverges = [](const CalibrationCoefficients& coefficients, float fieldOfView, int maxIterations) {
        const float halfExtent = std::tan(fieldOfView / 2.0F);
        constexpr int kSteps = 40;
        for (int i = 0; i <= kSteps; ++i) {
            for (int j = 0; j <= kSteps; ++j) {
                const float x = halfExtent * ((2.0F * static_cast<float>(i) / kSteps) - 1.0F);
                const float y = halfExtent * ((2.0F * static_cast<float>(j) / kSteps) - 1.0F);
                const UndistortedCoordinate result =
                    CobConverterAlgorithm::undistortNormalizedCoordinate(x, y, coefficients);
                EXPECT_TRUE(result.valid) << "x " << x << " y " << y;
                EXPECT_LE(result.iterations, maxIterations) << "x " << x << " y " << y;
            }
        }
    };
    constexpr float kDeg = std::numbers::pi_v<float> / 180.0F;
    // 60 deg full field of view (16).
    expectConverges({.k1 = -0.3F, .k2 = 0.1F, .k3 = -0.02F, .p1 = 1e-3F, .p2 = -5e-4F}, 60.0F * kDeg, 20);
    // 90 deg full field of view, with a lens that does not fold over the sensor (23).
    expectConverges({.k1 = -0.2F, .k2 = 0.05F, .k3 = 0.0F, .p1 = 1e-3F, .p2 = -5e-4F}, 90.0F * kDeg, 30);
}

// The phase-angle correction needs a range above the body radius. At or inside the radius the
// update is skipped and the output stays zeroed and invalid.
// Without a usable sunline, or at/inside the body radius, the COB is published as the COM.
TEST(CobConverterTest, PhaseCorrectionUnavailablePublishesCobTest) {
    constexpr float radius = 25.0e3F;  // [m]
    const CobConverterAlgorithm algorithm(CobConverterConfig::create(radius,
                                                                     /*radiusUncertainty=*/0.0F,
                                                                     Eigen::Matrix3f::Zero(),
                                                                     /*numStandardDeviations=*/3.0F,
                                                                     /*standardDeviation=*/100.0F,
                                                                     /*specifiedStandardDeviation=*/true,
                                                                     /*outlierDetectionEnabled=*/false,
                                                                     CalibrationCoefficients{},
                                                                     /*cameraId=*/0,
                                                                     /*fieldOfViewX=*/0.35F,
                                                                     /*fieldOfViewY=*/0.30F,
                                                                     /*resolutionX=*/512.0F,
                                                                     /*resolutionY=*/512.0F,
                                                                     Eigen::Vector3f::Zero()));
    const CobMeasurement cob{.cobValid = true,
                             .cobPixelsFound = 75,
                             .cobCenterOfBrightness = Eigen::Vector2f{256.0F, 256.0F},
                             .cobTimeTag = 12345U};
    const auto updateAt = [&](const Eigen::Vector3d& position, const Eigen::Vector3f& vehSunPntBdy) {
        const VehicleAttitude attitude{.sigma_BN = Eigen::Vector3f::Zero(), .vehSunPntBdy = vehSunPntBdy};
        const FilterState filter{.filterVehPosition = position,
                                 .filterVehPositionCovariance = Eigen::Matrix3d::Identity() * 50.0e3};
        return algorithm.updateState(cob, attitude, filter);
    };
    const auto expectCobPublished = [&](const CobConverterUpdateResult& result, const std::string& label) {
        EXPECT_TRUE(result.output.unitVecValid) << label;
        EXPECT_TRUE(result.diagnostic.comValid) << label;
        EXPECT_EQ(result.diagnostic.centerOfMass, cob.cobCenterOfBrightness) << label;
        EXPECT_EQ(result.diagnostic.offsetFactor, 0.0F) << label;
        // COB at the principal point with identity rotations: heading along -boresight.
        EXPECT_TRUE(result.output.rhat_BN_N.isApprox(Eigen::Vector3f{0.0F, 0.0F, -1.0F})) << label;
    };

    const Eigen::Vector3d direction = Eigen::Vector3d{-5.0, -3.0, 0.0}.normalized();
    const Eigen::Vector3f sunAlongPosition = direction.cast<float>();
    for (const double range : {0.5 * radius, static_cast<double>(radius)}) {
        expectCobPublished(updateAt(range * direction, sunAlongPosition), "range " + std::to_string(range));
    }
    expectCobPublished(updateAt(500.0e3 * direction, Eigen::Vector3f::Zero()), "zero sunline");

    // Control: the same geometry far outside the body is published as valid.
    EXPECT_TRUE(updateAt(500.0e3 * direction, sunAlongPosition).output.unitVecValid);
}

TEST(CobConverterTest, SetupTest) {
    const Eigen::Matrix3f zeroCovariance = Eigen::Matrix3f::Zero();
    const CalibrationCoefficients coefficients{};
    const Eigen::Vector3f zeroMrp = Eigen::Vector3f::Zero();

    // Builds a config from a fully-flattened, individually-overridable parameter list, each
    // defaulted to a known-valid nominal value -- so a single EXPECT_THROW case only needs to
    // spell out the fields up through (and including) the one under test, matching
    // CobConverterConfig::create()'s "throws on the first invalid field" ordering below.
    const auto makeConfig = [](float radius = 25.0e3F,
                               float radiusUncertainty = 8.0e3F,
                               const Eigen::Matrix3f& attitudeCovariance = Eigen::Matrix3f::Zero(),
                               float numStandardDeviations = 3.0F,
                               float standardDeviation = 100.0F,
                               bool specifiedStandardDeviation = true,
                               bool outlierDetectionEnabled = true,
                               const CalibrationCoefficients& calibrationCoefficients = CalibrationCoefficients{},
                               int cameraId = 0,
                               float fieldOfViewX = 0.35F,
                               float fieldOfViewY = 0.30F,
                               float resolutionX = 512.0F,
                               float resolutionY = 512.0F,
                               const Eigen::Vector3f& bodyToCameraMrp = Eigen::Vector3f::Zero()) {
        return CobConverterConfig::create(radius,
                                          radiusUncertainty,
                                          attitudeCovariance,
                                          numStandardDeviations,
                                          standardDeviation,
                                          specifiedStandardDeviation,
                                          outlierDetectionEnabled,
                                          calibrationCoefficients,
                                          cameraId,
                                          fieldOfViewX,
                                          fieldOfViewY,
                                          resolutionX,
                                          resolutionY,
                                          bodyToCameraMrp);
    };

    const float nan = std::numeric_limits<float>::quiet_NaN();

    // radius: must be > 0.
    EXPECT_TRUE(CobConverterConfig::isValidRadius(1.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadius(0.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadius(-1.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadius(nan));

    // radiusUncertainty: must be >= 0.
    EXPECT_TRUE(CobConverterConfig::isValidRadiusUncertainty(0.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadiusUncertainty(-1.0F));
    EXPECT_FALSE(CobConverterConfig::isValidRadiusUncertainty(nan));

    // attitudeCovariance: must be finite.
    EXPECT_TRUE(CobConverterConfig::isValidAttitudeCovariance(zeroCovariance));
    Eigen::Matrix3f nanCovariance = Eigen::Matrix3f::Zero();
    nanCovariance(0, 0) = nan;
    EXPECT_FALSE(CobConverterConfig::isValidAttitudeCovariance(nanCovariance));

    // numStandardDeviations: must be > 0.
    EXPECT_TRUE(CobConverterConfig::isValidNumStandardDeviations(1.0F));
    EXPECT_FALSE(CobConverterConfig::isValidNumStandardDeviations(0.0F));
    EXPECT_FALSE(CobConverterConfig::isValidNumStandardDeviations(-1.0F));

    // standardDeviation: only checked when specified.
    EXPECT_TRUE(CobConverterConfig::isValidStandardDeviation(-1.0F, /*specified=*/false));
    EXPECT_TRUE(CobConverterConfig::isValidStandardDeviation(1.0F, /*specified=*/true));
    EXPECT_FALSE(CobConverterConfig::isValidStandardDeviation(0.0F, /*specified=*/true));
    EXPECT_FALSE(CobConverterConfig::isValidStandardDeviation(-1.0F, /*specified=*/true));

    // calibrationCoefficients: must be finite.
    EXPECT_TRUE(CobConverterConfig::isValidCalibrationCoefficients(coefficients));
    CalibrationCoefficients nanCoefficients{};
    nanCoefficients.k1 = nan;
    EXPECT_FALSE(CobConverterConfig::isValidCalibrationCoefficients(nanCoefficients));

    // fieldOfViewX / fieldOfViewY: each must be in (0, pi), checked independently via the same
    // isValidFieldOfView helper.
    EXPECT_TRUE(CobConverterConfig::isValidFieldOfView(0.35F));
    EXPECT_FALSE(CobConverterConfig::isValidFieldOfView(0.0F));
    EXPECT_FALSE(CobConverterConfig::isValidFieldOfView(std::numbers::pi_v<float>));
    EXPECT_FALSE(CobConverterConfig::isValidFieldOfView(-0.1F));

    // resolutionX / resolutionY: must be > 0.
    EXPECT_TRUE(CobConverterConfig::isValidResolutionX(512.0F));
    EXPECT_FALSE(CobConverterConfig::isValidResolutionX(0.0F));
    EXPECT_TRUE(CobConverterConfig::isValidResolutionY(512.0F));
    EXPECT_FALSE(CobConverterConfig::isValidResolutionY(0.0F));

    // bodyToCameraMrp: must be finite.
    EXPECT_TRUE(CobConverterConfig::isValidBodyToCameraMrp(zeroMrp));
    EXPECT_FALSE(CobConverterConfig::isValidBodyToCameraMrp(Eigen::Vector3f{nan, 0.0F, 0.0F}));

    // create() throws on the first invalid field it encounters, so each case below spells out
    // every field up through the one under test (all valid except the last) and leaves the rest
    // at makeConfig's nominal defaults.
    EXPECT_THROW((void)makeConfig(0.0F /* invalid radius */), fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(25.0e3F, -1.0F /* invalid radiusUncertainty */), fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(25.0e3F, 8.0e3F, nanCovariance /* invalid attitudeCovariance */),
                 fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(25.0e3F,
                                  8.0e3F,
                                  zeroCovariance,
                                  3.0F,
                                  100.0F,
                                  true,
                                  true,
                                  nanCoefficients /* invalid calibrationCoefficients */),
                 fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(25.0e3F,
                                  8.0e3F,
                                  zeroCovariance,
                                  3.0F,
                                  100.0F,
                                  true,
                                  true,
                                  coefficients,
                                  0,
                                  std::numbers::pi_v<float> /* invalid fieldOfViewX */,
                                  0.30F),
                 fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(25.0e3F,
                                  8.0e3F,
                                  zeroCovariance,
                                  3.0F,
                                  100.0F,
                                  true,
                                  true,
                                  coefficients,
                                  0,
                                  0.35F,
                                  std::numbers::pi_v<float> /* invalid fieldOfViewY */),
                 fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(25.0e3F,
                                  8.0e3F,
                                  zeroCovariance,
                                  3.0F,
                                  100.0F,
                                  true,
                                  true,
                                  coefficients,
                                  0,
                                  0.35F,
                                  0.30F,
                                  0.0F /* invalid resolutionX */),
                 fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(25.0e3F,
                                  8.0e3F,
                                  zeroCovariance,
                                  3.0F,
                                  100.0F,
                                  true,
                                  true,
                                  coefficients,
                                  0,
                                  0.35F,
                                  0.30F,
                                  512.0F,
                                  0.0F /* invalid resolutionY */),
                 fsw::invalid_argument);
    // fieldOfViewX/fieldOfViewY are each individually valid (in (0, pi)) but their combination
    // pushes the camera model's internal tan(fieldOfView/2) argument within ~1 deg of the +/-pi/2
    // singularity that isValidCameraParam guards against (see cobConverterAlgorithm.h).
    EXPECT_THROW((void)makeConfig(25.0e3F,
                                  8.0e3F,
                                  zeroCovariance,
                                  3.0F,
                                  100.0F,
                                  true,
                                  true,
                                  coefficients,
                                  0,
                                  std::numbers::pi_v<float> - 0.01F /* fieldOfViewX pushes tan() near +/-pi/2 */,
                                  0.30F,
                                  512.0F,
                                  512.0F),
                 fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(25.0e3F,
                                  8.0e3F,
                                  zeroCovariance,
                                  3.0F,
                                  100.0F,
                                  true,
                                  true,
                                  coefficients,
                                  0,
                                  0.35F,
                                  0.30F,
                                  512.0F,
                                  512.0F,
                                  Eigen::Vector3f{nan, 0.0F, 0.0F} /* invalid bodyToCameraMrp */),
                 fsw::invalid_argument);

    // A fully valid config builds without throwing and round-trips its values through the
    // getters. fieldOfViewX/fieldOfViewY are deliberately distinct here to confirm they're
    // stored and retrieved independently.
    const CobConverterConfig cfg = makeConfig(/*radius=*/25.0e3F,
                                              /*radiusUncertainty=*/8.0e3F,
                                              zeroCovariance,
                                              /*numStandardDeviations=*/3.0F,
                                              /*standardDeviation=*/100.0F,
                                              /*specifiedStandardDeviation=*/true,
                                              /*outlierDetectionEnabled=*/true,
                                              coefficients,
                                              /*cameraId=*/7,
                                              /*fieldOfViewX=*/0.35F,
                                              /*fieldOfViewY=*/0.30F,
                                              /*resolutionX=*/512.0F,
                                              /*resolutionY=*/256.0F);
    EXPECT_FLOAT_EQ(cfg.getRadius(), 25.0e3F);
    EXPECT_FLOAT_EQ(cfg.getRadiusUncertainty(), 8.0e3F);
    EXPECT_FLOAT_EQ(cfg.getNumStandardDeviations(), 3.0F);
    EXPECT_FLOAT_EQ(cfg.getStandardDeviation(), 100.0F);
    EXPECT_TRUE(cfg.isStandardDeviationSpecified());
    EXPECT_TRUE(cfg.isOutlierDetectionEnabled());
    EXPECT_EQ(cfg.getCameraId(), 7);
    EXPECT_FLOAT_EQ(cfg.getFieldOfViewX(), 0.35F);
    EXPECT_FLOAT_EQ(cfg.getFieldOfViewY(), 0.30F);
    EXPECT_FLOAT_EQ(cfg.getResolutionX(), 512.0F);
    EXPECT_FLOAT_EQ(cfg.getResolutionY(), 256.0F);
}
