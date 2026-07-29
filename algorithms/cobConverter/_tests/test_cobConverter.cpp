#include "cobConverterTestHelpers.hpp"
#include <gtest/gtest.h>

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

    // run the regression test testCobConverter(...) defined in cobConverterTestHelpers
    testCobConverter(PhaseAngleCorrectionMethodAlgorithm::BinaryAlg,
                     /*radius=*/25.0e3F,
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

    const CobConverterConfig cfg = CobConverterConfig::create(PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg,
                                                              /*radius=*/25.0e3F,
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

    // Same COB pixel location and geometry throughout -- only cobPixelsFound differs.
    const auto makeInput = [](int32_t pixelsFound) {
        return CobConverterInput{.cobValid = true,
                                 .cobPixelsFound = pixelsFound,
                                 .cobCenterOfBrightness = Eigen::Vector2f{152.0F, 251.0F},
                                 .cobTimeTag = 12345U,
                                 .sigma_BN = Eigen::Vector3f::Zero(),
                                 .vehSunPntBdy = Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                 .filterVehPosition = Eigen::Vector3d{-500.0e3, -300.0e3, 0.0},
                                 .filterVehPositionCovariance = Eigen::Matrix3d::Identity() * 50.0e3};
    };

    const CobConverterOutput fewPixels = alg.updateState(makeInput(10));
    const CobConverterOutput manyPixels = alg.updateState(makeInput(1000));

    // A bigger detected blob (more pixels found) should widen, not shrink, the COM/COB position
    // uncertainty in every frame -- confirming this input feeds the algorithm as a size term.
    EXPECT_GT(manyPixels.unitVec.covar_B(0, 0), fewPixels.unitVec.covar_B(0, 0));
    EXPECT_GT(manyPixels.unitVec.covar_B(1, 1), fewPixels.unitVec.covar_B(1, 1));
    EXPECT_GT(manyPixels.unitVec.covar_N(0, 0), fewPixels.unitVec.covar_N(0, 0));
    EXPECT_GT(manyPixels.unitVec.covar_C(0, 0), fewPixels.unitVec.covar_C(0, 0));
}

TEST(CobConverterTest, SetupTest) {
    const Eigen::Matrix3f zeroCovariance = Eigen::Matrix3f::Zero();
    const CalibrationCoefficients coefficients{};
    const Eigen::Vector3f zeroMrp = Eigen::Vector3f::Zero();

    // Builds a config from a fixed valid baseline, overriding only the fields under test below.
    const auto makeConfig = [&](PhaseAngleCorrectionMethodAlgorithm method,
                                float radius,
                                float fieldOfViewX,
                                float fieldOfViewY,
                                int cameraId,
                                float resolutionY) {
        return CobConverterConfig::create(method,
                                          radius,
                                          8.0e3F,
                                          zeroCovariance,
                                          3.0F,
                                          100.0F,
                                          true,
                                          true,
                                          coefficients,
                                          cameraId,
                                          fieldOfViewX,
                                          fieldOfViewY,
                                          512.0F,
                                          resolutionY,
                                          zeroMrp);
    };

    const float nan = std::numeric_limits<float>::quiet_NaN();

    // phaseAngleCorrectionMethod: only NoCorrectionAlg/BinaryAlg are valid.
    EXPECT_TRUE(
        CobConverterConfig::isValidPhaseAngleCorrectionMethod(PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg));
    EXPECT_TRUE(CobConverterConfig::isValidPhaseAngleCorrectionMethod(PhaseAngleCorrectionMethodAlgorithm::BinaryAlg));
    EXPECT_FALSE(
        CobConverterConfig::isValidPhaseAngleCorrectionMethod(static_cast<PhaseAngleCorrectionMethodAlgorithm>(99)));

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

    // create() throws on the first invalid field it encounters.
    EXPECT_THROW(
        makeConfig(
            PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg, 0.0F /* invalid radius */, 0.35F, 0.30F, 0, 512.0F),
        fsw::invalid_argument);
    EXPECT_THROW(makeConfig(PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg,
                            25.0e3F,
                            std::numbers::pi_v<float> /* invalid fieldOfViewX */,
                            0.30F,
                            0,
                            512.0F),
                 fsw::invalid_argument);
    EXPECT_THROW(makeConfig(PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg,
                            25.0e3F,
                            0.35F,
                            std::numbers::pi_v<float> /* invalid fieldOfViewY */,
                            0,
                            512.0F),
                 fsw::invalid_argument);

    // A fully valid config builds without throwing and round-trips its values through the
    // getters. fieldOfViewX/fieldOfViewY are deliberately distinct here to confirm they're
    // stored and retrieved independently.
    const CobConverterConfig cfg =
        makeConfig(PhaseAngleCorrectionMethodAlgorithm::BinaryAlg, 25.0e3F, 0.35F, 0.30F, 7, 256.0F);
    EXPECT_EQ(cfg.getPhaseAngleCorrectionMethod(), PhaseAngleCorrectionMethodAlgorithm::BinaryAlg);
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
