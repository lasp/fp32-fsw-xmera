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
    const float fieldOfView = static_cast<float>(20.0 * std::numbers::pi / 180.0);

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
                     fieldOfView,
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

TEST(CobConverterTest, SetupTest) { testCobConverterSetup(); }
