#include "rateControlTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(rateControlTest, RegressionTest) {
    const Eigen::Matrix3f spacecraftInertia = Eigen::Matrix3f::Identity();
    const float derivativeGainP = 1.0F;
    const Eigen::Vector3f knownTorquePntB_B = Eigen::Vector3f::Ones();
    const Eigen::Vector3f omega_BR_B{1.0F, 2.0F, -3.0F};
    const Eigen::Vector3f domega_RN_B{11.0F, -2.6F, 0.9F};
    regressionTestRateControl(spacecraftInertia, derivativeGainP, knownTorquePntB_B, omega_BR_B, domega_RN_B);
}

TEST(rateControlTest, SetupTest) {
    RateControlAlgorithm alg;

    // 1) Set spacecraft inertia
    const Eigen::Matrix3f I_invalid = Eigen::Matrix3f{{2.0f, 0.0f, 0.0f}, {0.1f, 3.0f, 0.2f}, {0.0f, 0.2f, 4.0f}};
    EXPECT_THROW(alg.setSpacecraftInertia(I_invalid), fsw::invalid_argument);  // invalid inertia due to not symmetry
    const Eigen::Matrix3f I = Eigen::Matrix3f{{2.0f, 0.1f, 0.0f}, {0.1f, 3.0f, 0.2f}, {0.0f, 0.2f, 4.0f}};
    EXPECT_NO_THROW(alg.setSpacecraftInertia(I));

    // 2) Derivative gain P: allow zero and positive, reject negative
    EXPECT_NO_THROW(alg.setDerivativeGainP(0.0f));
    EXPECT_EQ(alg.getDerivativeGainP(), 0.0f);
    EXPECT_THROW(alg.setDerivativeGainP(-1.0f), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setDerivativeGainP(1.25f));
    EXPECT_EQ(alg.getDerivativeGainP(), 1.25f);

    // 3) Known external torque: set + getter correctness
    const Eigen::Vector3f tau = Eigen::Vector3f(0.4f, -0.2f, 0.1f);
    EXPECT_NO_THROW(alg.setKnownTorquePntB_B(tau));

    const Eigen::Vector3f& got = alg.getKnownTorquePntB_B();
    EXPECT_EQ(got.x(), tau.x());
    EXPECT_EQ(got.y(), tau.y());
    EXPECT_EQ(got.z(), tau.z());
}
