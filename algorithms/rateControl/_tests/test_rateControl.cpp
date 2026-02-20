#include "rateControlTestHelpers.hpp"
#include <gtest/gtest.h>

inline void testSetupRateControl() {
    RateControlAlgorithm alg;

    // 1) Set spacecraft inertia (just verify it doesn't throw)
    const Eigen::Matrix3f I = (Eigen::Matrix3f() << 2.0f, 0.1f, 0.0f, 0.1f, 3.0f, 0.2f, 0.0f, 0.2f, 4.0f).finished();
    EXPECT_NO_THROW(alg.setSpacecraftInertia(I));

    // 2) Derivative gain P: allow zero and positive, reject negative
    EXPECT_NO_THROW(alg.setDerivativeGainP(0.0f));
    EXPECT_FLOAT_EQ(alg.getDerivativeGainP(), 0.0f);

    EXPECT_NO_THROW(alg.setDerivativeGainP(1.25f));
    EXPECT_FLOAT_EQ(alg.getDerivativeGainP(), 1.25f);

    // If FS_THROW_INVALID_ARGUMENT throws a specific type in your project,
    // replace std::exception with that type.
    EXPECT_THROW(alg.setDerivativeGainP(-1.0f), std::exception);

    // 3) Known external torque: set + getter correctness
    const Eigen::Vector3f tau = Eigen::Vector3f(0.4f, -0.2f, 0.1f);
    EXPECT_NO_THROW(alg.setKnownTorquePntB_B(tau));

    const Eigen::Vector3f& got = alg.getKnownTorquePntB_B();
    EXPECT_FLOAT_EQ(got.x(), tau.x());
    EXPECT_FLOAT_EQ(got.y(), tau.y());
    EXPECT_FLOAT_EQ(got.z(), tau.z());
}

inline void testKnownSolution() {
    RateControlAlgorithm alg;

    // 1) Set spacecraft inertia (just verify it doesn't throw)
    const Eigen::Matrix3f I = (Eigen::Matrix3f() << 2.0f, 0.1f, 0.0f, 0.1f, 3.0f, 0.2f, 0.0f, 0.2f, 4.0f).finished();
    EXPECT_NO_THROW(alg.setSpacecraftInertia(I));
    EXPECT_NO_THROW(alg.setDerivativeGainP(0.0f));
    EXPECT_NO_THROW(alg.setDerivativeGainP(1.25f));
    const Eigen::Vector3f tau = Eigen::Vector3f(0.4f, -0.2f, 0.1f);
    EXPECT_NO_THROW(alg.setKnownTorquePntB_B(tau));

    // 4) Minimal sanity: with all-zero inputs, output should be -knownTorque
    InputGuidanceData in{};
    in.omega_BR_B.setZero();
    in.omega_RN_B.setZero();
    in.domega_RN_B.setZero();

    const Eigen::Vector3f out = alg.update(in);
    EXPECT_EQ(out.x(), -tau.x());
    EXPECT_EQ(out.y(), -tau.y());
    EXPECT_EQ(out.z(), -tau.z());
}

TEST(rateControlTest, RegressionTest) {
    const Eigen::Matrix3f spacecraftInertia = Eigen::Matrix3f::Identity();
    const float DrivativeGainP = 1.0F;
    const Eigen::Vector3f knownTorquePntB_B = Eigen::Vector3f::Ones();
    const Eigen::Vector3f omega_BR_B{1.0f, 2.0f, -3.0f};
    const Eigen::Vector3f omega_RN_B{-0.2f, 0.5f, 2.8f};
    const Eigen::Vector3f domega_RN_B{11.0f, -2.6f, 0.9f};
    regressionTestrateControl(
        spacecraftInertia, DrivativeGainP, knownTorquePntB_B, omega_BR_B, omega_RN_B, domega_RN_B);
}

TEST(rateControlTest, SetupTest) { testSetupRateControl(); }

TEST(rateControlTest, PropertyKnowlSolution) { testKnownSolution(); }
