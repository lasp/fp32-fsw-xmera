#include "mrpPDTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(MrpPDTest, RegressionTest) {
    regressionTestMrpPD(10,
                        200,
                        Eigen::Vector3f{0.2, -0.1, -0.4},
                        Eigen::Vector3f{-0.1, -0.4, 0},
                        Eigen::Vector3f{0.009, 0.007, -0.006},
                        Eigen::Vector3f{0.08, -0.001, -0.003});
}

TEST(MrpPDTest, PropertyTest) {
    // --- Test module with targeted inputs to ensure individual terms are properly implemented (Identity inertia) ---
    const Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();

    // All inputs are zeros except external torque should return the negated external torque.
    {
        const Eigen::Vector3f torque(1, 2, 3);
        MrpPDAlgorithm alg{MrpPDConfig::create(10, 200, torque, identity)};
        const Eigen::Vector3f out =
            alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out[i], -torque[i], 1e-6);
        }
    }

    // With zero torque, zero rates and accelerations, the torque is the input MRP scaled by the proportional gain.
    {
        MrpPDAlgorithm alg{MrpPDConfig::create(10, 200, Eigen::Vector3f::Zero(), identity)};
        const Eigen::Vector3f sigma_BR(0.5, 0.2, 0.1);
        const Eigen::Vector3f out = alg.update(sigma_BR, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out[i], -10.0F * sigma_BR[i], 1e-6);
        }

        // With Identity inertia, a pure acceleration input passes through as the domega term.
        const Eigen::Vector3f domega_RN_B(0.5, 0.2, 0.1);
        const Eigen::Vector3f outAccel = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), domega_RN_B);
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(outAccel[i], domega_RN_B[i], 1e-6);
        }

        // With all but omega_BR_B null, the torque is omega_BR_B scaled by the derivative gain.
        const Eigen::Vector3f omega_BR_B(-0.3, 0.1, -0.8);
        const Eigen::Vector3f outRate = alg.update(Eigen::Vector3f::Zero(), omega_BR_B, Eigen::Vector3f::Zero());
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(outRate[i], -200.0F * omega_BR_B[i], 1e-6);
        }
    }
}

TEST(MrpPDTest, SetupTest) {
    const Eigen::Vector3f torque(1.0, 2.0, 3.0);
    const Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();

    // Valid configuration round-trips its values.
    const auto config = MrpPDConfig::create(100.0F, 10.0F, torque, identity);
    EXPECT_NEAR(config.getProportionalGainK(), 100.0F, 1e-6);
    EXPECT_NEAR(config.getDerivativeGainP(), 10.0F, 1e-6);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(config.getKnownTorquePntB_B()[i], torque[i], 1e-6);
    }

    // Negative gains are rejected.
    EXPECT_THROW(MrpPDConfig::create(-0.1F, 10.0F, torque, identity), fsw::invalid_argument);
    EXPECT_THROW(MrpPDConfig::create(100.0F, -0.1F, torque, identity), fsw::invalid_argument);

    // Invalid inertia matrices are rejected.
    Eigen::Matrix3f badInertia{};
    badInertia << 1, 0, 0, 0, 1, 0, 0, 0, 0;  // singular
    EXPECT_THROW(MrpPDConfig::create(100.0F, 10.0F, torque, badInertia), fsw::invalid_argument);
    badInertia << 1, 0, 0, 0, 1, 0, 0, 1, 1;  // asymmetric
    EXPECT_THROW(MrpPDConfig::create(100.0F, 10.0F, torque, badInertia), fsw::invalid_argument);
    badInertia << 3, 0, 0, 0, 1, 0, 0, 0, 1;  // violates triangle inequality
    EXPECT_THROW(MrpPDConfig::create(100.0F, 10.0F, torque, badInertia), fsw::invalid_argument);

    EXPECT_TRUE(MrpPDConfig::isValidProportionalGainK(0.0F));
    EXPECT_FALSE(MrpPDConfig::isValidProportionalGainK(-1.0F));
    EXPECT_TRUE(MrpPDConfig::isValidDerivativeGainP(0.0F));
    EXPECT_FALSE(MrpPDConfig::isValidDerivativeGainP(-1.0F));
    EXPECT_TRUE(MrpPDConfig::isValidInertia(identity));
}
