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
    // --- Test module with targeted inputs to ensure individual terms are properly implemented ---
    MrpPDAlgorithm alg{};
    alg.setProportionalGainK(10);
    alg.setDerivativeGainP(200);

    Eigen::Vector3f torque = Eigen::Vector3f::Zero();

    // All inputs are zeros except external torque should return external torque
    torque << 1, 2, 3;
    alg.setKnownTorquePntB_B(torque);
    Eigen::Vector3f outputTorque = Eigen::Vector3f::Zero();
    EXPECT_NO_THROW(outputTorque =
                        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(outputTorque[i], -torque[i], 1e-6);
    }

    // If input rates and accelerations are null, the torque is the input mrp scaled by proportional gain
    torque << 0, 0, 0;
    alg.setKnownTorquePntB_B(torque);
    Eigen::Vector3f const sigma_BR(0.5, 0.2, 0.1);
    EXPECT_NO_THROW(outputTorque = alg.update(sigma_BR, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(outputTorque[i], -alg.getProportionalGainK() * sigma_BR[i], 1e-6);
    }

    // If input rates and accelerations are null, with Identity inertia matrix, the torque is the domega term
    Eigen::Vector3f const domega_RN_B(0.5, 0.2, 0.1);
    EXPECT_NO_THROW(outputTorque = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), domega_RN_B));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(outputTorque[i], domega_RN_B[i], 1e-6);
    }

    // If all but omega_BR_B null, the torque is omega_BR_B scaled by derivative gain
    Eigen::Vector3f const omega_BR_B(-0.3, 0.1, -0.8);
    EXPECT_NO_THROW(outputTorque = alg.update(Eigen::Vector3f::Zero(), omega_BR_B, Eigen::Vector3f::Zero()));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(outputTorque[i], -alg.getDerivativeGainP() * omega_BR_B[i], 1e-6);
    }
}

TEST(MrpPDTest, SetupTest) {
    MrpPDAlgorithm alg{};

    // --- Test expected exceptions ---

    // Negative feedback gains
    EXPECT_THROW(alg.setProportionalGainK(-0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setDerivativeGainP(-0.1), fsw::invalid_argument);

    Eigen::Matrix3f badInertia{};
    badInertia << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    EXPECT_THROW(alg.setSpacecraftInertia(badInertia), fsw::invalid_argument);
    badInertia << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    EXPECT_THROW(alg.setSpacecraftInertia(badInertia), fsw::invalid_argument);
    badInertia << 3, 0, 0, 0, 1, 0, 0, 0, 1;
    EXPECT_THROW(alg.setSpacecraftInertia(badInertia), fsw::invalid_argument);

    float K = 100;
    float P = 10;
    Eigen::Vector3f torque = Eigen::Vector3f(1., 2, 3);
    alg.setProportionalGainK(K);
    alg.setDerivativeGainP(P);
    alg.setKnownTorquePntB_B(torque);

    // Check setters and getters
    EXPECT_NEAR(alg.getProportionalGainK(), K, 1e-6);
    EXPECT_NEAR(alg.getDerivativeGainP(), P, 1e-6);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(alg.getKnownTorquePntB_B()[i], torque[i], 1e-6);
    }
}
