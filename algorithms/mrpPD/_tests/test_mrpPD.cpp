// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
    const float K = 10.0F;
    const float P = 200.0F;
    const Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();

    // All inputs are zeros except external torque should return -external torque
    {
        const Eigen::Vector3f torque{1.0F, 2.0F, 3.0F};
        const MrpPDAlgorithm alg{MrpPDConfig::create(K, P, torque, inertia)};
        Eigen::Vector3f outputTorque = Eigen::Vector3f::Zero();
        EXPECT_NO_THROW(outputTorque =
                            alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()));
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(outputTorque[i], -torque[i], 1e-6);
        }
    }

    // If input rates and accelerations are null, the torque is the input MRP scaled by -K
    {
        const MrpPDAlgorithm alg{MrpPDConfig::create(K, P, Eigen::Vector3f::Zero(), inertia)};
        const Eigen::Vector3f sigma_BR(0.5F, 0.2F, 0.1F);
        Eigen::Vector3f outputTorque = Eigen::Vector3f::Zero();
        EXPECT_NO_THROW(outputTorque = alg.update(sigma_BR, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()));
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(outputTorque[i], -K * sigma_BR[i], 1e-6);
        }
    }

    // With identity inertia, only the domega term contributes
    {
        const MrpPDAlgorithm alg{MrpPDConfig::create(K, P, Eigen::Vector3f::Zero(), inertia)};
        const Eigen::Vector3f domega_RN_B(0.5F, 0.2F, 0.1F);
        Eigen::Vector3f outputTorque = Eigen::Vector3f::Zero();
        EXPECT_NO_THROW(outputTorque = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), domega_RN_B));
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(outputTorque[i], domega_RN_B[i], 1e-6);
        }
    }

    // If only omega_BR_B is non-zero, the torque is omega_BR_B scaled by -P
    {
        const MrpPDAlgorithm alg{MrpPDConfig::create(K, P, Eigen::Vector3f::Zero(), inertia)};
        const Eigen::Vector3f omega_BR_B(-0.3F, 0.1F, -0.8F);
        Eigen::Vector3f outputTorque = Eigen::Vector3f::Zero();
        EXPECT_NO_THROW(outputTorque = alg.update(Eigen::Vector3f::Zero(), omega_BR_B, Eigen::Vector3f::Zero()));
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(outputTorque[i], -P * omega_BR_B[i], 1e-6);
        }
    }
}

TEST(MrpPDTest, SetupTest) {
    const Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
    const Eigen::Vector3f zeroTorque = Eigen::Vector3f::Zero();

    // Negative gains
    EXPECT_THROW(MrpPDConfig::create(-0.1F, 10.0F, zeroTorque, inertia), fsw::invalid_argument);
    EXPECT_THROW(MrpPDConfig::create(10.0F, -0.1F, zeroTorque, inertia), fsw::invalid_argument);

    // Invalid inertias
    Eigen::Matrix3f badInertia{};
    badInertia << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    EXPECT_THROW(MrpPDConfig::create(10.0F, 10.0F, zeroTorque, badInertia), fsw::invalid_argument);
    badInertia << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    EXPECT_THROW(MrpPDConfig::create(10.0F, 10.0F, zeroTorque, badInertia), fsw::invalid_argument);
    badInertia << 3, 0, 0, 0, 1, 0, 0, 0, 1;
    EXPECT_THROW(MrpPDConfig::create(10.0F, 10.0F, zeroTorque, badInertia), fsw::invalid_argument);

    // Round-trip
    const float K = 100.0F;
    const float P = 10.0F;
    const Eigen::Vector3f torque(1.0F, 2.0F, 3.0F);
    const auto cfg = MrpPDConfig::create(K, P, torque, inertia);
    EXPECT_NEAR(cfg.getProportionalGainK(), K, 1e-6);
    EXPECT_NEAR(cfg.getDerivativeGainP(), P, 1e-6);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(cfg.getKnownTorquePntB_B()[i], torque[i], 1e-6);
    }
}
