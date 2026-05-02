// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "mrpPDTestHelpers.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

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

    // Non-finite known torque
    const Eigen::Vector3f nanTorque{std::nanf(""), 0.0F, 0.0F};
    EXPECT_THROW(MrpPDConfig::create(10.0F, 10.0F, nanTorque, inertia), fsw::invalid_argument);
    const Eigen::Vector3f infTorque{std::numeric_limits<float>::infinity(), 0.0F, 0.0F};
    EXPECT_THROW(MrpPDConfig::create(10.0F, 10.0F, infTorque, inertia), fsw::invalid_argument);

    // Invalid inertias: zero diagonal, asymmetric, triangle-inequality violating
    Eigen::Matrix3f badInertia{};
    badInertia << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    EXPECT_THROW(MrpPDConfig::create(10.0F, 10.0F, zeroTorque, badInertia), fsw::invalid_argument);
    badInertia << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    EXPECT_THROW(MrpPDConfig::create(10.0F, 10.0F, zeroTorque, badInertia), fsw::invalid_argument);
    badInertia << 3, 0, 0, 0, 1, 0, 0, 0, 1;
    EXPECT_THROW(MrpPDConfig::create(10.0F, 10.0F, zeroTorque, badInertia), fsw::invalid_argument);

    // Zero gains are allowed (boundary of valid range)
    EXPECT_NO_THROW(MrpPDConfig::create(0.0F, 0.0F, zeroTorque, inertia));

    // Round-trip via getters
    const float K = 100.0F;
    const float P = 10.0F;
    const Eigen::Vector3f torque(1.0F, 2.0F, 3.0F);
    Eigen::Matrix3f principalInertia{};
    principalInertia << 1000.0F, 0.0F, 0.0F, 0.0F, 800.0F, 0.0F, 0.0F, 0.0F, 800.0F;
    const auto cfg = MrpPDConfig::create(K, P, torque, principalInertia);
    EXPECT_FLOAT_EQ(cfg.getProportionalGainK(), K);
    EXPECT_FLOAT_EQ(cfg.getDerivativeGainP(), P);
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(cfg.getKnownTorquePntB_B()[i], torque[i]);
    }
    EXPECT_TRUE(cfg.getSpacecraftInertia().isApprox(principalInertia));
}

TEST(MrpPDTest, FinitenessUnderRandomInputs) {
    // For any bounded valid configuration and bounded inputs, the output torque must be finite.
    Eigen::Matrix3f inertia{};
    inertia << 1000.0F, 0.0F, 0.0F, 0.0F, 800.0F, 0.0F, 0.0F, 0.0F, 800.0F;
    const MrpPDAlgorithm alg{MrpPDConfig::create(0.15F, 150.0F, Eigen::Vector3f{0.1F, 0.2F, 0.3F}, inertia)};

    const Eigen::Vector3f sigmaSamples[] = {
        {0.0F, 0.0F, 0.0F}, {0.999F, 0.0F, 0.0F}, {-0.5F, 0.5F, -0.5F}, {0.3F, -0.3F, 0.3F}};
    const Eigen::Vector3f omegaSamples[] = {
        {0.0F, 0.0F, 0.0F}, {1.0F, -1.0F, 1.0F}, {-10.0F, 10.0F, -10.0F}, {0.001F, -0.001F, 0.001F}};
    const Eigen::Vector3f domegaSamples[] = {{0.0F, 0.0F, 0.0F}, {0.1F, 0.0F, -0.1F}, {-1.0F, 1.0F, -1.0F}};
    for (const auto& sigma : sigmaSamples) {
        for (const auto& omega : omegaSamples) {
            for (const auto& domega : domegaSamples) {
                const Eigen::Vector3f Lr = alg.update(sigma, omega, domega);
                for (int i = 0; i < 3; ++i) {
                    EXPECT_TRUE(std::isfinite(Lr[i]));
                }
            }
        }
    }
}

TEST(MrpPDTest, ZeroGainsProduceOnlyFeedforwardTerms) {
    // With K = 0 and P = 0, the output collapses to [I] * domega_RN_B - knownTorque, regardless of
    // sigma and omega errors.
    const Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
    const Eigen::Vector3f knownTorque{1.0F, -2.0F, 3.0F};
    const MrpPDAlgorithm alg{MrpPDConfig::create(0.0F, 0.0F, knownTorque, inertia)};

    const Eigen::Vector3f sigma{0.7F, -0.5F, 0.2F};
    const Eigen::Vector3f omega{1.0F, 2.0F, 3.0F};
    const Eigen::Vector3f domega{0.4F, 0.5F, 0.6F};
    const Eigen::Vector3f Lr = alg.update(sigma, omega, domega);
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(Lr[i], domega[i] - knownTorque[i]);
    }
}

TEST(MrpPDTest, SetConfigUpdatesGainsAtRuntime) {
    // setConfig must replace the active configuration; a subsequent update reflects the new gains.
    const Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
    MrpPDAlgorithm alg{MrpPDConfig::create(1.0F, 2.0F, Eigen::Vector3f::Zero(), inertia)};

    const Eigen::Vector3f sigma{0.1F, 0.2F, 0.3F};
    const Eigen::Vector3f Lr1 = alg.update(sigma, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(Lr1[i], -1.0F * sigma[i]);
    }

    alg.setConfig(MrpPDConfig::create(5.0F, 2.0F, Eigen::Vector3f::Zero(), inertia));
    const Eigen::Vector3f Lr2 = alg.update(sigma, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(Lr2[i], -5.0F * sigma[i]);
    }
}

TEST(MrpPDTest, IsValidPredicates) {
    // Each predicate must accept the boundary value and reject the just-below/above case.
    EXPECT_TRUE(MrpPDConfig::isValidProportionalGainK(0.0F));
    EXPECT_FALSE(MrpPDConfig::isValidProportionalGainK(-1e-7F));

    EXPECT_TRUE(MrpPDConfig::isValidDerivativeGainP(0.0F));
    EXPECT_FALSE(MrpPDConfig::isValidDerivativeGainP(-1e-7F));

    EXPECT_TRUE(MrpPDConfig::isValidKnownTorquePntB_B(Eigen::Vector3f::Zero()));
    EXPECT_FALSE(MrpPDConfig::isValidKnownTorquePntB_B(Eigen::Vector3f{std::nanf(""), 0.0F, 0.0F}));

    EXPECT_TRUE(MrpPDConfig::isValidSpacecraftInertia(Eigen::Matrix3f::Identity()));
    Eigen::Matrix3f singular{};
    singular << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    EXPECT_FALSE(MrpPDConfig::isValidSpacecraftInertia(singular));
}
