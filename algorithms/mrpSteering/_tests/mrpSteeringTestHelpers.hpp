#ifndef TEST_MRPSTEERING_H
#define TEST_MRPSTEERING_H

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "mrpSteeringAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/RateCmdMsgF32Payload.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <numbers>
#include <vector>

// Reference computation for update
RateCmdMsgF32Payload referenceUpdate(const MrpSteeringConfig& config, AttGuidMsgF32Payload& msg) {
    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(msg.sigma_BR);
    Eigen::Vector3f omega_ast{};
    Eigen::Vector3f omega_ast_p{Eigen::Vector3f::Zero()};

    for (int i = 0; i < 3; ++i) {
        const float sigma_i = sigma_BR[i];
        const float f_i = atanf(static_cast<float>(std::numbers::pi) / 2 / config.getOmegaMax() *
                                (config.getK1() * sigma_i + config.getK3() * powf(sigma_i, 3))) /
                          (static_cast<float>(std::numbers::pi) / 2) * config.getOmegaMax();
        omega_ast[i] = -f_i;
    }

    if (!config.getIgnoreOuterLoopFeedforward()) {
        Eigen::Matrix3f B = bmatMrp(sigma_BR);

        Eigen::Vector3f sigmaDot_BR = 0.25 * B * omega_ast;

        for (int i = 0; i < 3; ++i) {
            const float sigma_i = sigma_BR[i];
            const float f_i = (3 * config.getK3() * powf(sigma_i, 2) + config.getK1()) /
                              (powf(static_cast<float>(std::numbers::pi / 2) / config.getOmegaMax() *
                                        (config.getK1() * sigma_i + config.getK3() * powf(sigma_i, 3)),
                                    2) +
                               1);
            omega_ast_p[i] = -f_i * sigmaDot_BR[i];
        }
    }

    RateCmdMsgF32Payload out{};
    eigenVectorToCArray(omega_ast, out.omega_BastR_B);
    eigenVectorToCArray(omega_ast_p, out.omegap_BastR_B);
    return out;
}

inline void testMrpSteeringConfigConstruction() {
    // --- Valid construction: a successfully created config means parameters are good ---
    EXPECT_NO_THROW(MrpSteeringConfig(0.15, 1.0, 0.5, false));
    EXPECT_NO_THROW(MrpSteeringConfig(0.15, 1.0, 0.5, true));

    // Boundary: K1 and K3 can be zero, omegaMax must be strictly positive
    EXPECT_NO_THROW(MrpSteeringConfig(0.0, 0.0, 0.001, false));

    // Factory method also succeeds for valid params
    EXPECT_NO_THROW(MrpSteeringAlgorithm::createConfig(0.15, 1.0, 0.5, false));

    // --- Invalid construction: callers catch exceptions to detect bad parameters ---

    // Negative K1
    EXPECT_THROW(MrpSteeringConfig(-0.1, 1.0, 0.5, false), fs::invalid_argument);
    // Negative K3
    EXPECT_THROW(MrpSteeringConfig(0.15, -0.1, 0.5, false), fs::invalid_argument);
    // Zero omegaMax (must be positive)
    EXPECT_THROW(MrpSteeringConfig(0.15, 1.0, 0.0, false), fs::invalid_argument);
    // Negative omegaMax
    EXPECT_THROW(MrpSteeringConfig(0.15, 1.0, -0.1, false), fs::invalid_argument);

    // Factory method throws identically
    EXPECT_THROW(MrpSteeringAlgorithm::createConfig(-0.1, 0.0, 0.5, false), fs::invalid_argument);
    EXPECT_THROW(MrpSteeringAlgorithm::createConfig(0.0, -0.1, 0.5, false), fs::invalid_argument);
    EXPECT_THROW(MrpSteeringAlgorithm::createConfig(0.0, 0.0, 0.0, false), fs::invalid_argument);
    EXPECT_THROW(MrpSteeringAlgorithm::createConfig(0.0, 0.0, -0.1, false), fs::invalid_argument);
}

inline void testMrpSteeringConfigValues() {
    // Config stores the exact values it was constructed with
    MrpSteeringConfig config(0.15, 1.0, 0.5, true);
    EXPECT_NEAR(config.getK1(), 0.15, 1e-6);
    EXPECT_NEAR(config.getK3(), 1.0, 1e-6);
    EXPECT_NEAR(config.getOmegaMax(), 0.5, 1e-6);
    EXPECT_TRUE(config.getIgnoreOuterLoopFeedforward());

    MrpSteeringConfig config2(0.0, 0.0, 0.001, false);
    EXPECT_NEAR(config2.getK1(), 0.0, 1e-6);
    EXPECT_NEAR(config2.getK3(), 0.0, 1e-6);
    EXPECT_NEAR(config2.getOmegaMax(), 0.001, 1e-6);
    EXPECT_FALSE(config2.getIgnoreOuterLoopFeedforward());
}

inline void testMrpSteeringConfigValidators() {
    // Static validators let callers pre-check parameters before attempting construction
    EXPECT_TRUE(MrpSteeringConfig::isValidK1(0.0));
    EXPECT_TRUE(MrpSteeringConfig::isValidK1(1.0));
    EXPECT_FALSE(MrpSteeringConfig::isValidK1(-0.01));

    EXPECT_TRUE(MrpSteeringConfig::isValidK3(0.0));
    EXPECT_TRUE(MrpSteeringConfig::isValidK3(1.0));
    EXPECT_FALSE(MrpSteeringConfig::isValidK3(-0.01));

    EXPECT_TRUE(MrpSteeringConfig::isValidOmegaMax(0.001));
    EXPECT_TRUE(MrpSteeringConfig::isValidOmegaMax(100.0));
    EXPECT_FALSE(MrpSteeringConfig::isValidOmegaMax(0.0));
    EXPECT_FALSE(MrpSteeringConfig::isValidOmegaMax(-0.01));
}

inline void testMrpSteeringConfigSetters() {
    MrpSteeringConfig config(1.0, 2.0, 3.0, false);

    // Valid setter updates value
    config.setK1(5.0);
    EXPECT_NEAR(config.getK1(), 5.0, 1e-6);

    // Invalid setter throws and preserves old value
    EXPECT_THROW(config.setK1(-1.0), fs::invalid_argument);
    EXPECT_NEAR(config.getK1(), 5.0, 1e-6);

    config.setK3(10.0);
    EXPECT_NEAR(config.getK3(), 10.0, 1e-6);
    EXPECT_THROW(config.setK3(-1.0), fs::invalid_argument);
    EXPECT_NEAR(config.getK3(), 10.0, 1e-6);

    config.setOmegaMax(7.0);
    EXPECT_NEAR(config.getOmegaMax(), 7.0, 1e-6);
    EXPECT_THROW(config.setOmegaMax(0.0), fs::invalid_argument);
    EXPECT_NEAR(config.getOmegaMax(), 7.0, 1e-6);
    EXPECT_THROW(config.setOmegaMax(-1.0), fs::invalid_argument);
    EXPECT_NEAR(config.getOmegaMax(), 7.0, 1e-6);

    config.setIgnoreOuterLoopFeedforward(true);
    EXPECT_TRUE(config.getIgnoreOuterLoopFeedforward());
    config.setIgnoreOuterLoopFeedforward(false);
    EXPECT_FALSE(config.getIgnoreOuterLoopFeedforward());
}

inline void testMrpSteeringAlgorithmConstruction() {
    // Algorithm constructed from valid config is immediately usable
    auto config = MrpSteeringAlgorithm::createConfig(0.15, 1.0, 0.5, true);
    MrpSteeringAlgorithm alg(config);

    EXPECT_NEAR(alg.getConfig().getK1(), 0.15, 1e-6);
    EXPECT_NEAR(alg.getConfig().getK3(), 1.0, 1e-6);
    EXPECT_NEAR(alg.getConfig().getOmegaMax(), 0.5, 1e-6);
    EXPECT_TRUE(alg.getConfig().getIgnoreOuterLoopFeedforward());

    // setConfig replaces the algorithm's config
    auto config2 = MrpSteeringAlgorithm::createConfig(0.3, 2.0, 1.0, false);
    alg.setConfig(config2);
    EXPECT_NEAR(alg.getConfig().getK1(), 0.3, 1e-6);
    EXPECT_NEAR(alg.getConfig().getK3(), 2.0, 1e-6);
    EXPECT_NEAR(alg.getConfig().getOmegaMax(), 1.0, 1e-6);
    EXPECT_FALSE(alg.getConfig().getIgnoreOuterLoopFeedforward());
}

inline void testMrpSteering(std::vector<float> sigma, float K1, float K3, float omegaMax, bool ignoreFF) {
    auto config = MrpSteeringAlgorithm::createConfig(K1, K3, omegaMax, ignoreFF);
    auto alg = MrpSteeringAlgorithm(config);

    Eigen::Vector3f sigma_BR = Eigen::Map<Eigen::Vector3f>(sigma.data());

    AttGuidMsgF32Payload msg{};
    eigenVectorToCArray(sigma_BR, msg.sigma_BR);

    // Reference
    RateCmdMsgF32Payload out{};
    RateCmdMsgF32Payload ref{};
    EXPECT_NO_THROW(out = alg.update(msg));
    EXPECT_NO_THROW(ref = referenceUpdate(config, msg));

    for (int i = 0; i < 3; ++i) {
        // --- General tests ---

        // Reference correctness
        EXPECT_NEAR(out.omega_BastR_B[i], ref.omega_BastR_B[i], 1e-6);
        EXPECT_NEAR(out.omegap_BastR_B[i], ref.omegap_BastR_B[i], 1e-6);

        // Finiteness
        EXPECT_TRUE(std::isfinite(out.omega_BastR_B[i]));
        EXPECT_TRUE(std::isfinite(out.omegap_BastR_B[i]));

        // --- Module specific tests ---

        // Omega smaller than specified max omega
        EXPECT_LE(std::abs(out.omega_BastR_B[i]), omegaMax + 1e-6);
    }

    // Linearity
    Eigen::Vector3f sigma1_BR = 0.01 * sigma_BR;
    AttGuidMsgF32Payload msg1{};
    eigenVectorToCArray(sigma1_BR, msg1.sigma_BR);
    RateCmdMsgF32Payload out1 = alg.update(msg1);
    float omegaMag1 = cArrayToEigenVector(out1.omega_BastR_B).norm();

    Eigen::Vector3f sigma2_BR = 0.1 * sigma_BR;
    AttGuidMsgF32Payload msg2{};
    eigenVectorToCArray(sigma2_BR, msg2.sigma_BR);
    RateCmdMsgF32Payload out2 = alg.update(msg2);
    float omegaMag2 = cArrayToEigenVector(out2.omega_BastR_B).norm();

    ASSERT_LT(omegaMag1, omegaMag2 + 1e-6);
}

#endif  // TEST_MRPSTEERING_H
