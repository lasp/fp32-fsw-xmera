#ifndef TEST_MRPSTEERING_H
#define TEST_MRPSTEERING_H

#include "../freestandingInvalidArgument.h"
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
RateCmdMsgF32Payload referenceUpdate(const MrpSteeringAlgorithm& alg, AttGuidMsgF32Payload& msg) {
    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(msg.sigma_BR);
    Eigen::Vector3f omega_ast{};
    Eigen::Vector3f omega_ast_p{Eigen::Vector3f::Zero()};

    for (int i = 0; i < 3; ++i) {
        const float sigma_i = sigma_BR[i];
        const float f_i = atanf(static_cast<float>(std::numbers::pi) / 2 / alg.getOmegaMax() *
                                (alg.getK1() * sigma_i + alg.getK3() * powf(sigma_i, 3))) /
                          (static_cast<float>(std::numbers::pi) / 2) * alg.getOmegaMax();
        omega_ast[i] = -f_i;
    }

    if (!alg.getIgnoreFeedforward()) {
        Eigen::Matrix3f B = bmatMrp(sigma_BR);

        Eigen::Vector3f sigmaDot_BR = 0.25 * B * omega_ast;

        for (int i = 0; i < 3; ++i) {
            const float sigma_i = sigma_BR[i];
            const float f_i = (3 * alg.getK3() * powf(sigma_i, 2) + alg.getK1()) /
                              (powf(static_cast<float>(std::numbers::pi / 2) / alg.getOmegaMax() *
                                        (alg.getK1() * sigma_i + alg.getK3() * powf(sigma_i, 3)),
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

inline void testMrpSteeringSetup() {
    MrpSteeringAlgorithm alg{};

    // --- Test expected exceptions ---

    // Negative feedback gains
    EXPECT_THROW(alg.setK1(-0.1), fs::invalid_argument);
    EXPECT_THROW(alg.setK3(-0.1), fs::invalid_argument);
    // Non-positive maximum rate
    EXPECT_THROW(alg.setOmegaMax(0.0), fs::invalid_argument);
    EXPECT_THROW(alg.setOmegaMax(-0.1), fs::invalid_argument);
}

inline void testMrpSteering(std::vector<float> sigma, float K1, float K3, float omegaMax, bool ignoreFF) {
    MrpSteeringAlgorithm alg{};

    alg.setK1(K1);
    alg.setK3(K3);
    alg.setOmegaMax(omegaMax);
    alg.setIgnoreFeedforward(ignoreFF);

    Eigen::Vector3f sigma_BR = Eigen::Map<Eigen::Vector3f>(sigma.data());

    AttGuidMsgF32Payload msg{};
    eigenVectorToCArray(sigma_BR, msg.sigma_BR);

    // Reference
    RateCmdMsgF32Payload out{};
    RateCmdMsgF32Payload ref{};
    EXPECT_NO_THROW(out = alg.update(msg));
    EXPECT_NO_THROW(ref = referenceUpdate(alg, msg));

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
