#ifndef TEST_MRPPD_H
#define TEST_MRPPD_H

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "mrpPDAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <numbers>
#include <vector>

// Reference computation for update
CmdTorqueBodyMsgF32Payload referenceUpdate(const MrpPDAlgorithm& alg, AttGuidMsgF32Payload& msg) {
    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(msg.sigma_BR);
    const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(msg.omega_BR_B);
    const Eigen::Vector3f omega_RN_B = cArrayToEigenVector(msg.omega_RN_B);
    const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(msg.domega_RN_B);
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    const Eigen::Vector3f Lr = -alg.getProportionalGainK() * sigma_BR - alg.getDerivativeGainP() * omega_BR_B +
                               omega_RN_B.cross(alg.getSpacecraftInertia() * omega_BN_B) +
                               alg.getSpacecraftInertia() * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                               alg.getKnownTorquePntB_B();

    CmdTorqueBodyMsgF32Payload out{};
    eigenVectorToCArray(Lr, out.torqueRequestBody);
    return out;
}

inline void testMrpPDSetup() {
    MrpPDAlgorithm alg{};

    // --- Test expected exceptions ---

    // Negative feedback gains
    EXPECT_THROW(alg.setProportionalGainK(-0.1), fs::invalid_argument);
    EXPECT_THROW(alg.setDerivativeGainP(-0.1), fs::invalid_argument);

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

inline void regressionTestMrpPD(float K,
                                float P,
                                std::vector<float> torque,
                                std::vector<float> sigma_BR,
                                std::vector<float> omega_BR_B,
                                std::vector<float> omega_RN_B,
                                std::vector<float> domega_RN_B) {
    // --- Regression test using expected update algorithm ---

    MrpPDAlgorithm alg{};

    alg.setProportionalGainK(K);
    alg.setDerivativeGainP(P);
    alg.setKnownTorquePntB_B(Eigen::Map<Eigen::Vector3f>(torque.data()));

    Eigen::Vector3f sigma_BR_input(sigma_BR.data());
    Eigen::Vector3f omega_BR_B_input(omega_BR_B.data());
    Eigen::Vector3f omega_RN_B_input(omega_RN_B.data());
    Eigen::Vector3f domega_RN_B_input(domega_RN_B.data());

    AttGuidMsgF32Payload msg{};
    eigenVectorToCArray(sigma_BR_input, msg.sigma_BR);
    eigenVectorToCArray(omega_BR_B_input, msg.omega_BR_B);
    eigenVectorToCArray(omega_RN_B_input, msg.omega_RN_B);
    eigenVectorToCArray(domega_RN_B_input, msg.domega_RN_B);

    // Reference
    CmdTorqueBodyMsgF32Payload out{};
    CmdTorqueBodyMsgF32Payload ref{};
    EXPECT_NO_THROW(out = alg.update(msg));
    EXPECT_NO_THROW(ref = referenceUpdate(alg, msg));

    for (int i = 0; i < 3; ++i) {
        // Reference correctness
        EXPECT_NEAR(out.torqueRequestBody[i], ref.torqueRequestBody[i], 1e-6);

        // Finiteness
        EXPECT_TRUE(std::isfinite(out.torqueRequestBody[i]));
    }
}

inline void propertyTestMrpPD() {
    // --- Test module with targeted inputs to ensure individual terms are properly implemented ---
    MrpPDAlgorithm alg{};
    alg.setProportionalGainK(10);
    alg.setDerivativeGainP(200);

    Eigen::Vector3f torque = Eigen::Vector3f::Zero();
    Eigen::Vector3f sigma_BR = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BR_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_B = Eigen::Vector3f::Zero();

    AttGuidMsgF32Payload msg{};
    eigenVectorToCArray(sigma_BR, msg.sigma_BR);
    eigenVectorToCArray(omega_BR_B, msg.omega_BR_B);
    eigenVectorToCArray(omega_RN_B, msg.omega_RN_B);
    eigenVectorToCArray(domega_RN_B, msg.domega_RN_B);

    // All inputs are zeros except external torque should return external torque
    torque << 1, 2, 3;
    alg.setKnownTorquePntB_B(torque);
    CmdTorqueBodyMsgF32Payload out{};
    EXPECT_NO_THROW(out = alg.update(msg));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.torqueRequestBody[i], -torque[i], 1e-6);
    }

    // If input rates and accelerations are null, the torque is the input mrp scaled by proportional gain
    torque << 0, 0, 0;
    alg.setKnownTorquePntB_B(torque);
    sigma_BR << 0.5, 0.2, 0.1;
    eigenVectorToCArray(sigma_BR, msg.sigma_BR);
    EXPECT_NO_THROW(out = alg.update(msg));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.torqueRequestBody[i], -alg.getProportionalGainK() * sigma_BR[i], 1e-6);
    }

    // If input rates and accelerations are null, with Identity inertia matrix, the torque is the domega term
    sigma_BR << 0, 0, 0;
    eigenVectorToCArray(sigma_BR, msg.sigma_BR);
    domega_RN_B << 0.5, 0.2, 0.1;
    eigenVectorToCArray(domega_RN_B, msg.domega_RN_B);
    EXPECT_NO_THROW(out = alg.update(msg));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.torqueRequestBody[i], domega_RN_B[i], 1e-6);
    }

    // If all but omega_BR_B null, the torque is omega_BR_B scaled by derivative gain
    domega_RN_B << 0, 0, 0;
    eigenVectorToCArray(domega_RN_B, msg.domega_RN_B);
    omega_BR_B << -0.3, 0.1, -0.8;
    eigenVectorToCArray(omega_BR_B, msg.omega_BR_B);
    EXPECT_NO_THROW(out = alg.update(msg));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.torqueRequestBody[i], -alg.getDerivativeGainP() * omega_BR_B[i], 1e-6);
    }

    // If everything is zero except omega_BR_B and omega_RN_B, the torque is twice the cross product of their sum
    alg.setDerivativeGainP(0);
    omega_BR_B << 0.0, 0.9, -0.2;
    eigenVectorToCArray(omega_BR_B, msg.omega_BR_B);
    omega_RN_B << 1.2, 0, 0;
    eigenVectorToCArray(omega_RN_B, msg.omega_RN_B);
    EXPECT_NO_THROW(out = alg.update(msg));
    Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.torqueRequestBody[i], 2 * omega_RN_B.cross(omega_BN_B)[i], 1e-6);
    }
}

#endif  // TEST_MRPPD_H
