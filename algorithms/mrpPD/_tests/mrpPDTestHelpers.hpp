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
inline Eigen::Vector3f referenceUpdate(const MrpPDAlgorithm& alg, const InputGuidanceData& msg) {
    const Eigen::Vector3f sigma_BR = msg.sigma_BR;
    const Eigen::Vector3f omega_BR_B = msg.omega_BR_B;
    const Eigen::Vector3f omega_RN_B = msg.omega_RN_B;
    const Eigen::Vector3f domega_RN_B = msg.domega_RN_B;
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    const Eigen::Vector3f Lr = -alg.getProportionalGainK() * sigma_BR - alg.getDerivativeGainP() * omega_BR_B +
                               alg.getSpacecraftInertia() * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                               alg.getKnownTorquePntB_B();

    return Lr;
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

    InputGuidanceData inputs{};
    inputs.sigma_BR = Eigen::Map<Eigen::Vector3f>(sigma_BR.data());
    inputs.omega_BR_B = Eigen::Map<Eigen::Vector3f>(omega_BR_B.data());
    inputs.omega_RN_B = Eigen::Map<Eigen::Vector3f>(omega_RN_B.data());
    inputs.domega_RN_B = Eigen::Map<Eigen::Vector3f>(domega_RN_B.data());

    // Reference
    Eigen::Vector3f outputTorque = Eigen::Vector3f::Zero();
    Eigen::Vector3f referenceTorque = Eigen::Vector3f::Zero();
    EXPECT_NO_THROW(outputTorque = alg.update(inputs));
    EXPECT_NO_THROW(referenceTorque = referenceUpdate(alg, inputs));

    for (int i = 0; i < 3; ++i) {
        // Reference correctness
        EXPECT_NEAR(outputTorque[i], referenceTorque[i], 1e-6);

        // Finiteness
        EXPECT_TRUE(std::isfinite(outputTorque[i]));
    }
}

#endif  // TEST_MRPPD_H
