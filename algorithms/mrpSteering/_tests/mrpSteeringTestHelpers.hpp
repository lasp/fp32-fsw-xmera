#ifndef TEST_MRPSTEERING_H
#define TEST_MRPSTEERING_H

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "mrpSteeringAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <numbers>
#include <vector>

typedef struct {
    Eigen::Vector3f Lr;
    Eigen::Vector3f z;
} ReferenceOutput;

// Reference computation for update
ReferenceOutput referenceUpdate(const MrpSteeringAlgorithm& alg,
                                RWArrayConfigMsgF32Payload rwConfigParams,
                                const Eigen::Matrix3f& ISCPntB_B,
                                Eigen::Vector3f z,
                                const float dt,
                                AttGuidMsgF32Payload guidCmd,
                                const RWSpeedMsgF32Payload& wheelSpeeds,
                                const RWAvailabilityMsgPayload& wheelsAvailability) {
    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(guidCmd.sigma_BR);
    Eigen::Vector3f omega_BastR_B{};
    Eigen::Vector3f omegap_BastR_B{Eigen::Vector3f::Zero()};

    for (int i = 0; i < 3; ++i) {
        const float sigma_i = sigma_BR[i];
        const float f_i = atanf(static_cast<float>(std::numbers::pi) / 2 / alg.getOmegaMax() *
                                (alg.getK1() * sigma_i + alg.getK3() * powf(sigma_i, 3))) /
                          (static_cast<float>(std::numbers::pi) / 2) * alg.getOmegaMax();
        omega_BastR_B[i] = -f_i;
    }

    if (!alg.getIgnoreFeedforward()) {
        Eigen::Matrix3f B = bmatMrp(sigma_BR);

        Eigen::Vector3f sigmaDot_BR = 0.25 * B * omega_BastR_B;

        for (int i = 0; i < 3; ++i) {
            const float sigma_i = sigma_BR[i];
            const float f_i = (3 * alg.getK3() * powf(sigma_i, 2) + alg.getK1()) /
                              (powf(static_cast<float>(std::numbers::pi / 2) / alg.getOmegaMax() *
                                        (alg.getK1() * sigma_i + alg.getK3() * powf(sigma_i, 3)),
                                    2) +
                               1);
            omegap_BastR_B[i] = -f_i * sigmaDot_BR[i];
        }
    }

    const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(guidCmd.omega_BR_B);
    const Eigen::Vector3f omega_RN_B = cArrayToEigenVector(guidCmd.omega_RN_B);
    const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(guidCmd.domega_RN_B);

    /*! - compute body rate */
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    /*! - compute the rate tracking error */
    const Eigen::Vector3f omega_BastN_B = omega_BastR_B + omega_RN_B;
    const Eigen::Vector3f omega_BBast_B = omega_BN_B - omega_BastN_B;

    /*! - integrate rate tracking error  */
    if (alg.getKi() > 0.0F) { /* check if integral feedback is turned on  */
        z += omega_BBast_B * dt;
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float intLimCheck = fabs(z[i]);
            if (intLimCheck > alg.getIntegralLimit()) {
                z[i] *= alg.getIntegralLimit() / intLimCheck;
            }
        }
    } else {
        /* integral feedback is turned off through a negative gain setting */
        z = Eigen::Vector3f::Zero();
    }

    const Eigen::Matrix<float, 3, RW_EFF_CNT> G_s_B =
        cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(rwConfigParams.GsMatrix_B);

    Eigen::Vector3f H_B = ISCPntB_B * omega_BN_B;
    for (Eigen::Index i = 0; i < rwConfigParams.numRW; ++i) {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) { /* check if wheel is available */
            const Eigen::Vector3f G_s_B_i = G_s_B.col(i);
            const Eigen::Vector3f h_s_i =
                rwConfigParams.JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.wheelSpeeds[i]) * G_s_B_i;
            H_B += h_s_i;
        }
    }

    /*! - evaluate required attitude control torque Lc */
    const Eigen::Vector3f Lc = alg.getP() * omega_BBast_B + alg.getKi() * z - omega_BastN_B.cross(H_B) -
                               ISCPntB_B * (omegap_BastR_B + domega_RN_B - omega_BN_B.cross(omega_RN_B)) +
                               alg.getKnownTorquePntB_B();

    /* Change sign to compute the net positive control torque onto the spacecraft */
    const Eigen::Vector3f Lr = -Lc;

    ReferenceOutput out{};

    out.Lr = Lr;
    out.z = z;

    return out;
}

inline void testMrpSteeringSetup() {
    MrpSteeringAlgorithm alg{};

    // --- Test expected exceptions ---

    // Negative feedback gains or integral limit
    EXPECT_THROW(alg.setK1(-0.1), fs::invalid_argument);
    EXPECT_THROW(alg.setK3(-0.1), fs::invalid_argument);
    EXPECT_THROW(alg.setP(-0.1), fs::invalid_argument);
    EXPECT_THROW(alg.setKi(-0.1), fs::invalid_argument);
    EXPECT_THROW(alg.setIntegralLimit(-0.1), fs::invalid_argument);
    // Non-positive maximum rate
    EXPECT_THROW(alg.setOmegaMax(0.0), fs::invalid_argument);
    EXPECT_THROW(alg.setOmegaMax(-0.1), fs::invalid_argument);
    //Non-positive control period
    EXPECT_THROW(alg.setControlPeriod(-0.1), fs::invalid_argument);
    // Invalid inertia matrix
    Eigen::Matrix3f badInertia{};
    badInertia << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    EXPECT_THROW(alg.setSpacecraftInertia(badInertia), fs::invalid_argument);
    badInertia << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    EXPECT_THROW(alg.setSpacecraftInertia(badInertia), fs::invalid_argument);
    badInertia << 3, 0, 0, 0, 1, 0, 0, 0, 1;
    EXPECT_THROW(alg.setSpacecraftInertia(badInertia), fs::invalid_argument);
}

inline void testMrpSteering(std::vector<float> sigma,
                            float K1,
                            float K3,
                            float omegaMax,
                            bool ignoreFF,
                            float P,
                            float Ki,
                            float integralLimit,
                            std::vector<float> knownTorquePntB_B,
                            std::vector<float> omega_BR_B,
                            std::vector<float> omega_RN_B,
                            std::vector<float> domega_RN_B,
                            std::vector<float> wheelSpeedsVec,
                            std::vector<bool> wheelAvailabilityBool,
                            int numRW,
                            std::vector<float> JsList,
                            std::vector<float> GsMatrix_B,
                            std::vector<float> ISCPntB_B,
                            bool rwIsLinked,
                            float dt) {
    MrpSteeringAlgorithm alg{};

    alg.setK1(K1);
    alg.setK3(K3);
    alg.setOmegaMax(omegaMax);
    alg.setIgnoreFeedforward(ignoreFF);
    alg.setP(P);
    alg.setKi(Ki);
    alg.setIntegralLimit(integralLimit);
    alg.setKnownTorquePntB_B(cArrayToEigenVector3(knownTorquePntB_B.data()));
    alg.setControlPeriod(dt);

    Eigen::Matrix3f ISC_B = cArrayToEigenMatrix3(ISCPntB_B.data());
    // Try setting inertia matrix. If invalid, skip test.
    try {
        alg.setSpacecraftInertia(ISC_B);
    }
    catch (fs::invalid_argument) {
        return;
    }

    // Populate messages
    AttGuidMsgF32Payload guidCmdMsg{};
    std::copy(sigma.begin(), sigma.end(), guidCmdMsg.sigma_BR);
    std::copy(omega_BR_B.begin(), omega_BR_B.end(), guidCmdMsg.omega_BR_B);
    std::copy(omega_RN_B.begin(), omega_RN_B.end(), guidCmdMsg.omega_RN_B);
    std::copy(domega_RN_B.begin(), domega_RN_B.end(), guidCmdMsg.domega_RN_B);

    RWSpeedMsgF32Payload wheelSpeedsMsg{};
    std::copy(wheelSpeedsVec.begin(), wheelSpeedsVec.end(), wheelSpeedsMsg.wheelSpeeds);

    RWAvailabilityMsgPayload wheelsAvailabilityMsg{};
    for (uint32_t i = 0U; i < wheelAvailabilityBool.size(); ++i) {
        if (wheelAvailabilityBool[i]) {
            wheelsAvailabilityMsg.wheelAvailability[i] = UNAVAILABLE;
        }
    }

    RWArrayConfigMsgF32Payload rwConfigMsg{};
    if (rwIsLinked) {
        rwConfigMsg.numRW = numRW;
        std::copy(JsList.begin(), JsList.end(), rwConfigMsg.JsList);
        std::copy(GsMatrix_B.begin(), GsMatrix_B.end(), rwConfigMsg.GsMatrix_B);
    }

    VehicleConfigMsgF32Payload vehConfigMsg{};
    std::copy(ISCPntB_B.begin(), ISCPntB_B.end(), vehConfigMsg.ISCPntB_B);

    // populate module input structs
    InputGuidanceData attGuidInputData{};
    attGuidInputData.sigma_BR = Eigen::Map<Eigen::Vector3f>(sigma.data());
    attGuidInputData.omega_BR_B = Eigen::Map<Eigen::Vector3f>(omega_BR_B.data());
    attGuidInputData.omega_RN_B = Eigen::Map<Eigen::Vector3f>(omega_RN_B.data());
    attGuidInputData.domega_RN_B = Eigen::Map<Eigen::Vector3f>(domega_RN_B.data());

    std::array<float, RW_EFF_CNT> wheelSpeeds{};
    std::copy(wheelSpeedsVec.begin(), wheelSpeedsVec.end(), wheelSpeeds.begin());

    std::array<FSWdeviceAvailability, RW_EFF_CNT> wheelAvailability{};
    for (uint32_t i = 0U; i < wheelAvailabilityBool.size(); ++i) {
        if (wheelAvailabilityBool[i]) {
            wheelAvailability[i] = UNAVAILABLE;
        }
    }

    InputRwData rwInputData{};
    rwInputData.GsMatrix_B = Eigen::Map<Eigen::Matrix<float, 3, RW_EFF_CNT>>(GsMatrix_B.data());
    std::copy(std::begin(JsList),
              std::end(JsList),
              std::begin(rwInputData.JsList));
    rwInputData.numRW = static_cast<uint32_t>(numRW);

    // Reset module
    EXPECT_NO_THROW(alg.reset(rwInputData, rwIsLinked));

    Eigen::Vector3f z{Eigen::Vector3f::Zero()};

    // Test over a few time steps
    int numSteps = 5;

    for (int step = 0; step < numSteps; ++step) {
        // Reference
        Eigen::Vector3f out{};
        ReferenceOutput refOutput{};
        EXPECT_NO_THROW(out = alg.update(attGuidInputData, wheelSpeeds, wheelAvailability));
        EXPECT_NO_THROW(
            refOutput = referenceUpdate(
                alg, rwConfigMsg, ISC_B, z, dt, guidCmdMsg, wheelSpeedsMsg, wheelsAvailabilityMsg));
        Eigen::Vector3f ref = refOutput.Lr;
        z = refOutput.z;

        for (int i = 0; i < 3; ++i) {
            // --- General tests ---

            // Reference correctness
            EXPECT_NEAR(out[i], ref[i], 1e-6);

            // Finiteness
            EXPECT_TRUE(std::isfinite(out[i]));
        }
    }
}

#endif  // TEST_MRPSTEERING_H
