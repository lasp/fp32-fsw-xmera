#ifndef TEST_MRPFEEDBACK_H
#define TEST_MRPFEEDBACK_H

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "mrpFeedbackAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/timeConstants.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <numbers>
#include <vector>

typedef struct {
    MrpFeedbackOutput mrpFeedbackOut;
    Eigen::Vector3f int_sigma;
    uint64_t priorTime;
} ReferenceOutput;

// Reference computation for update
ReferenceOutput referenceUpdate(const MrpFeedbackAlgorithm& alg,
                                RWArrayConfigMsgF32Payload rwConfigParams,
                                const Eigen::Matrix3f& ISCPntB_B,
                                Eigen::Vector3f int_sigma,
                                uint64_t priorTime,
                                const uint64_t callTime,
                                AttGuidMsgF32Payload guidCmd,
                                const RWSpeedMsgF32Payload& wheelSpeeds,
                                const RWAvailabilityMsgPayload& wheelsAvailability) {
    const float K = alg.getK();
    const float P = alg.getP();
    const float Ki = alg.getKi();
    const float integralLimit = alg.getIntegralLimit();
    const ControlLawType controlLawType = alg.getControlLawType();
    const Eigen::Vector3f knownTorquePntB_B = alg.getKnownTorquePntB_B();

    /*! - compute control update time */
    float dt{}; /* [s] control update period */
    if (priorTime == 0U) {
        dt = 0.0F;
    } else {
        dt = static_cast<float>(static_cast<double>(callTime - priorTime) * kNano2Sec);
    }
    priorTime = callTime;

    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(guidCmd.sigma_BR);
    const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(guidCmd.omega_BR_B);
    const Eigen::Vector3f omega_RN_B = cArrayToEigenVector(guidCmd.omega_RN_B);
    const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(guidCmd.domega_RN_B);

    /*! - compute body rate */
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    /*! - evaluate integral term */
    Eigen::Vector3f z{Eigen::Vector3f::Zero()};
    if (Ki > 0.0F) { /* check if integral feedback is turned on  */
        int_sigma += K * dt * sigma_BR;

        /* keep int_sigma less than integralLimit */
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float intCheck = fabsf(int_sigma[i]);
            if (intCheck > integralLimit) {
                int_sigma[i] *= integralLimit / intCheck;
            }
        }
        z = int_sigma + ISCPntB_B * omega_BR_B;
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

    Eigen::Vector3f momentumContribution{};
    if (controlLawType == ControlLawType::NORMAL) {
        momentumContribution = (omega_RN_B + Ki * z).cross(H_B);
    } else {
        momentumContribution = omega_BN_B.cross(H_B);
    }

    /*! - evaluate required attitude control torque Lc */
    const Eigen::Vector3f Lc = K * sigma_BR + P * omega_BR_B + P * Ki * z - momentumContribution +
                               ISCPntB_B * (omega_BN_B.cross(omega_RN_B) - domega_RN_B) + knownTorquePntB_B;

    const Eigen::Vector3f Lr = -Lc;
    const Eigen::Vector3f Li = -(P * Ki * z);

    ReferenceOutput out{};

    eigenVectorToCArray(Lr, out.mrpFeedbackOut.controlOut.torqueRequestBody);
    eigenVectorToCArray(Li, out.mrpFeedbackOut.intFeedbackOut.torqueRequestBody);
    out.int_sigma = int_sigma;
    out.priorTime = priorTime;

    return out;
}

inline void testMrpFeedbackSetup() {
    MrpFeedbackAlgorithm alg{};

    // --- Test expected exceptions ---

    // Negative feedback gains or integral limit
    EXPECT_THROW(alg.setK(-0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setP(-0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setKi(-0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setIntegralLimit(-0.1), fsw::invalid_argument);
}

inline void testMrpFeedback(const Eigen::Vector3f& sigma,
                            float K,
                            float P,
                            float Ki,
                            float integralLimit,
                            int controlLawType,
                            const Eigen::Vector3f& knownTorquePntB_B,
                            const Eigen::Vector3f& omega_BR_B,
                            const Eigen::Vector3f& omega_RN_B,
                            const Eigen::Vector3f& domega_RN_B,
                            std::vector<float> wheelSpeeds,
                            std::vector<bool> wheelAvailabilityBool,
                            int numRW,
                            std::vector<float> uMax,
                            std::vector<float> JsList,
                            std::vector<float> GsMatrix_B,
                            std::vector<float> ISCPntB_B,
                            bool rwIsLinked,
                            float dt) {
    MrpFeedbackAlgorithm alg{};

    ControlLawType controlLawTypeAlg{};
    if (controlLawType == 0) {
        controlLawTypeAlg = ControlLawType::NORMAL;
    } else {
        controlLawTypeAlg = ControlLawType::SIMPLE_INTEGRAL;
    }

    alg.setK(K);
    alg.setP(P);
    alg.setKi(Ki);
    alg.setIntegralLimit(integralLimit);
    alg.setControlLawType(controlLawTypeAlg);
    alg.setKnownTorquePntB_B(knownTorquePntB_B);

    // Populate messages
    AttGuidMsgF32Payload guidCmdMsg{};
    eigenVectorToCArray(sigma, guidCmdMsg.sigma_BR);
    eigenVectorToCArray(omega_BR_B, guidCmdMsg.omega_BR_B);
    eigenVectorToCArray(omega_RN_B, guidCmdMsg.omega_RN_B);
    eigenVectorToCArray(domega_RN_B, guidCmdMsg.domega_RN_B);

    RWSpeedMsgF32Payload wheelSpeedsMsg{};
    std::copy(wheelSpeeds.begin(), wheelSpeeds.end(), wheelSpeedsMsg.wheelSpeeds);

    RWAvailabilityMsgPayload wheelsAvailabilityMsg{};
    for (uint32_t i = 0U; i < wheelAvailabilityBool.size(); ++i) {
        if (wheelAvailabilityBool[i]) {
            wheelsAvailabilityMsg.wheelAvailability[i] = UNAVAILABLE;
        }
    }

    RWArrayConfigMsgF32Payload rwConfigMsg{};
    if (rwIsLinked) {
        rwConfigMsg.numRW = numRW;
        std::copy(uMax.begin(), uMax.end(), rwConfigMsg.uMax);
        std::copy(JsList.begin(), JsList.end(), rwConfigMsg.JsList);
        std::copy(GsMatrix_B.begin(), GsMatrix_B.end(), rwConfigMsg.GsMatrix_B);
    }

    VehicleConfigMsgF32Payload vehConfigMsg{};
    std::copy(ISCPntB_B.begin(), ISCPntB_B.end(), vehConfigMsg.ISCPntB_B);

    // Remaining variables needed by module
    Eigen::Matrix3f ISC_B = cArrayToEigenMatrix3(ISCPntB_B.data());

    // Reset module
    EXPECT_NO_THROW(alg.reset(vehConfigMsg, rwConfigMsg, rwIsLinked));

    Eigen::Vector3f int_sigma{Eigen::Vector3f::Zero()};
    uint64_t priorTime{};

    // Test over a few time steps
    int numSteps = 5;

    for (int step = 0; step < numSteps; ++step) {
        uint64_t callTime = priorTime + static_cast<uint64_t>(dt / kNano2Sec);

        // Reference
        MrpFeedbackOutput out{};
        ReferenceOutput refOutput{};
        EXPECT_NO_THROW(out = alg.update(callTime, guidCmdMsg, wheelSpeedsMsg, wheelsAvailabilityMsg));
        EXPECT_NO_THROW(refOutput = referenceUpdate(alg,
                                                    rwConfigMsg,
                                                    ISC_B,
                                                    int_sigma,
                                                    priorTime,
                                                    callTime,
                                                    guidCmdMsg,
                                                    wheelSpeedsMsg,
                                                    wheelsAvailabilityMsg));
        MrpFeedbackOutput ref = refOutput.mrpFeedbackOut;
        int_sigma = refOutput.int_sigma;
        priorTime = refOutput.priorTime;

        for (int i = 0; i < 3; ++i) {
            // --- General tests ---

            // Reference correctness
            EXPECT_NEAR(out.controlOut.torqueRequestBody[i], ref.controlOut.torqueRequestBody[i], 1e-6);
            EXPECT_NEAR(out.intFeedbackOut.torqueRequestBody[i], ref.intFeedbackOut.torqueRequestBody[i], 1e-6);

            // Finiteness
            EXPECT_TRUE(std::isfinite(out.controlOut.torqueRequestBody[i]));
            EXPECT_TRUE(std::isfinite(out.intFeedbackOut.torqueRequestBody[i]));
        }
    }
}

#endif  // TEST_MRPFEEDBACK_H
