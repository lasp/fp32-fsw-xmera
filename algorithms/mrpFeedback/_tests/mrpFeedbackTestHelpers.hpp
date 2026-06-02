#ifndef TEST_MRPFEEDBACK_H
#define TEST_MRPFEEDBACK_H

#include "architecture/utilities/eigenSupport.h"
#include "mrpFeedbackAlgorithm.h"
#include "mrpFeedbackTypes.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWAvailabilityMsgPayload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/timeConstants.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

struct ReferenceOutput {
    MrpFeedbackOutput mrpFeedbackOut;
    Eigen::Vector3f int_sigma;
    uint64_t priorTime;
};

inline ReferenceOutput referenceUpdate(const MrpFeedbackConfig& cfg,
                                       const RWArrayConfigMsgF32Payload& rwConfigParams,
                                       const Eigen::Matrix3f& ISCPntB_B,
                                       Eigen::Vector3f int_sigma,
                                       uint64_t priorTime,
                                       const uint64_t callTime,
                                       AttGuidMsgF32Payload guidCmd,
                                       const RWSpeedMsgF32Payload& wheelSpeeds,
                                       const RWAvailabilityMsgPayload& wheelsAvailability) {
    const float K = cfg.getK();
    const float P = cfg.getP();
    const float Ki = cfg.getKi();
    const float integralLimit = cfg.getIntegralLimit();
    const ControlLawType controlLawType = cfg.getControlLawType();
    const Eigen::Vector3f knownTorquePntB_B = cfg.getKnownTorquePntB_B();

    float dt{};
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

    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    Eigen::Vector3f z{Eigen::Vector3f::Zero()};
    if (Ki > 0.0F) {
        int_sigma += K * dt * sigma_BR;
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
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) {
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
    const Eigen::Matrix3f validInertia = Eigen::Matrix3f::Identity();

    // Valid config builds without throwing.
    EXPECT_NO_THROW({
        const MrpFeedbackConfig cfg = MrpFeedbackConfig::create(
            0.0F, 0.0F, 0.0F, 0.0F, ControlLawType::NORMAL, Eigen::Vector3f::Zero(), validInertia);
        const MrpFeedbackAlgorithm alg(cfg);
        (void)alg;
    });

    // Negative gains/limit are rejected by the Config factory.
    EXPECT_ANY_THROW({
        (void)MrpFeedbackConfig::create(
            -0.1F, 0.0F, 0.0F, 0.0F, ControlLawType::NORMAL, Eigen::Vector3f::Zero(), validInertia);
    });
    EXPECT_ANY_THROW({
        (void)MrpFeedbackConfig::create(
            0.0F, -0.1F, 0.0F, 0.0F, ControlLawType::NORMAL, Eigen::Vector3f::Zero(), validInertia);
    });
    EXPECT_ANY_THROW({
        (void)MrpFeedbackConfig::create(
            0.0F, 0.0F, -0.1F, 0.0F, ControlLawType::NORMAL, Eigen::Vector3f::Zero(), validInertia);
    });
    EXPECT_ANY_THROW({
        (void)MrpFeedbackConfig::create(
            0.0F, 0.0F, 0.0F, -0.1F, ControlLawType::NORMAL, Eigen::Vector3f::Zero(), validInertia);
    });

    // A non-physical inertia (here all-zero: not positive-definite) is rejected by the Config factory.
    EXPECT_ANY_THROW({
        (void)MrpFeedbackConfig::create(
            0.0F, 0.0F, 0.0F, 0.0F, ControlLawType::NORMAL, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Zero());
    });
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
    const ControlLawType controlLawTypeAlg =
        (controlLawType == 0) ? ControlLawType::NORMAL : ControlLawType::SIMPLE_INTEGRAL;

    const Eigen::Matrix3f ISC_B = cArrayToEigenMatrix3(ISCPntB_B.data());

    // The RW array config is now part of the immutable config. Build the (fully RW_EFF_CNT-sized)
    // payload first so it stays the canonical source for both the config inputs and the reference.
    RWArrayConfigMsgF32Payload rwConfigMsg{};
    if (rwIsLinked) {
        rwConfigMsg.numRW = numRW;
        std::copy(uMax.begin(), uMax.end(), rwConfigMsg.uMax);
        std::copy(JsList.begin(), JsList.end(), rwConfigMsg.JsList);
        std::copy(GsMatrix_B.begin(), GsMatrix_B.end(), rwConfigMsg.GsMatrix_B);
    }
    const Eigen::Matrix<float, 3, RW_EFF_CNT> Gs_B = cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(rwConfigMsg.GsMatrix_B);
    std::array<float, RW_EFF_CNT> jsListArr{};
    std::copy(std::begin(rwConfigMsg.JsList), std::end(rwConfigMsg.JsList), jsListArr.begin());

    // Inertia is now part of the validated config. The fuzz harness feeds arbitrary 3x3 data, so a
    // non-physical inertia must be rejected by the Config factory; confirm that and stop.
    if (!inertiaIsValid(ISC_B)) {
        EXPECT_ANY_THROW({
            (void)MrpFeedbackConfig::create(K,
                                            P,
                                            Ki,
                                            integralLimit,
                                            controlLawTypeAlg,
                                            knownTorquePntB_B,
                                            ISC_B,
                                            rwConfigMsg.numRW,
                                            Gs_B,
                                            jsListArr);
        });
        return;
    }

    const MrpFeedbackConfig cfg = MrpFeedbackConfig::create(
        K, P, Ki, integralLimit, controlLawTypeAlg, knownTorquePntB_B, ISC_B, rwConfigMsg.numRW, Gs_B, jsListArr);
    MrpFeedbackAlgorithm alg(cfg);

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

    EXPECT_NO_THROW(alg.reset());

    Eigen::Vector3f int_sigma{Eigen::Vector3f::Zero()};
    uint64_t priorTime{};

    constexpr int numSteps = 5;
    for (int step = 0; step < numSteps; ++step) {
        const uint64_t callTime = priorTime + static_cast<uint64_t>(dt / kNano2Sec);

        MrpFeedbackOutput out{};
        ReferenceOutput refOutput{};
        EXPECT_NO_THROW(out = alg.update(callTime, guidCmdMsg, wheelSpeedsMsg, wheelsAvailabilityMsg));
        EXPECT_NO_THROW(refOutput = referenceUpdate(cfg,
                                                    rwConfigMsg,
                                                    ISC_B,
                                                    int_sigma,
                                                    priorTime,
                                                    callTime,
                                                    guidCmdMsg,
                                                    wheelSpeedsMsg,
                                                    wheelsAvailabilityMsg));
        const MrpFeedbackOutput ref = refOutput.mrpFeedbackOut;
        int_sigma = refOutput.int_sigma;
        priorTime = refOutput.priorTime;

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.controlOut.torqueRequestBody[i], ref.controlOut.torqueRequestBody[i], 1e-6);
            EXPECT_NEAR(out.intFeedbackOut.torqueRequestBody[i], ref.intFeedbackOut.torqueRequestBody[i], 1e-6);

            EXPECT_TRUE(std::isfinite(out.controlOut.torqueRequestBody[i]));
            EXPECT_TRUE(std::isfinite(out.intFeedbackOut.torqueRequestBody[i]));
        }
    }
}

#endif  // TEST_MRPFEEDBACK_H
