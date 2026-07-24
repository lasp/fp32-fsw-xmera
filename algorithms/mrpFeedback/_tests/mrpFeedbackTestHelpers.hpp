#ifndef TEST_MRPFEEDBACK_H
#define TEST_MRPFEEDBACK_H

#include "mrpFeedbackAlgorithm.h"
#include "mrpFeedbackTypes.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWAvailabilityMsgPayload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

struct ReferenceOutput {
    MrpFeedbackOutput mrpFeedbackOut;
    Eigen::Vector3f int_sigma;
};

inline ReferenceOutput referenceUpdate(const MrpFeedbackConfig& cfg,
                                       const RWArrayConfigMsgF32Payload& rwConfigParams,
                                       const Eigen::Matrix3f& ISCPntB_B,
                                       Eigen::Vector3f int_sigma,
                                       AttGuidMsgF32Payload guidCmd,
                                       const RWSpeedMsgF32Payload& wheelSpeeds,
                                       const RWAvailabilityMsgPayload& wheelsAvailability) {
    const MrpFeedbackControlParameters& params = cfg.getControlParameters();
    const float K = params.K;
    const float P = params.P;
    const float Ki = params.Ki;
    const float integralLimit = params.integralLimit;
    const ControlLawType controlLawType = params.controlLawType;
    const Eigen::Vector3f knownTorquePntB_B = cfg.getKnownTorquePntB_B();

    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(guidCmd.sigma_BR);
    const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(guidCmd.omega_BR_B);
    const Eigen::Vector3f omega_RN_B = cArrayToEigenVector(guidCmd.omega_RN_B);
    const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(guidCmd.domega_RN_B);

    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    Eigen::Vector3f z{Eigen::Vector3f::Zero()};
    if (Ki > 0.0F) {
        int_sigma += K * params.controlPeriod * sigma_BR;
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
    return out;
}

inline MrpFeedbackControlParameters makeValidControlParameters() {
    return MrpFeedbackControlParameters{
        .K = 0.0F,
        .P = 0.0F,
        .Ki = 0.0F,
        .integralLimit = 0.0F,
        .controlLawType = ControlLawType::NORMAL,
        .controlPeriod = 0.1F,
    };
}

inline void testMrpFeedbackSetup() {
    // Valid config builds without throwing.
    EXPECT_NO_THROW({
        const MrpFeedbackConfig cfg = MrpFeedbackConfig::create(makeValidControlParameters(), Eigen::Vector3f::Zero());
        const MrpFeedbackAlgorithm alg(cfg);
        (void)alg;
    });

    // Negative gains/limit are rejected by the Config factory.
    for (float MrpFeedbackControlParameters::* gain : {&MrpFeedbackControlParameters::K,
                                                       &MrpFeedbackControlParameters::P,
                                                       &MrpFeedbackControlParameters::Ki,
                                                       &MrpFeedbackControlParameters::integralLimit}) {
        MrpFeedbackControlParameters params = makeValidControlParameters();
        params.*gain = -0.1F;
        EXPECT_ANY_THROW({ (void)MrpFeedbackConfig::create(params, Eigen::Vector3f::Zero()); });
    }

    // Non-positive control period is rejected.
    for (const float badPeriod : {0.0F, -0.1F}) {
        MrpFeedbackControlParameters params = makeValidControlParameters();
        params.controlPeriod = badPeriod;
        EXPECT_ANY_THROW({ (void)MrpFeedbackConfig::create(params, Eigen::Vector3f::Zero()); });
    }
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
                            float controlPeriod) {
    const ControlLawType controlLawTypeAlg =
        (controlLawType == 0) ? ControlLawType::NORMAL : ControlLawType::SIMPLE_INTEGRAL;

    const MrpFeedbackControlParameters params{
        .K = K,
        .P = P,
        .Ki = Ki,
        .integralLimit = integralLimit,
        .controlLawType = controlLawTypeAlg,
        .controlPeriod = controlPeriod,
    };
    const MrpFeedbackConfig cfg = MrpFeedbackConfig::create(params, knownTorquePntB_B);
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

    RWArrayConfigMsgF32Payload rwConfigMsg{};
    if (rwIsLinked) {
        rwConfigMsg.numRW = numRW;
        std::copy(uMax.begin(), uMax.end(), rwConfigMsg.uMax);
        std::copy(JsList.begin(), JsList.end(), rwConfigMsg.JsList);
        std::copy(GsMatrix_B.begin(), GsMatrix_B.end(), rwConfigMsg.GsMatrix_B);
    }

    VehicleConfigMsgF32Payload vehConfigMsg{};
    std::copy(ISCPntB_B.begin(), ISCPntB_B.end(), vehConfigMsg.ISCPntB_B);

    const Eigen::Matrix3f ISC_B = cArrayToEigenMatrix3(ISCPntB_B.data());

    EXPECT_NO_THROW(alg.reset(vehConfigMsg, rwConfigMsg, rwIsLinked));

    Eigen::Vector3f int_sigma{Eigen::Vector3f::Zero()};

    constexpr int numSteps = 5;
    for (int step = 0; step < numSteps; ++step) {
        MrpFeedbackOutput out{};
        ReferenceOutput refOutput{};
        EXPECT_NO_THROW(out = alg.update(guidCmdMsg, wheelSpeedsMsg, wheelsAvailabilityMsg));
        EXPECT_NO_THROW(refOutput = referenceUpdate(
                            cfg, rwConfigMsg, ISC_B, int_sigma, guidCmdMsg, wheelSpeedsMsg, wheelsAvailabilityMsg));
        const MrpFeedbackOutput ref = refOutput.mrpFeedbackOut;
        int_sigma = refOutput.int_sigma;

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.controlOut.torqueRequestBody[i], ref.controlOut.torqueRequestBody[i], 1e-6);
            EXPECT_NEAR(out.intFeedbackOut.torqueRequestBody[i], ref.intFeedbackOut.torqueRequestBody[i], 1e-6);

            EXPECT_TRUE(std::isfinite(out.controlOut.torqueRequestBody[i]));
            EXPECT_TRUE(std::isfinite(out.intFeedbackOut.torqueRequestBody[i]));
        }
    }
}

#endif  // TEST_MRPFEEDBACK_H
