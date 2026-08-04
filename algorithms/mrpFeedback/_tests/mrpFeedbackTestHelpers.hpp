#ifndef TEST_MRPFEEDBACK_H
#define TEST_MRPFEEDBACK_H

#include "mrpFeedbackAlgorithm.h"
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
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
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
            const Eigen::Vector3f G_s_B_i = G_s_B.col(i).normalized();
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

    ReferenceOutput out{};
    out.mrpFeedbackOut.controlTorque = -Lc;
    out.mrpFeedbackOut.integralFeedbackTorque = -(P * Ki * z);
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
    const Eigen::Vector3f knownTorque = Eigen::Vector3f::Zero();
    const Eigen::Matrix3f goodInertia = Eigen::Matrix3f::Identity();

    // Valid config builds without throwing.
    EXPECT_NO_THROW({
        const MrpFeedbackConfig cfg = MrpFeedbackConfig::create(makeValidControlParameters(), knownTorque, goodInertia);
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
        EXPECT_ANY_THROW({ (void)MrpFeedbackConfig::create(params, knownTorque, goodInertia); });
    }

    // Non-positive control period is rejected.
    for (const float badPeriod : {0.0F, -0.1F}) {
        MrpFeedbackControlParameters params = makeValidControlParameters();
        params.controlPeriod = badPeriod;
        EXPECT_ANY_THROW({ (void)MrpFeedbackConfig::create(params, knownTorque, goodInertia); });
    }

    // Invalid inertia matrices are rejected.
    Eigen::Matrix3f badInertia{};
    badInertia << 1, 0, 0, 0, 1, 0, 0, 0, 0;  // singular
    EXPECT_ANY_THROW({ (void)MrpFeedbackConfig::create(makeValidControlParameters(), knownTorque, badInertia); });
    badInertia << 1, 0, 0, 0, 1, 0, 0, 1, 1;  // asymmetric
    EXPECT_ANY_THROW({ (void)MrpFeedbackConfig::create(makeValidControlParameters(), knownTorque, badInertia); });
    badInertia << 3, 0, 0, 0, 1, 0, 0, 0, 1;  // violates triangle inequality
    EXPECT_ANY_THROW({ (void)MrpFeedbackConfig::create(makeValidControlParameters(), knownTorque, badInertia); });
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
                            std::vector<float> JsList,
                            std::vector<float> GsMatrix_B,
                            std::vector<float> ISCPntB_B,
                            bool rwIsLinked,
                            float controlPeriod) {
    // The payloads size their RW arrays from RW_EFF_CNT, so no caller can describe more wheels than the
    // mission holds. Fail loudly instead of writing past those arrays, and treat a short input as an error
    // rather than reading past its end.
    constexpr auto maxRw = static_cast<size_t>(RW_EFF_CNT);
    ASSERT_GE(numRW, 0);
    ASSERT_LE(numRW, RW_EFF_CNT);
    ASSERT_EQ(ISCPntB_B.size(), 9U);

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
    const Eigen::Matrix3f ISC_B = cArrayToEigenMatrix3(ISCPntB_B.data());

    // Build the RW spin-axis configuration, mirroring the adapter: it is only populated when the RW config
    // message is linked. Fill provided entries column-major into a zero matrix (matching the messaging-layer
    // Eigen::Map layout) so a short GsMatrix_B vector never reads out of bounds.
    MrpFeedbackInputRwData rwInputData{};
    if (rwIsLinked) {
        const std::size_t numGs = std::min<std::size_t>(GsMatrix_B.size(), static_cast<std::size_t>(RW_EFF_CNT) * 3U);
        for (std::size_t k = 0; k < numGs; ++k) {
            rwInputData.GsMatrix_B(static_cast<Eigen::Index>(k % 3), static_cast<Eigen::Index>(k / 3)) = GsMatrix_B[k];
        }
        std::copy(std::begin(JsList), std::end(JsList), std::begin(rwInputData.JsList));
        rwInputData.numRW = static_cast<uint32_t>(numRW);
        for (uint32_t i = 0U; i < wheelAvailabilityBool.size(); ++i) {
            if (wheelAvailabilityBool[i]) {
                rwInputData.wheelAvailability[i] = UNAVAILABLE;
            }
        }

        // The config requires (near-)unit spin axes; normalize the active columns before constructing it. Skip
        // inputs with a degenerate (near-zero) spin axis that cannot be normalized.
        for (uint32_t i = 0U; i < rwInputData.numRW; ++i) {
            const float colNorm = rwInputData.GsMatrix_B.col(static_cast<int>(i)).norm();
            if (colNorm < 1e-6F) {
                return;
            }
            rwInputData.GsMatrix_B.col(static_cast<int>(i)) /= colNorm;
        }
    }

    // Skip cases whose inertia matrix fails validation (e.g. random fuzz inputs).
    std::optional<MrpFeedbackConfig> config;
    try {
        config =
            MrpFeedbackConfig::create(params,
                                      knownTorquePntB_B,
                                      ISC_B,
                                      rwIsLinked ? std::optional<MrpFeedbackInputRwData>(rwInputData) : std::nullopt);
    } catch (const fsw::invalid_argument&) {
        return;
    }
    const MrpFeedbackConfig& cfg = *config;
    MrpFeedbackAlgorithm alg{cfg};

    AttGuidMsgF32Payload guidCmdMsg{};
    eigenVectorToCArray(sigma, guidCmdMsg.sigma_BR);
    eigenVectorToCArray(omega_BR_B, guidCmdMsg.omega_BR_B);
    eigenVectorToCArray(omega_RN_B, guidCmdMsg.omega_RN_B);
    eigenVectorToCArray(domega_RN_B, guidCmdMsg.domega_RN_B);

    RWSpeedMsgF32Payload wheelSpeedsMsg{};
    std::copy_n(wheelSpeeds.begin(), std::min(wheelSpeeds.size(), maxRw), wheelSpeedsMsg.wheelSpeeds);

    RWAvailabilityMsgPayload wheelsAvailabilityMsg{};
    for (size_t i = 0U; i < wheelAvailabilityBool.size() && i < maxRw; ++i) {
        if (wheelAvailabilityBool[i]) {
            wheelsAvailabilityMsg.wheelAvailability[i] = UNAVAILABLE;
        }
    }

    RWArrayConfigMsgF32Payload rwConfigMsg{};
    if (rwIsLinked) {
        rwConfigMsg.numRW = numRW;
        std::copy(JsList.begin(), JsList.end(), rwConfigMsg.JsList);
        // Feed the reference the same pre-normalized spin axes the algorithm uses (column-major), so its
        // normalization matches the config's and the reaction-wheel momentum term stays bit-identical.
        std::copy(rwInputData.GsMatrix_B.data(),
                  rwInputData.GsMatrix_B.data() + rwInputData.GsMatrix_B.size(),
                  rwConfigMsg.GsMatrix_B);
    }

    // Algorithm input structs (payload-free interface).
    const MrpFeedbackInputGuidance attGuidInputData{sigma, omega_BR_B, omega_RN_B, domega_RN_B};
    std::array<float, maxRw> wheelSpeedsArr{};
    std::copy(wheelSpeeds.begin(), wheelSpeeds.end(), wheelSpeedsArr.begin());

    Eigen::Vector3f int_sigma{Eigen::Vector3f::Zero()};

    constexpr int numSteps = 5;
    for (int step = 0; step < numSteps; ++step) {
        MrpFeedbackOutput out{};
        ReferenceOutput refOutput{};
        EXPECT_NO_THROW(out = alg.update(attGuidInputData, wheelSpeedsArr));
        EXPECT_NO_THROW(refOutput = referenceUpdate(
                            cfg, rwConfigMsg, ISC_B, int_sigma, guidCmdMsg, wheelSpeedsMsg, wheelsAvailabilityMsg));
        const MrpFeedbackOutput ref = refOutput.mrpFeedbackOut;
        int_sigma = refOutput.int_sigma;

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.controlTorque[i], ref.controlTorque[i], 1e-6);
            EXPECT_NEAR(out.integralFeedbackTorque[i], ref.integralFeedbackTorque[i], 1e-6);

            EXPECT_TRUE(std::isfinite(out.controlTorque[i]));
            EXPECT_TRUE(std::isfinite(out.integralFeedbackTorque[i]));
        }
    }
}

#endif  // TEST_MRPFEEDBACK_H
