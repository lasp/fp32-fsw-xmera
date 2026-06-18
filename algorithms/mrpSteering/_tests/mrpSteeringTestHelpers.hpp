#ifndef TEST_MRPSTEERING_H
#define TEST_MRPSTEERING_H

#include "mrpSteeringAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <vector>

typedef struct {
    Eigen::Vector3f Lr;
    Eigen::Vector3f z;
} ReferenceOutput;

// Reference computation for update
inline ReferenceOutput referenceUpdate(const MrpSteeringControlParameters& params,
                                       const Eigen::Vector3f& knownTorquePntB_B,
                                       RWArrayConfigMsgF32Payload rwConfigParams,
                                       const Eigen::Matrix3f& ISCPntB_B,
                                       Eigen::Vector3f z,
                                       AttGuidMsgF32Payload guidCmd,
                                       const RWSpeedMsgF32Payload& wheelSpeeds,
                                       const RWAvailabilityMsgPayload& wheelsAvailability) {
    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(guidCmd.sigma_BR);
    Eigen::Vector3f omega_BastR_B{};
    Eigen::Vector3f omegap_BastR_B{Eigen::Vector3f::Zero()};

    for (int i = 0; i < 3; ++i) {
        const float sigma_i = sigma_BR[i];
        const float f_i = atanf(static_cast<float>(std::numbers::pi) / 2 / params.omegaMax *
                                (params.K1 * sigma_i + params.K3 * powf(sigma_i, 3))) /
                          (static_cast<float>(std::numbers::pi) / 2) * params.omegaMax;
        omega_BastR_B[i] = -f_i;
    }

    if (!params.ignoreOuterLoopFeedforward) {
        Eigen::Matrix3f B = bmatMrp(sigma_BR);

        Eigen::Vector3f sigmaDot_BR = 0.25 * B * omega_BastR_B;

        for (int i = 0; i < 3; ++i) {
            const float sigma_i = sigma_BR[i];
            const float f_i = (3 * params.K3 * powf(sigma_i, 2) + params.K1) /
                              (powf(static_cast<float>(std::numbers::pi / 2) / params.omegaMax *
                                        (params.K1 * sigma_i + params.K3 * powf(sigma_i, 3)),
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
    if (params.Ki > 0.0F) { /* check if integral feedback is turned on  */
        z += omega_BBast_B * params.controlPeriod;
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float intLimCheck = fabs(z[i]);
            if (intLimCheck > params.integralLimit) {
                z[i] *= params.integralLimit / intLimCheck;
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
            const Eigen::Vector3f G_s_B_i = G_s_B.col(i).normalized();
            const Eigen::Vector3f h_s_i =
                rwConfigParams.JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.wheelSpeeds[i]) * G_s_B_i;
            H_B += h_s_i;
        }
    }

    /*! - evaluate required attitude control torque Lc */
    const Eigen::Vector3f Lc = params.P * omega_BBast_B + params.Ki * z - omega_BastN_B.cross(H_B) -
                               ISCPntB_B * (omegap_BastR_B + domega_RN_B - omega_BN_B.cross(omega_RN_B)) +
                               knownTorquePntB_B;

    /* Change sign to compute the net positive control torque onto the spacecraft */
    const Eigen::Vector3f Lr = -Lc;

    ReferenceOutput out{};

    out.Lr = Lr;
    out.z = z;

    return out;
}

inline MrpSteeringControlParameters makeValidControlParameters() {
    return MrpSteeringControlParameters{
        .K1 = 1.0F,
        .K3 = 1.0F,
        .omegaMax = 1.0F,
        .ignoreOuterLoopFeedforward = false,
        .P = 1.0F,
        .Ki = 1.0F,
        .integralLimit = 1.0F,
        .controlPeriod = 0.1F,
    };
}

inline void testMrpSteeringSetup() {
    const Eigen::Vector3f knownTorque = Eigen::Vector3f::Zero();
    const Eigen::Matrix3f goodInertia = Eigen::Matrix3f::Identity();
    const InputRwData rwData{};

    // --- Valid baseline does not throw ---
    EXPECT_NO_THROW(MrpSteeringConfig::create(makeValidControlParameters(), knownTorque, goodInertia, rwData, false));

    // --- Negative feedback gains or integral limit ---
    for (float MrpSteeringControlParameters::* gain : {&MrpSteeringControlParameters::K1,
                                                       &MrpSteeringControlParameters::K3,
                                                       &MrpSteeringControlParameters::P,
                                                       &MrpSteeringControlParameters::Ki,
                                                       &MrpSteeringControlParameters::integralLimit}) {
        MrpSteeringControlParameters params = makeValidControlParameters();
        params.*gain = -0.1F;
        EXPECT_THROW(MrpSteeringConfig::create(params, knownTorque, goodInertia, rwData, false), fsw::invalid_argument);
    }

    // --- Non-positive maximum rate ---
    for (const float badOmegaMax : {0.0F, -0.1F}) {
        MrpSteeringControlParameters params = makeValidControlParameters();
        params.omegaMax = badOmegaMax;
        EXPECT_THROW(MrpSteeringConfig::create(params, knownTorque, goodInertia, rwData, false), fsw::invalid_argument);
    }

    // --- Non-positive control period ---
    {
        MrpSteeringControlParameters params = makeValidControlParameters();
        params.controlPeriod = -0.1F;
        EXPECT_THROW(MrpSteeringConfig::create(params, knownTorque, goodInertia, rwData, false), fsw::invalid_argument);
    }

    // --- Invalid inertia matrix ---
    Eigen::Matrix3f badInertia{};
    badInertia << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    EXPECT_THROW(MrpSteeringConfig::create(makeValidControlParameters(), knownTorque, badInertia, rwData, false),
                 fsw::invalid_argument);
    badInertia << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    EXPECT_THROW(MrpSteeringConfig::create(makeValidControlParameters(), knownTorque, badInertia, rwData, false),
                 fsw::invalid_argument);
    badInertia << 3, 0, 0, 0, 1, 0, 0, 0, 1;
    EXPECT_THROW(MrpSteeringConfig::create(makeValidControlParameters(), knownTorque, badInertia, rwData, false),
                 fsw::invalid_argument);
}

inline void testMrpSteering(const Eigen::Vector3f& sigma,
                            float K1,
                            float K3,
                            float omegaMax,
                            bool ignoreFF,
                            float P,
                            float Ki,
                            float integralLimit,
                            const Eigen::Vector3f& knownTorquePntB_B,
                            const Eigen::Vector3f& omega_BR_B,
                            const Eigen::Vector3f& omega_RN_B,
                            const Eigen::Vector3f& domega_RN_B,
                            std::vector<float> wheelSpeedsVec,
                            std::vector<bool> wheelAvailabilityBool,
                            int numRW,
                            std::vector<float> JsList,
                            std::vector<float> GsMatrix_B,
                            std::vector<float> ISCPntB_B,
                            bool rwIsLinked,
                            float dt) {
    const Eigen::Matrix3f ISC_B = cArrayToEigenMatrix3(ISCPntB_B.data());

    // Build the RW spin-axis configuration, mirroring the adapter: it is only populated when the RW config
    // message is linked. Fill provided entries column-major into a zero matrix (matching the messaging-layer
    // Eigen::Map layout) so a short GsMatrix_B vector never reads out of bounds.
    InputRwData rwInputData{};
    if (rwIsLinked) {
        const std::size_t numGs = std::min<std::size_t>(GsMatrix_B.size(), static_cast<std::size_t>(RW_EFF_CNT) * 3U);
        for (std::size_t k = 0; k < numGs; ++k) {
            rwInputData.GsMatrix_B(static_cast<Eigen::Index>(k % 3), static_cast<Eigen::Index>(k / 3)) = GsMatrix_B[k];
        }
        std::copy(std::begin(JsList), std::end(JsList), std::begin(rwInputData.JsList));
        rwInputData.numRW = static_cast<uint32_t>(numRW);

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

    const MrpSteeringControlParameters params{
        .K1 = K1,
        .K3 = K3,
        .omegaMax = omegaMax,
        .ignoreOuterLoopFeedforward = ignoreFF,
        .P = P,
        .Ki = Ki,
        .integralLimit = integralLimit,
        .controlPeriod = dt,
    };

    // Build the validated configuration. If it is invalid (e.g. a degenerate inertia matrix), skip this case.
    std::optional<MrpSteeringConfig> config;
    try {
        config = MrpSteeringConfig::create(params, knownTorquePntB_B, ISC_B, rwInputData, rwIsLinked);
    } catch (const fsw::invalid_argument&) {
        return;
    }
    MrpSteeringAlgorithm alg{*config};

    // Populate messages
    AttGuidMsgF32Payload guidCmdMsg{};
    eigenVectorToCArray(sigma, guidCmdMsg.sigma_BR);
    eigenVectorToCArray(omega_BR_B, guidCmdMsg.omega_BR_B);
    eigenVectorToCArray(omega_RN_B, guidCmdMsg.omega_RN_B);
    eigenVectorToCArray(domega_RN_B, guidCmdMsg.domega_RN_B);

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
        // Feed the reference the same pre-normalized spin axes the algorithm uses (column-major), so its
        // normalization matches the config's and the reaction-wheel momentum term stays bit-identical.
        std::copy(rwInputData.GsMatrix_B.data(),
                  rwInputData.GsMatrix_B.data() + rwInputData.GsMatrix_B.size(),
                  rwConfigMsg.GsMatrix_B);
    }

    // populate module input structs
    InputGuidanceData attGuidInputData{};
    attGuidInputData.sigma_BR = sigma;
    attGuidInputData.omega_BR_B = omega_BR_B;
    attGuidInputData.omega_RN_B = omega_RN_B;
    attGuidInputData.domega_RN_B = domega_RN_B;

    std::array<float, RW_EFF_CNT> wheelSpeeds{};
    std::copy(wheelSpeedsVec.begin(), wheelSpeedsVec.end(), wheelSpeeds.begin());

    std::array<FSWdeviceAvailability, RW_EFF_CNT> wheelAvailability{};
    for (uint32_t i = 0U; i < wheelAvailabilityBool.size(); ++i) {
        if (wheelAvailabilityBool[i]) {
            wheelAvailability[i] = UNAVAILABLE;
        }
    }

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
                params, knownTorquePntB_B, rwConfigMsg, ISC_B, z, guidCmdMsg, wheelSpeedsMsg, wheelsAvailabilityMsg));
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
