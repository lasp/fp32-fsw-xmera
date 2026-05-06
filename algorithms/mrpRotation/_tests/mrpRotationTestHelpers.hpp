#ifndef TEST_MRPROTATION_H
#define TEST_MRPROTATION_H

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "mrpRotationAlgorithm.h"
#include "mrpRotationTypes.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/timeConstants.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cstdint>

// Reference implementation that independently computes one update step of the MRP rotation
// algorithm. Mirrors the production algorithm's formulation:
//   sigma_dot_RR0 = (1/4) B(sigma_RR0) * omega_RR0_R
//   sigma_RR0(k+1) = mrpSwitch(sigma_RR0(k) + dt * sigma_dot_RR0)
//   [RN] = [RR0(sigma_RR0(k+1))] * [R0N(sigma_R0N)]
//   omega_RR0_N = [RN]^T * omega_RR0_R
//   omega_RN_N = omega_RR0_N + omega_R0N_N
//   domega_RN_N = omega_R0N_N x omega_RR0_N + domega_R0N_N
struct MrpRotationReferenceState {
    Eigen::Vector3f sigma_RR0;
    Eigen::Vector3f omega_RR0_R;
};

struct MrpRotationReferenceOutput {
    Eigen::Vector3f sigma_RN;
    Eigen::Vector3f omega_RN_N;
    Eigen::Vector3f domega_RN_N;
};

inline MrpRotationReferenceOutput referenceUpdate(MrpRotationReferenceState& state,
                                                  const Eigen::Vector3f& sigma_R0N,
                                                  const Eigen::Vector3f& omega_R0N_N,
                                                  const Eigen::Vector3f& domega_R0N_N,
                                                  float dt) {
    const Eigen::Matrix3f B = bmatMrp(state.sigma_RR0);
    const Eigen::Vector3f sigmaDot_RR0 = 0.25F * B * state.omega_RR0_R;
    const Eigen::Vector3f mrpSetNew = state.sigma_RR0 + sigmaDot_RR0 * dt;
    state.sigma_RR0 = mrpSwitch(mrpSetNew, 1.0F);

    const Eigen::Matrix3f dcm_RR0 = mrpToDcm(state.sigma_RR0);
    const Eigen::Matrix3f dcm_R0N = mrpToDcm(sigma_R0N);
    const Eigen::Matrix3f dcm_RN = dcm_RR0 * dcm_R0N;

    MrpRotationReferenceOutput out{};
    out.sigma_RN = dcmToMrp(dcm_RN);
    const Eigen::Vector3f omega_RR0_N = dcm_RN.transpose() * state.omega_RR0_R;
    out.omega_RN_N = omega_RR0_N + omega_R0N_N;
    const Eigen::Vector3f domega_RR0_N = omega_R0N_N.cross(omega_RR0_N);
    out.domega_RN_N = domega_RR0_N + domega_R0N_N;
    return out;
}

// Build an AttRefMsgF32Payload from individual Eigen vectors.
inline AttRefMsgF32Payload buildAttRef(const Eigen::Vector3f& sigma,
                                       const Eigen::Vector3f& omega,
                                       const Eigen::Vector3f& domega) {
    AttRefMsgF32Payload payload{};
    eigenVectorToCArray(sigma, payload.sigma_RN);
    eigenVectorToCArray(omega, payload.omega_RN_N);
    eigenVectorToCArray(domega, payload.domega_RN_N);
    return payload;
}

inline AttStateMsgF32Payload buildAttState(const Eigen::Vector3f& state, const Eigen::Vector3f& rate) {
    AttStateMsgF32Payload payload{};
    eigenVectorToCArray(state, payload.state);
    eigenVectorToCArray(rate, payload.rate);
    return payload;
}

// ---------------------------------------------------------------------------
// Regression test helper: drive the algorithm through several time steps and compare
// to the reference implementation.
// ---------------------------------------------------------------------------
inline void regressionTestMrpRotation(const Eigen::Vector3f& initialSigmaRR0,
                                      const Eigen::Vector3f& omegaRR0R,
                                      const Eigen::Vector3f& sigma_R0N,
                                      const Eigen::Vector3f& omega_R0N_N,
                                      const Eigen::Vector3f& domega_R0N_N,
                                      float updateTimeSec,
                                      int numSteps) {
    const auto config = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, false);
    MrpRotationAlgorithm alg{config};
    alg.reset();

    MrpRotationReferenceState refState{initialSigmaRR0, omegaRR0R};

    const auto inputRef = buildAttRef(sigma_R0N, omega_R0N_N, domega_R0N_N);
    const AttStateMsgF32Payload emptyState{};

    // Start callTime at one update period (not 0). The algorithm's computeTimeStep keys off
    // priorTime: if it is exactly 0, dt is forced to 0. priorTime is set to callTime at the end of
    // each update, so calling first with callTime=0 leaves priorTime at 0 -- which then forces
    // dt=0 on the *second* call too. Starting at step_ns gives priorTime a non-zero value after
    // the first call, so step 0 has dt=0 (no integration) and step 1 onward integrates by
    // updateTimeSec, exactly as this test expects.
    const uint64_t step_ns = static_cast<uint64_t>(updateTimeSec * static_cast<float>(kSec2Nano));
    uint64_t callTime = step_ns;
    for (int k = 0; k < numSteps; ++k) {
        const float dt = (k == 0) ? 0.0F : updateTimeSec;
        const AttRefMsgF32Payload algOut = alg.update(callTime, inputRef, emptyState);
        const auto refOut = referenceUpdate(refState, sigma_R0N, omega_R0N_N, domega_R0N_N, dt);

        constexpr float tol = 1e-5F;
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.sigma_RN[i], refOut.sigma_RN(i), tol);
            EXPECT_NEAR(algOut.omega_RN_N[i], refOut.omega_RN_N(i), tol);
            EXPECT_NEAR(algOut.domega_RN_N[i], refOut.domega_RN_N(i), tol);
        }

        callTime += step_ns;
    }
}

#endif  // TEST_MRPROTATION_H
