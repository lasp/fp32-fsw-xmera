#ifndef TEST_MRPROTATION_H
#define TEST_MRPROTATION_H

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "mrpRotationAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/timeConstants.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <algorithm>
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

// ---------------------------------------------------------------------------
// Regression test helper: drive the algorithm through several time steps and compare
// to the reference implementation. The algorithm now integrates by the configured
// controlPeriod on every call (no priorTime-based first-step gating), so every k advances
// the reference by the same dt.
// ---------------------------------------------------------------------------
inline void regressionTestMrpRotation(const Eigen::Vector3f& initialSigmaRR0,
                                      const Eigen::Vector3f& omegaRR0R,
                                      const Eigen::Vector3f& sigma_R0N,
                                      const Eigen::Vector3f& omega_R0N_N,
                                      const Eigen::Vector3f& domega_R0N_N,
                                      float updateTimeSec,
                                      int numSteps) {
    const auto config = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, updateTimeSec, false);
    MrpRotationAlgorithm alg{config};
    alg.reset();

    MrpRotationReferenceState refState{initialSigmaRR0, omegaRR0R};

    const MrpRotationAttRefInputs attRef{sigma_R0N, omega_R0N_N, domega_R0N_N};
    const MrpRotationAttStateInputs emptyState{};

    for (int k = 0; k < numSteps; ++k) {
        const MrpRotationOutput algOut = alg.update(attRef, emptyState);
        const auto refOut = referenceUpdate(refState, sigma_R0N, omega_R0N_N, domega_R0N_N, updateTimeSec);

        constexpr float tol = 1e-5F;
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.sigma_RN(i), refOut.sigma_RN(i), tol);
            EXPECT_NEAR(algOut.omega_RN_N(i), refOut.omega_RN_N(i), tol);
            EXPECT_NEAR(algOut.domega_RN_N(i), refOut.domega_RN_N(i), tol);
        }
    }
}

// ---------------------------------------------------------------------------
// Fuzz-compatible regression helper: drives regressionTestMrpRotation with three
// fuzz-supplied Eigen::Vector3f inputs.
// ---------------------------------------------------------------------------
inline void fuzzRegressionMrpRotation(const Eigen::Vector3f& initialSigmaRR0,
                                      const Eigen::Vector3f& omegaRR0R,
                                      const Eigen::Vector3f& sigma_R0N,
                                      const Eigen::Vector3f& omega_R0N_N,
                                      float updateTimeSec) {
    regressionTestMrpRotation(
        initialSigmaRR0, omegaRR0R, sigma_R0N, omega_R0N_N, Eigen::Vector3f::Zero(), updateTimeSec, 3);
}

// ---------------------------------------------------------------------------
// Property test helpers
// ---------------------------------------------------------------------------

// Output reference is finite for any finite inputs and finite configuration.
inline void propertyOutputIsFinite(const Eigen::Vector3f& initialSigmaRR0, const Eigen::Vector3f& omegaRR0R) {
    constexpr float kPropertyControlPeriod = 0.5F;
    const auto config = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, kPropertyControlPeriod, false);
    MrpRotationAlgorithm alg{config};
    alg.reset();

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.1F, 0.2F, 0.3F},
        Eigen::Vector3f{0.05F, 0.0F, 0.0F},
        Eigen::Vector3f::Zero(),
    };
    const MrpRotationAttStateInputs emptyState{};

    MrpRotationOutput out0{};
    EXPECT_NO_THROW(out0 = alg.update(attRef, emptyState));
    MrpRotationOutput out1{};
    EXPECT_NO_THROW(out1 = alg.update(attRef, emptyState));

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out0.sigma_RN(i)));
        EXPECT_TRUE(std::isfinite(out0.omega_RN_N(i)));
        EXPECT_TRUE(std::isfinite(out0.domega_RN_N(i)));
        EXPECT_TRUE(std::isfinite(out1.sigma_RN(i)));
        EXPECT_TRUE(std::isfinite(out1.omega_RN_N(i)));
        EXPECT_TRUE(std::isfinite(out1.domega_RN_N(i)));
    }
}

#endif  // TEST_MRPROTATION_H
