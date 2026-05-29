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
    const auto config = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, updateTimeSec);
    MrpRotationAlgorithm alg{config};

    // The algorithm bounds the seed MRP via mrpSwitch in MrpRotationConfig::create, so the
    // reference must start from the same bounded representative to stay in lock-step.
    MrpRotationReferenceState refState{mrpSwitch(initialSigmaRR0, 1.0F), omegaRR0R};

    const MrpRotationAttRefInputs attRef{sigma_R0N, omega_R0N_N, domega_R0N_N};

    for (int k = 0; k < numSteps; ++k) {
        const MrpRotationOutput algOut = alg.update(attRef);
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
    const auto config = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, kPropertyControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.1F, 0.2F, 0.3F},
        Eigen::Vector3f{0.05F, 0.0F, 0.0F},
        Eigen::Vector3f::Zero(),
    };

    MrpRotationOutput out0{};
    EXPECT_NO_THROW(out0 = alg.update(attRef));
    MrpRotationOutput out1{};
    EXPECT_NO_THROW(out1 = alg.update(attRef));

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out0.sigma_RN(i)));
        EXPECT_TRUE(std::isfinite(out0.omega_RN_N(i)));
        EXPECT_TRUE(std::isfinite(out0.domega_RN_N(i)));
        EXPECT_TRUE(std::isfinite(out1.sigma_RN(i)));
        EXPECT_TRUE(std::isfinite(out1.omega_RN_N(i)));
        EXPECT_TRUE(std::isfinite(out1.domega_RN_N(i)));
    }
}

// With sigma_R0N = 0 the input reference frame coincides with N, so dcm_RN = dcm_RR0 and the
// output sigma_RN must encode the same rotation as the algorithm's internal sigma_RR0. We track
// the latter independently in a MrpRotationReferenceState and assert agreement at every step.
// dcmToMrp can return either MRP shadow-set representative near norm = 1, so we accept whichever is
// closer.
inline void propertySigmaRNEqualsSigmaRR0WhenInputRefIsIdentity(const Eigen::Vector3f& initialSigmaRR0,
                                                                const Eigen::Vector3f& omegaRR0R) {
    constexpr float kControlPeriod = 0.1F;
    constexpr int kNumSteps = 10;

    const auto config = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs identityRef{
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
    };
    // The algorithm bounds the seed MRP via mrpSwitch in MrpRotationConfig::create, so the
    // reference must start from the same bounded representative to stay in lock-step.
    MrpRotationReferenceState refState{mrpSwitch(initialSigmaRR0, 1.0F), omegaRR0R};

    constexpr float tol = 1e-5F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput algOut = alg.update(identityRef);
        (void)referenceUpdate(
            refState, identityRef.sigma_R0N, identityRef.omega_R0N_N, identityRef.domega_R0N_N, kControlPeriod);
        // algOut.sigma_RN and refState.sigma_RR0 encode the same rotation. Accept either
        // shadow-set representative when they straddle the |sigma| = 1 boundary.
        const Eigen::Vector3f principal = refState.sigma_RR0;
        const float principalNormSq = principal.squaredNorm();
        const Eigen::Vector3f shadow =
            (principalNormSq > 1e-5F) ? Eigen::Vector3f(-principal / principalNormSq) : principal;
        const float errPrincipal = (algOut.sigma_RN - principal).norm();
        const float errShadow = (algOut.sigma_RN - shadow).norm();
        EXPECT_LT(std::min(errPrincipal, errShadow), tol);
    }
}

// dcmToMrp's contract is to return the MRP with |sigma| <= 1, so the algorithm's output sigma_RN
// must satisfy that bound on every step. This invariant is what mrpSwitch exists to enforce; drive
// many steps with a R-frame omega large enough to repeatedly cross the boundary and assert the
// bound holds throughout.
inline void propertySigmaRNNormLessOrEqualToOne(const Eigen::Vector3f& initialSigmaRR0,
                                                const Eigen::Vector3f& omegaRR0R) {
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 100;
    constexpr float kMrpNormBound = 1.0F + 1e-5F;  // small fp32 slack at the boundary

    const auto config = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.1F, 0.2F, 0.3F},
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
    };

    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(attRef);
        EXPECT_LE(out.sigma_RN.norm(), kMrpNormBound);
    }
}

// The algebraic identity omega_RN_N = [RN]^T * omega_RR0_R + omega_R0N_N follows from the
// transport theorem. Reconstruct dcm_RN from out.sigma_RN (mrpToDcm gives the same DCM whether
// sigma_RN is the principal or shadow representative) and verify the decomposition holds on every
// step, independently of the reference implementation.
inline void propertyOmegaRNDecomposesCorrectly(const Eigen::Vector3f& initialSigmaRR0,
                                               const Eigen::Vector3f& omegaRR0R,
                                               const Eigen::Vector3f& sigma_R0N,
                                               const Eigen::Vector3f& omega_R0N_N) {
    constexpr float kControlPeriod = 0.25F;
    constexpr int kNumSteps = 5;

    const auto config = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{sigma_R0N, omega_R0N_N, Eigen::Vector3f::Zero()};

    constexpr float tol = 1e-5F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(attRef);
        const Eigen::Matrix3f dcm_RN = mrpToDcm(out.sigma_RN);
        const Eigen::Vector3f expectedOmegaRR0_N = dcm_RN.transpose() * omegaRR0R;
        const Eigen::Vector3f actualOmegaRR0_N = out.omega_RN_N - omega_R0N_N;
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(actualOmegaRR0_N(i), expectedOmegaRR0_N(i), tol);
        }
    }
}

// With sigma_RR0 = 0 and omega_RR0_R = 0 the algorithm's internal R/R0 rotation is identity and a
// zero R-frame rate never advances it, so the output reference frame R must coincide with the input
// reference frame R0 on every step: sigma_RN == sigma_R0N, omega_RN_N == omega_R0N_N, and
// domega_RN_N == domega_R0N_N. Compare the attitude via its DCM so the check is independent of which
// shadow-set representative dcmToMrp returns (fuzzed sigma_R0N may have norm > 1).
inline void propertyOutputRefEqualsInputRefWhenRotationIsZero(const Eigen::Vector3f& sigma_R0N,
                                                              const Eigen::Vector3f& omega_R0N_N,
                                                              const Eigen::Vector3f& domega_R0N_N) {
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 5;

    const auto config = MrpRotationConfig::create(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{sigma_R0N, omega_R0N_N, domega_R0N_N};

    const Eigen::Matrix3f dcm_R0N = mrpToDcm(sigma_R0N);

    constexpr float tol = 1e-5F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(attRef);

        const Eigen::Matrix3f dcm_RN = mrpToDcm(out.sigma_RN);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                EXPECT_NEAR(dcm_RN(r, c), dcm_R0N(r, c), tol);
            }
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.omega_RN_N(i), omega_R0N_N(i), tol);
            EXPECT_NEAR(out.domega_RN_N(i), domega_R0N_N(i), tol);
        }
    }
}

#endif  // TEST_MRPROTATION_H
