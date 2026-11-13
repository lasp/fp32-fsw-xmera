#ifndef TEST_MRPROTATION_H
#define TEST_MRPROTATION_H

#include "mrpRotationAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

// Reference implementation that independently computes one update step of the MRP rotation
// algorithm:
//   sigmaDot_RRp = (1/4) B(sigma_RRp) * [RRp(sigma_RRp(k))] * omega_RRp_Rp
//   sigma_RRp(k+1) = mrpSwitch(sigma_RRp(k) + dt * sigmaDot_RRp)
//   [RN] = [RRp(sigma_RRp(k+1))] * [RpRc] * [RcR0] * [R0N]
//   omega_RRp_N = ([RpRc] * [RcR0] * [R0N])^T * omega_RRp_Rp
//   omega_RN_N = omega_RRp_N + omega_R0N_N
//   domega_RN_N = omega_R0N_N x omega_RRp_N + domega_R0N_N
struct MrpRotationReferenceState {
    Eigen::Vector3f sigma_RcR0{Eigen::Vector3f::Zero()};
    Eigen::Vector3f sigma_RpRc{Eigen::Vector3f::Zero()};
    Eigen::Vector3f omega_RRp_Rp{Eigen::Vector3f::Zero()};
    Eigen::Vector3f sigma_RRp{Eigen::Vector3f::Zero()};
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
    const Eigen::Matrix3f B = bmatMrp(state.sigma_RRp);
    const Eigen::Vector3f sigmaDot_RRp = 0.25F * B * (mrpToDcm(state.sigma_RRp) * state.omega_RRp_Rp);
    const Eigen::Vector3f mrpSetNew = state.sigma_RRp + sigmaDot_RRp * dt;
    state.sigma_RRp = mrpSwitch(mrpSetNew);

    const Eigen::Matrix3f dcm_RRp = mrpToDcm(state.sigma_RRp);
    const Eigen::Matrix3f dcm_RpRc = mrpToDcm(state.sigma_RpRc);
    const Eigen::Matrix3f dcm_RcR0 = mrpToDcm(state.sigma_RcR0);
    const Eigen::Matrix3f dcm_R0N = mrpToDcm(sigma_R0N);
    const Eigen::Matrix3f dcm_RpN = dcm_RpRc * dcm_RcR0 * dcm_R0N;
    const Eigen::Matrix3f dcm_RN = dcm_RRp * dcm_RpN;

    MrpRotationReferenceOutput out{};
    out.sigma_RN = dcmToMrp(dcm_RN);
    const Eigen::Vector3f omega_RRp_N = dcm_RpN.transpose() * state.omega_RRp_Rp;
    out.omega_RN_N = omega_RRp_N + omega_R0N_N;
    const Eigen::Vector3f domega_RRp_N = omega_R0N_N.cross(omega_RRp_N);
    out.domega_RN_N = domega_RRp_N + domega_R0N_N;
    return out;
}

// Max absolute element difference between two DCMs. Comparing DCMs makes a check independent of which
// shadow-set representative dcmToMrp returns.
inline float dcmMaxAbsDiff(const Eigen::Matrix3f& a, const Eigen::Matrix3f& b) { return (a - b).cwiseAbs().maxCoeff(); }

// ---------------------------------------------------------------------------
// Regression test helper: drive the algorithm through several time steps and compare
// to the reference implementation.
// ---------------------------------------------------------------------------
inline void regressionTestMrpRotation(const Eigen::Vector3f& sigma_RcR0,
                                      const Eigen::Vector3f& sigma_RpRc,
                                      const Eigen::Vector3f& omega_RRp_Rp,
                                      const Eigen::Vector3f& sigma_R0N,
                                      const Eigen::Vector3f& omega_R0N_N,
                                      const Eigen::Vector3f& domega_R0N_N,
                                      float updateTimeSec,
                                      int numSteps) {
    const auto config = MrpRotationConfig::create(sigma_RcR0, sigma_RpRc, omega_RRp_Rp, updateTimeSec);
    MrpRotationAlgorithm alg{config};

    MrpRotationReferenceState refState{sigma_RcR0, sigma_RpRc, omega_RRp_Rp};

    const MrpRotationAttRefInputs attRef{sigma_R0N, omega_R0N_N, domega_R0N_N};

    // Per-component fp32 error of omega is ~ N * eps * (||omega_RRp_Rp|| + ||omega_R0N_N||), summing the extra 3x3
    // product from the cached [RpRc][RcR0] in the algorithm (~3 ops), the two sigma_RRp integration paths (~1 op),
    // the matvec that takes omega_RRp_Rp to N (~3 ops) and the final sum (~1 op), so N ~ 8. domega carries the
    // omega_RRp_N error scaled by ||omega_R0N_N|| plus the cross product and sum, about the same N. A 2x safety
    // margin guards the adversarial fuzz corners; the 1e-5 floor covers the small-rate base noise.
    constexpr float tol = 1e-5F;
    constexpr float kRoundingOps = 8.0F;
    constexpr float kSafetyMargin = 2.0F;
    constexpr float kRelTol = kSafetyMargin * kRoundingOps * std::numeric_limits<float>::epsilon();
    const float omegaTol = tol + kRelTol * (omega_RRp_Rp.stableNorm() + omega_R0N_N.stableNorm());
    const float domegaTol =
        tol + kRelTol * (omega_R0N_N.stableNorm() * omega_RRp_Rp.stableNorm() + domega_R0N_N.stableNorm());
    for (int k = 0; k < numSteps; ++k) {
        const MrpRotationOutput algOut = alg.update(attRef);
        const auto refOut = referenceUpdate(refState, sigma_R0N, omega_R0N_N, domega_R0N_N, updateTimeSec);

        EXPECT_LT(dcmMaxAbsDiff(mrpToDcm(algOut.sigma_RN), mrpToDcm(refOut.sigma_RN)), tol);
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.omega_RN_N(i), refOut.omega_RN_N(i), omegaTol);
            EXPECT_NEAR(algOut.domega_RN_N(i), refOut.domega_RN_N(i), domegaTol);
        }
    }
}

// ---------------------------------------------------------------------------
// Fuzz-compatible regression helper.
// ---------------------------------------------------------------------------
inline void fuzzRegressionMrpRotation(const Eigen::Vector3f& sigma_RcR0,
                                      const Eigen::Vector3f& sigma_RpRc,
                                      const Eigen::Vector3f& omega_RRp_Rp,
                                      const Eigen::Vector3f& sigma_R0N,
                                      const Eigen::Vector3f& omega_R0N_N,
                                      float updateTimeSec) {
    regressionTestMrpRotation(
        sigma_RcR0, sigma_RpRc, omega_RRp_Rp, sigma_R0N, omega_R0N_N, Eigen::Vector3f::Zero(), updateTimeSec, 3);
}

// ---------------------------------------------------------------------------
// Property test helpers
// ---------------------------------------------------------------------------

// Output reference is finite for any finite inputs and finite configuration.
inline void propertyOutputIsFinite(const Eigen::Vector3f& sigma_RcR0,
                                   const Eigen::Vector3f& sigma_RpRc,
                                   const Eigen::Vector3f& omega_RRp_Rp) {
    constexpr float kPropertyControlPeriod = 0.5F;
    const auto config = MrpRotationConfig::create(sigma_RcR0, sigma_RpRc, omega_RRp_Rp, kPropertyControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.1F, 0.2F, 0.3F},
        Eigen::Vector3f{0.05F, 0.0F, 0.0F},
        Eigen::Vector3f::Zero(),
    };

    for (int k = 0; k < 2; ++k) {
        MrpRotationOutput out{};
        EXPECT_NO_THROW(out = alg.update(attRef));
        EXPECT_TRUE(out.sigma_RN.allFinite());
        EXPECT_TRUE(out.omega_RN_N.allFinite());
        EXPECT_TRUE(out.domega_RN_N.allFinite());
    }
}

// With the input reference and both offsets at identity, the output sigma_RN encodes the integrated
// rotation sigma_RRp, which the reference tracks independently. dcmToMrp can return either shadow-set
// representative near norm = 1, so accept whichever is closer.
inline void propertySigmaRNEqualsSigmaRRpWhenOffsetsAndInputRefAreIdentity(const Eigen::Vector3f& omega_RRp_Rp) {
    constexpr float kControlPeriod = 0.1F;
    constexpr int kNumSteps = 10;

    const auto config =
        MrpRotationConfig::create(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), omega_RRp_Rp, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs identityRef{};
    MrpRotationReferenceState refState{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), omega_RRp_Rp};

    constexpr float tol = 1e-5F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput algOut = alg.update(identityRef);
        (void)referenceUpdate(
            refState, identityRef.sigma_R0N, identityRef.omega_R0N_N, identityRef.domega_R0N_N, kControlPeriod);
        const Eigen::Vector3f principal = refState.sigma_RRp;
        const float principalNormSq = principal.squaredNorm();
        const Eigen::Vector3f shadow =
            (principalNormSq > 1e-5F) ? Eigen::Vector3f(-principal / principalNormSq) : principal;
        const float errPrincipal = (algOut.sigma_RN - principal).stableNorm();
        const float errShadow = (algOut.sigma_RN - shadow).stableNorm();
        EXPECT_LT(std::min(errPrincipal, errShadow), tol);
    }
}

// dcmToMrp's contract is to return the MRP with |sigma| <= 1, so the output sigma_RN must satisfy that
// bound on every step. Drive many steps with a rate large enough to repeatedly cross the boundary.
inline void propertySigmaRNNormLessOrEqualToOne(const Eigen::Vector3f& sigma_RcR0,
                                                const Eigen::Vector3f& sigma_RpRc,
                                                const Eigen::Vector3f& omega_RRp_Rp) {
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 100;
    constexpr float kMrpNormBound = 1.0F + 1e-5F;  // small fp32 slack at the boundary

    const auto config = MrpRotationConfig::create(sigma_RcR0, sigma_RpRc, omega_RRp_Rp, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.1F, 0.2F, 0.3F},
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
    };

    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(attRef);
        EXPECT_LE(out.sigma_RN.stableNorm(), kMrpNormBound);
    }
}

// The identity omega_RN_N = [RpN]^T * omega_RRp_Rp + omega_R0N_N follows from the transport theorem and
// from Rp being fixed in R0. Build [RpN] from the configured offsets and the input reference and verify the
// decomposition on every step, independently of the reference implementation.
inline void propertyOmegaRNDecomposesCorrectly(const Eigen::Vector3f& sigma_RcR0,
                                               const Eigen::Vector3f& sigma_RpRc,
                                               const Eigen::Vector3f& omega_RRp_Rp,
                                               const Eigen::Vector3f& sigma_R0N,
                                               const Eigen::Vector3f& omega_R0N_N) {
    constexpr float kControlPeriod = 0.25F;
    constexpr int kNumSteps = 5;

    const auto config = MrpRotationConfig::create(sigma_RcR0, sigma_RpRc, omega_RRp_Rp, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{sigma_R0N, omega_R0N_N, Eigen::Vector3f::Zero()};

    // Per-component fp32 error of this check is ~ N * eps * (||omega_RRp_Rp|| + ||omega_R0N_N||), summing the
    // sum/subtract cancellation (~2 ops), the 3-term dcm_RpN^T * omega_RRp_Rp matvec (~3 ops), and the
    // algorithm's different path to N through the integrated [RRp] (~3 ops), so N ~ 8. A 2x safety margin guards the
    // adversarial fuzz corners; the 1e-5 floor covers the small-rate base noise.
    constexpr float kRoundingOps = 8.0F;
    constexpr float kSafetyMargin = 2.0F;
    constexpr float kRelTol = kSafetyMargin * kRoundingOps * std::numeric_limits<float>::epsilon();
    const float tol = 1e-5F + kRelTol * (omega_RRp_Rp.stableNorm() + omega_R0N_N.stableNorm());
    const Eigen::Matrix3f dcm_RpN = mrpToDcm(sigma_RpRc) * mrpToDcm(sigma_RcR0) * mrpToDcm(sigma_R0N);
    const Eigen::Vector3f expectedOmega_RRp_N = dcm_RpN.transpose() * omega_RRp_Rp;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(attRef);
        const Eigen::Vector3f actualOmega_RRp_N = out.omega_RN_N - omega_R0N_N;
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(actualOmega_RRp_N(i), expectedOmega_RRp_N(i), tol);
        }
    }
}

// With both offsets at identity and zero rate, the output reference R must coincide with the input
// reference R0 on every step.
inline void propertyOutputRefEqualsInputRefWhenRotationIsZero(const Eigen::Vector3f& sigma_R0N,
                                                              const Eigen::Vector3f& omega_R0N_N,
                                                              const Eigen::Vector3f& domega_R0N_N) {
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 5;

    const auto config = MrpRotationConfig::create(
        Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{sigma_R0N, omega_R0N_N, domega_R0N_N};

    const Eigen::Matrix3f dcm_R0N = mrpToDcm(sigma_R0N);

    constexpr float tol = 1e-5F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(attRef);

        EXPECT_LT(dcmMaxAbsDiff(mrpToDcm(out.sigma_RN), dcm_R0N), tol);
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.omega_RN_N(i), omega_R0N_N(i), tol);
            EXPECT_NEAR(out.domega_RN_N(i), domega_R0N_N(i), tol);
        }
    }
}

// With zero rate, the output attitude is the input reference with both constant offsets applied:
// [RN] = [RpRc][RcR0][R0N] on every step.
inline void propertyOutputEqualsOffsetInputRefWhenRateIsZero(const Eigen::Vector3f& sigma_RcR0,
                                                             const Eigen::Vector3f& sigma_RpRc,
                                                             const Eigen::Vector3f& sigma_R0N) {
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 5;

    const auto config = MrpRotationConfig::create(sigma_RcR0, sigma_RpRc, Eigen::Vector3f::Zero(), kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{sigma_R0N, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
    const Eigen::Matrix3f expectedDcm_RN = mrpToDcm(sigma_RpRc) * mrpToDcm(sigma_RcR0) * mrpToDcm(sigma_R0N);

    constexpr float tol = 1e-5F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(attRef);
        EXPECT_LT(dcmMaxAbsDiff(mrpToDcm(out.sigma_RN), expectedDcm_RN), tol);
    }
}

// The rotation starts at identity and its axis omegaHat is constant in Rp, so the axis stays fixed in the input
// reference R0 at [RpR0]^T * omegaHat over many steps, whatever the offsets. The axis of a rotation has the same
// components in R and Rp, so [RR0]^T * omegaHat must give the same vector. The input reference is identity, so
// dcm_RN = dcm_RR0.
// Forward Euler keeps the integrated MRP on the axis only for small steps: above about 0.2 rad per step,
// fp32 rounding off the axis grows exponentially, so callers keep |omega_RRp_Rp| * controlPeriod well below that.
inline void propertyRotationAxisFixedInInputRef(const Eigen::Vector3f& sigma_RcR0,
                                                const Eigen::Vector3f& sigma_RpRc,
                                                const Eigen::Vector3f& omega_RRp_Rp) {
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 2000;
    constexpr float tol = 1e-4F;

    if (omega_RRp_Rp.stableNorm() < 1e-3F) {
        return;  // the axis is undefined for a near-zero rate
    }

    const auto config = MrpRotationConfig::create(sigma_RcR0, sigma_RpRc, omega_RRp_Rp, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const Eigen::Vector3f omegaHat = omega_RRp_Rp.stableNormalized();
    const Eigen::Matrix3f dcm_RpR0 = mrpToDcm(sigma_RpRc) * mrpToDcm(sigma_RcR0);
    const Eigen::Vector3f expectedAxis_R0 = dcm_RpR0.transpose() * omegaHat;

    const MrpRotationAttRefInputs identityRef{};
    float maxErr = 0.0F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(identityRef);
        const Eigen::Vector3f axis_R0 = mrpToDcm(out.sigma_RN).transpose() * omegaHat;
        maxErr = std::max(maxErr, (axis_R0 - expectedAxis_R0).cwiseAbs().maxCoeff());
    }
    EXPECT_LT(maxErr, tol);
}

#endif  // TEST_MRPROTATION_H
