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

// Reference implementation that independently computes one update step of the MRP rotation algorithm. The offset
// and the spin axis are given in component axes. The reference turns the offset into body axes by conjugating with
// [BC], where the algorithm turns its MRP like a vector:
//   omega_RRp_Rp = [BC] * omega_RRp_C
//   sigmaDot_RRp = (1/4) B(sigma_RRp) * [RRp(sigma_RRp(k))] * omega_RRp_Rp
//   sigma_RRp(k+1) = mrpSwitch(sigma_RRp(k) + dt * sigmaDot_RRp)
//   [RcRb] = [BC] * [CnB],  [RpRc] = [BC] * [RpRc(sigma_RpRc_C)] * [CB]
//   [RN] = [RRp(sigma_RRp(k+1))] * [RpRc] * [RcRb] * [RbR0] * [R0N]
//   omega_RRp_N = ([RpRc] * [RcRb] * [RbR0] * [R0N])^T * omega_RRp_Rp
//   omega_RN_N = omega_RRp_N + omega_R0N_N
//   domega_RN_N = omega_R0N_N x omega_RRp_N + domega_R0N_N
struct MrpRotationReferenceState {
    Eigen::Vector3f sigma_RbR0{Eigen::Vector3f::Zero()};
    Eigen::Vector3f sigma_CnB{Eigen::Vector3f::Zero()};
    Eigen::Vector3f sigma_CB{Eigen::Vector3f::Zero()};
    Eigen::Vector3f sigma_RpRc_C{Eigen::Vector3f::Zero()};
    Eigen::Vector3f omega_RRp_C{Eigen::Vector3f::Zero()};
    Eigen::Vector3f sigma_RRp{Eigen::Vector3f::Zero()};
};

struct MrpRotationReferenceOutput {
    Eigen::Vector3f sigma_RN;
    Eigen::Vector3f omega_RN_N;
    Eigen::Vector3f domega_RN_N;
};

// [RpN] of the reference: the base, the component and the offset, with the offset conjugated into body axes.
inline Eigen::Matrix3f referenceDcm_RpN(const Eigen::Vector3f& sigma_RbR0,
                                        const Eigen::Vector3f& sigma_CnB,
                                        const Eigen::Vector3f& sigma_CB,
                                        const Eigen::Vector3f& sigma_RpRc_C,
                                        const Eigen::Vector3f& sigma_R0N) {
    const Eigen::Matrix3f dcm_BC = mrpToDcm(sigma_CB).transpose();
    const Eigen::Matrix3f dcm_RcRb = dcm_BC * mrpToDcm(sigma_CnB);
    const Eigen::Matrix3f dcm_RpRc = dcm_BC * mrpToDcm(sigma_RpRc_C) * dcm_BC.transpose();
    return dcm_RpRc * dcm_RcRb * mrpToDcm(sigma_RbR0) * mrpToDcm(sigma_R0N);
}

inline MrpRotationReferenceOutput referenceUpdate(MrpRotationReferenceState& state,
                                                  const Eigen::Vector3f& sigma_R0N,
                                                  const Eigen::Vector3f& omega_R0N_N,
                                                  const Eigen::Vector3f& domega_R0N_N,
                                                  float dt) {
    const Eigen::Vector3f omega_RRp_Rp = mrpToDcm(state.sigma_CB).transpose() * state.omega_RRp_C;
    const Eigen::Matrix3f B = bmatMrp(state.sigma_RRp);
    const Eigen::Vector3f sigmaDot_RRp = 0.25F * B * (mrpToDcm(state.sigma_RRp) * omega_RRp_Rp);
    const Eigen::Vector3f mrpSetNew = state.sigma_RRp + sigmaDot_RRp * dt;
    state.sigma_RRp = mrpSwitch(mrpSetNew);

    const Eigen::Matrix3f dcm_RpN =
        referenceDcm_RpN(state.sigma_RbR0, state.sigma_CnB, state.sigma_CB, state.sigma_RpRc_C, sigma_R0N);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(state.sigma_RRp) * dcm_RpN;

    MrpRotationReferenceOutput out{};
    out.sigma_RN = dcmToMrp(dcm_RN);
    const Eigen::Vector3f omega_RRp_N = dcm_RpN.transpose() * omega_RRp_Rp;
    out.omega_RN_N = omega_RRp_N + omega_R0N_N;
    const Eigen::Vector3f domega_RRp_N = omega_R0N_N.cross(omega_RRp_N);
    out.domega_RN_N = domega_RRp_N + domega_R0N_N;
    return out;
}

// Max absolute element difference between two DCMs. Comparing DCMs makes a check independent of which
// shadow-set representative dcmToMrp returns.
inline float dcmMaxAbsDiff(const Eigen::Matrix3f& a, const Eigen::Matrix3f& b) { return (a - b).cwiseAbs().maxCoeff(); }

// MRP of a rotation by angle [rad] about a unit axis.
inline Eigen::Vector3f mrpFromAxisAngle(const Eigen::Vector3f& axis, float angle) {
    return std::tan(angle / 4.0F) * axis.stableNormalized();
}

// Angle [rad] between two unit vectors.
inline float angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return std::atan2(a.cross(b).stableNorm(), a.dot(b));
}

// Fuzz domains give a raw spin axis. The helpers normalize it before create(), as a caller must; a near-zero
// vector has no direction.
inline bool hasSpinDirection(const Eigen::Vector3f& spinAxis_C) { return spinAxis_C.stableNorm() >= 1e-3F; }

// ---------------------------------------------------------------------------
// Regression test helper: drive the algorithm through several time steps and compare
// to the reference implementation.
// ---------------------------------------------------------------------------
inline void regressionTestMrpRotation(const Eigen::Vector3f& sigma_RbR0,
                                      const Eigen::Vector3f& sigma_CnB,
                                      const Eigen::Vector3f& sigma_CB,
                                      const Eigen::Vector3f& sigma_RpRc_C,
                                      const Eigen::Vector3f& spinAxis_C,
                                      float spinRate,
                                      const Eigen::Vector3f& sigma_R0N,
                                      const Eigen::Vector3f& omega_R0N_N,
                                      const Eigen::Vector3f& domega_R0N_N,
                                      float updateTimeSec,
                                      int numSteps) {
    if (!hasSpinDirection(spinAxis_C)) {
        return;
    }
    const Eigen::Vector3f spinHat_C = spinAxis_C.stableNormalized();
    const Eigen::Vector3f omega_RRp_C = spinRate * spinHat_C;
    const auto config =
        MrpRotationConfig::create(sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, spinHat_C, spinRate, updateTimeSec);
    MrpRotationAlgorithm alg{config};

    MrpRotationReferenceState refState{sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, omega_RRp_C};

    const MrpRotationAttRefInputs attRef{sigma_R0N, omega_R0N_N, domega_R0N_N};

    // Per-component fp32 error of omega is ~ N * eps * (||omega_RRp_C|| + ||omega_R0N_N||), summing the 3x3 products
    // of the cached [RpRc][RcRb][RbR0] in the algorithm and of the conjugated [RpRc] in the reference (~8 ops), the
    // two sigma_RRp integration paths (~1 op), the matvec that takes omega_RRp_Rp to N (~3 ops) and the final sum
    // (~1 op), so N ~ 13. domega carries the omega_RRp_N error scaled by ||omega_R0N_N|| plus the cross product and
    // sum, about the same N. A 2x safety margin guards the adversarial fuzz corners; the 1e-5 floor covers the
    // small-rate base noise.
    constexpr float tol = 1e-5F;
    constexpr float kRoundingOps = 13.0F;
    constexpr float kSafetyMargin = 2.0F;
    constexpr float kRelTol = kSafetyMargin * kRoundingOps * std::numeric_limits<float>::epsilon();
    const float omegaTol = tol + kRelTol * (omega_RRp_C.stableNorm() + omega_R0N_N.stableNorm());
    const float domegaTol =
        tol + kRelTol * (omega_R0N_N.stableNorm() * omega_RRp_C.stableNorm() + domega_R0N_N.stableNorm());
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
inline void fuzzRegressionMrpRotation(const Eigen::Vector3f& sigma_RbR0,
                                      const Eigen::Vector3f& sigma_CnB,
                                      const Eigen::Vector3f& sigma_CB,
                                      const Eigen::Vector3f& sigma_RpRc_C,
                                      const Eigen::Vector3f& spinAxis_C,
                                      float spinRate,
                                      const Eigen::Vector3f& sigma_R0N,
                                      const Eigen::Vector3f& omega_R0N_N,
                                      float updateTimeSec) {
    regressionTestMrpRotation(sigma_RbR0,
                              sigma_CnB,
                              sigma_CB,
                              sigma_RpRc_C,
                              spinAxis_C,
                              spinRate,
                              sigma_R0N,
                              omega_R0N_N,
                              Eigen::Vector3f::Zero(),
                              updateTimeSec,
                              3);
}

// ---------------------------------------------------------------------------
// Property test helpers
// ---------------------------------------------------------------------------

// Output reference is finite for any finite inputs and finite configuration.
inline void propertyOutputIsFinite(const Eigen::Vector3f& sigma_RbR0,
                                   const Eigen::Vector3f& sigma_CnB,
                                   const Eigen::Vector3f& sigma_CB,
                                   const Eigen::Vector3f& sigma_RpRc_C,
                                   const Eigen::Vector3f& spinAxis_C,
                                   float spinRate) {
    if (!hasSpinDirection(spinAxis_C)) {
        return;
    }
    const Eigen::Vector3f spinHat_C = spinAxis_C.stableNormalized();
    constexpr float kPropertyControlPeriod = 0.5F;
    const auto config = MrpRotationConfig::create(
        sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, spinHat_C, spinRate, kPropertyControlPeriod);
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

// With the input reference, the base frame, both mountings and the offset at identity, the output sigma_RN encodes
// the integrated spin sigma_RRp, which the reference tracks independently. dcmToMrp can return either shadow-set
// representative near norm = 1, so accept whichever is closer.
inline void propertySigmaRNEqualsSpinWhenFixedRotationsAndInputRefAreIdentity(const Eigen::Vector3f& spinAxis_C,
                                                                              float spinRate) {
    if (!hasSpinDirection(spinAxis_C)) {
        return;
    }
    const Eigen::Vector3f spinHat_C = spinAxis_C.stableNormalized();
    const Eigen::Vector3f omega_RRp_C = spinRate * spinHat_C;
    constexpr float kControlPeriod = 0.1F;
    constexpr int kNumSteps = 10;

    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const auto config = MrpRotationConfig::create(zero, zero, zero, zero, spinHat_C, spinRate, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs identityRef{};
    MrpRotationReferenceState refState{zero, zero, zero, zero, omega_RRp_C};

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
inline void propertySigmaRNNormLessOrEqualToOne(const Eigen::Vector3f& sigma_RbR0,
                                                const Eigen::Vector3f& sigma_CnB,
                                                const Eigen::Vector3f& sigma_CB,
                                                const Eigen::Vector3f& sigma_RpRc_C,
                                                const Eigen::Vector3f& spinAxis_C,
                                                float spinRate) {
    if (!hasSpinDirection(spinAxis_C)) {
        return;
    }
    const Eigen::Vector3f spinHat_C = spinAxis_C.stableNormalized();
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 100;
    constexpr float kMrpNormBound = 1.0F + 1e-5F;  // small fp32 slack at the boundary

    const auto config =
        MrpRotationConfig::create(sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, spinHat_C, spinRate, kControlPeriod);
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

// The identity omega_RN_N = [RpN]^T * [BC] * omega_RRp_C + omega_R0N_N follows from the transport theorem and from Rp
// being fixed in R0. Build [RpN] from the configured base frame, mountings and offset and the input reference, and
// verify the decomposition on every step, independently of the reference implementation.
inline void propertyOmegaRNDecomposesCorrectly(const Eigen::Vector3f& sigma_RbR0,
                                               const Eigen::Vector3f& sigma_CnB,
                                               const Eigen::Vector3f& sigma_CB,
                                               const Eigen::Vector3f& sigma_RpRc_C,
                                               const Eigen::Vector3f& spinAxis_C,
                                               float spinRate,
                                               const Eigen::Vector3f& sigma_R0N,
                                               const Eigen::Vector3f& omega_R0N_N) {
    if (!hasSpinDirection(spinAxis_C)) {
        return;
    }
    const Eigen::Vector3f spinHat_C = spinAxis_C.stableNormalized();
    const Eigen::Vector3f omega_RRp_C = spinRate * spinHat_C;
    constexpr float kControlPeriod = 0.25F;
    constexpr int kNumSteps = 5;

    const auto config =
        MrpRotationConfig::create(sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, spinHat_C, spinRate, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{sigma_R0N, omega_R0N_N, Eigen::Vector3f::Zero()};

    // Per-component fp32 error of this check is ~ N * eps * (||omega_RRp_C|| + ||omega_R0N_N||), summing the
    // sum/subtract cancellation (~2 ops), the [BC] and dcm_RpN^T matvecs (~4 ops), and the algorithm's different path
    // to N through the integrated [RRp] and the cached product (~7 ops), so N ~ 13. A 2x safety margin guards the
    // adversarial fuzz corners; the 1e-5 floor covers the small-rate base noise.
    constexpr float kRoundingOps = 13.0F;
    constexpr float kSafetyMargin = 2.0F;
    constexpr float kRelTol = kSafetyMargin * kRoundingOps * std::numeric_limits<float>::epsilon();
    const float tol = 1e-5F + kRelTol * (omega_RRp_C.stableNorm() + omega_R0N_N.stableNorm());
    const Eigen::Matrix3f dcm_RpN = referenceDcm_RpN(sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, sigma_R0N);
    const Eigen::Vector3f expectedOmega_RRp_N = dcm_RpN.transpose() * (mrpToDcm(sigma_CB).transpose() * omega_RRp_C);
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(attRef);
        const Eigen::Vector3f actualOmega_RRp_N = out.omega_RN_N - omega_R0N_N;
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(actualOmega_RRp_N(i), expectedOmega_RRp_N(i), tol);
        }
    }
}

// With the base frame, both mountings and the offset at identity and zero rate, the output reference R must
// coincide with the input reference R0 on every step.
inline void propertyOutputRefEqualsInputRefWhenRotationIsZero(const Eigen::Vector3f& sigma_R0N,
                                                              const Eigen::Vector3f& omega_R0N_N,
                                                              const Eigen::Vector3f& domega_R0N_N) {
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 5;

    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const auto config = MrpRotationConfig::create(zero,
                                                  zero,
                                                  zero,
                                                  zero,
                                                  /* spinAxis_C = */ Eigen::Vector3f::UnitX(),
                                                  /* spinRate = */ 0.0F,
                                                  kControlPeriod);
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

// With zero rate, the output attitude is the input reference with the constant rotations applied:
// [RN] = [RpRc][RcRb][RbR0][R0N] on every step.
inline void propertyOutputEqualsOffsetInputRefWhenRateIsZero(const Eigen::Vector3f& sigma_RbR0,
                                                             const Eigen::Vector3f& sigma_CnB,
                                                             const Eigen::Vector3f& sigma_CB,
                                                             const Eigen::Vector3f& sigma_RpRc_C,
                                                             const Eigen::Vector3f& sigma_R0N) {
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 5;

    const auto config = MrpRotationConfig::create(sigma_RbR0,
                                                  sigma_CnB,
                                                  sigma_CB,
                                                  sigma_RpRc_C,
                                                  /* spinAxis_C = */ Eigen::Vector3f::UnitX(),
                                                  /* spinRate = */ 0.0F,
                                                  kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{sigma_R0N, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
    const Eigen::Matrix3f expectedDcm_RN = referenceDcm_RpN(sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, sigma_R0N);

    constexpr float tol = 1e-5F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(attRef);
        EXPECT_LT(dcmMaxAbsDiff(mrpToDcm(out.sigma_RN), expectedDcm_RN), tol);
    }
}

// With zero offset and zero rate, the measured component frame C sits exactly where the nominal frame Cn sits when
// the body is on the base: [CB][RN] = [CnB][RbR0][R0N]. Every component axis, the boresight included, points where
// its nominal counterpart points in the base attitude, whatever the misalignment between the two mountings.
inline void propertyComponentFrameSitsOnNominalFrameAtBase(const Eigen::Vector3f& sigma_RbR0,
                                                           const Eigen::Vector3f& sigma_CnB,
                                                           const Eigen::Vector3f& sigma_CB,
                                                           const Eigen::Vector3f& sigma_R0N) {
    constexpr float kControlPeriod = 0.5F;

    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const auto config = MrpRotationConfig::create(sigma_RbR0,
                                                  sigma_CnB,
                                                  sigma_CB,
                                                  zero,
                                                  /* spinAxis_C = */ Eigen::Vector3f::UnitZ(),
                                                  /* spinRate = */ 0.0F,
                                                  kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const MrpRotationAttRefInputs attRef{sigma_R0N, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
    const MrpRotationOutput out = alg.update(attRef);

    const Eigen::Matrix3f dcm_CN = mrpToDcm(sigma_CB) * mrpToDcm(out.sigma_RN);
    const Eigen::Matrix3f expectedDcm_CN = mrpToDcm(sigma_CnB) * mrpToDcm(sigma_RbR0) * mrpToDcm(sigma_R0N);
    EXPECT_LT(dcmMaxAbsDiff(dcm_CN, expectedDcm_CN), 1e-5F);
}

// The spin starts at identity and its axis spinHat_C is constant in component axes, so the component axis along
// spinHat_C, which is [BC] * spinHat_C in body axes, stays fixed in the input reference R0 at
// [RpR0]^T * [BC] * spinHat_C over many steps, whatever the base frame, mountings and offset. A spin about the
// boresight therefore holds the boresight on the offset point. The axis has the same components in R and Rp, so
// [RN]^T [BC] * spinHat_C must give that vector. The input reference is identity, so dcm_RN = dcm_RR0.
// Forward Euler keeps the integrated MRP on the axis only for small steps: above about 0.2 rad per step,
// fp32 rounding off the axis grows exponentially, so callers keep |spinRate| * controlPeriod well below that.
inline void propertyRotationAxisFixedInInputRef(const Eigen::Vector3f& sigma_RbR0,
                                                const Eigen::Vector3f& sigma_CnB,
                                                const Eigen::Vector3f& sigma_CB,
                                                const Eigen::Vector3f& sigma_RpRc_C,
                                                const Eigen::Vector3f& spinAxis_C,
                                                float spinRate) {
    if (!hasSpinDirection(spinAxis_C)) {
        return;
    }
    const Eigen::Vector3f spinHat_C = spinAxis_C.stableNormalized();
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 2000;
    constexpr float tol = 1e-4F;

    const auto config =
        MrpRotationConfig::create(sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, spinHat_C, spinRate, kControlPeriod);
    MrpRotationAlgorithm alg{config};

    const Eigen::Vector3f spinHat_B = mrpToDcm(sigma_CB).transpose() * spinHat_C;
    const Eigen::Matrix3f dcm_RpR0 =
        referenceDcm_RpN(sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, Eigen::Vector3f::Zero());
    const Eigen::Vector3f expectedAxis_R0 = dcm_RpR0.transpose() * spinHat_B;

    const MrpRotationAttRefInputs identityRef{};
    float maxErr = 0.0F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(identityRef);
        const Eigen::Vector3f axis_R0 = mrpToDcm(out.sigma_RN).transpose() * spinHat_B;
        maxErr = std::max(maxErr, (axis_R0 - expectedAxis_R0).cwiseAbs().maxCoeff());
    }
    EXPECT_LT(maxErr, tol);
}

#endif  // TEST_MRPROTATION_H
