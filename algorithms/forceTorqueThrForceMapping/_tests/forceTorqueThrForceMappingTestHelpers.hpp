#ifndef TEST_FORCE_TORQUE_THR_FORCE_MAPPING_H
#define TEST_FORCE_TORQUE_THR_FORCE_MAPPING_H

#include "forceTorqueThrForceMappingAlgorithm.h"
#include "msgPayloadDef/definitions.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

// Forward declarations for layout fixtures defined at the bottom of this file. The balanced-layout
// property helper below uses these directly; the fixtures themselves stay grouped with the other
// regression-test geometries.
inline std::vector<Eigen::Vector3f> rcsPositions1();
inline std::vector<Eigen::Vector3f> rcsDirections1();

// Property/regression helpers below run on layouts including fuzzer inputs and the deliberately
// rank-deficient layout 1, so they opt every axis out of the controllability assertion (a true entry
// would throw at construction on a rank-deficient layout). Tests that specifically exercise the
// assertion pass their own axes to ForceTorqueThrForceMappingConfig::create.
inline constexpr std::array<bool, 6> kNoAxisAssertion{false, false, false, false, false, false};

// Build a configured algorithm from a thruster array, CoM, and controllability assertion vector. The
// mapping is computed in the constructor and throws if an asserted axis is uncontrollable.
inline ForceTorqueThrForceMappingAlgorithm makeMappingAlgorithm(const ThrusterArrayConfiguration& config,
                                                                const Eigen::Vector3f& CoM,
                                                                const std::array<bool, 6>& axes = kNoAxisAssertion) {
    return ForceTorqueThrForceMappingAlgorithm{ForceTorqueThrForceMappingConfig::create(config, CoM, axes)};
}

// Combined atol + rtol tolerance for fp32 comparisons.
inline float combinedTolerance(float expected, float atol, float rtol) { return atol + rtol * std::fabs(expected); }

// Build the DG matrix (6 x MAX_EFF_CNT) from a configured thruster array and CoM. Rows 0-2 are
// moment arms (r-CoM)×g, rows 3-5 are thrust directions g. Trailing columns beyond numThrusters
// are zero. Used by property helpers that need to compute the achieved force/torque from the
// algorithm's output independently of the algorithm's internal storage.
inline Eigen::Matrix<float, 6, MAX_EFF_CNT> buildDG(const ThrusterArrayConfiguration& config,
                                                    const Eigen::Vector3f& CoM) {
    Eigen::Matrix<float, 6, MAX_EFF_CNT> DG = Eigen::Matrix<float, 6, MAX_EFF_CNT>::Zero();
    for (std::uint32_t i = 0; i < config.numThrusters; ++i) {
        const Eigen::Vector3f r(
            config.thrusters.at(i).r_TB_B[0], config.thrusters.at(i).r_TB_B[1], config.thrusters.at(i).r_TB_B[2]);
        const Eigen::Vector3f g(
            config.thrusters.at(i).tHat_B[0], config.thrusters.at(i).tHat_B[1], config.thrusters.at(i).tHat_B[2]);
        const Eigen::Vector3f arm = r - CoM;
        DG.col(static_cast<int>(i)).head<3>() = arm.cross(g);
        DG.col(static_cast<int>(i)).tail<3>() = g;
    }
    return DG;
}

// Build a ThrusterArrayConfiguration from raw per-thruster vectors. `directions` are normalized here so the
// resulting config is always valid for `setThrusters`. Returns false if the inputs cannot produce a
// valid configuration (wrong count, size mismatch, or near-zero direction vector), in which case the
// caller should skip the test input. This is used by both gtest and fuzz harnesses.
inline bool buildThrusterConfig(std::uint32_t numThrusters,
                                const std::vector<Eigen::Vector3f>& positions,
                                const std::vector<Eigen::Vector3f>& directions,
                                ThrusterArrayConfiguration& config) {
    if (numThrusters < 1U || numThrusters > MAX_EFF_CNT) {
        return false;
    }
    if (positions.size() < numThrusters || directions.size() < numThrusters) {
        return false;
    }

    config = ThrusterArrayConfiguration{};
    config.numThrusters = numThrusters;
    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        config.thrusters.at(i).r_TB_B = {positions[i].x(), positions[i].y(), positions[i].z()};

        Eigen::Vector3f dir = directions[i];
        const float norm = dir.stableNorm();
        if (norm < 1e-3F) {
            return false;
        }
        dir /= norm;
        config.thrusters.at(i).tHat_B = {dir.x(), dir.y(), dir.z()};
    }
    return true;
}

// Independent truth implementation of update(). Mirrors the algorithm's truncated-SVD pseudo-inverse
// in fp64 so numeric disagreement reflects real fp32 round-off rather than algorithmic divergence.
// Two details must match the algorithm exactly:
//   1. DG has the same shape (6 × MAX_EFF_CNT with trailing zero columns), so the SVD's left
//      singular vectors and the kept singular values are computed on an identical operator.
//   2. The truncation cutoff uses fp32 epsilon scaled by max(6, MAX_EFF_CNT) — the algorithm's
//      noise floor — instead of fp64 epsilon. Otherwise the reference keeps singular values in the
//      gap [eps_d, eps_f] that the algorithm correctly drops as fp32 noise, and 1/sv blows up.
// Assumes `directions` are already unit vectors (call buildThrusterConfig first if needed).
inline Eigen::Vector<float, MAX_EFF_CNT> referenceUpdate(std::uint32_t numThrusters,
                                                         const std::vector<Eigen::Vector3f>& positions,
                                                         const std::vector<Eigen::Vector3f>& directions,
                                                         const Eigen::Vector3f& CoM_B,
                                                         const Eigen::Vector3f& cmdTorque_B,
                                                         const Eigen::Vector3f& cmdForce_B,
                                                         float* absErrorScale = nullptr) {
    Eigen::Vector<float, MAX_EFF_CNT> result = Eigen::Vector<float, MAX_EFF_CNT>::Zero();
    if (numThrusters < 1U || numThrusters > MAX_EFF_CNT) {
        return result;
    }

    Eigen::Matrix<double, 6, MAX_EFF_CNT> DG = Eigen::Matrix<double, 6, MAX_EFF_CNT>::Zero();
    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        const Eigen::Vector3d r = positions[i].cast<double>();
        const Eigen::Vector3d g = directions[i].cast<double>();
        const Eigen::Vector3d arm = r - CoM_B.cast<double>();
        const Eigen::Vector3d cross = arm.cross(g);
        DG.col(static_cast<int>(i)).head<3>() = cross;
        DG.col(static_cast<int>(i)).tail<3>() = g;
    }

    Eigen::Matrix<double, 6, 1> ft;
    ft << cmdTorque_B.cast<double>(), cmdForce_B.cast<double>();

    Eigen::JacobiSVD<Eigen::Matrix<double, 6, MAX_EFF_CNT>> svd(DG, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix<double, 6, 1> sv = svd.singularValues();
    constexpr int kMaxDim = (6 > MAX_EFF_CNT) ? 6 : MAX_EFF_CNT;
    const double tol =
        sv(0) * static_cast<double>(std::numeric_limits<float>::epsilon()) * static_cast<double>(kMaxDim);

    Eigen::Matrix<double, 6, 1> invSv = Eigen::Matrix<double, 6, 1>::Zero();
    double minKeptSv = sv(0);
    for (int i = 0; i < 6; ++i) {
        if (sv(i) > tol) {
            invSv(i) = 1.0 / sv(i);
            minKeptSv = sv(i);  // sv is sorted descending, so this ends on the smallest kept value
        }
    }
    const Eigen::Matrix<double, MAX_EFF_CNT, 1> thrForces =
        svd.matrixV().leftCols<6>() * invSv.asDiagonal() * svd.matrixU().transpose() * ft;

    // Error scale for the fp32-vs-fp64 comparison: cond * ||F_pre||_inf. The fp32 solver's relative
    // error ~eps*cond makes the absolute per-entry error ~eps*cond*||F_pre||_inf.
    if (absErrorScale != nullptr) {
        const double cond = sv(0) / minKeptSv;
        const double preShiftMaxAbs = thrForces.head(numThrusters).cwiseAbs().maxCoeff();
        *absErrorScale = static_cast<float>(cond * preShiftMaxAbs);
    }

    // min-shift over the active head only, matching the algorithm.
    const double minForce = thrForces.head(numThrusters).minCoeff();
    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        result[static_cast<int>(i)] = static_cast<float>(thrForces(i) - minForce);
    }
    return result;
}

// Configures the algorithm, runs update(), and compares against the SVD reference.
inline void runRegressionCase(std::uint32_t numThrusters,
                              std::vector<Eigen::Vector3f> positions,
                              std::vector<Eigen::Vector3f> directions,
                              const Eigen::Vector3f& CoM,
                              const Eigen::Vector3f& cmdTorque,
                              const Eigen::Vector3f& cmdForce) {
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(numThrusters, positions, directions, config)) {
        return;
    }
    // create() rejects ill-conditioned or uncontrollable configs; skip the inputs it would reject.
    if (!ForceTorqueThrForceMappingConfig::isValidMapping(config, CoM, kNoAxisAssertion)) {
        return;
    }

    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM);

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(cmdTorque, cmdForce);

    // The reference uses the post-normalization directions stored in the config so that both
    // implementations start from identical unit-norm inputs.
    std::vector<Eigen::Vector3f> unitDirs(numThrusters);
    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        unitDirs[i] = Eigen::Vector3f(
            config.thrusters.at(i).tHat_B[0], config.thrusters.at(i).tHat_B[1], config.thrusters.at(i).tHat_B[2]);
    }
    float absErrorScale = 0.0F;
    const Eigen::Vector<float, MAX_EFF_CNT> ref =
        referenceUpdate(numThrusters, positions, unitDirs, CoM, cmdTorque, cmdForce, &absErrorScale);

    // Flat 1e-3 budget plus the fp32 algorithm error floor sqrt(n)*eps*absErrorScale: absolute error is
    // ~eps*cond*||F_pre||_inf (= absErrorScale), and sqrt(n) is the inf<-2 norm conversion bounding
    // the per-entry error. This term dominates for large commands on near-cond-100 layouts.
    constexpr float kAtol = 1e-3F;
    constexpr float kRtol = 1e-3F;
    const float fpErrorTol =
        std::sqrt(static_cast<float>(numThrusters)) * std::numeric_limits<float>::epsilon() * absErrorScale;
    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        const int idx = static_cast<int>(i);
        EXPECT_TRUE(std::isfinite(out[idx]));
        EXPECT_NEAR(out[idx], ref[idx], combinedTolerance(ref[idx], kAtol, kRtol) + fpErrorTol);
    }
    for (int i = static_cast<int>(numThrusters); i < MAX_EFF_CNT; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0F);
    }
}

// ---------------------------------------------------------------------------
// Property test helper functions — each asserts an invariant that must hold
// for any valid thruster configuration and any commanded torque/force.
// Each has an early-return guard for invalid inputs so the fuzz harness can
// drop unusable samples silently.
// ---------------------------------------------------------------------------

// Every active thruster force is non-negative (min-shift guarantees this).
inline void propertyNonNegativeForces(std::uint32_t numThrusters,
                                      std::vector<Eigen::Vector3f> positions,
                                      std::vector<Eigen::Vector3f> directions,
                                      const Eigen::Vector3f& CoM,
                                      const Eigen::Vector3f& cmdTorque,
                                      const Eigen::Vector3f& cmdForce) {
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(numThrusters, positions, directions, config)) {
        return;
    }
    // create() rejects ill-conditioned or uncontrollable configs; skip the inputs it would reject.
    if (!ForceTorqueThrForceMappingConfig::isValidMapping(config, CoM, kNoAxisAssertion)) {
        return;
    }

    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM);

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(cmdTorque, cmdForce);

    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        EXPECT_GE(out[static_cast<int>(i)], -1e-5F);  // small slack for fp32 round-off
    }
}

// The minimum active thruster force is zero (post-shift property — the min element must be exactly
// the subtracted value, leaving a zero).
inline void propertyMinimumIsZero(std::uint32_t numThrusters,
                                  std::vector<Eigen::Vector3f> positions,
                                  std::vector<Eigen::Vector3f> directions,
                                  const Eigen::Vector3f& CoM,
                                  const Eigen::Vector3f& cmdTorque,
                                  const Eigen::Vector3f& cmdForce) {
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(numThrusters, positions, directions, config)) {
        return;
    }
    // create() rejects ill-conditioned or uncontrollable configs; skip the inputs it would reject.
    if (!ForceTorqueThrForceMappingConfig::isValidMapping(config, CoM, kNoAxisAssertion)) {
        return;
    }

    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM);

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(cmdTorque, cmdForce);

    float minVal = out[0];
    for (std::uint32_t i = 1; i < numThrusters; ++i) {
        minVal = std::min(minVal, out[static_cast<int>(i)]);
    }
    EXPECT_NEAR(minVal, 0.0F, 1e-5F);
}

// Output entries beyond numThrusters are exactly zero — the algorithm must not touch unused slots.
inline void propertyPaddingIsZero(std::uint32_t numThrusters,
                                  std::vector<Eigen::Vector3f> positions,
                                  std::vector<Eigen::Vector3f> directions,
                                  const Eigen::Vector3f& CoM,
                                  const Eigen::Vector3f& cmdTorque,
                                  const Eigen::Vector3f& cmdForce) {
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(numThrusters, positions, directions, config)) {
        return;
    }
    // create() rejects ill-conditioned or uncontrollable configs; skip the inputs it would reject.
    if (!ForceTorqueThrForceMappingConfig::isValidMapping(config, CoM, kNoAxisAssertion)) {
        return;
    }

    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM);

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(cmdTorque, cmdForce);

    for (int i = static_cast<int>(numThrusters); i < MAX_EFF_CNT; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0F);
    }
}

// Raw direction norms must be large enough that normalization doesn't turn fp32 noise into an
// arbitrary unit vector. Without this, two thrusters whose raw directions are ~1e-3 can normalize
// to near-identical unit vectors, making DG near-rank-deficient and pseudo-inverse entries grow
// to ~1e5 — at which point any fp32 noise in the arm computation swamps the invariance under test.
// Used by the numerically-sensitive property helpers (scale, CoM translation).
inline bool rawDirectionsWellScaled(std::uint32_t numThrusters, const std::vector<Eigen::Vector3f>& directions) {
    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        if (directions[i].stableNorm() < 0.5F) {
            return false;
        }
    }
    return true;
}

// Positive scaling of the commanded torque and force scales the output by the same factor.
// Rationale: pseudoInverseDG * (k * v) = k * (pseudoInverseDG * v); for k > 0 the min-shift is also
// scaled by k, so the post-shift output scales linearly. create() rejects ill-conditioned layouts, so the
// kept singular subspace has condition number <= 100 and the linear-scaling residual stays within tolerance.
inline void propertyScaleInvariance(std::uint32_t numThrusters,
                                    std::vector<Eigen::Vector3f> positions,
                                    std::vector<Eigen::Vector3f> directions,
                                    const Eigen::Vector3f& CoM,
                                    const Eigen::Vector3f& cmdTorque,
                                    const Eigen::Vector3f& cmdForce,
                                    float scale) {
    if (!(scale > 0.0F) || !std::isfinite(scale)) {
        return;
    }
    if (!rawDirectionsWellScaled(numThrusters, directions)) {
        return;
    }
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(numThrusters, positions, directions, config)) {
        return;
    }
    // create() rejects ill-conditioned or uncontrollable configs; skip the inputs it would reject.
    if (!ForceTorqueThrForceMappingConfig::isValidMapping(config, CoM, kNoAxisAssertion)) {
        return;
    }

    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM);

    const Eigen::Vector<float, MAX_EFF_CNT> baseOut = alg.update(cmdTorque, cmdForce);
    const Eigen::Vector<float, MAX_EFF_CNT> scaledOut = alg.update(scale * cmdTorque, scale * cmdForce);

    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        const int idx = static_cast<int>(i);
        const float expected = scale * baseOut[idx];
        EXPECT_NEAR(scaledOut[idx], expected, combinedTolerance(expected, 1e-4F, 1e-4F));
    }
}

// update() is const — repeated calls with the same inputs must return bitwise-identical output.
inline void propertyStateless(std::uint32_t numThrusters,
                              std::vector<Eigen::Vector3f> positions,
                              std::vector<Eigen::Vector3f> directions,
                              const Eigen::Vector3f& CoM,
                              const Eigen::Vector3f& cmdTorque,
                              const Eigen::Vector3f& cmdForce) {
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(numThrusters, positions, directions, config)) {
        return;
    }
    // create() rejects ill-conditioned or uncontrollable configs; skip the inputs it would reject.
    if (!ForceTorqueThrForceMappingConfig::isValidMapping(config, CoM, kNoAxisAssertion)) {
        return;
    }

    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM);

    const Eigen::Vector<float, MAX_EFF_CNT> first = alg.update(cmdTorque, cmdForce);
    for (int step = 0; step < 5; ++step) {
        const Eigen::Vector<float, MAX_EFF_CNT> again = alg.update(cmdTorque, cmdForce);
        for (int i = 0; i < MAX_EFF_CNT; ++i) {
            EXPECT_EQ(first[i], again[i]);
        }
    }
}

// All output components are finite for finite inputs — no NaN/Inf leakage even for rank-deficient D.
inline void propertyFiniteOutput(std::uint32_t numThrusters,
                                 std::vector<Eigen::Vector3f> positions,
                                 std::vector<Eigen::Vector3f> directions,
                                 const Eigen::Vector3f& CoM,
                                 const Eigen::Vector3f& cmdTorque,
                                 const Eigen::Vector3f& cmdForce) {
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(numThrusters, positions, directions, config)) {
        return;
    }
    // create() rejects ill-conditioned or uncontrollable configs; skip the inputs it would reject.
    if (!ForceTorqueThrForceMappingConfig::isValidMapping(config, CoM, kNoAxisAssertion)) {
        return;
    }

    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM);

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(cmdTorque, cmdForce);
    for (int i = 0; i < MAX_EFF_CNT; ++i) {
        EXPECT_TRUE(std::isfinite(out[i]));
    }
}

// Reachable commands are reproduced exactly on a balanced layout. Pinned to layout 1 (verified
// analytically: sum(g_i) = 0 and sum(r_i × g_i) = 0), so DG·1 = 0 regardless of CoM (the CoM
// translation contribution is -CoM × sum(g) = 0). With DG·1 = 0, the algorithm's min-shift
// (x_alg = x_pre - min·1) shifts strictly within the null space and so doesn't disturb the
// achieved FT — the algorithm reproduces cmd exactly modulo fp32 noise. CoM and per-thruster
// forces fuzz; the layout stays fixed.
inline void propertyAchievesCommandForBalancedLayout(const Eigen::Vector3f& CoM,
                                                     const Eigen::Matrix<float, 8, 1>& testForces) {
    constexpr std::uint32_t numThrusters = 8U;
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(numThrusters, rcsPositions1(), rcsDirections1(), config)) {
        return;
    }

    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM);

    const Eigen::Matrix<float, 6, MAX_EFF_CNT> DG = buildDG(config, CoM);

    // Build a non-negative test force vector — cmd = DG·x_test is then in the row space of DG.
    Eigen::Vector<float, MAX_EFF_CNT> x_test = Eigen::Vector<float, MAX_EFF_CNT>::Zero();
    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        x_test[static_cast<int>(i)] = std::fabs(testForces[static_cast<int>(i)]);
    }
    const Eigen::Vector<float, 6> cmd = DG * x_test;

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(cmd.head<3>(), cmd.tail<3>());
    const Eigen::Vector<float, 6> achieved = DG * out;

    // 1e-4 atol covers fp32 absolute round-off in DG·out when the commanded component happens to
    // be zero — the rtol budget collapses to atol alone in that case, and fuzz x_test up to 10
    // drives ||DG·out|| accumulator noise to ~1e-5 absolute, just past 1e-5.
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(achieved[i], cmd[i], combinedTolerance(cmd[i], 1e-4F, 1e-4F));
    }
}

// Output magnitude stays bounded for any input within the configured fuzz domain. The truncated-SVD
// pinv has ‖pinv‖ ≤ 1/(σ_max · eps · max(m,n)); combined with the bounded cmd magnitude, ‖out‖ is
// bounded above. The threshold below is loose enough to absorb worst-case ill-conditioning within
// the configured fuzz ranges, but tight enough to catch a genuine numerical regression in the
// truncation logic (e.g. a removed eps · max(m,n) factor would let ‖pinv‖ blow up by ~1e6).
inline void propertyOutputMagnitudeBounded(std::uint32_t numThrusters,
                                           std::vector<Eigen::Vector3f> positions,
                                           std::vector<Eigen::Vector3f> directions,
                                           const Eigen::Vector3f& CoM,
                                           const Eigen::Vector3f& cmdTorque,
                                           const Eigen::Vector3f& cmdForce) {
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(numThrusters, positions, directions, config)) {
        return;
    }
    // create() rejects ill-conditioned or uncontrollable configs; skip the inputs it would reject.
    if (!ForceTorqueThrForceMappingConfig::isValidMapping(config, CoM, kNoAxisAssertion)) {
        return;
    }

    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM);

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(cmdTorque, cmdForce);

    constexpr float kMagnitudeBound = 1e7F;
    const float maxAbs = out.head(numThrusters).cwiseAbs().maxCoeff();
    EXPECT_LT(maxAbs, kMagnitudeBound);
}

// ---------------------------------------------------------------------------
// Fixed test-vector constants — mirror the Python test parametrizations so the
// C++ regression suite exercises the same geometries.
// ---------------------------------------------------------------------------

// 8-thruster RCS layout 1: directions lie in the y-z plane (no X-axis thrust component), so
// force_X is uncontrollable and gets dropped by the selector. torque_X, torque_Y, torque_Z,
// force_Y, force_Z are all controllable. The layout is balanced: sum(g_i) = 0 and
// sum(r_i × g_i) = 0, so DG·1 = 0 and the min-shift preserves the requested force and torque.
inline std::vector<Eigen::Vector3f> rcsPositions1() {
    return {{0.964717F, 0.88138F, 1.800225F},
            {0.964717F, 0.88138F, 0.565785F},
            {-0.964717F, 0.88138F, 1.800225F},
            {-0.964717F, 0.88138F, 0.565785F},
            {-0.964717F, -0.88138F, 1.800225F},
            {-0.964717F, -0.88138F, 0.565785F},
            {0.964717F, -0.88138F, 1.800225F},
            {0.964717F, -0.88138F, 0.565785F}};
}

inline std::vector<Eigen::Vector3f> rcsDirections1() {
    constexpr float s = 0.70710678F;
    return {{0.0F, -s, s},
            {0.0F, -s, -s},
            {0.0F, -s, s},
            {0.0F, -s, -s},
            {0.0F, s, s},
            {0.0F, s, -s},
            {0.0F, s, s},
            {0.0F, s, -s}};
}

// 12-thruster layout 2 — thrusters cover all six axes so DG is full-rank.
inline std::vector<Eigen::Vector3f> rcsPositions2() {
    return {{-1.0F, -1.0F, 1.0F},
            {-1.0F, -1.0F, 1.0F},
            {-1.0F, -1.0F, 1.0F},
            {1.0F, 1.0F, 1.0F},
            {1.0F, 1.0F, 1.0F},
            {1.0F, 1.0F, 1.0F},
            {1.0F, 1.0F, -1.0F},
            {1.0F, 1.0F, -1.0F},
            {1.0F, 1.0F, -1.0F},
            {-1.0F, -1.0F, -1.0F},
            {-1.0F, -1.0F, -1.0F},
            {-1.0F, -1.0F, -1.0F}};
}

inline std::vector<Eigen::Vector3f> rcsDirections2() {
    return {{1.0F, 0.0F, 0.0F},
            {0.0F, 1.0F, 0.0F},
            {0.0F, 0.0F, -1.0F},
            {0.0F, 0.0F, -1.0F},
            {0.0F, -1.0F, 0.0F},
            {-1.0F, 0.0F, 0.0F},
            {0.0F, -1.0F, 0.0F},
            {-1.0F, 0.0F, 0.0F},
            {0.0F, 0.0F, 1.0F},
            {1.0F, 0.0F, 0.0F},
            {0.0F, 1.0F, 0.0F},
            {0.0F, 0.0F, 1.0F}};
}

#endif  // TEST_FORCE_TORQUE_THR_FORCE_MAPPING_H
