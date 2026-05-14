#include "forceTorqueThrForceMappingTestHelpers.hpp"
#include "utilities/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <array>

// ---------------------------------------------------------------------------
// Regression tests — mirror the Python test fixtures so the C++ suite pins the
// same numerical behavior against an independent SVD-based reference.
// ---------------------------------------------------------------------------

TEST(ForceTorqueThrForceMappingTest, RegressionUncontrollableXAxis) {
    runRegressionCase(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionPureForceZeroTorque) {
    runRegressionCase(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionNoTorqueCommand) {
    // Mirrors the "no torque message connected" Python case: torque defaults to zero at the adapter
    // boundary, so at the algorithm layer it's indistinguishable from an explicit zero command.
    runRegressionCase(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionAllDirectionsCovered) {
    runRegressionCase(
        12U, rcsPositions2(), rcsDirections2(), {0.1F, 0.1F, 0.1F}, {0.0F, 0.0F, 0.0F}, {0.9F, 1.1F, 1.0F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionCoMAtOrigin) {
    runRegressionCase(
        8U, rcsPositions1(), rcsDirections1(), {0.0F, 0.0F, 0.0F}, {0.2F, -0.1F, 0.3F}, {0.0F, 0.5F, -0.4F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionLargeCoMOffset) {
    runRegressionCase(
        8U, rcsPositions1(), rcsDirections1(), {1.0F, 1.0F, 1.0F}, {0.2F, -0.1F, 0.3F}, {0.0F, 0.5F, -0.4F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionSingleThruster) {
    runRegressionCase(
        1U, {{0.0F, 0.0F, 0.0F}}, {{1.0F, 0.0F, 0.0F}}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F});
}

// ---------------------------------------------------------------------------
// Setup test — default state, getter/setter round-trips, and exception paths.
// ---------------------------------------------------------------------------

TEST(ForceTorqueThrForceMappingTest, SetupTest) {
    ForceTorqueThrForceMappingAlgorithm alg{};

    // Default state: CoM at origin, no thrusters configured.
    const Eigen::Vector3f defaultCoM = alg.getCoM_B();
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(defaultCoM[i], 0.0F, 1e-6F);
    }
    EXPECT_EQ(alg.getThrusters().numThrusters, 0U);

    // setCoM_B / getCoM_B round-trip
    const Eigen::Vector3f CoM(0.25F, -0.5F, 1.0F);
    alg.setCoM_B(CoM);
    const Eigen::Vector3f readBackCoM = alg.getCoM_B();
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(readBackCoM[i], CoM[i], 1e-6F);
    }

    // setThrusters / getThrusters round-trip — positions preserved, directions normalized.
    ThrusterArrayConfig config{};
    config.numThrusters = 2U;
    config.thrusters[0].rThrust_B = {1.0F, 2.0F, 3.0F};
    config.thrusters[0].tHatThrust_B = {1.0F, 0.0F, 0.0F};
    config.thrusters[1].rThrust_B = {-1.0F, 0.5F, 0.0F};
    config.thrusters[1].tHatThrust_B = {0.0F, 1.0F, 0.0F};
    EXPECT_NO_THROW(alg.setThrusters(config));

    const ThrusterArrayConfig readBack = alg.getThrusters();
    EXPECT_EQ(readBack.numThrusters, 2U);
    for (std::uint32_t t = 0; t < readBack.numThrusters; ++t) {
        for (int i = 0; i < 3; ++i) {
            EXPECT_FLOAT_EQ(readBack.thrusters[t].rThrust_B[i], config.thrusters[t].rThrust_B[i]);
        }
        const Eigen::Vector3f dir(readBack.thrusters[t].tHatThrust_B[0],
                                  readBack.thrusters[t].tHatThrust_B[1],
                                  readBack.thrusters[t].tHatThrust_B[2]);
        EXPECT_NEAR(dir.stableNorm(), 1.0F, 1e-6F);
    }

    // Invalid configurations throw fsw::invalid_argument.
    ThrusterArrayConfig bad = config;
    bad.numThrusters = 0U;
    EXPECT_THROW(alg.setThrusters(bad), fsw::invalid_argument);

    bad = config;
    bad.numThrusters = MAX_EFF_CNT + 1U;
    EXPECT_THROW(alg.setThrusters(bad), fsw::invalid_argument);

    bad = config;
    bad.thrusters[0].tHatThrust_B = {0.5F, 0.0F, 0.0F};  // norm = 0.5, far outside 1e-3 tolerance
    EXPECT_THROW(alg.setThrusters(bad), fsw::invalid_argument);

    bad = config;
    bad.thrusters[0].tHatThrust_B = {1.01F, 0.0F, 0.0F};  // norm = 1.01, outside 1e-3 tolerance
    EXPECT_THROW(alg.setThrusters(bad), fsw::invalid_argument);

    // Directions within the 1e-3 tolerance band are accepted (and normalized).
    ThrusterArrayConfig nearUnit = config;
    nearUnit.thrusters[0].tHatThrust_B = {1.0F + 5e-4F, 0.0F, 0.0F};
    nearUnit.thrusters[1].tHatThrust_B = {1.0F - 5e-4F, 0.0F, 0.0F};
    EXPECT_NO_THROW(alg.setThrusters(nearUnit));
}

// update() returns the zero vector if computeThrusterMapping() has not been called — the
// pseudoInverseDG matrix is zero-initialized, so the matrix product is zero and the min-shift leaves
// it zero.
TEST(ForceTorqueThrForceMappingTest, UpdateBeforeComputeIsZero) {
    ForceTorqueThrForceMappingAlgorithm alg{};
    ThrusterArrayConfig config{};
    config.numThrusters = 2U;
    config.thrusters[0].rThrust_B = {1.0F, 0.0F, 0.0F};
    config.thrusters[0].tHatThrust_B = {1.0F, 0.0F, 0.0F};
    config.thrusters[1].rThrust_B = {-1.0F, 0.0F, 0.0F};
    config.thrusters[1].tHatThrust_B = {-1.0F, 0.0F, 0.0F};
    alg.setThrusters(config);
    // Deliberately skip computeThrusterMapping().

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update({1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F});
    for (int i = 0; i < MAX_EFF_CNT; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0F);
    }
}

// ---------------------------------------------------------------------------
// Property tests — fixed representative inputs exercising invariants. The
// same helpers are re-run under fuzz inputs in test_..._fuzz.cpp.
// ---------------------------------------------------------------------------

TEST(ForceTorqueThrForceMappingTest, PropertyNonNegativeForces) {
    propertyNonNegativeForces(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyMinimumIsZero) {
    propertyMinimumIsZero(
        12U, rcsPositions2(), rcsDirections2(), {0.0F, 0.0F, 0.0F}, {0.3F, -0.2F, 0.1F}, {0.9F, 1.1F, 1.0F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyPaddingIsZero) {
    propertyPaddingIsZero(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyScaleInvariance) {
    propertyScaleInvariance(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F}, 2.5F);
}

TEST(ForceTorqueThrForceMappingTest, PropertyStateless) {
    propertyStateless(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyFiniteOutput) {
    propertyFiniteOutput(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyAchievesCommandForBalancedLayout) {
    propertyAchievesCommandForBalancedLayout({0.1F, 0.1F, 0.1F}, {0.5F, 0.3F, 0.1F, 0.7F, 0.2F, 0.4F, 0.6F, 0.8F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyOutputMagnitudeBounded) {
    propertyOutputMagnitudeBounded(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

// Documents a known limitation: for an unbalanced layout (DG·1 ≠ 0), the min-shift step perturbs
// the achieved FT by min_shift·(DG·1) whenever pinv·cmd has a negative entry. The test pins a
// fully-controllable but unbalanced 6-thruster layout — three positions each carrying two
// orthogonal thrusters, so rank(DG) = 6 (any FT command is reachable in principle) but
// DG·1 = (−1, −1, −1, 2, 2, 2) is nonzero on every axis. Commanding a pure τ_x forces a negative
// entry in pinv·cmd, which the min-shift then translates into an FT offset along DG·1.
//
// This is a behavioral test, not a property test — it freezes the algorithm's documented behavior
// so a future refactor (e.g. a null-space-projected min-shift) would intentionally break it and
// force a deliberate update of the behavior.
TEST(ForceTorqueThrForceMappingTest, UnbalancedLayoutAchievedFTDiffersFromCommand) {
    // Three positions, each carrying two orthogonal thrusters. Pairs (thr 0, 5), (thr 1, 3),
    // (thr 2, 4) share a position but point along different axes — gives rank-6 DG with
    // DG·1 = (−1, −1, −1, 2, 2, 2).
    const std::vector<Eigen::Vector3f> positions = {{1.0F, 0.0F, 0.0F},
                                                    {0.0F, 1.0F, 0.0F},
                                                    {0.0F, 0.0F, 1.0F},
                                                    {0.0F, 1.0F, 0.0F},
                                                    {0.0F, 0.0F, 1.0F},
                                                    {1.0F, 0.0F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {{1.0F, 0.0F, 0.0F},
                                                     {0.0F, 1.0F, 0.0F},
                                                     {0.0F, 0.0F, 1.0F},
                                                     {1.0F, 0.0F, 0.0F},
                                                     {0.0F, 1.0F, 0.0F},
                                                     {0.0F, 0.0F, 1.0F}};

    ThrusterArrayConfig config{};
    ASSERT_TRUE(buildThrusterConfig(6U, positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.0F, 0.0F, 0.0F});
    alg.setThrusters(config);
    disableDesiredControlAxesAssertion(alg);
    alg.computeThrusterMapping();

    // Pure τ_x = 1. pinv·cmd = (0, 1, 0, 0, −1, 0) — the −1 at thr 4 forces min_shift = −1, so
    // achieved = cmd − min_shift·(DG·1) = cmd + DG·1 = (0, −1, −1, 2, 2, 2). Far from the
    // commanded (1, 0, 0, 0, 0, 0) on every component.
    const Eigen::Vector3f cmdTorque{1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f cmdForce = Eigen::Vector3f::Zero();
    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(cmdTorque, cmdForce);

    const Eigen::Matrix<float, 6, MAX_EFF_CNT> DG = buildDG(config, Eigen::Vector3f::Zero());
    const Eigen::Vector<float, 6> achieved = DG * out;
    Eigen::Vector<float, 6> cmd;
    cmd << cmdTorque, cmdForce;

    // Achieved differs from cmd by ~DG·1, whose largest component is 2. Assert the discrepancy is
    // at least 1 unit on the largest axis — a clear, layout-balance-driven mismatch, not fp32 noise.
    const Eigen::Vector<float, 6> diff = cmd - achieved;
    EXPECT_GT(diff.cwiseAbs().maxCoeff(), 1.0F);
}

// ---------------------------------------------------------------------------
// Edge-case tests — boundary conditions and degenerate geometries.
// ---------------------------------------------------------------------------

// Zero commanded torque and force produces the zero-thrust solution. The raw pseudo-inverse of
// [0;0] is zero, and min-shift of zero is zero.
TEST(ForceTorqueThrForceMappingTest, ZeroCommandProducesZeroOutput) {
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.1F, 0.1F, 0.1F});
    ThrusterArrayConfig config{};
    if (!buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config)) {
        FAIL() << "buildThrusterConfig failed for rcs1 layout";
    }
    alg.setThrusters(config);
    disableDesiredControlAxesAssertion(alg);
    alg.computeThrusterMapping();

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    for (int i = 0; i < MAX_EFF_CNT; ++i) {
        EXPECT_NEAR(out[i], 0.0F, 1e-6F);
    }
}

// All thrusters parallel: only force_x and torque_z rows of DG are nonzero (selector drops the
// other four). With 4 symmetric +X thrusters about origin and a pure force_x = 1 command, the
// min-norm LS solution is x = [0.25, 0.25, 0.25, 0.25]; the min-shift then zeros all four
// outputs. Verifies the algorithm handles rank-deficient DG without NaN or crash and matches the
// analytic solution.
TEST(ForceTorqueThrForceMappingTest, AllThrustersParallel) {
    const std::vector<Eigen::Vector3f> positions = {
        {0.5F, 0.0F, 0.0F}, {-0.5F, 0.0F, 0.0F}, {0.0F, 0.5F, 0.0F}, {0.0F, -0.5F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {
        {1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}};

    ThrusterArrayConfig config{};
    ASSERT_TRUE(buildThrusterConfig(4U, positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.0F, 0.0F, 0.0F});
    alg.setThrusters(config);
    disableDesiredControlAxesAssertion(alg);
    alg.computeThrusterMapping();

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(Eigen::Vector3f::Zero(), {1.0F, 0.0F, 0.0F});
    for (int i = 0; i < MAX_EFF_CNT; ++i) {
        EXPECT_NEAR(out[i], 0.0F, 1e-6F);
    }
}

// A thruster located at the CoM has zero moment arm and contributes only to the force block.
// Uses a two-thruster +Y configuration with CoM at thruster 0: active DG is [[0, -1], [1, 1]]
// (rows tau_z and F_y), and a force_y = 1 command yields the exact analytic output [1, 0].
TEST(ForceTorqueThrForceMappingTest, CoMCoincidesWithThruster) {
    const std::vector<Eigen::Vector3f> positions = {{0.5F, 0.0F, 0.0F}, {-0.5F, 0.0F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {{0.0F, 1.0F, 0.0F}, {0.0F, 1.0F, 0.0F}};

    ThrusterArrayConfig config{};
    ASSERT_TRUE(buildThrusterConfig(2U, positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.5F, 0.0F, 0.0F});
    alg.setThrusters(config);
    disableDesiredControlAxesAssertion(alg);
    alg.computeThrusterMapping();

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(Eigen::Vector3f::Zero(), {0.0F, 1.0F, 0.0F});
    EXPECT_NEAR(out[0], 1.0F, 1e-5F);
    EXPECT_NEAR(out[1], 0.0F, 1e-5F);
    for (int i = 2; i < MAX_EFF_CNT; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0F);
    }
}

// Smoke test at full MAX_EFF_CNT capacity. Positions and directions are arbitrary but
// well-conditioned — asserts no buffer overruns and a finite output over the full 36-thruster array.
TEST(ForceTorqueThrForceMappingTest, MaxThrusterCount) {
    std::vector<Eigen::Vector3f> positions(MAX_EFF_CNT);
    std::vector<Eigen::Vector3f> directions(MAX_EFF_CNT);
    for (int i = 0; i < MAX_EFF_CNT; ++i) {
        const float theta = static_cast<float>(i) * 0.175F;
        positions[static_cast<std::size_t>(i)] = {std::cos(theta), std::sin(theta), 0.1F * static_cast<float>(i % 5)};
        directions[static_cast<std::size_t>(i)] = {-std::sin(theta), std::cos(theta), 0.0F};
    }
    propertyFiniteOutput(static_cast<std::uint32_t>(MAX_EFF_CNT),
                         positions,
                         directions,
                         {0.0F, 0.0F, 0.0F},
                         {0.3F, 0.2F, 0.1F},
                         {1.0F, 0.5F, 0.2F});
    propertyNonNegativeForces(static_cast<std::uint32_t>(MAX_EFF_CNT),
                              positions,
                              directions,
                              {0.0F, 0.0F, 0.0F},
                              {0.3F, 0.2F, 0.1F},
                              {1.0F, 0.5F, 0.2F});
    propertyPaddingIsZero(static_cast<std::uint32_t>(MAX_EFF_CNT),
                          positions,
                          directions,
                          {0.0F, 0.0F, 0.0F},
                          {0.3F, 0.2F, 0.1F},
                          {1.0F, 0.5F, 0.2F});
}

// A direction vector with norm 1 + 9e-4 is inside the 1e-3 acceptance band — must not throw and
// must be normalized on store.
TEST(ForceTorqueThrForceMappingTest, DirectionAtNormToleranceBoundary) {
    ThrusterArrayConfig config{};
    config.numThrusters = 1U;
    config.thrusters[0].rThrust_B = {0.0F, 0.0F, 0.0F};
    config.thrusters[0].tHatThrust_B = {1.0F + 9e-4F, 0.0F, 0.0F};

    ForceTorqueThrForceMappingAlgorithm alg{};
    EXPECT_NO_THROW(alg.setThrusters(config));

    const ThrusterArrayConfig readBack = alg.getThrusters();
    const Eigen::Vector3f dir(readBack.thrusters[0].tHatThrust_B[0],
                              readBack.thrusters[0].tHatThrust_B[1],
                              readBack.thrusters[0].tHatThrust_B[2]);
    EXPECT_NEAR(dir.stableNorm(), 1.0F, 1e-6F);
}

// CubeSat-scale moment arms. With ~5e-3 m moment arms, individual torque-row entries (r×g) are at
// most ~5e-3 — well above the SVD's relative tol (sigma_max * eps * max(m,n) ~ 1e-7 here) but
// below the prior algorithm's hard-coded 1e-4 absolute threshold. Verifies the new tolerance
// preserves controllability for small bodies that the old code would have dropped, and that small
// commanded torques are reproduced rather than zeroed out by the pseudo-inverse.
TEST(ForceTorqueThrForceMappingTest, SmallMomentArmsControllable) {
    constexpr float r = 5e-3F;
    const std::vector<Eigen::Vector3f> positions = {
        {r, 0.0F, 0.0F}, {-r, 0.0F, 0.0F}, {0.0F, r, 0.0F}, {0.0F, -r, 0.0F}, {0.0F, 0.0F, r}, {0.0F, 0.0F, -r}};
    const std::vector<Eigen::Vector3f> directions = {{0.0F, 1.0F, 0.0F},
                                                     {0.0F, -1.0F, 0.0F},
                                                     {0.0F, 0.0F, 1.0F},
                                                     {0.0F, 0.0F, -1.0F},
                                                     {1.0F, 0.0F, 0.0F},
                                                     {-1.0F, 0.0F, 0.0F}};

    ThrusterArrayConfig config{};
    ASSERT_TRUE(buildThrusterConfig(6U, positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.0F, 0.0F, 0.0F});
    alg.setThrusters(config);
    disableDesiredControlAxesAssertion(alg);
    alg.computeThrusterMapping();

    // Small commanded torque, no force — needs the small-moment-arm rows to be retained.
    const Eigen::Vector3f torque{1e-4F, 2e-4F, -1e-4F};
    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(torque, Eigen::Vector3f::Zero());

    // The layout pairs flip both r and g, so each pair's r×g terms add (DG·1 = (2r, 2r, 2r, 0, 0, 0)
    // ≠ 0) and the min-shift introduces an FT offset that's part of the algorithm's documented
    // behavior — see UnbalancedLayoutAchievedFTDiffersFromCommand. We can't compare achieved
    // against commanded directly, but we can compare against the fp64 SVD reference, which
    // applies the same min-shift; agreement confirms fp32 didn't drop the small-arm rows.
    std::vector<Eigen::Vector3f> unitDirs(config.numThrusters);
    for (std::uint32_t i = 0; i < config.numThrusters; ++i) {
        unitDirs[i] = Eigen::Vector3f(config.thrusters.at(i).tHatThrust_B[0],
                                      config.thrusters.at(i).tHatThrust_B[1],
                                      config.thrusters.at(i).tHatThrust_B[2]);
    }
    const Eigen::Vector<float, MAX_EFF_CNT> ref = referenceUpdate(
        config.numThrusters, positions, unitDirs, Eigen::Vector3f::Zero(), torque, Eigen::Vector3f::Zero());

    // At least one active thruster must be non-zero — pinning the small-moment-arm intent: if SVD
    // truncation had dropped the torque rows the entire output would be zero post-min-shift.
    EXPECT_GT(out.head(config.numThrusters).cwiseAbs().maxCoeff(), 1e-6F);

    for (std::uint32_t i = 0; i < config.numThrusters; ++i) {
        const int idx = static_cast<int>(i);
        EXPECT_TRUE(std::isfinite(out[idx]));
        EXPECT_NEAR(out[idx], ref[idx], combinedTolerance(ref[idx], 1e-4F, 1e-4F));
    }
}

// A commanded force along a completely uncontrollable axis (x, for the 8-thruster layout 1 whose
// directions all lie in the y-z plane) is silently dropped by the selector. With no other
// commands, every kept row of ft is zero, so the pseudo-inverse product is zero and the output
// must be exactly zero.
TEST(ForceTorqueThrForceMappingTest, CommandOnUncontrollableAxis) {
    ThrusterArrayConfig config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config));
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.1F, 0.1F, 0.1F});
    alg.setThrusters(config);
    disableDesiredControlAxesAssertion(alg);
    alg.computeThrusterMapping();

    const Eigen::Vector<float, MAX_EFF_CNT> out = alg.update(Eigen::Vector3f::Zero(), {1.0F, 0.0F, 0.0F});
    for (int i = 0; i < MAX_EFF_CNT; ++i) {
        EXPECT_NEAR(out[i], 0.0F, 1e-5F);
    }
}

// ---------------------------------------------------------------------------
// desiredControlAxes_B — per-axis controllability assertion vector. Default is all-true (full
// controllability asserted); each true entry requires that axis to lie in the column space of DG
// when the mapping is computed. Set entries to false to opt out per axis.
// ---------------------------------------------------------------------------

TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesDefaultIsAllTrue) {
    ForceTorqueThrForceMappingAlgorithm alg{};
    const std::array<bool, 6> axes = alg.getDesiredControlAxes();
    for (bool flag : axes) {
        EXPECT_TRUE(flag);
    }
}

TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesRoundTrip) {
    ForceTorqueThrForceMappingAlgorithm alg{};
    const std::array<bool, 6> axes{true, false, true, false, true, false};
    alg.setDesiredControlAxes(axes);
    const std::array<bool, 6> readBack = alg.getDesiredControlAxes();
    for (std::size_t i = 0; i < 6U; ++i) {
        EXPECT_EQ(readBack.at(i), axes.at(i));
    }
}

// 12-thruster layout 2 is full-rank — every axis is controllable, so an all-true assertion must
// pass and the resulting mapping must still match the unconstrained reference.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesAllTrueOnFullRankLayout) {
    ThrusterArrayConfig config{};
    ASSERT_TRUE(buildThrusterConfig(12U, rcsPositions2(), rcsDirections2(), config));
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.1F, 0.1F, 0.1F});
    alg.setThrusters(config);
    alg.setDesiredControlAxes({true, true, true, true, true, true});
    EXPECT_NO_THROW(alg.computeThrusterMapping());
}

// 8-thruster layout 1 has all directions in the y-z plane: torque_xyz and force_yz are controllable
// through moment arms / direction sums, but force_x is not. Asserting controllability on force_x
// must throw; asserting controllability only on the other axes must pass.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesUncontrollableForceXThrows) {
    ThrusterArrayConfig config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config));
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.1F, 0.1F, 0.1F});
    alg.setThrusters(config);

    // force_x (index 3) is the uncontrollable axis.
    alg.setDesiredControlAxes({false, false, false, true, false, false});
    EXPECT_THROW(alg.computeThrusterMapping(), fsw::invalid_argument);

    // Same layout, but only the controllable axes are asserted: must pass.
    alg.setDesiredControlAxes({true, true, true, false, true, true});
    EXPECT_NO_THROW(alg.computeThrusterMapping());
}

// All-false desiredControlAxes_B opts every axis out of the assertion, so computeThrusterMapping()
// must not gate on controllability — useful for callers that don't want any controllability check.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesAllFalseAcceptsUncontrollableLayout) {
    ThrusterArrayConfig config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config));
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.1F, 0.1F, 0.1F});
    alg.setThrusters(config);
    alg.setDesiredControlAxes({false, false, false, false, false, false});
    EXPECT_NO_THROW(alg.computeThrusterMapping());
}

// The all-true default is strict: on a layout with an uncontrollable axis (layout 1's force_x),
// computeThrusterMapping() must throw if the caller hasn't opted out.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesDefaultThrowsOnUncontrollableLayout) {
    ThrusterArrayConfig config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config));
    ForceTorqueThrForceMappingAlgorithm alg{};
    alg.setCoM_B({0.1F, 0.1F, 0.1F});
    alg.setThrusters(config);
    // No setDesiredControlAxes call — default is all-true.
    EXPECT_THROW(alg.computeThrusterMapping(), fsw::invalid_argument);
}
