#include "forceTorqueThrForceMappingTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <array>
#include <limits>

// ---------------------------------------------------------------------------
// Regression tests — mirror the Python test fixtures so the C++ suite pins the
// same numerical behavior against an independent SVD-based reference.
// ---------------------------------------------------------------------------

TEST(ForceTorqueThrForceMappingTest, RegressionUncontrollableXAxis) {
    runRegressionCase(rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionPureForceZeroTorque) {
    runRegressionCase(rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionNoTorqueCommand) {
    // Mirrors the "no torque message connected" Python case: torque defaults to zero at the adapter
    // boundary, so at the algorithm layer it's indistinguishable from an explicit zero command.
    runRegressionCase(rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionAllDirectionsCovered) {
    runRegressionCase(rcsPositions2(), rcsDirections2(), {0.1F, 0.1F, 0.1F}, {0.0F, 0.0F, 0.0F}, {0.9F, 1.1F, 1.0F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionCoMAtOrigin) {
    runRegressionCase(rcsPositions1(), rcsDirections1(), {0.0F, 0.0F, 0.0F}, {0.2F, -0.1F, 0.3F}, {0.0F, 0.5F, -0.4F});
}

TEST(ForceTorqueThrForceMappingTest, RegressionLargeCoMOffset) {
    runRegressionCase(rcsPositions1(), rcsDirections1(), {1.0F, 1.0F, 1.0F}, {0.2F, -0.1F, 0.3F}, {0.0F, 0.5F, -0.4F});
}

// ---------------------------------------------------------------------------
// Config validation — ForceTorqueThrForceMappingConfig::create round-trips the
// stored values through its getters and rejects invalid configurations.
// ---------------------------------------------------------------------------

TEST(ForceTorqueThrForceMappingTest, ConfigValidationAndRoundTrip) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(rcsPositions2(), rcsDirections2(), config));
    const Eigen::Vector3f CoM(0.25F, -0.5F, 1.0F);

    // create() round-trips the stored configuration via the getters (positions and directions are
    // preserved verbatim; normalization happens later when the mapping is computed).
    const ForceTorqueThrForceMappingConfig cfg =
        ForceTorqueThrForceMappingConfig::create(config, CoM, kNoAxisAssertion);
    for (std::uint32_t t = 0; t < kMaxThrusterCount; ++t) {
        for (int i = 0; i < 3; ++i) {
            EXPECT_FLOAT_EQ(cfg.getThrusters().thrusters[t].r_TB_B[i], config.thrusters[t].r_TB_B[i]);
            EXPECT_FLOAT_EQ(cfg.getThrusters().thrusters[t].tHat_B[i], config.thrusters[t].tHat_B[i]);
        }
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(cfg.getCenterOfMass_B()[i], CoM[i], 1e-6F);
    }
    for (std::size_t i = 0; i < 6U; ++i) {
        EXPECT_EQ(cfg.getDesiredControlAxes().at(i), kNoAxisAssertion.at(i));
    }

    EXPECT_NO_THROW(makeMappingAlgorithm(config, CoM, kNoAxisAssertion));

    // Invalid configurations throw fsw::invalid_argument from create().
    ThrusterArrayConfiguration bad = config;
    bad.thrusters[0].tHat_B = {0.5F, 0.0F, 0.0F};  // norm = 0.5, below unit length
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(bad, CoM, kNoAxisAssertion), fsw::invalid_argument);

    bad = config;
    bad.thrusters[0].tHat_B = {1.5F, 0.0F, 0.0F};  // norm = 1.5, above unit length
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(bad, CoM, kNoAxisAssertion), fsw::invalid_argument);

    // A non-finite center of mass is rejected.
    const Eigen::Vector3f badCoM(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(config, badCoM, kNoAxisAssertion), fsw::invalid_argument);

    // Directions within the 1e-3 tolerance band are accepted.
    ThrusterArrayConfiguration nearUnit = config;
    nearUnit.thrusters[0].tHat_B = {0.0F, 1.0F + 5e-4F, 0.0F};
    nearUnit.thrusters[1].tHat_B = {0.0F, 0.0F, -(1.0F - 5e-4F)};
    EXPECT_NO_THROW(ForceTorqueThrForceMappingConfig::create(nearUnit, CoM, kNoAxisAssertion));
}

// ---------------------------------------------------------------------------
// Property tests — fixed representative inputs exercising invariants. The
// same helpers are re-run under fuzz inputs in test_..._fuzz.cpp.
// ---------------------------------------------------------------------------

TEST(ForceTorqueThrForceMappingTest, PropertyNonNegativeForces) {
    propertyNonNegativeForces(
        rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyMinimumIsZero) {
    propertyMinimumIsZero(
        rcsPositions2(), rcsDirections2(), {0.0F, 0.0F, 0.0F}, {0.3F, -0.2F, 0.1F}, {0.9F, 1.1F, 1.0F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyScaleInvariance) {
    propertyScaleInvariance(
        rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F}, 2.5F);
}

TEST(ForceTorqueThrForceMappingTest, PropertyStateless) {
    propertyStateless(rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyFiniteOutput) {
    propertyFiniteOutput(rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyAchievesCommandForBalancedLayout) {
    propertyAchievesCommandForBalancedLayout({0.1F, 0.1F, 0.1F}, {0.5F, 0.3F, 0.1F, 0.7F, 0.2F, 0.4F, 0.6F, 0.8F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyOutputMagnitudeBounded) {
    propertyOutputMagnitudeBounded(
        rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

// Documents a known limitation: for an unbalanced layout (DG·1 ≠ 0), the min-shift step perturbs
// the achieved FT by min_shift·(DG·1) whenever pinv·cmd has a negative entry. The test pins a
// fully-controllable but unbalanced layout — three positions carrying orthogonal thrusters, so
// rank(DG) = 6 (any FT command is reachable in principle) but DG·1 = (−1, −1, −1, 3, 2, 3) is
// nonzero on every axis. Commanding a pure τ_x forces a negative entry in pinv·cmd, which the
// min-shift then translates into an FT offset along DG·1.
//
// This is a behavioral test, not a property test — it freezes the algorithm's documented behavior
// so a future refactor (e.g. a null-space-projected min-shift) would intentionally break it and
// force a deliberate update of the behavior.
TEST(ForceTorqueThrForceMappingTest, UnbalancedLayoutAchievedFTDiffersFromCommand) {
    // Three positions, each carrying thrusters along different axes — gives rank-6 DG with
    // DG·1 = (−1, −1, −1, 3, 2, 3).
    const std::vector<Eigen::Vector3f> positions = {{1.0F, 0.0F, 0.0F},
                                                    {0.0F, 1.0F, 0.0F},
                                                    {0.0F, 0.0F, 1.0F},
                                                    {0.0F, 1.0F, 0.0F},
                                                    {0.0F, 0.0F, 1.0F},
                                                    {1.0F, 0.0F, 0.0F},
                                                    {0.0F, 0.0F, 1.0F},
                                                    {1.0F, 0.0F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {{1.0F, 0.0F, 0.0F},
                                                     {0.0F, 1.0F, 0.0F},
                                                     {0.0F, 0.0F, 1.0F},
                                                     {1.0F, 0.0F, 0.0F},
                                                     {0.0F, 1.0F, 0.0F},
                                                     {0.0F, 0.0F, 1.0F},
                                                     {1.0F, 0.0F, 0.0F},
                                                     {0.0F, 0.0F, 1.0F}};

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, Eigen::Vector3f::Zero());

    // Pure τ_x = 1. pinv·cmd carries a negative entry, forcing min_shift = −1, so
    // achieved = cmd − min_shift·(DG·1) = cmd + DG·1 = (0, −1, −1, 3, 2, 3). Far from the
    // commanded (1, 0, 0, 0, 0, 0) on every component.
    const Eigen::Vector3f cmdTorque{1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f cmdForce = Eigen::Vector3f::Zero();
    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(cmdTorque, cmdForce);

    const Eigen::Matrix<float, 6, kMaxThrusterCount> DG = buildDG(config, Eigen::Vector3f::Zero());
    const Eigen::Vector<float, 6> achieved = DG * out;
    Eigen::Vector<float, 6> cmd;
    cmd << cmdTorque, cmdForce;

    // Achieved differs from cmd by ~DG·1, whose largest component is 3. Assert the discrepancy is
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
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(rcsPositions1(), rcsDirections1(), config)) {
        FAIL() << "buildThrusterConfig failed for rcs1 layout";
    }
    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F});

    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    for (int i = 0; i < kMaxThrusterCount; ++i) {
        EXPECT_NEAR(out[i], 0.0F, 1e-6F);
    }
}

// All thrusters parallel: only force_x and torque_z rows of DG are nonzero (selector drops the
// other four). With 8 symmetric +X thrusters about origin and a pure force_x = 1 command, the
// min-norm LS solution spreads the thrust evenly; the min-shift then zeros every output.
// Verifies the algorithm handles rank-deficient DG without NaN or crash and matches the
// analytic solution.
TEST(ForceTorqueThrForceMappingTest, AllThrustersParallel) {
    const std::vector<Eigen::Vector3f> positions = {{0.5F, 0.0F, 0.0F},
                                                    {-0.5F, 0.0F, 0.0F},
                                                    {0.0F, 0.5F, 0.0F},
                                                    {0.0F, -0.5F, 0.0F},
                                                    {0.5F, 0.0F, 0.0F},
                                                    {-0.5F, 0.0F, 0.0F},
                                                    {0.0F, 0.5F, 0.0F},
                                                    {0.0F, -0.5F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions(kMaxThrusterCount, Eigen::Vector3f{1.0F, 0.0F, 0.0F});

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, Eigen::Vector3f::Zero());

    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), {1.0F, 0.0F, 0.0F});
    for (int i = 0; i < kMaxThrusterCount; ++i) {
        EXPECT_NEAR(out[i], 0.0F, 1e-6F);
    }
}

// A thruster located at the CoM has zero moment arm and contributes only to the force block.
// Eight +Y thrusters alternate between x = +0.5 and x = -0.5 with the CoM at x = +0.5, so the four
// even-indexed thrusters sit on the CoM. A force_y = 1 command is met by those four alone, 0.25
// each, leaving the off-CoM thrusters at zero so no torque is produced.
TEST(ForceTorqueThrForceMappingTest, CoMCoincidesWithThruster) {
    const std::vector<Eigen::Vector3f> positions = {{0.5F, 0.0F, 0.0F},
                                                    {-0.5F, 0.0F, 0.0F},
                                                    {0.5F, 0.0F, 0.0F},
                                                    {-0.5F, 0.0F, 0.0F},
                                                    {0.5F, 0.0F, 0.0F},
                                                    {-0.5F, 0.0F, 0.0F},
                                                    {0.5F, 0.0F, 0.0F},
                                                    {-0.5F, 0.0F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions(kMaxThrusterCount, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, {0.5F, 0.0F, 0.0F});

    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), {0.0F, 1.0F, 0.0F});
    for (int i = 0; i < kMaxThrusterCount; i += 2) {
        EXPECT_NEAR(out[i], 0.25F, 1e-5F);
        EXPECT_NEAR(out[i + 1], 0.0F, 1e-5F);
    }
}

// A direction vector with norm 1 + 9e-4 is inside the 1e-3 acceptance band — Config::create must
// accept it (it is normalized internally when the mapping is computed).
TEST(ForceTorqueThrForceMappingTest, DirectionAtNormToleranceBoundary) {
    ThrusterArrayConfiguration config{};
    for (std::uint32_t i = 0; i < kMaxThrusterCount; ++i) {
        config.thrusters.at(i).r_TB_B = {0.5F * (i % 2 == 0 ? 1.0F : -1.0F), 0.0F, 0.0F};
        config.thrusters.at(i).tHat_B = {1.0F + 9e-4F, 0.0F, 0.0F};
    }

    EXPECT_NO_THROW(ForceTorqueThrForceMappingConfig::create(config, Eigen::Vector3f::Zero(), kNoAxisAssertion));
}

// A commanded force along a completely uncontrollable axis (x, for the 8-thruster layout 1 whose
// directions all lie in the y-z plane) is silently dropped by the selector. With no other
// commands, every kept row of ft is zero, so the pseudo-inverse product is zero and the output
// must be exactly zero.
TEST(ForceTorqueThrForceMappingTest, CommandOnUncontrollableAxis) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(rcsPositions1(), rcsDirections1(), config));
    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F});

    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), {1.0F, 0.0F, 0.0F});
    for (int i = 0; i < kMaxThrusterCount; ++i) {
        EXPECT_NEAR(out[i], 0.0F, 1e-5F);
    }
}

// ---------------------------------------------------------------------------
// desiredControlAxes_B — per-axis controllability assertion vector carried by the configuration.
// Each true entry requires that axis to lie in the column space of DG when the mapping is computed
// (i.e. at construction); set entries to false to opt out per axis.
// ---------------------------------------------------------------------------

// 8-thruster layout 2 is full-rank — every axis is controllable, so an all-true assertion must
// construct without throwing.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesAllTrueOnFullRankLayout) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(rcsPositions2(), rcsDirections2(), config));
    EXPECT_NO_THROW(makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F}, {true, true, true, true, true, true}));
}

// 8-thruster layout 1 has all directions in the y-z plane: torque_xyz and force_yz are controllable
// through moment arms / direction sums, but force_x is not. Asserting controllability on force_x
// must throw at construction; asserting controllability only on the other axes must succeed.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesUncontrollableForceXThrows) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(rcsPositions1(), rcsDirections1(), config));
    const Eigen::Vector3f CoM{0.1F, 0.1F, 0.1F};

    // force_x (index 3) is the uncontrollable axis.
    EXPECT_THROW(makeMappingAlgorithm(config, CoM, {false, false, false, true, false, false}), fsw::invalid_argument);

    // Same layout, but only the controllable axes are asserted: must succeed.
    EXPECT_NO_THROW(makeMappingAlgorithm(config, CoM, {true, true, true, false, true, true}));
}

// All-false desiredControlAxes_B opts every axis out of the assertion, so construction must not gate
// on controllability — useful for callers that don't want any controllability check.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesAllFalseAcceptsUncontrollableLayout) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(rcsPositions1(), rcsDirections1(), config));
    EXPECT_NO_THROW(makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F}, kNoAxisAssertion));
}

// The all-true assertion is strict: on a layout with an uncontrollable axis (layout 1's force_x),
// construction must throw.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesAllTrueThrowsOnUncontrollableLayout) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(rcsPositions1(), rcsDirections1(), config));
    EXPECT_THROW(makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F}, {true, true, true, true, true, true}),
                 fsw::invalid_argument);
}

// An ill-conditioned (but full-rank) layout is rejected by create() even with no controllability assertion.
// Thrusters with 5 mm moment arms span all six axes, but the torque rows are ~5e-3 while the force rows
// are unit, so cond(DG) ~ 200 (> 100). create() must reject it.
TEST(ForceTorqueThrForceMappingTest, IllConditionedLayoutRejected) {
    constexpr float r = 5e-3F;
    const std::vector<Eigen::Vector3f> positions = {{r, 0.0F, 0.0F},
                                                    {-r, 0.0F, 0.0F},
                                                    {0.0F, r, 0.0F},
                                                    {0.0F, -r, 0.0F},
                                                    {0.0F, 0.0F, r},
                                                    {0.0F, 0.0F, -r},
                                                    {r, 0.0F, 0.0F},
                                                    {0.0F, r, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {{0.0F, 1.0F, 0.0F},
                                                     {0.0F, -1.0F, 0.0F},
                                                     {0.0F, 0.0F, 1.0F},
                                                     {0.0F, 0.0F, -1.0F},
                                                     {1.0F, 0.0F, 0.0F},
                                                     {-1.0F, 0.0F, 0.0F},
                                                     {0.0F, 1.0F, 0.0F},
                                                     {0.0F, 0.0F, 1.0F}};

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(positions, directions, config));
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(config, Eigen::Vector3f::Zero(), kNoAxisAssertion),
                 fsw::invalid_argument);
}
