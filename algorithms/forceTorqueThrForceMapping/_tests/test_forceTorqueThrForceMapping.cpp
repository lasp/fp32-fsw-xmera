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
        8U, rcsPositions2(), rcsDirections2(), {0.1F, 0.1F, 0.1F}, {0.0F, 0.0F, 0.0F}, {0.9F, 1.1F, 1.0F});
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
// Config validation — ForceTorqueThrForceMappingConfig::create round-trips the
// stored values through its getters and rejects invalid configurations.
// ---------------------------------------------------------------------------

TEST(ForceTorqueThrForceMappingTest, ConfigValidationAndRoundTrip) {
    ThrusterArrayConfiguration config{};
    config.numThrusters = 2U;
    config.thrusters[0].r_TB_B = {0.0F, 0.5F, 0.0F};
    config.thrusters[0].tHat_B = {1.0F, 0.0F, 0.0F};
    config.thrusters[1].r_TB_B = {0.0F, -0.5F, 0.0F};
    config.thrusters[1].tHat_B = {1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f CoM(0.25F, -0.5F, 1.0F);
    // The opposed +x pair controls torque_z about this CoM and nothing else on its own.
    constexpr std::array<bool, 6> kTorqueZOnly{false, false, true, false, false, false};

    // create() round-trips the stored configuration via the getters (positions and directions are
    // preserved verbatim; normalization happens later when the mapping is computed).
    const ForceTorqueThrForceMappingConfig cfg = ForceTorqueThrForceMappingConfig::create(config, CoM, kTorqueZOnly);
    EXPECT_EQ(cfg.getThrusters().numThrusters, 2U);
    for (std::uint32_t t = 0; t < 2U; ++t) {
        for (int i = 0; i < 3; ++i) {
            EXPECT_FLOAT_EQ(cfg.getThrusters().thrusters[t].r_TB_B[i], config.thrusters[t].r_TB_B[i]);
            EXPECT_FLOAT_EQ(cfg.getThrusters().thrusters[t].tHat_B[i], config.thrusters[t].tHat_B[i]);
        }
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(cfg.getCenterOfMass_B()[i], CoM[i], 1e-6F);
    }
    for (std::size_t i = 0; i < 6U; ++i) {
        EXPECT_EQ(cfg.getDesiredControlAxes().at(i), kTorqueZOnly.at(i));
    }

    // Constructing the algorithm from a valid config succeeds.
    EXPECT_NO_THROW(makeMappingAlgorithm(config, CoM, kTorqueZOnly));

    // Invalid configurations throw fsw::invalid_argument from create().
    ThrusterArrayConfiguration bad = config;
    bad.numThrusters = 0U;
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(bad, CoM, kTorqueZOnly), fsw::invalid_argument);

    bad = config;
    bad.numThrusters = kMaxThrusterCount + 1U;
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(bad, CoM, kTorqueZOnly), fsw::invalid_argument);

    bad = config;
    bad.thrusters[0].tHat_B = {0.5F, 0.0F, 0.0F};  // norm = 0.5, below unit length
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(bad, CoM, kTorqueZOnly), fsw::invalid_argument);

    bad = config;
    bad.thrusters[0].tHat_B = {1.5F, 0.0F, 0.0F};  // norm = 1.5, above unit length
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(bad, CoM, kTorqueZOnly), fsw::invalid_argument);

    // A non-finite center of mass is rejected.
    const Eigen::Vector3f badCoM(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(config, badCoM, kTorqueZOnly), fsw::invalid_argument);

    // Directions within the 1e-3 tolerance band are accepted.
    ThrusterArrayConfiguration nearUnit = config;
    nearUnit.thrusters[0].tHat_B = {1.0F + 5e-4F, 0.0F, 0.0F};
    nearUnit.thrusters[1].tHat_B = {1.0F - 5e-4F, 0.0F, 0.0F};
    EXPECT_NO_THROW(ForceTorqueThrForceMappingConfig::create(nearUnit, CoM, kTorqueZOnly));
}

// ---------------------------------------------------------------------------
// Property tests — fixed representative inputs exercising invariants. The
// same helpers are re-run under fuzz inputs in test_..._fuzz.cpp.
// ---------------------------------------------------------------------------

TEST(ForceTorqueThrForceMappingTest, PropertyNonNegativeForces) {
    propertyNonNegativeForces(
        8U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

TEST(ForceTorqueThrForceMappingTest, PropertyMinimumIsZeroForBalancedLayout) {
    propertyMinimumIsZeroForBalancedLayout({0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
}

// Six of layout 1's eight thrusters, so two slots sit past numThrusters and the padding assertion has
// something to check. At the full eight the loop over [numThrusters, kMaxThrusterCount) is empty, and
// the property passes without testing anything; the boundary case is covered separately below.
TEST(ForceTorqueThrForceMappingTest, PropertyPaddingIsZero) {
    propertyPaddingIsZero(
        6U, rcsPositions1(), rcsDirections1(), {0.1F, 0.1F, 0.1F}, {0.4F, 0.2F, 0.4F}, {0.0F, 0.9F, 1.1F});
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

// The one remaining way the achieved force and torque can leave the command: the clamp. This layout is
// rank 6 with 6 thrusters, so there is no null space to shift along and a pure tau_x command keeps a
// negative entry that only the clamp can remove. Behavioral, not a property -- it freezes the
// documented limitation so a change to the non-negativity handling has to update it deliberately.
TEST(ForceTorqueThrForceMappingTest, ClampedSolutionAchievedFTDiffersFromCommand) {
    // Three positions, each carrying two orthogonal thrusters. Pairs (thr 0, 5), (thr 1, 3),
    // (thr 2, 4) share a position but point along different axes -- gives rank-6 DG.
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

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(6U, positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, Eigen::Vector3f::Zero(), kAllControlAxes);

    // Minimum-norm solution (0, 1, 0, 0, -1, 0); the -1 clamps away, leaving a pure F_y.
    const Eigen::Vector3f cmdTorque{1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f cmdForce = Eigen::Vector3f::Zero();
    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(cmdTorque, cmdForce);

    for (int i = 0; i < 6; ++i) {
        EXPECT_GE(out[i], 0.0F);
    }

    const Eigen::Matrix<float, 6, kMaxThrusterCount> DG = buildDG(config, Eigen::Vector3f::Zero());
    const Eigen::Vector<float, 6> achieved = DG * out;
    Eigen::Vector<float, 6> cmd;
    cmd << cmdTorque, cmdForce;

    const Eigen::Vector<float, 6> diff = cmd - achieved;
    EXPECT_GT(diff.cwiseAbs().maxCoeff(), 0.5F);
}

// Two parallel +z thrusters split a commanded +z force evenly, at any center of mass: there is no
// shift direction to cancel the pair, and the parasitic torque falls on an unselected axis.
TEST(ForceTorqueThrForceMappingTest, DvPairSplitsCommandedForceEvenly) {
    const std::vector<Eigen::Vector3f> positions = {{-1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {{0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 1.0F}};
    constexpr std::array<bool, 6> kForceZOnly{false, false, false, false, false, true};

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(2U, positions, directions, config));

    for (const Eigen::Vector3f& CoM :
         {Eigen::Vector3f{0.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 0.0F, 0.5F}, Eigen::Vector3f{0.2F, -0.1F, 0.5F}}) {
        const ForceTorqueThrForceMappingAlgorithm alg = makeMappingAlgorithm(config, CoM, kForceZOnly);
        const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), {0.0F, 0.0F, 10.0F});

        EXPECT_NEAR(out[0], 5.0F, 1e-4F);
        EXPECT_NEAR(out[1], 5.0F, 1e-4F);
        for (int i = 2; i < kMaxThrusterCount; ++i) {
            EXPECT_FLOAT_EQ(out[i], 0.0F);
        }

        const Eigen::Matrix<float, 6, kMaxThrusterCount> DG = buildDG(config, CoM);
        EXPECT_NEAR((DG * out)[5], 10.0F, 1e-3F);
    }
}

// A retro request on a +z-only pair has no non-negative solution, so the clamp takes the output to
// zero rather than commanding a pull.
TEST(ForceTorqueThrForceMappingTest, DvPairRejectsRetroThrust) {
    const std::vector<Eigen::Vector3f> positions = {{-1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {{0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 1.0F}};

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(2U, positions, directions, config));
    const ForceTorqueThrForceMappingAlgorithm alg =
        makeMappingAlgorithm(config, {0.2F, -0.1F, 0.5F}, {false, false, false, false, false, true});

    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), {0.0F, 0.0F, -10.0F});
    for (int i = 0; i < kMaxThrusterCount; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0F);
    }
}

// ---------------------------------------------------------------------------
// Edge-case tests — boundary conditions and degenerate geometries.
// ---------------------------------------------------------------------------

// Zero commanded torque and force produces the zero-thrust solution. The raw pseudo-inverse of
// [0;0] is zero, and min-shift of zero is zero.
TEST(ForceTorqueThrForceMappingTest, ZeroCommandProducesZeroOutput) {
    ThrusterArrayConfiguration config{};
    if (!buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config)) {
        FAIL() << "buildThrusterConfig failed for rcs1 layout";
    }
    ForceTorqueThrForceMappingAlgorithm alg =
        makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F}, {true, true, true, false, true, true});

    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    for (int i = 0; i < kMaxThrusterCount; ++i) {
        EXPECT_NEAR(out[i], 0.0F, 1e-6F);
    }
}

// All thrusters parallel: only the force_x and torque_z rows of DG are nonzero, and those are the
// selected axes. A pure force_x = 1 command gives x = [0.25, 0.25, 0.25, 0.25]; there is no shift
// direction here, so the common thrust survives and the array delivers the commanded 1 N.
TEST(ForceTorqueThrForceMappingTest, AllThrustersParallel) {
    const std::vector<Eigen::Vector3f> positions = {
        {0.5F, 0.0F, 0.0F}, {-0.5F, 0.0F, 0.0F}, {0.0F, 0.5F, 0.0F}, {0.0F, -0.5F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {
        {1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}};

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(4U, positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg =
        makeMappingAlgorithm(config, Eigen::Vector3f::Zero(), {false, false, true, true, false, false});

    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), {1.0F, 0.0F, 0.0F});
    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(out[i], 0.25F, 1e-6F);
    }
    for (int i = 4; i < kMaxThrusterCount; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0F);
    }
}

// A thruster located at the CoM has zero moment arm and contributes only to the force block.
// Uses a two-thruster +Y configuration with CoM at thruster 0: active DG is [[0, -1], [1, 1]]
// (rows tau_z and F_y), and a force_y = 1 command yields the exact analytic output [1, 0].
TEST(ForceTorqueThrForceMappingTest, CoMCoincidesWithThruster) {
    const std::vector<Eigen::Vector3f> positions = {{0.5F, 0.0F, 0.0F}, {-0.5F, 0.0F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {{0.0F, 1.0F, 0.0F}, {0.0F, 1.0F, 0.0F}};

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(2U, positions, directions, config));
    ForceTorqueThrForceMappingAlgorithm alg =
        makeMappingAlgorithm(config, {0.5F, 0.0F, 0.0F}, {false, false, true, false, true, false});

    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), {0.0F, 1.0F, 0.0F});
    EXPECT_NEAR(out[0], 1.0F, 1e-5F);
    EXPECT_NEAR(out[1], 0.0F, 1e-5F);
    for (int i = 2; i < kMaxThrusterCount; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0F);
    }
}

// Smoke test at full kMaxThrusterCount capacity. Positions and directions are arbitrary but
// well-conditioned — asserts no buffer overruns and a finite output over the full 36-thruster array.
TEST(ForceTorqueThrForceMappingTest, MaxThrusterCount) {
    std::vector<Eigen::Vector3f> positions(kMaxThrusterCount);
    std::vector<Eigen::Vector3f> directions(kMaxThrusterCount);
    for (int i = 0; i < kMaxThrusterCount; ++i) {
        const float theta = static_cast<float>(i) * 0.175F;
        positions[static_cast<std::size_t>(i)] = {std::cos(theta), std::sin(theta), 0.1F * static_cast<float>(i % 5)};
        directions[static_cast<std::size_t>(i)] = {-std::sin(theta), std::cos(theta), 0.0F};
    }
    propertyFiniteOutput(static_cast<std::uint32_t>(kMaxThrusterCount),
                         positions,
                         directions,
                         {0.0F, 0.0F, 0.0F},
                         {0.3F, 0.2F, 0.1F},
                         {1.0F, 0.5F, 0.2F});
    propertyNonNegativeForces(static_cast<std::uint32_t>(kMaxThrusterCount),
                              positions,
                              directions,
                              {0.0F, 0.0F, 0.0F},
                              {0.3F, 0.2F, 0.1F},
                              {1.0F, 0.5F, 0.2F});
    propertyPaddingIsZero(static_cast<std::uint32_t>(kMaxThrusterCount),
                          positions,
                          directions,
                          {0.0F, 0.0F, 0.0F},
                          {0.3F, 0.2F, 0.1F},
                          {1.0F, 0.5F, 0.2F});
}

// A direction vector with norm 1 + 9e-4 is inside the 1e-3 acceptance band — Config::create must
// accept it (it is normalized internally when the mapping is computed).
TEST(ForceTorqueThrForceMappingTest, DirectionAtNormToleranceBoundary) {
    ThrusterArrayConfiguration config{};
    config.numThrusters = 1U;
    config.thrusters[0].r_TB_B = {0.0F, 0.0F, 0.0F};
    config.thrusters[0].tHat_B = {1.0F + 9e-4F, 0.0F, 0.0F};

    EXPECT_NO_THROW(ForceTorqueThrForceMappingConfig::create(
        config, Eigen::Vector3f::Zero(), {false, false, false, true, false, false}));
}

// A commanded force along a completely uncontrollable axis (x, for the 8-thruster layout 1 whose
// directions all lie in the y-z plane) is silently dropped by the selector. With no other
// commands, every kept row of ft is zero, so the pseudo-inverse product is zero and the output
// must be exactly zero.
TEST(ForceTorqueThrForceMappingTest, CommandOnUncontrollableAxis) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config));
    ForceTorqueThrForceMappingAlgorithm alg =
        makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F}, {true, true, true, false, true, true});

    const Eigen::Vector<float, kMaxThrusterCount> out = alg.update(Eigen::Vector3f::Zero(), {1.0F, 0.0F, 0.0F});
    for (int i = 0; i < kMaxThrusterCount; ++i) {
        EXPECT_NEAR(out[i], 0.0F, 1e-5F);
    }
}

// ---------------------------------------------------------------------------
// desiredControlAxes_B — the axis selection carried by the configuration. Only the selected rows of
// DG enter the solve, and each selected axis must lie in the column space of DG when the mapping is
// computed (i.e. at construction). The selection must contain a minimum of one axis.
// ---------------------------------------------------------------------------

// 8-thruster layout 2 is full-rank — every axis is controllable, so selecting all six must
// construct without throwing.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesAllTrueOnFullRankLayout) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions2(), rcsDirections2(), config));
    EXPECT_NO_THROW(makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F}, {true, true, true, true, true, true}));
}

// 8-thruster layout 1 has all directions in the y-z plane: torque_xyz and force_yz are controllable
// through moment arms / direction sums, but force_x is not. Selecting force_x must throw at
// construction; selecting only the controllable axes must succeed.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesUncontrollableForceXThrows) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config));
    const Eigen::Vector3f CoM{0.1F, 0.1F, 0.1F};

    // force_x (index 3) is the uncontrollable axis.
    EXPECT_THROW(makeMappingAlgorithm(config, CoM, {false, false, false, true, false, false}), fsw::invalid_argument);

    // Same layout, but only the controllable axes are selected: must succeed.
    EXPECT_NO_THROW(makeMappingAlgorithm(config, CoM, {true, true, true, false, true, true}));
}

// An all-false selection names no axis for the mapping to control, which would command zero thrust for
// every input. create() rejects it rather than returning a mapping that ignores every request.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesAllFalseRejected) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions2(), rcsDirections2(), config));
    constexpr std::array<bool, 6> kNoAxisSelected{false, false, false, false, false, false};
    EXPECT_THROW(makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F}, kNoAxisSelected), fsw::invalid_argument);
}

// An unselected axis places no condition on the solve. Layout 1 is balanced, so the min-shift preserves
// the achieved force and torque and the comparison below is exact. Under a torque-only selection the
// commanded torque is met and the resulting body force is left wherever the minimum-norm solution puts
// it. Adding force_yz to the selection holds that same force at the commanded zero, with the torque
// still met -- so the unselected axes really are free rather than quietly driven to zero.
TEST(ForceTorqueThrForceMappingTest, UnselectedAxisIsUnconstrained) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config));
    const Eigen::Vector3f CoM{0.1F, 0.1F, 0.1F};
    const Eigen::Vector3f cmdTorque{0.4F, 0.2F, 0.4F};
    const Eigen::Vector3f cmdForce{0.0F, 0.0F, 0.0F};
    const Eigen::Matrix<float, 6, kMaxThrusterCount> DG = buildDG(config, CoM);

    const Eigen::Vector<float, 6> torqueOnly =
        DG * makeMappingAlgorithm(config, CoM, {true, true, true, false, false, false}).update(cmdTorque, cmdForce);
    const Eigen::Vector<float, 6> torqueAndForce =
        DG * makeMappingAlgorithm(config, CoM, {true, true, true, false, true, true}).update(cmdTorque, cmdForce);

    // Both selections meet the commanded torque.
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR(torqueOnly[axis], cmdTorque[axis], 1e-5F);
        EXPECT_NEAR(torqueAndForce[axis], cmdTorque[axis], 1e-5F);
    }

    // Unselected, force_yz drifts away from the commanded zero; selected, it is held there.
    EXPECT_GT(torqueOnly.tail<2>().cwiseAbs().maxCoeff(), 1e-2F);
    EXPECT_LT(torqueAndForce.tail<2>().cwiseAbs().maxCoeff(), 1e-5F);
}

// A layout that cannot reach a selected axis is rejected: selecting all six on layout 1 must throw,
// because its force_x row is zero.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesAllTrueThrowsOnUncontrollableLayout) {
    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(8U, rcsPositions1(), rcsDirections1(), config));
    EXPECT_THROW(makeMappingAlgorithm(config, {0.1F, 0.1F, 0.1F}, kAllControlAxes), fsw::invalid_argument);
}

// The DV case: two parallel +z thrusters mounted symmetrically about the body z axis. They control
// force_z and nothing else, and a center of mass off the thruster-pair symmetry line makes a
// torque-free +z force unreachable. Selecting force_z alone must therefore configure at any center of
// mass — the torque rows the array cannot null are left out of the solve.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesForceZOnlyOnDvPair) {
    const std::vector<Eigen::Vector3f> positions = {{-1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}};
    const std::vector<Eigen::Vector3f> directions = {{0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 1.0F}};
    constexpr std::array<bool, 6> kForceZOnly{false, false, false, false, false, true};

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(2U, positions, directions, config));

    EXPECT_NO_THROW(makeMappingAlgorithm(config, Eigen::Vector3f::Zero(), kForceZOnly));
    EXPECT_NO_THROW(makeMappingAlgorithm(config, {0.0F, 0.0F, 0.5F}, kForceZOnly));
    EXPECT_NO_THROW(makeMappingAlgorithm(config, {0.2F, -0.1F, 0.5F}, kForceZOnly));

    // Selecting a torque axis the pair cannot reach is still rejected.
    EXPECT_THROW(makeMappingAlgorithm(config, {0.2F, -0.1F, 0.5F}, kAllControlAxes), fsw::invalid_argument);

    // Both thrust directions are parallel to the body z axis, so every moment arm crosses into the
    // body x-y plane and the torque_z row of DG is identically zero. The array has no authority about
    // torque_z at any center of mass, and selecting it must throw.
    constexpr std::array<bool, 6> kTorqueZOnly{false, false, true, false, false, false};
    EXPECT_THROW(makeMappingAlgorithm(config, Eigen::Vector3f::Zero(), kTorqueZOnly), fsw::invalid_argument);
    EXPECT_THROW(makeMappingAlgorithm(config, {0.2F, -0.1F, 0.5F}, kTorqueZOnly), fsw::invalid_argument);
}

// An axis is controllable only when it can be commanded independently of the other selected axes.
// Two parallel thrusters offset along one line produce torque along a single body direction: the
// torque_y and torque_z rows of DG are both nonzero, but they are proportional, so neither axis can
// be commanded while the other is held at zero. Selecting both must throw even though no row is zero.
TEST(ForceTorqueThrForceMappingTest, DesiredControlAxesCoupledTorqueAxesRejected) {
    const std::vector<Eigen::Vector3f> positions = {{0.0F, 0.5F, 0.3F}, {0.0F, -0.5F, -0.3F}};
    const std::vector<Eigen::Vector3f> directions = {{1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}};

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(2U, positions, directions, config));

    EXPECT_THROW(makeMappingAlgorithm(config, Eigen::Vector3f::Zero(), {false, true, true, false, false, false}),
                 fsw::invalid_argument);

    // Either torque axis on its own lies in the column space, so it stays controllable.
    EXPECT_NO_THROW(makeMappingAlgorithm(config, Eigen::Vector3f::Zero(), {false, true, false, false, false, false}));
    EXPECT_NO_THROW(makeMappingAlgorithm(config, Eigen::Vector3f::Zero(), {false, false, true, false, false, false}));
}

// An ill-conditioned (but full-rank) layout is rejected by create() even with no controllability assertion.
// Six thrusters with 5 mm moment arms span all six axes, but the torque rows are ~5e-3 while the force rows
// are unit, so cond(DG) ~ 200 (> 100). create() must reject it.
TEST(ForceTorqueThrForceMappingTest, IllConditionedLayoutRejected) {
    constexpr float r = 5e-3F;
    const std::vector<Eigen::Vector3f> positions = {
        {r, 0.0F, 0.0F}, {-r, 0.0F, 0.0F}, {0.0F, r, 0.0F}, {0.0F, -r, 0.0F}, {0.0F, 0.0F, r}, {0.0F, 0.0F, -r}};
    const std::vector<Eigen::Vector3f> directions = {{0.0F, 1.0F, 0.0F},
                                                     {0.0F, -1.0F, 0.0F},
                                                     {0.0F, 0.0F, 1.0F},
                                                     {0.0F, 0.0F, -1.0F},
                                                     {1.0F, 0.0F, 0.0F},
                                                     {-1.0F, 0.0F, 0.0F}};

    ThrusterArrayConfiguration config{};
    ASSERT_TRUE(buildThrusterConfig(6U, positions, directions, config));
    EXPECT_THROW(ForceTorqueThrForceMappingConfig::create(config, Eigen::Vector3f::Zero(), kAllControlAxes),
                 fsw::invalid_argument);
}
