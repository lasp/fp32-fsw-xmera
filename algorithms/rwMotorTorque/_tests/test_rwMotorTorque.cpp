#include "rwMotorTorqueTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Regression tests — pin update() against the independent fp32 reference.
// ---------------------------------------------------------------------------

// Control torque only (zero gain), four wheels, one unavailable, two summed command torques.
TEST(RwMotorTorqueTest, RegressionControlOnly) {
    runRegressionCase(Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                      Eigen::Vector3f{0.2F, -0.4F, 0.7F},
                      std::vector<bool>{false, false, true, false},
                      true,
                      true,
                      4,
                      std::vector<float>{0.4F, 0.1F, -0.3F, 1.2F, 0.4F, 0.1F, -0.3F, 1.2F, 0.4F, 0.1F, -0.3F, 1.2F},
                      3,
                      std::vector<float>{},
                      std::vector<float>{},
                      0.0F);
}

// Control torque plus an active null-space term (four wheels, non-zero gain and speeds).
TEST(RwMotorTorqueTest, RegressionNullSpace) {
    runRegressionCase(Eigen::Vector3f{0.3F, -0.5F, 0.8F},
                      Eigen::Vector3f::Zero(),
                      std::vector<bool>{},
                      false,
                      false,
                      4,
                      std::vector<float>{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F},
                      3,
                      std::vector<float>{100.0F, -50.0F, 30.0F, 80.0F},
                      std::vector<float>{10.0F, 10.0F, 10.0F, 10.0F},
                      0.5F);
}

// ---------------------------------------------------------------------------
// Setup test — RwMotorTorqueConfig validation / exception paths.
// ---------------------------------------------------------------------------

TEST(RwMotorTorqueTest, SetupTest) {
    const RwMotorTorqueArrayConfiguration rwConfiguration{};

    // No control axis selected is rejected.
    EXPECT_THROW(RwMotorTorqueConfig::create({false, false, false}, rwConfiguration), fsw::invalid_argument);

    // A non-unit RW spin axis is rejected.
    RwMotorTorqueArrayConfiguration nonUnitRw{};
    nonUnitRw.numRW = 1U;
    nonUnitRw.GsMatrix_B.col(0) = Eigen::Vector3f{2.0F, 0.0F, 0.0F};
    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(1U), nonUnitRw), fsw::invalid_argument);

    // A negative null-space feedback gain is rejected.
    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(1U), rwConfiguration, -1.0F), fsw::invalid_argument);

    // Control mapping not full rank (3 control axes but no reaction wheels): create() validates the mapping
    // and rejects the rank-deficient configuration.
    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration), fsw::invalid_argument);
}

// ---------------------------------------------------------------------------
// Property tests — fixed representative inputs exercising invariants. The same
// helpers are re-run under fuzz inputs in test_rwMotorTorque_fuzz.cpp.
// ---------------------------------------------------------------------------

TEST(RwMotorTorqueTest, PropertyOutputIsFinite) {
    propertyOutputIsFinite(Eigen::Vector3f{0.3F, -0.5F, 0.8F},
                           Eigen::Vector3f::Zero(),
                           std::vector<bool>{},
                           false,
                           false,
                           4,
                           std::vector<float>{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F},
                           3,
                           std::vector<float>{100.0F, -50.0F, 30.0F, 80.0F},
                           std::vector<float>{},
                           0.5F);
}

TEST(RwMotorTorqueTest, PropertyExcludedWheelsZeroTorque) {
    propertyExcludedWheelsZeroTorque(
        Eigen::Vector3f{0.3F, -0.5F, 0.8F},
        Eigen::Vector3f::Zero(),
        std::vector<bool>{false, false, true, false, false},
        false,
        true,
        5,
        std::vector<float>{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, -1.0F, 0.5F},
        3,
        std::vector<float>{100.0F, -50.0F, 30.0F, 80.0F, -20.0F},
        std::vector<float>{},
        0.5F);
}

TEST(RwMotorTorqueTest, PropertyNullSpaceAddsNoBodyTorque) {
    propertyNullSpaceAddsNoBodyTorque(
        std::vector<bool>{},
        false,
        4,
        std::vector<float>{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F},
        3,
        std::vector<float>{100.0F, -50.0F, 30.0F, 80.0F},
        std::vector<float>{10.0F, 10.0F, 10.0F, 10.0F},
        0.5F);
}

TEST(RwMotorTorqueTest, PropertyZeroGainDisablesNullSpace) {
    propertyZeroGainDisablesNullSpace(
        Eigen::Vector3f{0.3F, -0.5F, 0.8F},
        Eigen::Vector3f::Zero(),
        std::vector<bool>{},
        false,
        false,
        4,
        std::vector<float>{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F},
        3,
        std::vector<float>{100.0F, -50.0F, 30.0F, 80.0F},
        std::vector<float>{});
}

TEST(RwMotorTorqueTest, PropertyControlTorqueRealized) {
    propertyControlTorqueRealized(
        Eigen::Vector3f{0.3F, -0.5F, 0.8F},
        Eigen::Vector3f::Zero(),
        std::vector<bool>{false, false, true, false},
        false,
        true,
        4,
        std::vector<float>{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F},
        3);
}

// ---------------------------------------------------------------------------
// Edge-case tests — boundary geometries and config canonicalization.
// ---------------------------------------------------------------------------

// Three spanning wheels leave no null space, so the null-space term is zero even with a non-zero gain.
TEST(RwMotorTorqueTest, ThreeSpanningWheelsHaveNoNullSpace) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    const RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration, 0.5F)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<3>() = Eigen::Vector3f{100.0F, -50.0F, 30.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> withSpeeds = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_FLOAT_EQ(withSpeeds[i], controlOnly[i]);
    }
}

// A near-degenerate control geometry (two wheels nearly aligned, so the second control axis is barely
// reachable) is rejected by create(): cond([CGs]) exceeds the limit even though the mapping is full rank.
TEST(RwMotorTorqueTest, IllConditionedControlMappingRejected) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 2U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{1.0F, 5e-3F, 0.0F}.normalized();  // ~0.3 deg off wheel 0

    // Control body x and y: y is reachable only through the tiny y-component of the near-parallel wheels.
    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(2U), rwConfiguration), fsw::invalid_argument);
}

// Four near-coplanar wheels: the control mapping (body x, y) is well-conditioned, but the null-space
// geometry is ill-conditioned (cond([Gs]) > 100), so create() rejects the configuration regardless of gain.
TEST(RwMotorTorqueTest, IllConditionedNullSpaceGeometryRejected) {
    constexpr float kOutOfPlane = 1e-3F;  // tiny z component -> [Gs] barely spans the third dimension
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 4U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, kOutOfPlane}.normalized();
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, kOutOfPlane}.normalized();
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{-1.0F, 0.0F, kOutOfPlane}.normalized();
    rwConfiguration.GsMatrix_B.col(3) = Eigen::Vector3f{0.0F, -1.0F, kOutOfPlane}.normalized();

    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(2U), rwConfiguration, 0.5F), fsw::invalid_argument);
}

// With every wheel marked unavailable there is nothing to control with, so the (otherwise valid) configuration
// is uncontrollable and create() rejects it.
TEST(RwMotorTorqueTest, AllWheelsUnavailableRejected) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    rwConfiguration.wheelAvailability[0] = UNAVAILABLE;
    rwConfiguration.wheelAvailability[1] = UNAVAILABLE;
    rwConfiguration.wheelAvailability[2] = UNAVAILABLE;

    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration), fsw::invalid_argument);
}

// Determined case: three orthogonal wheels and three control axes give a square mapping with no null space, so
// the motor torques are the exact unique solution (each wheel negates the commanded torque on its own axis).
TEST(RwMotorTorqueTest, SquareMappingIsExact) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    const RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration)};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, RwMotorTorqueSpeeds{});

    EXPECT_NEAR(out[0], -Lr_B[0], 1e-6);
    EXPECT_NEAR(out[1], -Lr_B[1], 1e-6);
    EXPECT_NEAR(out[2], -Lr_B[2], 1e-6);
    for (uint32_t i = 3U; i < kMaxNumRw; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0F);
    }
}

// Non-consecutive control axes: selecting body x and z (skipping y) must control exactly those axes. Exercises
// the "pack selected axes to the top" approach in computeRwMapping for a gapped selection -- a bug that
// controlled x and y instead would fail the z assertion.
TEST(RwMotorTorqueTest, NonConsecutiveControlAxes) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    const RwMotorTorqueConfig config = RwMotorTorqueConfig::create({true, false, true}, rwConfiguration);
    const RwMotorTorqueAlgorithm alg{config};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector3f bodyTorque = availableGs(config) * alg.update(Lr_B, RwMotorTorqueSpeeds{});

    // The controlled axes x and z realize the commanded torque (-Lr); the uncontrolled y axis is left alone
    // (its orthogonal wheel is commanded zero, so no body torque about y).
    EXPECT_NEAR(bodyTorque[0], -Lr_B[0], 1e-5F);
    EXPECT_NEAR(bodyTorque[2], -Lr_B[2], 1e-5F);
    EXPECT_NEAR(bodyTorque[1], 0.0F, 1e-5F);
}

// setConfig swaps in a new configuration: the algorithm recomputes the mapping, so update() matches the new
// configuration's reference (here switching from three wheels to four with an active null-space).
TEST(RwMotorTorqueTest, SetConfigSwitchesConfiguration) {
    RwMotorTorqueArrayConfiguration rwA{};
    rwA.numRW = 3U;
    rwA.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwA.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwA.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(makeControlAxes(3U), rwA)};

    RwMotorTorqueArrayConfiguration rwB{};
    rwB.numRW = 4U;
    rwB.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwB.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwB.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    rwB.GsMatrix_B.col(3) = Eigen::Vector3f{1.0F, 1.0F, 1.0F}.normalized();
    constexpr float kOmegaGain = 0.5F;
    alg.setConfig(RwMotorTorqueConfig::create(makeControlAxes(3U), rwB, kOmegaGain));

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<4>() = Eigen::Vector4f{100.0F, -50.0F, 30.0F, 80.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, speeds);
    const Eigen::Vector<double, kMaxNumRw> ref = referenceUpdate(makeControlAxes(3U),
                                                                 rwB.GsMatrix_B.cast<double>(),
                                                                 rwB.numRW,
                                                                 rwB.wheelAvailability,
                                                                 Lr_B.cast<double>(),
                                                                 speeds.rwSpeeds.cast<double>(),
                                                                 speeds.rwDesiredSpeeds.cast<double>(),
                                                                 static_cast<double>(kOmegaGain));

    const float refScale = static_cast<float>(ref.cwiseAbs().maxCoeff());
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(out[i], static_cast<float>(ref[i]), 1e-4F + 1e-3F * refScale);
    }
}

// With the RW speeds already at their desired values the speed error is zero, so the null-space term vanishes even
// with a non-zero gain and a non-trivial null space: the output equals the control-only update.
TEST(RwMotorTorqueTest, NullSpaceZeroAtDesiredSpeed) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 4U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    rwConfiguration.GsMatrix_B.col(3) = Eigen::Vector3f{1.0F, 1.0F, 1.0F}.normalized();

    const RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration, 0.5F)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<4>() = Eigen::Vector4f{100.0F, -50.0F, 30.0F, 80.0F};
    speeds.rwDesiredSpeeds.head<4>() = Eigen::Vector4f{100.0F, -50.0F, 30.0F, 80.0F};  // zero speed error

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_FLOAT_EQ(out[i], controlOnly[i]);
    }
}
