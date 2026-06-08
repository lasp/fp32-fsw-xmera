#include "rwMotorTorqueTestHelpers.hpp"
#include "utilities/freestandingInvalidArgument.h"
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

// Control torque plus an active null-space despin term (four wheels, non-zero gain and speeds).
TEST(RwMotorTorqueTest, RegressionDespin) {
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
    const RwMotorTorqueAvailability availability{};

    // A non-unit control axis is rejected.
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    controlAxes_B.row(0) = Eigen::Vector3f{2.0F, 0.0F, 0.0F};
    EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability), fsw::invalid_argument);

    // Non-orthogonal control axes are rejected.
    controlAxes_B = Eigen::Matrix3f::Zero();
    controlAxes_B.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    controlAxes_B.row(1) = Eigen::Vector3f{0.70710678F, 0.70710678F, 0.0F};
    EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability), fsw::invalid_argument);

    // A non-unit RW spin axis is rejected.
    RwMotorTorqueArrayConfiguration nonUnitRw{};
    nonUnitRw.numRW = 1U;
    nonUnitRw.GsMatrix_B.col(0) = Eigen::Vector3f{2.0F, 0.0F, 0.0F};
    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(1U), nonUnitRw, availability), fsw::invalid_argument);

    // A negative despin gain is rejected.
    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(1U), rwConfiguration, availability, -1.0F),
                 fsw::invalid_argument);

    // Control mapping not full rank (3 control axes but no reaction wheels): create() validates the mapping
    // and rejects the rank-deficient configuration.
    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration, availability),
                 fsw::invalid_argument);
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

TEST(RwMotorTorqueTest, PropertyDespinAddsNoBodyTorque) {
    propertyDespinAddsNoBodyTorque(
        Eigen::Vector3f{0.3F, -0.5F, 0.8F},
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

TEST(RwMotorTorqueTest, PropertyZeroGainDisablesDespin) {
    propertyZeroGainDisablesDespin(
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

// ---------------------------------------------------------------------------
// Edge-case tests — boundary geometries and config canonicalization.
// ---------------------------------------------------------------------------

// Three spanning wheels leave no null space, so the despin term is zero even with a non-zero gain.
TEST(RwMotorTorqueTest, ThreeSpanningWheelsHaveNoDespin) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    const RwMotorTorqueAlgorithm alg{
        RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration, RwMotorTorqueAvailability{}, 0.5F)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<3>() = Eigen::Vector3f{100.0F, -50.0F, 30.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> withSpeeds = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_FLOAT_EQ(withSpeeds[i], controlOnly[i]);
    }
}

// Control axes may sit in any rows; only the spanned subspace matters. Controlling body x and z via
// non-contiguous rows {0, 2} must produce the same motor torques as the contiguous rows {0, 1}.
TEST(RwMotorTorqueTest, NonContiguousControlAxes) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    const RwMotorTorqueAvailability availability{};

    Eigen::Matrix3f contiguous{Eigen::Matrix3f::Zero()};
    contiguous.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    contiguous.row(1) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    Eigen::Matrix3f nonContiguous{Eigen::Matrix3f::Zero()};
    nonContiguous.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    nonContiguous.row(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    const RwMotorTorqueAlgorithm algContiguous{RwMotorTorqueConfig::create(contiguous, rwConfiguration, availability)};
    const RwMotorTorqueAlgorithm algNonContiguous{
        RwMotorTorqueConfig::create(nonContiguous, rwConfiguration, availability)};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> outContiguous = algContiguous.update(Lr_B, RwMotorTorqueSpeeds{});
    const Eigen::Vector<float, kMaxNumRw> outNonContiguous = algNonContiguous.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(outContiguous[i], outNonContiguous[i], 1e-6);
    }
}

// Control axes that are unit but slightly non-orthogonal (within the validation tolerance) are stored
// orthonormalized, and zero (uncontrolled) rows stay zero.
TEST(RwMotorTorqueTest, ControlAxesAreOrthonormalized) {
    Eigen::Matrix3f controlAxes{Eigen::Matrix3f::Zero()};
    controlAxes.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    controlAxes.row(1) = Eigen::Vector3f{8e-4F, 1.0F, 0.0F}.normalized();  // unit, ~8e-4 off orthogonal to row 0

    // Three wheels spanning body x, y, z so create() accepts the (controllable) configuration.
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    const RwMotorTorqueConfig config =
        RwMotorTorqueConfig::create(controlAxes, rwConfiguration, RwMotorTorqueAvailability{});
    const Eigen::Matrix3f& stored = config.getControlAxes();

    EXPECT_NEAR(stored.row(0).norm(), 1.0F, 1e-6);
    EXPECT_NEAR(stored.row(1).norm(), 1.0F, 1e-6);
    EXPECT_NEAR(stored.row(0).dot(stored.row(1)), 0.0F, 1e-6);  // exactly orthogonal after Gram-Schmidt
    EXPECT_NEAR(stored.row(2).norm(), 0.0F, 1e-6);              // uncontrolled row untouched
}

// A near-degenerate control geometry (two wheels nearly aligned, so the second control axis is barely
// reachable) is rejected by create(): cond([CGs]) exceeds the limit even though the mapping is full rank.
TEST(RwMotorTorqueTest, IllConditionedControlMappingRejected) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 2U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{1.0F, 5e-3F, 0.0F}.normalized();  // ~0.3 deg off wheel 0

    // Control body x and y: y is reachable only through the tiny y-component of the near-parallel wheels.
    Eigen::Matrix3f controlAxes{Eigen::Matrix3f::Zero()};
    controlAxes.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    controlAxes.row(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};

    EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes, rwConfiguration, RwMotorTorqueAvailability{}),
                 fsw::invalid_argument);
}

// Four near-coplanar wheels: the control mapping (body x, y) is well-conditioned, but the null-space (despin)
// geometry is ill-conditioned (cond([Gs]) > 100), so create() rejects the configuration regardless of gain.
TEST(RwMotorTorqueTest, IllConditionedDespinGeometryRejected) {
    constexpr float kOutOfPlane = 1e-3F;  // tiny z component -> [Gs] barely spans the third dimension
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 4U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, kOutOfPlane}.normalized();
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, kOutOfPlane}.normalized();
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{-1.0F, 0.0F, kOutOfPlane}.normalized();
    rwConfiguration.GsMatrix_B.col(3) = Eigen::Vector3f{0.0F, -1.0F, kOutOfPlane}.normalized();

    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(2U), rwConfiguration, RwMotorTorqueAvailability{}, 0.5F),
                 fsw::invalid_argument);
}
