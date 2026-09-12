#include "thrustVectoringTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <limits>

namespace {
constexpr float kAccuracy = 1e-4F;
}  // namespace

// ---------------------------------------------------------------------------
// Regression tests (thruster aligned with the center of mass when no torque is requested)
// ---------------------------------------------------------------------------

TEST(ThrustVectoringTest, RegressionAxisAlignedThrust) {
    regressionTestThrustVectoring({0.0F, 0.1F, 1.4F},       // r_MB_B
                                  {0.05F, 0.02F, 0.1F},     // r_CB_B
                                  10.0F,                    // thrust
                                  Eigen::Vector3f::Zero(),  // Lreq_B
                                  kAccuracy);
}

TEST(ThrustVectoringTest, RegressionAxisAlignedThrustWithRequestedTorque) {
    regressionTestThrustVectoring({0.0F, 0.1F, 1.4F}, {0.05F, 0.02F, 0.1F}, 10.0F, {0.4F, -0.2F, 0.3F}, kAccuracy);
}

TEST(ThrustVectoringTest, RegressionOffsetThrustPoint) {
    regressionTestThrustVectoring({0.0F, 0.1F, 1.4F}, {0.2F, -0.1F, 0.15F}, 5.0F, Eigen::Vector3f::Zero(), kAccuracy);
}

TEST(ThrustVectoringTest, RegressionOffsetThrustPointWithRequestedTorque) {
    regressionTestThrustVectoring({0.0F, 0.1F, 1.4F}, {0.2F, -0.1F, 0.15F}, 5.0F, {-0.3F, 0.15F, 0.25F}, kAccuracy);
}

TEST(ThrustVectoringTest, RegressionArbitraryGeometry) {
    regressionTestThrustVectoring({0.1F, -0.2F, 0.9F}, {0.3F, 0.25F, -0.1F}, 12.0F, Eigen::Vector3f::Zero(), kAccuracy);
}

TEST(ThrustVectoringTest, RegressionArbitraryGeometryWithRequestedTorque) {
    regressionTestThrustVectoring({0.1F, -0.2F, 0.9F}, {0.3F, 0.25F, -0.1F}, 12.0F, {0.5F, 0.6F, -0.4F}, kAccuracy);
}

// A request beyond thrust * |r_MC| is regression-checked too: the helper expects the saturated torque.
TEST(ThrustVectoringTest, RegressionSaturatedRequest) {
    regressionTestThrustVectoring({0.0F, 0.1F, 1.4F}, {0.05F, 0.02F, 0.1F}, 10.0F, {1e3F, -5e2F, 8e2F}, kAccuracy);
}

// ---------------------------------------------------------------------------
// Setup tests (configuration validation)
// ---------------------------------------------------------------------------

TEST(ThrustVectoringTest, SetupTest) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f com{0.0F, 0.0F, -1.0F};  // clear of M at the origin, so a direction is defined
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    const auto create = [](const Eigen::Vector3f& r_MB_B, float thrust, const Eigen::Vector3f& r_CB_B) {
        return ThrustVectoringConfig::create(r_MB_B, thrust, r_CB_B);
    };

    // A finite geometry with positive thrust is accepted.
    EXPECT_NO_THROW((void)create(zero, 10.0F, com));

    // A non-finite thrust point is rejected.
    EXPECT_THROW((void)create({0.0F, nan, 0.0F}, 10.0F, com), fsw::invalid_argument);

    // A non-positive or non-finite thrust magnitude is rejected: it leaves no line of action to point.
    EXPECT_THROW((void)create(zero, 0.0F, com), fsw::invalid_argument);
    EXPECT_THROW((void)create(zero, -1.0F, com), fsw::invalid_argument);
    EXPECT_THROW((void)create(zero, nan, com), fsw::invalid_argument);

    // A non-finite center of mass is rejected.
    EXPECT_THROW((void)create(zero, 10.0F, {0.0F, nan, 0.0F}), fsw::invalid_argument);
}

// A center of mass on (or within kMinR_CM of) M leaves the thrust with no moment arm, so the configuration is
// rejected rather than left to produce a meaningless direction at run time.
TEST(ThrustVectoringTest, SetupRejectsCenterOfMassOnTheThrustPoint) {
    // M has a zero x component, so offsetting the centre of mass along x is exact in float -- no cancellation
    // against the thrust point, so the threshold is straddled exactly.
    const Eigen::Vector3f r_MB_B{0.0F, 0.1F, 1.4F};
    const auto create = [&](const Eigen::Vector3f& r_CB_B) {
        return ThrustVectoringConfig::create(r_MB_B, 10.0F, r_CB_B);
    };

    // Straddle the threshold: the offset is measured from M, not from the body origin.
    EXPECT_THROW((void)create(r_MB_B), fsw::invalid_argument);  // exactly on it
    EXPECT_THROW((void)create(r_MB_B + Eigen::Vector3f(0.5F * kMinR_CM, 0.0F, 0.0F)), fsw::invalid_argument);
    EXPECT_THROW((void)create(r_MB_B + Eigen::Vector3f(kMinR_CM, 0.0F, 0.0F)), fsw::invalid_argument);
    EXPECT_NO_THROW((void)create(r_MB_B + Eigen::Vector3f(2.0F * kMinR_CM, 0.0F, 0.0F)));
    EXPECT_NO_THROW((void)create(Eigen::Vector3f::Zero()));
}

// Both endpoints can be finite while their difference is not, which would leave the moment arm infinite and its
// direction undefined. The finiteness of the difference is checked, not just of each endpoint.
TEST(ThrustVectoringTest, SetupRejectsAnUnrepresentableMomentArm) {
    const Eigen::Vector3f farOut{3e38F, 0.0F, 0.0F};
    ASSERT_TRUE(farOut.allFinite()) << "Test setup: each endpoint must be finite on its own";
    ASSERT_FALSE((farOut - -farOut).allFinite()) << "Test setup: their difference must overflow";

    EXPECT_THROW((void)ThrustVectoringConfig::create(farOut, 10.0F, -farOut), fsw::invalid_argument);
}

// The solve divides by thrust * |r_MC|. Each factor can pass its own check while the product flushes to zero,
// which would leave no torque scale to work against.
TEST(ThrustVectoringTest, SetupRejectsAnUnrepresentableMaximumTorque) {
    const Eigen::Vector3f r_MB_B{1e-2F, 0.0F, 0.0F};  // a moment arm above kMinR_CM
    constexpr float tinyThrust = 1e-44F;              // positive and finite, but denormal
    ASSERT_TRUE(ThrustVectoringConfig::isValidThrust(tinyThrust)) << "Test setup: the thrust alone must pass";
    ASSERT_TRUE(ThrustVectoringConfig::isValidR_CM(Eigen::Vector3f::Zero(), r_MB_B))
        << "Test setup: the moment arm alone must pass";
    ASSERT_EQ(tinyThrust * r_MB_B.stableNorm(), 0.0F) << "Test setup: their product must flush to zero";

    EXPECT_THROW((void)ThrustVectoringConfig::create(r_MB_B, tinyThrust, Eigen::Vector3f::Zero()),
                 fsw::invalid_argument);

    // An overflowing product is rejected for the same reason.
    EXPECT_THROW((void)ThrustVectoringConfig::create({1e30F, 0.0F, 0.0F}, 1e30F, Eigen::Vector3f::Zero()),
                 fsw::invalid_argument);
}

// The configuration getters return the values supplied to create().
TEST(ThrustVectoringTest, ConfigRoundTrip) {
    const Eigen::Vector3f r_MB_B(0.0F, 0.1F, 1.4F);
    const Eigen::Vector3f r_CB_B(0.05F, 0.02F, 0.1F);
    const ThrustVectoringConfig cfg = ThrustVectoringConfig::create(r_MB_B, 12.0F, r_CB_B);
    EXPECT_TRUE(cfg.getR_MB_B().isApprox(r_MB_B));
    EXPECT_FLOAT_EQ(cfg.getThrust(), 12.0F);
    EXPECT_TRUE(cfg.getR_CB_B().isApprox(r_CB_B));
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// The reported thrust heading is a unit vector.
TEST(ThrustVectoringTest, PropertyHeadingIsUnit) {
    const ThrustVectoringAlgorithm alg{makeConfig({0.0F, 0.1F, 1.4F}, {0.1F, 0.2F, -0.1F}, 7.5F)};
    const Eigen::Vector3f tHat_B = alg.update({0.05F, -0.02F, 0.03F});

    EXPECT_NEAR(tHat_B.norm(), 1.0F, 1e-6F);
}

// The solve is closed form, so a repeated call returns exactly the same reference: there is no state and no
// dependence on the previous cycle.
TEST(ThrustVectoringTest, PropertyRepeatedUpdateIsIdentical) {
    const ThrustVectoringAlgorithm alg{makeConfig({0.0F, 0.1F, 1.4F}, {0.05F, 0.02F, 0.1F}, 10.0F)};
    const Eigen::Vector3f Lreq_B{0.1F, -0.05F, 0.08F};

    const Eigen::Vector3f first = alg.update(Lreq_B);
    const Eigen::Vector3f second = alg.update(Lreq_B);

    EXPECT_TRUE(second.isApprox(first));

    // A different request moves the reference, so the previous check is not vacuous.
    EXPECT_GT((alg.update(-Lreq_B) - first).norm(), kAccuracy);
}

// The solve delivers the requested torque in a single call -- no iteration -- up to the component the geometry
// cannot reach, which is the part parallel to the center-of-mass offset from M.
TEST(ThrustVectoringTest, AchievesRequestedTorqueInOneCall) {
    const Eigen::Vector3f r_MB_B{0.0F, 0.1F, 1.4F};
    const Eigen::Vector3f r_CB_B{0.05F, 0.02F, 0.1F};
    const Eigen::Vector3f Lreq_B{0.1F, -0.05F, 0.08F};
    const ThrustVectoringAlgorithm alg{makeConfig(r_MB_B, r_CB_B, 10.0F)};

    const Eigen::Vector3f tHat_B = alg.update(Lreq_B);

    const Eigen::Vector3f rHat_CM_B = (r_CB_B - r_MB_B).normalized();
    const Eigen::Vector3f LreqReachable_B = Lreq_B - (rHat_CM_B * rHat_CM_B.dot(Lreq_B));
    EXPECT_LT((achievedTorque_B(tHat_B, 10.0F, r_MB_B, r_CB_B) - LreqReachable_B).norm(), kAccuracy);
}

// The component of the request along the center-of-mass offset can never be produced: the torque of a force whose
// line passes through M is perpendicular to that offset by construction.
TEST(ThrustVectoringTest, TorqueAlongTheMomentArmIsUnreachable) {
    const Eigen::Vector3f r_MB_B{0.0F, 0.0F, 1.0F};
    const Eigen::Vector3f r_CB_B{0.0F, 0.0F, 0.0F};  // r_MC is +z, so a +z torque request is unreachable
    const ThrustVectoringAlgorithm alg{makeConfig(r_MB_B, r_CB_B, 10.0F)};

    const Eigen::Vector3f tHat_B = alg.update({0.0F, 0.0F, 0.5F});

    // The request is entirely unreachable, so the module falls back to the zero-torque alignment.
    EXPECT_LT(achievedTorque_B(tHat_B, 10.0F, r_MB_B, r_CB_B).norm(), kAccuracy);
}

// A request beyond thrust * |r_MC| saturates on the largest torque the geometry can deliver, in the requested
// direction, rather than failing or overshooting.
TEST(ThrustVectoringTest, SaturatesAtTheMaximumAchievableTorque) {
    const Eigen::Vector3f r_MB_B{0.0F, 0.0F, 1.0F};
    const Eigen::Vector3f r_CB_B = Eigen::Vector3f::Zero();
    constexpr float thrust = 10.0F;
    const float maxTorque = thrust * (r_MB_B - r_CB_B).norm();  // thrust * |r_MC|
    const ThrustVectoringAlgorithm alg{makeConfig(r_MB_B, r_CB_B, thrust)};

    const Eigen::Vector3f achieved = achievedTorque_B(alg.update({1e3F, 0.0F, 0.0F}), thrust, r_MB_B, r_CB_B);

    EXPECT_NEAR(achieved.norm(), maxTorque, 1e-3F);
    EXPECT_GT(achieved.normalized().dot(Eigen::Vector3f::UnitX()), 1.0F - kAccuracy);  // still along the request
}

// Both signs of the free parameter in the solve give the same torque. The reference takes the one that fires the
// thrust from M towards the center of mass, so the thrust acts on the vehicle from outside it -- here that is
// -z, not the +z flip that would produce the same zero torque.
TEST(ThrustVectoringTest, PicksTheSolutionFiringTowardsTheCenterOfMass) {
    // r_MC points along +z, so the thrust must fire along -z.
    const ThrustVectoringAlgorithm alg{makeConfig({0.0F, 0.0F, 1.0F}, Eigen::Vector3f::Zero(), 10.0F)};
    const Eigen::Vector3f tHat_B = alg.update(Eigen::Vector3f::Zero());

    ASSERT_LT(achievedTorque_B(tHat_B, 10.0F, {0.0F, 0.0F, 1.0F}, Eigen::Vector3f::Zero()).norm(), kAccuracy)
        << "Test setup: torque must vanish";
    EXPECT_GT(tHat_B.dot(-Eigen::Vector3f::UnitZ()), 1.0F - kAccuracy);
}

// The same choice holds for M displaced in an arbitrary direction, not only along a body axis.
TEST(ThrustVectoringTest, PicksTheSolutionFiringTowardsTheCenterOfMassForAnObliqueThrustPoint) {
    const Eigen::Vector3f r_MB_B = Eigen::Vector3f(0.6F, -0.5F, 0.4F);
    const ThrustVectoringAlgorithm alg{makeConfig(r_MB_B, Eigen::Vector3f::Zero(), 10.0F)};
    const Eigen::Vector3f tHat_B = alg.update(Eigen::Vector3f::Zero());

    ASSERT_LT(achievedTorque_B(tHat_B, 10.0F, r_MB_B, Eigen::Vector3f::Zero()).norm(), kAccuracy)
        << "Test setup: torque must vanish";
    EXPECT_GT(tHat_B.dot(-r_MB_B.normalized()), 1.0F - kAccuracy);
}

// setConfig() swaps the configuration on a running instance: a new center of mass changes the pointing.
TEST(ThrustVectoringTest, SetConfigAppliesNewCenterOfMass) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f r_MB_B{0.0F, 0.1F, 1.4F};
    ThrustVectoringAlgorithm alg{makeConfig(r_MB_B, {0.05F, 0.02F, 0.1F}, 10.0F)};
    const Eigen::Vector3f before = alg.update(zero);

    alg.setConfig(makeConfig(r_MB_B, {-0.05F, 0.15F, 0.1F}, 10.0F));
    const Eigen::Vector3f after = alg.update(zero);

    EXPECT_GT((after - before).norm(), kAccuracy);
}
