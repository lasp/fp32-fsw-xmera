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
    regressionTestThrustVectoring({0.0F, 0.0F, 0.0F},       // sigma_MB
                                  {0.0F, 0.1F, 1.4F},       // r_MB_B
                                  {0.05F, 0.02F, 0.1F},     // r_CB_B
                                  0.1F,                     // armLength
                                  10.0F,                    // thrust
                                  Eigen::Vector3f::Zero(),  // Lreq_B
                                  kAccuracy);
}

TEST(ThrustVectoringTest, RegressionAxisAlignedThrustWithRequestedTorque) {
    regressionTestThrustVectoring(
        {0.0F, 0.0F, 0.0F}, {0.0F, 0.1F, 1.4F}, {0.05F, 0.02F, 0.1F}, 0.1F, 10.0F, {0.4F, -0.2F, 0.3F}, kAccuracy);
}

TEST(ThrustVectoringTest, RegressionTiltedMFrame) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(0.087F, 0.175F, 0.0F)));
    regressionTestThrustVectoring(
        sigma_MB, {0.0F, 0.1F, 1.4F}, {0.2F, -0.1F, 0.15F}, 0.1F, 5.0F, Eigen::Vector3f::Zero(), kAccuracy);
}

TEST(ThrustVectoringTest, RegressionTiltedMFrameWithRequestedTorque) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(0.087F, 0.175F, 0.0F)));
    regressionTestThrustVectoring(
        sigma_MB, {0.0F, 0.1F, 1.4F}, {0.2F, -0.1F, 0.15F}, 0.1F, 5.0F, {-0.3F, 0.15F, 0.25F}, kAccuracy);
}

TEST(ThrustVectoringTest, RegressionArbitraryGeometry) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(-0.2F, 0.1F, 0.0F)));
    regressionTestThrustVectoring(
        sigma_MB, {0.1F, -0.2F, 0.9F}, {0.3F, 0.25F, -0.1F}, 0.25F, 12.0F, Eigen::Vector3f::Zero(), kAccuracy);
}

TEST(ThrustVectoringTest, RegressionArbitraryGeometryWithRequestedTorque) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(-0.2F, 0.1F, 0.0F)));
    regressionTestThrustVectoring(
        sigma_MB, {0.1F, -0.2F, 0.9F}, {0.3F, 0.25F, -0.1F}, 0.25F, 12.0F, {0.5F, 0.6F, -0.4F}, kAccuracy);
}

// A zero arm length puts the thruster on the joint itself, which the pointing solve does not care about.
TEST(ThrustVectoringTest, RegressionZeroArmLength) {
    regressionTestThrustVectoring(
        {0.0F, 0.0F, 0.0F}, {0.0F, 0.1F, 1.4F}, {0.05F, 0.02F, 0.1F}, 0.0F, 10.0F, Eigen::Vector3f::Zero(), kAccuracy);
}

// A request beyond thrust * |r_MC| is regression-checked too: the helper expects the saturated torque.
TEST(ThrustVectoringTest, RegressionSaturatedRequest) {
    regressionTestThrustVectoring(
        {0.0F, 0.0F, 0.0F}, {0.0F, 0.1F, 1.4F}, {0.05F, 0.02F, 0.1F}, 0.1F, 10.0F, {1e3F, -5e2F, 8e2F}, kAccuracy);
}

// ---------------------------------------------------------------------------
// Setup tests (configuration validation)
// ---------------------------------------------------------------------------

TEST(ThrustVectoringTest, SetupTest) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f com{0.0F, 0.0F, -1.0F};  // clear of the joint at the origin, so the pointing is defined
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    const auto create = [&](const ThrustVectoringPlatformConfiguration& platform,
                            const ThrustVectoringThrusterConfiguration& thruster,
                            const Eigen::Vector3f& r_CB_B) {
        return ThrustVectoringConfig::create(platform, thruster, r_CB_B);
    };
    const ThrustVectoringThrusterConfiguration thruster{.armLength = 0.1F, .thrust = 10.0F};

    // A finite geometry with a valid cone half-angle and positive thrust is accepted.
    EXPECT_NO_THROW((void)create(makePlatformConfig(zero, zero, 1.0F), thruster, com));

    // Non-finite platform geometry is rejected.
    EXPECT_THROW((void)create(makePlatformConfig({nan, 0.0F, 0.0F}, zero, 1.0F), thruster, com), fsw::invalid_argument);
    EXPECT_THROW((void)create(makePlatformConfig(zero, {0.0F, nan, 0.0F}, 1.0F), thruster, com), fsw::invalid_argument);

    // A cone half-angle outside the open interval (0, pi) is rejected.
    EXPECT_THROW((void)create(makePlatformConfig(zero, zero, 0.0F), thruster, com), fsw::invalid_argument);
    EXPECT_THROW((void)create(makePlatformConfig(zero, zero, 4.0F), thruster, com), fsw::invalid_argument);
    EXPECT_THROW((void)create(makePlatformConfig(zero, zero, nan), thruster, com), fsw::invalid_argument);

    // A negative or non-finite arm length is rejected; zero is allowed.
    EXPECT_THROW((void)create(makePlatformConfig(zero, zero, 1.0F), {.armLength = -0.1F, .thrust = 10.0F}, com),
                 fsw::invalid_argument);
    EXPECT_THROW((void)create(makePlatformConfig(zero, zero, 1.0F), {.armLength = nan, .thrust = 10.0F}, com),
                 fsw::invalid_argument);
    EXPECT_NO_THROW((void)create(makePlatformConfig(zero, zero, 1.0F), {.armLength = 0.0F, .thrust = 10.0F}, com));

    // A non-positive or non-finite thrust magnitude is rejected: it leaves no line of action to point.
    EXPECT_THROW((void)create(makePlatformConfig(zero, zero, 1.0F), {.armLength = 0.1F, .thrust = 0.0F}, com),
                 fsw::invalid_argument);
    EXPECT_THROW((void)create(makePlatformConfig(zero, zero, 1.0F), {.armLength = 0.1F, .thrust = -1.0F}, com),
                 fsw::invalid_argument);
    EXPECT_THROW((void)create(makePlatformConfig(zero, zero, 1.0F), {.armLength = 0.1F, .thrust = nan}, com),
                 fsw::invalid_argument);

    // A non-finite center of mass is rejected.
    EXPECT_THROW((void)create(makePlatformConfig(zero, zero, 1.0F), thruster, {0.0F, nan, 0.0F}),
                 fsw::invalid_argument);
}

// A center of mass on (or within kMinR_CM of) the platform joint leaves the thruster with no moment arm, so the
// configuration is rejected rather than left to produce a meaningless reference at run time.
TEST(ThrustVectoringTest, SetupRejectsCenterOfMassOnTheJoint) {
    // The joint has a zero x component and the mount is tilted so its thrust leans along +x. Offsetting the
    // centre of mass along x is then exact in float -- no cancellation against the joint position, so the
    // threshold is straddled exactly -- while still describing a thrust that fires inboard.
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(0.0F, -0.6F, 0.0F)));
    const Eigen::Vector3f r_MB_B{0.0F, 0.1F, 1.4F};
    const auto create = [&](const Eigen::Vector3f& r_CB_B) {
        return ThrustVectoringConfig::create(
            makePlatformConfig(sigma_MB, r_MB_B, 1.0F), {.armLength = 0.1F, .thrust = 10.0F}, r_CB_B);
    };

    // Straddle the threshold: the offset is measured from the joint, not from the body origin.
    EXPECT_THROW((void)create(r_MB_B), fsw::invalid_argument);  // exactly on it
    EXPECT_THROW((void)create(r_MB_B + Eigen::Vector3f(0.5F * kMinR_CM, 0.0F, 0.0F)), fsw::invalid_argument);
    EXPECT_THROW((void)create(r_MB_B + Eigen::Vector3f(kMinR_CM, 0.0F, 0.0F)), fsw::invalid_argument);
    EXPECT_NO_THROW((void)create(r_MB_B + Eigen::Vector3f(2.0F * kMinR_CM, 0.0F, 0.0F)));
    EXPECT_NO_THROW((void)create(Eigen::Vector3f::Zero()));
}

// The configuration getters return the values supplied to create().
TEST(ThrustVectoringTest, ConfigRoundTrip) {
    const Eigen::Vector3f sigma_MB(0.1F, -0.2F, 0.3F);
    const Eigen::Vector3f r_MB_B(0.0F, 0.1F, 1.4F);
    const Eigen::Vector3f r_CB_B(0.05F, 0.02F, 0.1F);
    const ThrustVectoringConfig cfg = ThrustVectoringConfig::create(
        makePlatformConfig(sigma_MB, r_MB_B, 0.7F), {.armLength = 0.25F, .thrust = 12.0F}, r_CB_B);
    EXPECT_TRUE(cfg.getPlatformConfiguration().sigma_MB.isApprox(sigma_MB));
    EXPECT_TRUE(cfg.getPlatformConfiguration().r_MB_B.isApprox(r_MB_B));
    EXPECT_FLOAT_EQ(cfg.getPlatformConfiguration().thetaMax, 0.7F);
    EXPECT_FLOAT_EQ(cfg.getThrusterConfiguration().armLength, 0.25F);
    EXPECT_FLOAT_EQ(cfg.getThrusterConfiguration().thrust, 12.0F);
    EXPECT_TRUE(cfg.getR_CB_B().isApprox(r_CB_B));
}

// create() bounds sigma_MB to the principal MRP set: a value with norm > 1 is stored as its
// shadow-set representative (-sigma / |sigma|^2), which has norm <= 1.
TEST(ThrustVectoringTest, SigmaMbSwitchedToShadowSetWhenNormExceedsOne) {
    const Eigen::Vector3f largeSigma{0.8F, 0.6F, 0.6F};  // |sigma|^2 = 1.36 > 1
    ASSERT_GT(largeSigma.norm(), 1.0F) << "Test setup: sigma_MB must exceed the norm-1 boundary";

    const ThrustVectoringConfig cfg = makeConfig(largeSigma, Eigen::Vector3f::Zero(), {0.0F, 0.0F, 1.0F}, 0.1F, 10.0F);
    const Eigen::Vector3f stored = cfg.getPlatformConfiguration().sigma_MB;

    EXPECT_LE(stored.norm(), 1.0F);
    const Eigen::Vector3f expectedShadow = -largeSigma / largeSigma.squaredNorm();
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(stored(i), expectedShadow(i));
    }
}

// A sigma_MB already within the principal set (norm <= 1) is stored unchanged.
TEST(ThrustVectoringTest, SigmaMbWithinBoundStoredUnchanged) {
    const Eigen::Vector3f sigma{0.3F, -0.4F, 0.2F};  // norm < 1
    ASSERT_LE(sigma.norm(), 1.0F);

    const ThrustVectoringConfig cfg = makeConfig(sigma, Eigen::Vector3f::Zero(), {0.0F, 0.0F, 1.0F}, 0.1F, 10.0F);
    const Eigen::Vector3f stored = cfg.getPlatformConfiguration().sigma_MB;
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(stored(i), sigma(i));
    }
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// The reported thrust heading is a unit vector and the magnitude is passed through exactly.
TEST(ThrustVectoringTest, PropertyHeadingIsUnitAndThrustPreserved) {
    const ThrustVectoringAlgorithm alg{
        makeConfig({0.05F, 0.1F, -0.2F}, {0.0F, 0.1F, 1.4F}, {0.1F, 0.2F, -0.1F}, 0.1F, 7.5F)};
    const ThrustVectoringOutput out = alg.update({0.05F, -0.02F, 0.03F});

    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, 1e-6F);
    EXPECT_FLOAT_EQ(out.thrust, 7.5F);
}

// The solve is closed form, so a repeated call returns exactly the same reference: there is no state and no
// dependence on the previous cycle.
TEST(ThrustVectoringTest, PropertyRepeatedUpdateIsIdentical) {
    const ThrustVectoringAlgorithm alg{
        makeConfig({0.0F, 0.0F, 0.0F}, {0.0F, 0.1F, 1.4F}, {0.05F, 0.02F, 0.1F}, 0.1F, 10.0F)};
    const Eigen::Vector3f Lreq_B{0.1F, -0.05F, 0.08F};

    const ThrustVectoringOutput first = alg.update(Lreq_B);
    const ThrustVectoringOutput second = alg.update(Lreq_B);

    EXPECT_TRUE(second.tHat_B.isApprox(first.tHat_B));
    EXPECT_TRUE(second.r_TB_B.isApprox(first.r_TB_B));

    // A different request moves the reference, so the previous check is not vacuous.
    EXPECT_GT((alg.update(-Lreq_B).tHat_B - first.tHat_B).norm(), kAccuracy);
}

// The platform delivers the requested torque in a single call -- no iteration -- up to the component the geometry
// cannot reach, which is the part parallel to the center-of-mass offset from the joint.
TEST(ThrustVectoringTest, AchievesRequestedTorqueInOneCall) {
    const Eigen::Vector3f r_MB_B{0.0F, 0.1F, 1.4F};
    const Eigen::Vector3f r_CB_B{0.05F, 0.02F, 0.1F};
    const Eigen::Vector3f Lreq_B{0.1F, -0.05F, 0.08F};
    const ThrustVectoringAlgorithm alg{makeConfig(Eigen::Vector3f::Zero(), r_MB_B, r_CB_B, 0.1F, 10.0F)};

    const ThrustVectoringOutput out = alg.update(Lreq_B);

    const Eigen::Vector3f rHat_CM_B = (r_CB_B - r_MB_B).normalized();
    const Eigen::Vector3f LreqReachable_B = Lreq_B - (rHat_CM_B * rHat_CM_B.dot(Lreq_B));
    EXPECT_LT((achievedTorque_B(out, r_CB_B) - LreqReachable_B).norm(), kAccuracy);
}

// The request is interpreted in the body frame, so a tilted mount frame does not change the delivered torque.
TEST(ThrustVectoringTest, AchievesRequestedTorqueWithTiltedMountFrame) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(0.087F, 0.175F, 0.0F)));
    const Eigen::Vector3f r_MB_B{0.0F, 0.1F, 1.4F};
    const Eigen::Vector3f r_CB_B{0.05F, 0.02F, 0.1F};
    const Eigen::Vector3f Lreq_B{0.1F, -0.05F, 0.08F};
    const ThrustVectoringAlgorithm alg{makeConfig(sigma_MB, r_MB_B, r_CB_B, 0.1F, 10.0F)};

    const ThrustVectoringOutput out = alg.update(Lreq_B);

    const Eigen::Vector3f rHat_CM_B = (r_CB_B - r_MB_B).normalized();
    const Eigen::Vector3f LreqReachable_B = Lreq_B - (rHat_CM_B * rHat_CM_B.dot(Lreq_B));
    EXPECT_LT((achievedTorque_B(out, r_CB_B) - LreqReachable_B).norm(), kAccuracy);
}

// The component of the request along the center-of-mass offset can never be produced: the torque of a force whose
// line runs through the joint is perpendicular to that offset by construction.
TEST(ThrustVectoringTest, TorqueAlongTheMomentArmIsUnreachable) {
    const Eigen::Vector3f r_MB_B{0.0F, 0.0F, 1.0F};
    const Eigen::Vector3f r_CB_B{0.0F, 0.0F, 0.0F};  // r_MC is +z, so a +z torque request is unreachable
    const ThrustVectoringAlgorithm alg{makeConfig(Eigen::Vector3f::Zero(), r_MB_B, r_CB_B, 0.1F, 10.0F)};

    const ThrustVectoringOutput out = alg.update({0.0F, 0.0F, 0.5F});

    // The request is entirely unreachable, so the module falls back to the zero-torque alignment.
    EXPECT_LT(achievedTorque_B(out, r_CB_B).norm(), kAccuracy);
}

// A request beyond thrust * |r_MC| saturates on the largest torque the geometry can deliver, in the requested
// direction, rather than failing or overshooting.
TEST(ThrustVectoringTest, SaturatesAtTheMaximumAchievableTorque) {
    const Eigen::Vector3f r_MB_B{0.0F, 0.0F, 1.0F};
    const Eigen::Vector3f r_CB_B = Eigen::Vector3f::Zero();
    constexpr float thrust = 10.0F;
    const float maxTorque = thrust * (r_MB_B - r_CB_B).norm();  // thrust * |r_MC|
    const ThrustVectoringAlgorithm alg{makeConfig(Eigen::Vector3f::Zero(), r_MB_B, r_CB_B, 0.1F, thrust)};

    const Eigen::Vector3f achieved = achievedTorque_B(alg.update({1e3F, 0.0F, 0.0F}), r_CB_B);

    EXPECT_NEAR(achieved.norm(), maxTorque, 1e-3F);
    EXPECT_GT(achieved.normalized().dot(Eigen::Vector3f::UnitX()), 1.0F - kAccuracy);  // still along the request
}

// Both signs of the free parameter in the solve give the same torque, so the reference takes the one nearer the
// un-deflected direction. With the center of mass on the far side of the joint along -z, that is -z (no
// deflection) rather than +z (a 180 degree flip that would produce the same zero torque).
TEST(ThrustVectoringTest, PicksTheSolutionNearerTheUndeflectedDirection) {
    // M == B, so the neutral thrust direction is -z; r_MC points along +z.
    const ThrustVectoringAlgorithm alg{
        makeConfig(Eigen::Vector3f::Zero(), {0.0F, 0.0F, 1.0F}, Eigen::Vector3f::Zero(), 0.1F, 10.0F)};
    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    ASSERT_LT(achievedTorque_B(out, Eigen::Vector3f::Zero()).norm(), kAccuracy) << "Test setup: torque must vanish";
    EXPECT_GT(out.tHat_B.dot(-Eigen::Vector3f::UnitZ()), 1.0F - kAccuracy);
}

// A geometry that would require a large deflection is clamped so the thrust direction stays on the cone: the
// angle between the reported heading and its neutral direction equals thetaMax.
TEST(ThrustVectoringTest, ThrustDeflectionClampedToCone) {
    constexpr float thetaMax = 0.5F;
    // M == B, so the neutral thrust direction is -z; the center of mass sits far off that axis.
    const ThrustVectoringAlgorithm alg{
        makeConfig(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), {1.0F, 0.0F, -0.1F}, 0.1F, 5.0F, thetaMax)};

    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    const float deflection = std::acos((-Eigen::Vector3f::UnitZ()).dot(out.tHat_B));
    EXPECT_NEAR(deflection, thetaMax, 1e-4F);
}

namespace {
// A mount tilted far enough that its axis has a positive body-frame z component. Every quantity measured against
// the mount axis then disagrees in sign with the same quantity measured against the un-deflected body -z, so a
// test built on this tilt fails if the implementation confuses the two.
Eigen::Vector3f steeplyTiltedMount() { return dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(2.5F, 0.0F, 0.0F))); }
Eigen::Vector3f neutralDirection_B(const Eigen::Vector3f& sigma_MB) {
    // Derived the opposite way round from the implementation, so a transposed conversion cannot pass unnoticed.
    return mrpToDcm(sigma_MB).transpose() * -Eigen::Vector3f::UnitZ();
}
}  // namespace

// The cone is measured about the mount frame's axis, not the body -z axis. With the centre of mass on that axis
// the solution needs no deflection at all, so nothing may be clamped -- even though the axis lies far outside a
// cone drawn about body -z.
TEST(ThrustVectoringTest, NoClampWhenAlignedWithATiltedMountAxis) {
    const Eigen::Vector3f sigma_MB = steeplyTiltedMount();
    const Eigen::Vector3f neutral_B = neutralDirection_B(sigma_MB);
    constexpr float thetaMax = 0.3F;
    ASSERT_GT(std::acos(neutral_B.dot(-Eigen::Vector3f::UnitZ())), thetaMax)
        << "Test setup: the mount axis must lie outside a cone about body -z";

    // Centre of mass along the mount axis: the zero-torque solution is the axis itself.
    const ThrustVectoringAlgorithm alg{makeConfig(sigma_MB, Eigen::Vector3f::Zero(), neutral_B, 0.1F, 10.0F, thetaMax)};
    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    // Compared as vectors rather than through acos: for near-parallel directions acos is ill-conditioned, and
    // amplifies a one-ulp dot product into an apparent angle of sqrt(2 * eps), about 5e-4 rad.
    EXPECT_LT((out.tHat_B - neutral_B).norm(), 1e-5F);
}

// The clamp itself also works about the mount axis: a geometry needing more deflection than the cone allows is
// pulled back to exactly thetaMax measured from that axis.
TEST(ThrustVectoringTest, ThrustDeflectionClampedToConeWithTiltedMountFrame) {
    const Eigen::Vector3f sigma_MB = steeplyTiltedMount();
    const Eigen::Vector3f neutral_B = neutralDirection_B(sigma_MB);
    constexpr float thetaMax = 0.5F;

    // Centre of mass very nearly perpendicular to the mount axis, tipped just far enough onto the thrust side
    // that the mounting is legal: the solution still wants close to 90 degrees of deflection.
    const Eigen::Vector3f r_CB_B = (neutral_B.unitOrthogonal() + (0.01F * neutral_B)).normalized();
    const ThrustVectoringAlgorithm alg{makeConfig(sigma_MB, Eigen::Vector3f::Zero(), r_CB_B, 0.1F, 5.0F, thetaMax)};
    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    EXPECT_NEAR(std::acos(neutral_B.dot(out.tHat_B)), thetaMax, 1e-4F);
}

// The sign of the free parameter is chosen against the mount axis too. With the centre of mass on the far side of
// the joint, the two zero-torque solutions are +/- the axis, and the reference must take the undeflected one.
TEST(ThrustVectoringTest, PicksTheNearerSolutionWithTiltedMountFrame) {
    const Eigen::Vector3f sigma_MB = steeplyTiltedMount();
    const Eigen::Vector3f neutral_B = neutralDirection_B(sigma_MB);
    ASSERT_GT(neutral_B.z(), 0.0F) << "Test setup: the mount axis must oppose body -z in the sign of its z";

    // Joint displaced opposite the mount axis, so the un-deflected thrust fires from it back towards the centre
    // of mass and the thruster lands outboard.
    const ThrustVectoringAlgorithm alg{makeConfig(sigma_MB, -neutral_B, Eigen::Vector3f::Zero(), 0.1F, 10.0F)};
    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    ASSERT_LT(achievedTorque_B(out, Eigen::Vector3f::Zero()).norm(), kAccuracy) << "Test setup: torque must vanish";
    EXPECT_GT(out.tHat_B.dot(neutral_B), 1.0F - kAccuracy);
}

// setConfig() swaps the configuration on a running instance: a new center of mass changes the pointing.
TEST(ThrustVectoringTest, SetConfigAppliesNewCenterOfMass) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f r_MB_B{0.0F, 0.1F, 1.4F};
    ThrustVectoringAlgorithm alg{makeConfig(zero, r_MB_B, {0.05F, 0.02F, 0.1F}, 0.1F, 10.0F)};
    const ThrustVectoringOutput before = alg.update(zero);

    alg.setConfig(makeConfig(zero, r_MB_B, {-0.05F, 0.15F, 0.1F}, 0.1F, 10.0F));
    const ThrustVectoringOutput after = alg.update(zero);

    EXPECT_GT((after.tHat_B - before.tHat_B).norm(), kAccuracy);
}
