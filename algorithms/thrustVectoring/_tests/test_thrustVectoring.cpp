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
    regressionTestThrustVectoring({0.0F, 0.0F, 0.0F},      // sigma_MB
                                  {0.0F, -0.1F, -1.4F},    // r_MB_B
                                  {0.0F, 0.0F, -0.1F},     // r_FM_F
                                  {0.05F, 0.02F, 0.1F},    // r_CB_B
                                  {-0.01F, 0.03F, 0.02F},  // rThrust_F
                                  {1.0F, 1.0F, 10.0F},     // tHatThrust_F (normalized in helper)
                                  10.0F,                   // maxThrust
                                  kAccuracy);
}

TEST(ThrustVectoringTest, RegressionTiltedMFrame) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(0.087F, 0.175F, 0.0F)));
    regressionTestThrustVectoring(sigma_MB,                // sigma_MB
                                  {0.0F, -0.1F, -1.4F},    // r_MB_B
                                  {0.0F, 0.0F, -0.1F},     // r_FM_F
                                  {0.2F, -0.1F, 0.15F},    // r_CB_B
                                  {-0.01F, 0.03F, 0.02F},  // rThrust_F
                                  {0.0F, 0.0F, 1.0F},      // tHatThrust_F
                                  5.0F,                    // maxThrust
                                  kAccuracy);
}

TEST(ThrustVectoringTest, RegressionArbitraryGeometry) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(-0.2F, 0.1F, 0.0F)));
    regressionTestThrustVectoring(sigma_MB,                // sigma_MB
                                  {-0.1F, 0.2F, -0.9F},    // r_MB_B
                                  {0.02F, -0.05F, -0.2F},  // r_FM_F
                                  {0.3F, 0.25F, -0.1F},    // r_CB_B
                                  {0.04F, -0.02F, 0.05F},  // rThrust_F
                                  {2.0F, -1.0F, 8.0F},     // tHatThrust_F
                                  12.0F,                   // maxThrust
                                  kAccuracy);
}

// ---------------------------------------------------------------------------
// Setup tests (configuration validation)
// ---------------------------------------------------------------------------

TEST(ThrustVectoringTest, SetupTest) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f unitZ{0.0F, 0.0F, 1.0F};
    const Eigen::Vector3f com{0.0F, 0.0F, 1.0F};  // clear of the joint at the origin, so the pointing is defined
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    const auto platform = [&](float thetaMax) { return makePlatformConfig(zero, zero, zero, thetaMax); };
    const ThrustVectoringThrusterConfiguration thruster{.r_TF_F = zero, .tHat_F = unitZ, .thrust = 10.0F};

    // A finite geometry with a valid cone half-angle, unit thrust direction and positive thrust is accepted.
    EXPECT_NO_THROW(ThrustVectoringConfig::create(platform(1.0F), thruster, com));

    // Non-finite platform geometry is rejected.
    EXPECT_THROW(ThrustVectoringConfig::create(makePlatformConfig({nan, 0.0F, 0.0F}, zero, zero, 1.0F), thruster, com),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(makePlatformConfig(zero, {0.0F, nan, 0.0F}, zero, 1.0F), thruster, com),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(makePlatformConfig(zero, zero, {0.0F, 0.0F, nan}, 1.0F), thruster, com),
                 fsw::invalid_argument);

    // A cone half-angle outside the open interval (0, pi) is rejected.
    EXPECT_THROW(ThrustVectoringConfig::create(platform(0.0F), thruster, com), fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(platform(4.0F), thruster, com), fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(platform(nan), thruster, com), fsw::invalid_argument);

    // A non-finite thrust application point is rejected.
    EXPECT_THROW(ThrustVectoringConfig::create(
                     platform(1.0F), {.r_TF_F = {nan, 0.0F, 0.0F}, .tHat_F = unitZ, .thrust = 10.0F}, com),
                 fsw::invalid_argument);

    // A thrust direction that is not (close to) a unit vector is rejected, including the zero vector.
    EXPECT_THROW(ThrustVectoringConfig::create(
                     platform(1.0F), {.r_TF_F = zero, .tHat_F = {0.0F, 0.0F, 2.0F}, .thrust = 10.0F}, com),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(platform(1.0F), {.r_TF_F = zero, .tHat_F = zero, .thrust = 10.0F}, com),
                 fsw::invalid_argument);

    // A non-positive or non-finite thrust magnitude is rejected: it leaves no line of action to point.
    EXPECT_THROW(ThrustVectoringConfig::create(platform(1.0F), {.r_TF_F = zero, .tHat_F = unitZ, .thrust = 0.0F}, com),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(platform(1.0F), {.r_TF_F = zero, .tHat_F = unitZ, .thrust = -1.0F}, com),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(platform(1.0F), {.r_TF_F = zero, .tHat_F = unitZ, .thrust = nan}, com),
                 fsw::invalid_argument);

    // A non-finite center of mass is rejected.
    EXPECT_THROW(ThrustVectoringConfig::create(platform(1.0F), thruster, {0.0F, nan, 0.0F}), fsw::invalid_argument);
}

// A center of mass on (or within kMinR_CM of) the platform joint leaves the reference pointing undefined, so the
// configuration is rejected rather than left to produce a meaningless reference at run time.
TEST(ThrustVectoringTest, SetupRejectsCenterOfMassOnTheJoint) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f unitZ{0.0F, 0.0F, 1.0F};
    const Eigen::Vector3f r_MB_B{0.0F, -0.1F, -1.4F};
    const ThrustVectoringThrusterConfiguration thruster{.r_TF_F = zero, .tHat_F = unitZ, .thrust = 10.0F};
    const auto create = [&](const Eigen::Vector3f& r_CB_B) {
        return ThrustVectoringConfig::create(makePlatformConfig(zero, r_MB_B, zero, 1.0F), thruster, r_CB_B);
    };

    // Straddle the threshold: the offset is measured from the joint, not from the body origin.
    EXPECT_THROW((void)create(r_MB_B), fsw::invalid_argument);  // exactly on it
    EXPECT_THROW((void)create(r_MB_B + Eigen::Vector3f(0.5F * kMinR_CM, 0.0F, 0.0F)), fsw::invalid_argument);
    EXPECT_THROW((void)create(r_MB_B + Eigen::Vector3f(kMinR_CM, 0.0F, 0.0F)), fsw::invalid_argument);
    EXPECT_NO_THROW((void)create(r_MB_B + Eigen::Vector3f(2.0F * kMinR_CM, 0.0F, 0.0F)));
    EXPECT_NO_THROW((void)create(r_MB_B + Eigen::Vector3f(0.0F, 0.0F, 1.0F)));

    // The body origin itself is fine as long as it is clear of the joint.
    EXPECT_NO_THROW((void)create(zero));
}

// The configuration getters return the values supplied to create().
TEST(ThrustVectoringTest, ConfigRoundTrip) {
    const Eigen::Vector3f sigma_MB(0.1F, -0.2F, 0.3F);
    const Eigen::Vector3f r_MB_B(0.0F, 0.1F, 1.4F);
    const Eigen::Vector3f r_FM_F(0.0F, 0.0F, -0.1F);
    const Eigen::Vector3f r_TF_F(-0.01F, 0.03F, 0.02F);
    const Eigen::Vector3f r_CB_B(0.05F, 0.02F, 0.1F);
    const ThrustVectoringConfig cfg =
        ThrustVectoringConfig::create(makePlatformConfig(sigma_MB, r_MB_B, r_FM_F, 0.7F),
                                      {.r_TF_F = r_TF_F, .tHat_F = {0.0F, 0.0F, 1.0F}, .thrust = 12.0F},
                                      r_CB_B);
    EXPECT_TRUE(cfg.getPlatformConfiguration().sigma_MB.isApprox(sigma_MB));
    EXPECT_TRUE(cfg.getPlatformConfiguration().r_MB_B.isApprox(r_MB_B));
    EXPECT_TRUE(cfg.getPlatformConfiguration().r_FM_F.isApprox(r_FM_F));
    EXPECT_FLOAT_EQ(cfg.getPlatformConfiguration().thetaMax, 0.7F);
    EXPECT_TRUE(cfg.getThrusterConfiguration().r_TF_F.isApprox(r_TF_F));
    EXPECT_TRUE(cfg.getThrusterConfiguration().tHat_F.isApprox(Eigen::Vector3f(0.0F, 0.0F, 1.0F)));
    EXPECT_FLOAT_EQ(cfg.getThrusterConfiguration().thrust, 12.0F);
    EXPECT_TRUE(cfg.getR_CB_B().isApprox(r_CB_B));
}

// A thrust direction within tolerance of unit length is normalized exactly on construction.
TEST(ThrustVectoringTest, ThrustDirectionNormalized) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const ThrustVectoringConfig cfg =
        ThrustVectoringConfig::create(makePlatformConfig(zero, zero, zero, 1.0F),
                                      {.r_TF_F = zero, .tHat_F = {1.0005F, 0.0F, 0.0F}, .thrust = 10.0F},
                                      Eigen::Vector3f(0.0F, 0.0F, 1.0F));
    EXPECT_NEAR(cfg.getThrusterConfiguration().tHat_F.norm(), 1.0F, 1e-6F);
}

// create() bounds sigma_MB to the principal MRP set: a value with norm > 1 is stored as its
// shadow-set representative (-sigma / |sigma|^2), which has norm <= 1.
TEST(ThrustVectoringTest, SigmaMbSwitchedToShadowSetWhenNormExceedsOne) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f largeSigma{0.8F, 0.6F, 0.6F};  // |sigma|^2 = 1.36 > 1
    ASSERT_GT(largeSigma.norm(), 1.0F) << "Test setup: sigma_MB must exceed the norm-1 boundary";

    const ThrustVectoringConfig cfg =
        ThrustVectoringConfig::create(makePlatformConfig(largeSigma, zero, zero, 1.0F),
                                      {.r_TF_F = zero, .tHat_F = {0.0F, 0.0F, 1.0F}, .thrust = 10.0F},
                                      Eigen::Vector3f(0.0F, 0.0F, 1.0F));
    const Eigen::Vector3f stored = cfg.getPlatformConfiguration().sigma_MB;

    EXPECT_LE(stored.norm(), 1.0F);
    const Eigen::Vector3f expectedShadow = -largeSigma / largeSigma.squaredNorm();
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(stored(i), expectedShadow(i));
    }
}

// A sigma_MB already within the principal set (norm <= 1) is stored unchanged.
TEST(ThrustVectoringTest, SigmaMbWithinBoundStoredUnchanged) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f sigma{0.3F, -0.4F, 0.2F};  // norm < 1
    ASSERT_LE(sigma.norm(), 1.0F);

    const ThrustVectoringConfig cfg =
        ThrustVectoringConfig::create(makePlatformConfig(sigma, zero, zero, 1.0F),
                                      {.r_TF_F = zero, .tHat_F = {0.0F, 0.0F, 1.0F}, .thrust = 10.0F},
                                      Eigen::Vector3f(0.0F, 0.0F, 1.0F));
    const Eigen::Vector3f stored = cfg.getPlatformConfiguration().sigma_MB;
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(stored(i), sigma(i));
    }
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// All outputs are finite for an arbitrary valid configuration and input.
TEST(ThrustVectoringTest, PropertyOutputsFinite) {
    ThrustVectoringAlgorithm alg{makeConfig({0.1F, -0.2F, 0.3F},
                                            {0.0F, -0.1F, -1.4F},
                                            {0.0F, 0.0F, -0.1F},
                                            {0.2F, -0.1F, 0.15F},
                                            {-0.01F, 0.03F, 0.02F},
                                            {1.0F, 1.0F, 10.0F},
                                            10.0F)};
    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    EXPECT_TRUE(out.r_TB_B.allFinite());
    EXPECT_TRUE(out.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(out.thrust));
}

// The reported thrust headings are unit vectors and the reported thrust magnitude matches the input.
TEST(ThrustVectoringTest, PropertyHeadingsAreUnitAndThrustPreserved) {
    ThrustVectoringAlgorithm alg{makeConfig({0.05F, 0.1F, -0.2F},
                                            {0.0F, -0.1F, -1.4F},
                                            {0.0F, 0.0F, -0.1F},
                                            {0.1F, 0.2F, -0.1F},
                                            {-0.01F, 0.03F, 0.02F},
                                            {2.0F, -1.0F, 8.0F},
                                            7.5F)};
    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, 1e-5F);
    EXPECT_NEAR(out.thrust, 7.5F, 1e-5F);
}

// A non-zero requested torque produces finite outputs, on the seeded first cycle and on a later one.
TEST(ThrustVectoringTest, PropertyRequestedTorqueFinite) {
    ThrustVectoringAlgorithm alg{makeConfig({0.0F, 0.0F, 0.0F},
                                            {0.0F, -0.1F, -1.4F},
                                            {0.0F, 0.0F, -0.1F},
                                            {0.1F, 0.05F, 0.1F},
                                            {-0.01F, 0.03F, 0.02F},
                                            {1.0F, 1.0F, 10.0F},
                                            10.0F)};
    const Eigen::Vector3f Lreq_B{0.1F, -0.2F, 0.15F};

    const ThrustVectoringOutput first = alg.update(Lreq_B);
    EXPECT_TRUE(first.r_TB_B.allFinite());
    EXPECT_TRUE(first.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(first.thrust));

    const ThrustVectoringOutput second = alg.update(Lreq_B);
    EXPECT_TRUE(second.r_TB_B.allFinite());
    EXPECT_TRUE(second.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(second.thrust));
}

namespace {
// Run the algorithm on a constant requested torque until the platform-frame torque conversion settles on the
// pointing it produces, and return the body-frame torque the thruster then delivers about the center of mass.
Eigen::Vector3f settledAchievedTorque_B(const ThrustVectoringConfig& cfg,
                                        const Eigen::Vector3f& Lreq_B,
                                        Eigen::Vector3f& tHatSettled_B) {
    constexpr int kSettlingSteps = 20;  // the conversion reuses the previous cycle's pointing, so it needs iterating
    ThrustVectoringAlgorithm alg{cfg};
    ThrustVectoringOutput out{};
    for (int step = 0; step < kSettlingSteps; ++step) {
        out = alg.update(Lreq_B);
    }
    tHatSettled_B = out.tHat_B;
    return (out.r_TB_B - cfg.getR_CB_B()).cross(out.thrust * out.tHat_B);
}
}  // namespace

// The platform points the thruster so it delivers the requested body-frame torque about the center of mass, up to
// the component along the thrust, which no thruster force can produce.
TEST(ThrustVectoringTest, AchievesRequestedTorque) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();  // sigma_MB == 0 -> M == B
    const Eigen::Vector3f Lreq_B(0.1F, -0.05F, 0.08F);
    const ThrustVectoringConfig cfg = makeConfig(zero,
                                                 {0.0F, -0.1F, -1.4F},
                                                 {0.0F, 0.0F, -0.1F},
                                                 {0.05F, 0.02F, 0.1F},
                                                 {-0.01F, 0.03F, 0.02F},
                                                 {1.0F, 1.0F, 10.0F},
                                                 10.0F);

    Eigen::Vector3f tHat_B = Eigen::Vector3f::Zero();
    const Eigen::Vector3f Lachieved_B = settledAchievedTorque_B(cfg, Lreq_B, tHat_B);

    const Eigen::Vector3f LreqPerp_B = Lreq_B - (tHat_B * tHat_B.dot(Lreq_B));
    EXPECT_LT((Lachieved_B - LreqPerp_B).norm(), kAccuracy);
}

// The request is interpreted in the body frame, so a tilted mount frame does not change the delivered torque: the
// conversion through [MB] and [FM] is exercised end to end.
TEST(ThrustVectoringTest, AchievesRequestedTorqueWithTiltedMountFrame) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(0.087F, 0.175F, 0.0F)));
    const Eigen::Vector3f Lreq_B(0.1F, -0.05F, 0.08F);
    const ThrustVectoringConfig cfg = makeConfig(sigma_MB,
                                                 {0.0F, -0.1F, -1.4F},
                                                 {0.0F, 0.0F, -0.1F},
                                                 {0.05F, 0.02F, 0.1F},
                                                 {-0.01F, 0.03F, 0.02F},
                                                 {1.0F, 1.0F, 10.0F},
                                                 10.0F);

    Eigen::Vector3f tHat_B = Eigen::Vector3f::Zero();
    const Eigen::Vector3f Lachieved_B = settledAchievedTorque_B(cfg, Lreq_B, tHat_B);

    const Eigen::Vector3f LreqPerp_B = Lreq_B - (tHat_B * tHat_B.dot(Lreq_B));
    EXPECT_LT((Lachieved_B - LreqPerp_B).norm(), kAccuracy);
}

// reInitialize() drops the stored pointing, so the next cycle reproduces the seeded first-cycle result exactly.
TEST(ThrustVectoringTest, ReInitializeRestoresFirstCycleBehavior) {
    ThrustVectoringAlgorithm alg{makeConfig({0.0F, 0.0F, 0.0F},
                                            {0.0F, -0.1F, -1.4F},
                                            {0.0F, 0.0F, -0.1F},
                                            {0.05F, 0.02F, 0.1F},
                                            {-0.01F, 0.03F, 0.02F},
                                            {1.0F, 1.0F, 10.0F},
                                            10.0F)};
    const Eigen::Vector3f Lreq_B{0.1F, -0.05F, 0.08F};

    const ThrustVectoringOutput first = alg.update(Lreq_B);
    const ThrustVectoringOutput second = alg.update(Lreq_B);
    ASSERT_GT((second.tHat_B - first.tHat_B).norm(), 0.0F) << "Test setup: the pointing must move between cycles";

    alg.reInitialize();
    const ThrustVectoringOutput afterReset = alg.update(Lreq_B);
    EXPECT_TRUE(afterReset.tHat_B.isApprox(first.tHat_B));
    EXPECT_TRUE(afterReset.r_TB_B.isApprox(first.r_TB_B));
}

// setConfig() swaps the configuration on a running instance: a new center of mass changes the pointing, without
// disturbing the runtime state.
TEST(ThrustVectoringTest, SetConfigAppliesNewCenterOfMass) {
    const Eigen::Vector3f r_MB_B{0.0F, -0.1F, -1.4F};
    const Eigen::Vector3f r_FM_F{0.0F, 0.0F, -0.1F};
    const Eigen::Vector3f rThrust_F{-0.01F, 0.03F, 0.02F};
    const Eigen::Vector3f tHat_F{1.0F, 1.0F, 10.0F};
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

    ThrustVectoringAlgorithm alg{makeConfig(zero, r_MB_B, r_FM_F, {0.05F, 0.02F, 0.1F}, rThrust_F, tHat_F, 10.0F)};
    const ThrustVectoringOutput before = alg.update(zero);

    alg.setConfig(makeConfig(zero, r_MB_B, r_FM_F, {-0.05F, 0.15F, 0.1F}, rThrust_F, tHat_F, 10.0F));
    const ThrustVectoringOutput after = alg.update(zero);

    EXPECT_GT((after.tHat_B - before.tHat_B).norm(), kAccuracy);
}

// A geometry that would require a large deflection is clamped so the thrust direction stays on the cone: the angle
// between the reported thrust heading and its neutral direction equals thetaMax.
TEST(ThrustVectoringTest, ThrustDeflectionClampedToCone) {
    constexpr float thetaMax = 0.5F;
    // M == B, thruster fires along +z through the joint, CM placed far off that axis (large required deflection).
    ThrustVectoringAlgorithm alg{makeConfig({0.0F, 0.0F, 0.0F},
                                            {0.0F, 0.0F, 0.0F},
                                            {0.0F, 0.0F, 0.0F},
                                            {1.0F, 0.0F, 0.1F},
                                            {0.0F, 0.0F, 0.0F},
                                            {0.0F, 0.0F, 1.0F},
                                            5.0F,
                                            thetaMax)};
    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    // Neutral thrust direction in the body frame (M == B, [FM] == I) is +z.
    const Eigen::Vector3f neutral_B(0.0F, 0.0F, 1.0F);
    const float deflection = std::acos(neutral_B.dot(out.tHat_B));
    EXPECT_NEAR(deflection, thetaMax, 1e-4F);
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

// With the center of mass already on the thrust line the platform is left unrotated, so the body-frame thrust
// direction matches the (M == B) platform-frame thrust axis and the net thruster torque vanishes.
TEST(ThrustVectoringTest, EdgeCenterOfMassOnThrustLine) {
    // M == B, thruster fires along +z from the F origin, CM placed straight ahead on that axis.
    ThrustVectoringAlgorithm alg{makeConfig({0.0F, 0.0F, 0.0F},
                                            {0.0F, 0.0F, -1.0F},
                                            {0.0F, 0.0F, 0.0F},
                                            {0.0F, 0.0F, 0.0F},
                                            {0.0F, 0.0F, 0.0F},
                                            {0.0F, 0.0F, 1.0F},
                                            5.0F)};
    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    EXPECT_LT((out.tHat_B - Eigen::Vector3f(0.0F, 0.0F, 1.0F)).norm(), 1e-5F);
    EXPECT_LT(out.r_TB_B.cross(out.thrust * out.tHat_B).norm(), 1e-5F);  // CM at the B origin, so r_TC_B == r_TB_B
}
