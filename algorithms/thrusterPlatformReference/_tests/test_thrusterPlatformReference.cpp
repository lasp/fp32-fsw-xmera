#include "thrusterPlatformReferenceTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <limits>

namespace {
constexpr float kAccuracy = 1e-4F;
}  // namespace

// ---------------------------------------------------------------------------
// Regression tests (thruster aligned with the center of mass when there is no wheel momentum to dump)
// ---------------------------------------------------------------------------

TEST(ThrusterPlatformReferenceTest, RegressionAxisAlignedThrust) {
    regressionTestThrusterPlatformReference({0.0F, 0.0F, 0.0F},      // sigma_MB
                                            {0.0F, -0.1F, -1.4F},    // r_MB_B
                                            {0.0F, 0.0F, -0.1F},     // r_FM_F
                                            {0.05F, 0.02F, 0.1F},    // r_CB_B
                                            {-0.01F, 0.03F, 0.02F},  // rThrust_F
                                            {1.0F, 1.0F, 10.0F},     // tHatThrust_F (normalized in helper)
                                            10.0F,                   // maxThrust
                                            kAccuracy);
}

TEST(ThrusterPlatformReferenceTest, RegressionTiltedMFrame) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(0.087F, 0.175F, 0.0F)));
    regressionTestThrusterPlatformReference(sigma_MB,                // sigma_MB
                                            {0.0F, -0.1F, -1.4F},    // r_MB_B
                                            {0.0F, 0.0F, -0.1F},     // r_FM_F
                                            {0.2F, -0.1F, 0.15F},    // r_CB_B
                                            {-0.01F, 0.03F, 0.02F},  // rThrust_F
                                            {0.0F, 0.0F, 1.0F},      // tHatThrust_F
                                            5.0F,                    // maxThrust
                                            kAccuracy);
}

TEST(ThrusterPlatformReferenceTest, RegressionArbitraryGeometry) {
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(-0.2F, 0.1F, 0.0F)));
    regressionTestThrusterPlatformReference(sigma_MB,                // sigma_MB
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

TEST(ThrusterPlatformReferenceTest, SetupTest) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const ThrusterPlatformReferenceRwArrayConfiguration noRw{};
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();

    // A finite configuration with a positive proportional gain and a valid cone half-angle is accepted.
    EXPECT_NO_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, 0.0F, 1.0F, 1.0F, noRw));

    // Non-finite geometry is rejected.
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(
                     Eigen::Vector3f(nan, 0.0F, 0.0F), zero, zero, 1.0F, 0.0F, 1.0F, 1.0F, noRw),
                 fsw::invalid_argument);

    // A non-positive proportional gain is rejected (momentum dumping is always active).
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 0.0F, 0.0F, 1.0F, 1.0F, noRw),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, -1.0F, 0.0F, 1.0F, 1.0F, noRw),
                 fsw::invalid_argument);

    // A negative integral gain is rejected.
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, -1.0F, 1.0F, 1.0F, noRw),
                 fsw::invalid_argument);

    // A non-positive control period is rejected.
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, noRw),
                 fsw::invalid_argument);

    // A cone half-angle outside the open interval (0, pi) is rejected.
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, 0.0F, 1.0F, 0.0F, noRw),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, 0.0F, 1.0F, 4.0F, noRw),
                 fsw::invalid_argument);

    // Too many reaction wheels is rejected.
    ThrusterPlatformReferenceRwArrayConfiguration tooManyRw{};
    tooManyRw.numRW = static_cast<uint32_t>(kMaxNumRw) + 1U;
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, 0.0F, 1.0F, 1.0F, tooManyRw),
                 fsw::invalid_argument);

    // A non-unit reaction-wheel spin axis is rejected.
    ThrusterPlatformReferenceRwArrayConfiguration nonUnitRw{};
    nonUnitRw.numRW = 1U;
    nonUnitRw.GsMatrix_B.col(0) = Eigen::Vector3f(2.0F, 0.0F, 0.0F);
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, 0.0F, 1.0F, 1.0F, nonUnitRw),
                 fsw::invalid_argument);
}

// A reaction-wheel spin axis within tolerance of unit length is normalized exactly on construction.
TEST(ThrusterPlatformReferenceTest, RwSpinAxisNormalized) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    ThrusterPlatformReferenceRwArrayConfiguration rw{};
    rw.numRW = 1U;
    rw.GsMatrix_B.col(0) = Eigen::Vector3f(1.0005F, 0.0F, 0.0F);
    rw.JsList(0) = 0.01F;
    const ThrusterPlatformReferenceConfig cfg =
        ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, 0.0F, 1.0F, 1.0F, rw);
    EXPECT_NEAR(cfg.getRwConfig().GsMatrix_B.col(0).norm(), 1.0F, 1e-6F);
}

// The configuration getters return the values supplied to create().
TEST(ThrusterPlatformReferenceTest, ConfigRoundTrip) {
    const Eigen::Vector3f sigma_MB(0.1F, -0.2F, 0.3F);
    const Eigen::Vector3f r_MB_B(0.0F, 0.1F, 1.4F);
    const Eigen::Vector3f r_FM_F(0.0F, 0.0F, -0.1F);
    const ThrusterPlatformReferenceConfig cfg = ThrusterPlatformReferenceConfig::create(
        sigma_MB, r_MB_B, r_FM_F, 5.0F, 0.5F, 2.0F, 0.7F, ThrusterPlatformReferenceRwArrayConfiguration{});
    EXPECT_TRUE(cfg.getSigma_MB().isApprox(sigma_MB));
    EXPECT_TRUE(cfg.getR_MB_B().isApprox(r_MB_B));
    EXPECT_TRUE(cfg.getR_FM_F().isApprox(r_FM_F));
    EXPECT_FLOAT_EQ(cfg.getK(), 5.0F);
    EXPECT_FLOAT_EQ(cfg.getKi(), 0.5F);
    EXPECT_FLOAT_EQ(cfg.getControlPeriod(), 2.0F);
    EXPECT_FLOAT_EQ(cfg.getThetaMax(), 0.7F);
}

// create() bounds sigma_MB to the principal MRP set: a value with norm > 1 is stored as its
// shadow-set representative (-sigma / |sigma|^2), which has norm <= 1.
TEST(ThrusterPlatformReferenceTest, SigmaMbSwitchedToShadowSetWhenNormExceedsOne) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f largeSigma{0.8F, 0.6F, 0.6F};  // |sigma|^2 = 1.36 > 1
    ASSERT_GT(largeSigma.norm(), 1.0F) << "Test setup: sigma_MB must exceed the norm-1 boundary";

    const ThrusterPlatformReferenceConfig cfg = ThrusterPlatformReferenceConfig::create(
        largeSigma, zero, zero, 1.0F, 0.0F, 1.0F, 1.0F, ThrusterPlatformReferenceRwArrayConfiguration{});
    const Eigen::Vector3f stored = cfg.getSigma_MB();

    EXPECT_LE(stored.norm(), 1.0F);
    const Eigen::Vector3f expectedShadow = -largeSigma / largeSigma.squaredNorm();
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(stored(i), expectedShadow(i));
    }
}

// A sigma_MB already within the principal set (norm <= 1) is stored unchanged.
TEST(ThrusterPlatformReferenceTest, SigmaMbWithinBoundStoredUnchanged) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f sigma{0.3F, -0.4F, 0.2F};  // norm < 1
    ASSERT_LE(sigma.norm(), 1.0F);

    const ThrusterPlatformReferenceConfig cfg = ThrusterPlatformReferenceConfig::create(
        sigma, zero, zero, 1.0F, 0.0F, 1.0F, 1.0F, ThrusterPlatformReferenceRwArrayConfiguration{});
    const Eigen::Vector3f stored = cfg.getSigma_MB();
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(stored(i), sigma(i));
    }
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// All outputs are finite for an arbitrary valid configuration and input.
TEST(ThrusterPlatformReferenceTest, PropertyOutputsFinite) {
    ThrusterPlatformReferenceAlgorithm alg{
        makeAlignmentConfig({0.1F, -0.2F, 0.3F}, {0.0F, -0.1F, -1.4F}, {0.0F, 0.0F, -0.1F})};
    const ThrusterPlatformReferenceOutput out =
        alg.update(makeInputs({0.2F, -0.1F, 0.15F}, {-0.01F, 0.03F, 0.02F}, {1.0F, 1.0F, 10.0F}, 10.0F));

    EXPECT_TRUE(out.Lcomp_B.allFinite());
    EXPECT_TRUE(out.r_TB_B.allFinite());
    EXPECT_TRUE(out.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(out.thrust));
}

// The reported thrust headings are unit vectors and the reported thrust magnitude matches the input.
TEST(ThrusterPlatformReferenceTest, PropertyHeadingsAreUnitAndThrustPreserved) {
    ThrusterPlatformReferenceAlgorithm alg{
        makeAlignmentConfig({0.05F, 0.1F, -0.2F}, {0.0F, -0.1F, -1.4F}, {0.0F, 0.0F, -0.1F})};
    const ThrusterPlatformReferenceOutput out =
        alg.update(makeInputs({0.1F, 0.2F, -0.1F}, {-0.01F, 0.03F, 0.02F}, {2.0F, -1.0F, 8.0F}, 7.5F));

    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, 1e-5F);
    EXPECT_NEAR(out.thrust, 7.5F, 1e-5F);
}

// The momentum-dumping path (non-zero wheel momentum) produces finite outputs.
TEST(ThrusterPlatformReferenceTest, PropertyMomentumDumpingFinite) {
    ThrusterPlatformReferenceRwArrayConfiguration rw{};
    rw.numRW = 3U;
    rw.GsMatrix_B.col(0) = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
    rw.GsMatrix_B.col(1) = Eigen::Vector3f(0.0F, 1.0F, 0.0F);
    rw.GsMatrix_B.col(2) = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
    rw.JsList(0) = 0.01F;
    rw.JsList(1) = 0.01F;
    rw.JsList(2) = 0.01F;
    ThrusterPlatformReferenceAlgorithm alg{ThrusterPlatformReferenceConfig::create(
        {0.0F, 0.0F, 0.0F}, {0.0F, -0.1F, -1.4F}, {0.0F, 0.0F, -0.1F}, 5.0F, 1.0F, 1.0F, 3.0F, rw)};

    ThrusterPlatformReferenceInputs in =
        makeInputs({0.1F, 0.05F, 0.1F}, {-0.01F, 0.03F, 0.02F}, {1.0F, 1.0F, 10.0F}, 10.0F);
    in.wheelSpeeds(0) = 100.0F;
    in.wheelSpeeds(1) = 100.0F;
    in.wheelSpeeds(2) = 100.0F;

    // advance two steps so the momentum integral accumulates over more than one control period
    alg.update(in);
    const ThrusterPlatformReferenceOutput out = alg.update(in);

    EXPECT_TRUE(out.Lcomp_B.allFinite());
    EXPECT_TRUE(out.r_TB_B.allFinite());
    EXPECT_TRUE(out.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(out.thrust));
}

// The momentum-dumping path points the thruster so it produces the desired thruster torque -(K*hs + Ki*hsInt); the
// reported reaction-wheel torque (out.Lcomp_B) is its opposite, K*hs (Ki = 0 here), projected onto the component
// achievable by the thrust.
TEST(ThrusterPlatformReferenceTest, MomentumDumpingAchievesRequestedTorque) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();  // sigma_MB == 0 -> M == B
    constexpr float K = 1.0F;
    ThrusterPlatformReferenceRwArrayConfiguration rw{};
    rw.numRW = 3U;
    rw.GsMatrix_B.col(0) = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
    rw.GsMatrix_B.col(1) = Eigen::Vector3f(0.0F, 1.0F, 0.0F);
    rw.GsMatrix_B.col(2) = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
    rw.JsList(0) = 0.01F;
    rw.JsList(1) = 0.01F;
    rw.JsList(2) = 0.01F;
    // Ki = 0 so the reported reaction-wheel torque is exactly K * hs (the thruster produces its opposite).
    ThrusterPlatformReferenceAlgorithm alg{ThrusterPlatformReferenceConfig::create(
        zero, {0.0F, -0.1F, -1.4F}, {0.0F, 0.0F, -0.1F}, K, 0.0F, 1.0F, 3.0F, rw)};

    ThrusterPlatformReferenceInputs in =
        makeInputs({0.05F, 0.02F, 0.1F}, {-0.01F, 0.03F, 0.02F}, {1.0F, 1.0F, 10.0F}, 10.0F);
    in.wheelSpeeds(0) = 10.0F;
    in.wheelSpeeds(1) = 10.0F;
    in.wheelSpeeds(2) = 10.0F;
    const ThrusterPlatformReferenceOutput out = alg.update(in);

    // expected reported reaction-wheel torque about the CM (sigma_MB == 0, so B frame); only the component
    // perpendicular to the thrust is achievable
    const Eigen::Vector3f hs_B(0.01F * 10.0F, 0.01F * 10.0F, 0.01F * 10.0F);
    const Eigen::Vector3f Lcomp_B = K * hs_B;
    const Eigen::Vector3f tHat_B = out.tHat_B;
    const Eigen::Vector3f LcompPerp_B = Lcomp_B - (tHat_B * tHat_B.dot(Lcomp_B));
    EXPECT_LT((out.Lcomp_B - LcompPerp_B).norm(), 1e-3F);
}

// A geometry that would require a large deflection is clamped so the thrust direction stays on the cone: the angle
// between the reported thrust heading and its neutral direction equals thetaMax.
TEST(ThrusterPlatformReferenceTest, ThrustDeflectionClampedToCone) {
    constexpr float thetaMax = 0.5F;
    // M == B, thruster fires along +z through the joint, CM placed far off that axis (large required deflection).
    ThrusterPlatformReferenceAlgorithm alg{
        makeAlignmentConfig({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, thetaMax)};
    const ThrusterPlatformReferenceOutput out =
        alg.update(makeInputs({1.0F, 0.0F, 0.1F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 5.0F));

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
TEST(ThrusterPlatformReferenceTest, EdgeCenterOfMassOnThrustLine) {
    // M == B, thruster fires along +z from the F origin, CM placed straight ahead on that axis.
    ThrusterPlatformReferenceAlgorithm alg{
        makeAlignmentConfig({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}, {0.0F, 0.0F, 0.0F})};
    const ThrusterPlatformReferenceOutput out =
        alg.update(makeInputs({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 5.0F));

    EXPECT_LT((out.tHat_B - Eigen::Vector3f(0.0F, 0.0F, 1.0F)).norm(), 1e-5F);
    EXPECT_LT(out.Lcomp_B.norm(), 1e-5F);
}
