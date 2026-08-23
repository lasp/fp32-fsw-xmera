#include "thrustVectoringTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <limits>

namespace {
constexpr float kAccuracy = 1e-4F;
}  // namespace

// ---------------------------------------------------------------------------
// Regression tests (thruster aligned with the center of mass when there is no wheel momentum to dump)
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
    const ThrustVectoringRwArrayConfiguration noRw{};
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();

    // A finite configuration with a positive proportional gain and a valid cone half-angle is accepted.
    EXPECT_NO_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F, noRw));

    // Non-finite geometry is rejected.
    EXPECT_THROW(
        ThrustVectoringConfig::create(Eigen::Vector3f(nan, 0.0F, 0.0F), zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F, noRw),
        fsw::invalid_argument);

    // A non-positive proportional gain is rejected (momentum dumping is always active).
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, noRw),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, -1.0F, 0.0F, 0.0F, 1.0F, 1.0F, noRw),
                 fsw::invalid_argument);

    // A negative integral gain is rejected.
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, -1.0F, 0.0F, 1.0F, 1.0F, noRw),
                 fsw::invalid_argument);

    // A negative momentum-integral limit is rejected, as is a zero limit while the integral term is active.
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 0.0F, -1.0F, 1.0F, 1.0F, noRw),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 1.0F, 0.0F, 1.0F, 1.0F, noRw),
                 fsw::invalid_argument);
    EXPECT_NO_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, noRw));

    // A non-positive control period is rejected.
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, noRw),
                 fsw::invalid_argument);

    // A cone half-angle outside the open interval (0, pi) is rejected.
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, noRw),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, 4.0F, noRw),
                 fsw::invalid_argument);

    // Too many reaction wheels is rejected.
    ThrustVectoringRwArrayConfiguration tooManyRw{};
    tooManyRw.numRW = static_cast<uint32_t>(kMaxNumRw) + 1U;
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F, tooManyRw),
                 fsw::invalid_argument);

    // A non-unit reaction-wheel spin axis is rejected.
    ThrustVectoringRwArrayConfiguration nonUnitRw{};
    nonUnitRw.numRW = 1U;
    nonUnitRw.GsMatrix_B.col(0) = Eigen::Vector3f(2.0F, 0.0F, 0.0F);
    EXPECT_THROW(ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F, nonUnitRw),
                 fsw::invalid_argument);
}

// A reaction-wheel spin axis within tolerance of unit length is normalized exactly on construction.
TEST(ThrustVectoringTest, RwSpinAxisNormalized) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    ThrustVectoringRwArrayConfiguration rw{};
    rw.numRW = 1U;
    rw.GsMatrix_B.col(0) = Eigen::Vector3f(1.0005F, 0.0F, 0.0F);
    rw.JsList(0) = 0.01F;
    const ThrustVectoringConfig cfg = ThrustVectoringConfig::create(zero, zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F, rw);
    EXPECT_NEAR(cfg.getRwConfig().GsMatrix_B.col(0).norm(), 1.0F, 1e-6F);
}

// The configuration getters return the values supplied to create().
TEST(ThrustVectoringTest, ConfigRoundTrip) {
    const Eigen::Vector3f sigma_MB(0.1F, -0.2F, 0.3F);
    const Eigen::Vector3f r_MB_B(0.0F, 0.1F, 1.4F);
    const Eigen::Vector3f r_FM_F(0.0F, 0.0F, -0.1F);
    const ThrustVectoringConfig cfg = ThrustVectoringConfig::create(
        sigma_MB, r_MB_B, r_FM_F, 5.0F, 0.5F, 3.0F, 2.0F, 0.7F, ThrustVectoringRwArrayConfiguration{});
    EXPECT_TRUE(cfg.getSigma_MB().isApprox(sigma_MB));
    EXPECT_TRUE(cfg.getR_MB_B().isApprox(r_MB_B));
    EXPECT_TRUE(cfg.getR_FM_F().isApprox(r_FM_F));
    EXPECT_FLOAT_EQ(cfg.getK(), 5.0F);
    EXPECT_FLOAT_EQ(cfg.getKi(), 0.5F);
    EXPECT_FLOAT_EQ(cfg.getIntegralLimit(), 3.0F);
    EXPECT_FLOAT_EQ(cfg.getControlPeriod(), 2.0F);
    EXPECT_FLOAT_EQ(cfg.getThetaMax(), 0.7F);
}

// create() bounds sigma_MB to the principal MRP set: a value with norm > 1 is stored as its
// shadow-set representative (-sigma / |sigma|^2), which has norm <= 1.
TEST(ThrustVectoringTest, SigmaMbSwitchedToShadowSetWhenNormExceedsOne) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f largeSigma{0.8F, 0.6F, 0.6F};  // |sigma|^2 = 1.36 > 1
    ASSERT_GT(largeSigma.norm(), 1.0F) << "Test setup: sigma_MB must exceed the norm-1 boundary";

    const ThrustVectoringConfig cfg = ThrustVectoringConfig::create(
        largeSigma, zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F, ThrustVectoringRwArrayConfiguration{});
    const Eigen::Vector3f stored = cfg.getSigma_MB();

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

    const ThrustVectoringConfig cfg = ThrustVectoringConfig::create(
        sigma, zero, zero, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F, ThrustVectoringRwArrayConfiguration{});
    const Eigen::Vector3f stored = cfg.getSigma_MB();
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(stored(i), sigma(i));
    }
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// All outputs are finite for an arbitrary valid configuration and input.
TEST(ThrustVectoringTest, PropertyOutputsFinite) {
    ThrustVectoringAlgorithm alg{makeAlignmentConfig({0.1F, -0.2F, 0.3F}, {0.0F, -0.1F, -1.4F}, {0.0F, 0.0F, -0.1F})};
    const ThrustVectoringOutput out =
        alg.update(makeInputs({0.2F, -0.1F, 0.15F}, {-0.01F, 0.03F, 0.02F}, {1.0F, 1.0F, 10.0F}, 10.0F));

    EXPECT_TRUE(out.r_TB_B.allFinite());
    EXPECT_TRUE(out.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(out.thrust));
}

// The reported thrust headings are unit vectors and the reported thrust magnitude matches the input.
TEST(ThrustVectoringTest, PropertyHeadingsAreUnitAndThrustPreserved) {
    ThrustVectoringAlgorithm alg{makeAlignmentConfig({0.05F, 0.1F, -0.2F}, {0.0F, -0.1F, -1.4F}, {0.0F, 0.0F, -0.1F})};
    const ThrustVectoringOutput out =
        alg.update(makeInputs({0.1F, 0.2F, -0.1F}, {-0.01F, 0.03F, 0.02F}, {2.0F, -1.0F, 8.0F}, 7.5F));

    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, 1e-5F);
    EXPECT_NEAR(out.thrust, 7.5F, 1e-5F);
}

// The momentum-dumping path (non-zero wheel momentum) produces finite outputs.
TEST(ThrustVectoringTest, PropertyMomentumDumpingFinite) {
    ThrustVectoringRwArrayConfiguration rw{};
    rw.numRW = 3U;
    rw.GsMatrix_B.col(0) = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
    rw.GsMatrix_B.col(1) = Eigen::Vector3f(0.0F, 1.0F, 0.0F);
    rw.GsMatrix_B.col(2) = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
    rw.JsList(0) = 0.01F;
    rw.JsList(1) = 0.01F;
    rw.JsList(2) = 0.01F;
    ThrustVectoringAlgorithm alg{ThrustVectoringConfig::create(
        {0.0F, 0.0F, 0.0F}, {0.0F, -0.1F, -1.4F}, {0.0F, 0.0F, -0.1F}, 5.0F, 1.0F, 10.0F, 1.0F, 3.0F, rw)};

    ThrustVectoringInputs in = makeInputs({0.1F, 0.05F, 0.1F}, {-0.01F, 0.03F, 0.02F}, {1.0F, 1.0F, 10.0F}, 10.0F);
    in.wheelSpeeds(0) = 100.0F;
    in.wheelSpeeds(1) = 100.0F;
    in.wheelSpeeds(2) = 100.0F;

    // advance two steps so the momentum integral accumulates over more than one control period
    alg.update(in);
    const ThrustVectoringOutput out = alg.update(in);

    EXPECT_TRUE(out.r_TB_B.allFinite());
    EXPECT_TRUE(out.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(out.thrust));
}

// The momentum-dumping path points the thruster so it produces the desired thruster torque -(K*hs + Ki*hsInt),
// projected onto the component achievable by the thrust.
TEST(ThrustVectoringTest, MomentumDumpingAchievesRequestedTorque) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();  // sigma_MB == 0 -> M == B
    constexpr float K = 1.0F;
    ThrustVectoringRwArrayConfiguration rw{};
    rw.numRW = 3U;
    rw.GsMatrix_B.col(0) = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
    rw.GsMatrix_B.col(1) = Eigen::Vector3f(0.0F, 1.0F, 0.0F);
    rw.GsMatrix_B.col(2) = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
    rw.JsList(0) = 0.01F;
    rw.JsList(1) = 0.01F;
    rw.JsList(2) = 0.01F;
    // Ki = 0 so the requested thruster torque is exactly -K * hs.
    ThrustVectoringAlgorithm alg{
        ThrustVectoringConfig::create(zero, {0.0F, -0.1F, -1.4F}, {0.0F, 0.0F, -0.1F}, K, 0.0F, 0.0F, 1.0F, 3.0F, rw)};

    const Eigen::Vector3f r_CB_B(0.05F, 0.02F, 0.1F);
    ThrustVectoringInputs in = makeInputs(r_CB_B, {-0.01F, 0.03F, 0.02F}, {1.0F, 1.0F, 10.0F}, 10.0F);
    in.wheelSpeeds(0) = 10.0F;
    in.wheelSpeeds(1) = 10.0F;
    in.wheelSpeeds(2) = 10.0F;
    const ThrustVectoringOutput out = alg.update(in);

    // requested thruster torque about the CM (sigma_MB == 0, so B frame); only the component perpendicular to the
    // thrust is achievable
    const Eigen::Vector3f hs_B(0.01F * 10.0F, 0.01F * 10.0F, 0.01F * 10.0F);
    const Eigen::Vector3f Lreq_B = -K * hs_B;
    const Eigen::Vector3f tHat_B = out.tHat_B;
    const Eigen::Vector3f LreqPerp_B = Lreq_B - (tHat_B * tHat_B.dot(Lreq_B));

    const Eigen::Vector3f Lachieved_B = (out.r_TB_B - r_CB_B).cross(out.thrust * tHat_B);
    EXPECT_LT((Lachieved_B - LreqPerp_B).norm(), 1e-3F);
}

// The momentum integral is clamped to integralLimit, so a sustained wheel momentum cannot wind the integral term up:
// with a limit small enough to suppress it, the achieved torque still matches the proportional-only request after
// many cycles, whereas an effectively unlimited integral drives the torque far away from it.
TEST(ThrustVectoringTest, IntegralLimitBoundsTheIntegralTerm) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();  // sigma_MB == 0 -> M == B
    constexpr float K = 1.0F;
    constexpr float Ki = 0.1F;
    constexpr int numSteps = 100;
    ThrustVectoringRwArrayConfiguration rw{};
    rw.numRW = 3U;
    rw.GsMatrix_B.col(0) = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
    rw.GsMatrix_B.col(1) = Eigen::Vector3f(0.0F, 1.0F, 0.0F);
    rw.GsMatrix_B.col(2) = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
    rw.JsList(0) = 0.01F;
    rw.JsList(1) = 0.01F;
    rw.JsList(2) = 0.01F;

    const Eigen::Vector3f r_CB_B(0.05F, 0.02F, 0.1F);
    ThrustVectoringInputs in = makeInputs(r_CB_B, {-0.01F, 0.03F, 0.02F}, {1.0F, 1.0F, 10.0F}, 10.0F);
    in.wheelSpeeds(0) = 10.0F;
    in.wheelSpeeds(1) = 10.0F;
    in.wheelSpeeds(2) = 10.0F;

    const auto runToTorque = [&in, &rw](float integralLimit) {
        ThrustVectoringAlgorithm alg{ThrustVectoringConfig::create(
            Eigen::Vector3f::Zero(), {0.0F, -0.1F, -1.4F}, {0.0F, 0.0F, -0.1F}, K, Ki, integralLimit, 1.0F, 3.0F, rw)};
        ThrustVectoringOutput out{};
        for (int step = 0; step < numSteps; ++step) {
            out = alg.update(in);
        }
        return out;
    };

    // proportional-only request (Ki * hsInt suppressed by the tight limit); only its perpendicular part is achievable
    const Eigen::Vector3f hs_B(0.01F * 10.0F, 0.01F * 10.0F, 0.01F * 10.0F);
    const ThrustVectoringOutput clamped = runToTorque(1e-6F);
    const Eigen::Vector3f LreqP_B = -K * hs_B;
    const Eigen::Vector3f LreqPPerp_B = LreqP_B - (clamped.tHat_B * clamped.tHat_B.dot(LreqP_B));
    const Eigen::Vector3f LclampedAchieved_B = (clamped.r_TB_B - r_CB_B).cross(clamped.thrust * clamped.tHat_B);
    EXPECT_LT((LclampedAchieved_B - LreqPPerp_B).norm(), 1e-3F);

    // an effectively unlimited integral winds up over the same run and lands far from the proportional-only torque
    const ThrustVectoringOutput unclamped = runToTorque(1e3F);
    const Eigen::Vector3f LunclampedAchieved_B = (unclamped.r_TB_B - r_CB_B).cross(unclamped.thrust * unclamped.tHat_B);
    EXPECT_GT((LunclampedAchieved_B - LclampedAchieved_B).norm(), 0.1F);
}

// A geometry that would require a large deflection is clamped so the thrust direction stays on the cone: the angle
// between the reported thrust heading and its neutral direction equals thetaMax.
TEST(ThrustVectoringTest, ThrustDeflectionClampedToCone) {
    constexpr float thetaMax = 0.5F;
    // M == B, thruster fires along +z through the joint, CM placed far off that axis (large required deflection).
    ThrustVectoringAlgorithm alg{
        makeAlignmentConfig({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, thetaMax)};
    const ThrustVectoringOutput out =
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
TEST(ThrustVectoringTest, EdgeCenterOfMassOnThrustLine) {
    // M == B, thruster fires along +z from the F origin, CM placed straight ahead on that axis.
    ThrustVectoringAlgorithm alg{makeAlignmentConfig({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}, {0.0F, 0.0F, 0.0F})};
    const ThrustVectoringOutput out =
        alg.update(makeInputs({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 5.0F));

    EXPECT_LT((out.tHat_B - Eigen::Vector3f(0.0F, 0.0F, 1.0F)).norm(), 1e-5F);
    EXPECT_LT(out.r_TB_B.cross(out.thrust * out.tHat_B).norm(), 1e-5F);  // CM at the B origin, so r_TC_B == r_TB_B
}
