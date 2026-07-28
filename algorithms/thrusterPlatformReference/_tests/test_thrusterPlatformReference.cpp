#include "thrusterPlatformReferenceTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <limits>

namespace {
constexpr float kAccuracy = 1e-4F;
}  // namespace

// ---------------------------------------------------------------------------
// Regression tests (thruster aligned with the center of mass for K = 0)
// ---------------------------------------------------------------------------

TEST(ThrusterPlatformReferenceTest, RegressionAxisAlignedThrust) {
    regressionTestThrusterPlatformReference({0.0F, 0.0F, 0.0F},      // sigma_MB
                                            {0.0F, 0.1F, 1.4F},      // r_BM_M
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
                                            {0.0F, 0.1F, 1.4F},      // r_BM_M
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
                                            {0.1F, -0.2F, 0.9F},     // r_BM_M
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
    constexpr float inf = std::numeric_limits<float>::infinity();

    // A finite, non-negative-gain, finite-bound configuration is accepted.
    EXPECT_NO_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 0.0F, 0.0F, -1.0F, -1.0F, false, noRw));

    // Non-finite geometry is rejected.
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(
                     Eigen::Vector3f(nan, 0.0F, 0.0F), zero, zero, 0.0F, 0.0F, -1.0F, -1.0F, false, noRw),
                 fsw::invalid_argument);

    // Negative gains are rejected.
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, -1.0F, 0.0F, -1.0F, -1.0F, false, noRw),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 0.0F, -1.0F, -1.0F, -1.0F, false, noRw),
                 fsw::invalid_argument);

    // Non-finite angle bounds are rejected.
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 0.0F, 0.0F, inf, -1.0F, false, noRw),
                 fsw::invalid_argument);

    // Too many reaction wheels is rejected.
    ThrusterPlatformReferenceRwArrayConfiguration tooManyRw{};
    tooManyRw.numRW = static_cast<uint32_t>(kMaxNumRw) + 1U;
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 0.0F, 0.0F, -1.0F, -1.0F, true, tooManyRw),
                 fsw::invalid_argument);

    // A non-unit reaction-wheel spin axis is rejected.
    ThrusterPlatformReferenceRwArrayConfiguration nonUnitRw{};
    nonUnitRw.numRW = 1U;
    nonUnitRw.GsMatrix_B.col(0) = Eigen::Vector3f(2.0F, 0.0F, 0.0F);
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 0.0F, 0.0F, -1.0F, -1.0F, true, nonUnitRw),
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
        ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, 0.0F, -1.0F, -1.0F, true, rw);
    EXPECT_NEAR(cfg.getRwConfig().GsMatrix_B.col(0).norm(), 1.0F, 1e-6F);
}

// The configuration getters return the values supplied to create().
TEST(ThrusterPlatformReferenceTest, ConfigRoundTrip) {
    const Eigen::Vector3f sigma_MB(0.1F, -0.2F, 0.3F);
    const Eigen::Vector3f r_BM_M(0.0F, 0.1F, 1.4F);
    const Eigen::Vector3f r_FM_F(0.0F, 0.0F, -0.1F);
    const ThrusterPlatformReferenceConfig cfg = ThrusterPlatformReferenceConfig::create(
        sigma_MB, r_BM_M, r_FM_F, 5.0F, 0.5F, 0.2F, 0.3F, false, ThrusterPlatformReferenceRwArrayConfiguration{});
    EXPECT_TRUE(cfg.getSigma_MB().isApprox(sigma_MB));
    EXPECT_TRUE(cfg.getR_BM_M().isApprox(r_BM_M));
    EXPECT_TRUE(cfg.getR_FM_F().isApprox(r_FM_F));
    EXPECT_FLOAT_EQ(cfg.getK(), 5.0F);
    EXPECT_FLOAT_EQ(cfg.getKi(), 0.5F);
    EXPECT_FLOAT_EQ(cfg.getTheta1Max(), 0.2F);
    EXPECT_FLOAT_EQ(cfg.getTheta2Max(), 0.3F);
    EXPECT_FALSE(cfg.getMomentumDumping());
}

// create() bounds sigma_MB to the principal MRP set: a value with norm > 1 is stored as its
// shadow-set representative (-sigma / |sigma|^2), which has norm <= 1.
TEST(ThrusterPlatformReferenceTest, SigmaMbSwitchedToShadowSetWhenNormExceedsOne) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const Eigen::Vector3f largeSigma{0.8F, 0.6F, 0.6F};  // |sigma|^2 = 1.36 > 1
    ASSERT_GT(largeSigma.norm(), 1.0F) << "Test setup: sigma_MB must exceed the norm-1 boundary";

    const ThrusterPlatformReferenceConfig cfg = ThrusterPlatformReferenceConfig::create(
        largeSigma, zero, zero, 0.0F, 0.0F, -1.0F, -1.0F, false, ThrusterPlatformReferenceRwArrayConfiguration{});
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
        sigma, zero, zero, 0.0F, 0.0F, -1.0F, -1.0F, false, ThrusterPlatformReferenceRwArrayConfiguration{});
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
        makeAlignmentConfig({0.1F, -0.2F, 0.3F}, {0.0F, 0.1F, 1.4F}, {0.0F, 0.0F, -0.1F}, -1.0F, -1.0F)};
    const ThrusterPlatformReferenceOutput out =
        alg.update(makeInputs({0.2F, -0.1F, 0.15F}, {-0.01F, 0.03F, 0.02F}, {1.0F, 1.0F, 10.0F}, 10.0F), 0);

    EXPECT_TRUE(std::isfinite(out.theta1));
    EXPECT_TRUE(std::isfinite(out.theta2));
    EXPECT_TRUE(out.Lreq_B.allFinite());
    EXPECT_TRUE(out.r_TB_B.allFinite());
    EXPECT_TRUE(out.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(out.thrust));
}

// The reported thrust headings are unit vectors and the reported thrust magnitude matches the input.
TEST(ThrusterPlatformReferenceTest, PropertyHeadingsAreUnitAndThrustPreserved) {
    ThrusterPlatformReferenceAlgorithm alg{
        makeAlignmentConfig({0.05F, 0.1F, -0.2F}, {0.0F, 0.1F, 1.4F}, {0.0F, 0.0F, -0.1F}, -1.0F, -1.0F)};
    const ThrusterPlatformReferenceOutput out =
        alg.update(makeInputs({0.1F, 0.2F, -0.1F}, {-0.01F, 0.03F, 0.02F}, {2.0F, -1.0F, 8.0F}, 7.5F), 0);

    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, 1e-5F);
    EXPECT_NEAR(out.thrust, 7.5F, 1e-5F);
}

// When angle bounds are set, the reported tip/tilt angles stay within them.
TEST(ThrusterPlatformReferenceTest, PropertyAngleBoundsRespected) {
    constexpr float bound = 0.05F;
    ThrusterPlatformReferenceAlgorithm alg{
        makeAlignmentConfig({0.0F, 0.0F, 0.0F}, {0.0F, 0.5F, 1.4F}, {0.0F, 0.0F, -0.1F}, bound, bound)};
    const ThrusterPlatformReferenceOutput out =
        alg.update(makeInputs({0.4F, 0.3F, 0.1F}, {-0.05F, 0.06F, 0.02F}, {1.0F, 1.0F, 3.0F}, 5.0F), 0);

    EXPECT_LE(std::fabs(out.theta1), bound + 1e-5F);
    EXPECT_LE(std::fabs(out.theta2), bound + 1e-5F);
}

// The momentum-dumping path (K > 0 with a valid RW configuration) produces finite outputs.
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
        {0.0F, 0.0F, 0.0F}, {0.0F, 0.1F, 1.4F}, {0.0F, 0.0F, -0.1F}, 5.0F, 1.0F, -1.0F, -1.0F, true, rw)};

    ThrusterPlatformReferenceInputs in =
        makeInputs({0.1F, 0.05F, 0.1F}, {-0.01F, 0.03F, 0.02F}, {1.0F, 1.0F, 10.0F}, 10.0F);
    in.wheelSpeeds(0) = 100.0F;
    in.wheelSpeeds(1) = 100.0F;
    in.wheelSpeeds(2) = 100.0F;

    // advance two steps so the integral term accumulates a non-zero dt (1 s in nanoseconds)
    constexpr uint64_t stepNs = 1000000000ULL;
    alg.update(in, stepNs);
    const ThrusterPlatformReferenceOutput out = alg.update(in, 2ULL * stepNs);

    EXPECT_TRUE(std::isfinite(out.theta1));
    EXPECT_TRUE(std::isfinite(out.theta2));
    EXPECT_TRUE(out.Lreq_B.allFinite());
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

// With the center of mass already on the thrust line the reference angles are near zero.
TEST(ThrusterPlatformReferenceTest, EdgeCenterOfMassOnThrustLine) {
    // M == B, thruster fires along +z from the F origin, CM placed straight ahead on that axis.
    ThrusterPlatformReferenceAlgorithm alg{
        makeAlignmentConfig({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F}, -1.0F, -1.0F)};
    const ThrusterPlatformReferenceOutput out =
        alg.update(makeInputs({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 5.0F), 0);

    EXPECT_NEAR(out.theta1, 0.0F, 1e-5F);
    EXPECT_NEAR(out.theta2, 0.0F, 1e-5F);
}
