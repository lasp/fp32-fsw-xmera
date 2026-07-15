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
    const ThrusterPlatformReferenceRwArrayConfig noRw{};
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
    ThrusterPlatformReferenceRwArrayConfig tooManyRw{};
    tooManyRw.numRW = static_cast<uint32_t>(kMaxNumRw) + 1U;
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 0.0F, 0.0F, -1.0F, -1.0F, true, tooManyRw),
                 fsw::invalid_argument);

    // A non-unit reaction-wheel spin axis is rejected.
    ThrusterPlatformReferenceRwArrayConfig nonUnitRw{};
    nonUnitRw.numRW = 1U;
    nonUnitRw.GsMatrix_B.col(0) = Eigen::Vector3f(2.0F, 0.0F, 0.0F);
    EXPECT_THROW(ThrusterPlatformReferenceConfig::create(zero, zero, zero, 0.0F, 0.0F, -1.0F, -1.0F, true, nonUnitRw),
                 fsw::invalid_argument);
}

// A reaction-wheel spin axis within tolerance of unit length is normalized exactly on construction.
TEST(ThrusterPlatformReferenceTest, RwSpinAxisNormalized) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    ThrusterPlatformReferenceRwArrayConfig rw{};
    rw.numRW = 1U;
    rw.GsMatrix_B.col(0) = Eigen::Vector3f(1.0005F, 0.0F, 0.0F);
    rw.JsList(0) = 0.01F;
    const ThrusterPlatformReferenceConfig cfg =
        ThrusterPlatformReferenceConfig::create(zero, zero, zero, 1.0F, 0.0F, -1.0F, -1.0F, true, rw);
    EXPECT_NEAR(cfg.getRwConfig().GsMatrix_B.col(0).norm(), 1.0F, 1e-6F);
}
