#include "dvGuidanceTestHelpers.hpp"
#include <gtest/gtest.h>
#include <limits>

TEST(DvGuidanceTest, ReferenceTest) {
    // The preferred 2nd-axis direction is not orthogonal to the delta-V command.
    testDvGuidanceRegression(Eigen::Vector3f{5.0F, 5.0F, 5.0F},   // cmdDv_N
                             Eigen::Vector3f{1.0F, 0.0F, 0.0F});  // rHatPref_N
}

TEST(DvGuidanceTest, ReferenceTestRHatPrefPerpendicularToDeltaV) {
    // The preferred 2nd-axis direction is orthogonal to the delta-V command, so it becomes the 2nd axis.
    testDvGuidanceRegression(Eigen::Vector3f{0.0F, 0.0F, 3.0F},   // cmdDv_N
                             Eigen::Vector3f{0.0F, 1.0F, 0.0F});  // rHatPref_N
}

TEST(DvGuidanceTest, SigmaNormBounded) {
    // A delta-V command along -z with the 2nd axis preferred along -y gives a 180 deg rotation about x,
    // the MRP shadow-set boundary: |sigma_RN| is expected to remain <= 1 + tolerance.
    propertySigmaNormBounded(Eigen::Vector3f{0.0F, 0.0F, -1.0F}, Eigen::Vector3f{0.0F, -1.0F, 0.0F});
}

TEST(DvGuidanceTest, ZeroDeltaVCommand) {
    // A zero delta-V command does not define a burn direction, so the configuration is rejected.
    testDvGuidanceInvalidConfigRejected(Eigen::Vector3f{0.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
}

TEST(DvGuidanceTest, RHatPrefParallelToDeltaV) {
    // rHatPref_N parallel to cmdDv_N collapses the cross product, so the configuration is rejected.
    const Eigen::Vector3f cmdDv_N{1.0F, 2.0F, -3.0F};
    testDvGuidanceInvalidConfigRejected(cmdDv_N, cmdDv_N.stableNormalized());
}

TEST(DvGuidanceTest, ZeroRHatPref) {
    // A zero rHatPref_N is not a unit direction, so the configuration is rejected.
    testDvGuidanceInvalidConfigRejected(Eigen::Vector3f{1.0F, 2.0F, -3.0F}, Eigen::Vector3f{0.0F, 0.0F, 0.0F});
}

TEST(DvGuidanceTest, RHatPrefAntiParallelToDeltaV) {
    // rHatPref_N antiparallel to cmdDv_N collapses the cross product, so the configuration is rejected.
    const Eigen::Vector3f cmdDv_N{1.0F, 2.0F, -3.0F};
    testDvGuidanceInvalidConfigRejected(cmdDv_N, -cmdDv_N.stableNormalized());
}

TEST(DvGuidanceTest, RHatPrefBelowMinCross) {
    // rHatPref_N tilted from cmdDv_N by half of kMinCross falls below kMinCross, so the configuration is rejected.
    testDvGuidanceInvalidConfigRejected(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                        Eigen::Vector3f{1.0F, 0.5F * DvGuidanceConfig::kMinCross, 0.0F});
}

TEST(DvGuidanceTest, DeltaVNormBoundary) {
    // cmdDv_N norm is tested at kMinNorm and immediately below it:
    // equality is expected to be accepted, while the value below is rejected.
    testDvGuidanceDeltaVNormBoundary(Eigen::Vector3f{0.0F, 1.0F, 0.0F});
}

TEST(DvGuidanceTest, OutputIsFinite) {
    // A valid configuration is provided:
    // all sigma_RN, omega_RN_N, and domega_RN_N components are expected to remain finite.
    propertyOutputIsFinite(Eigen::Vector3f{2.0F, -1.0F, 4.0F}, Eigen::Vector3f{0.0F, 0.0F, 1.0F});
}

TEST(DvGuidanceTest, SetupTest) {
    // Valid config should not throw
    EXPECT_NO_THROW(DvGuidanceConfig::create(Eigen::Vector3f::UnitZ(), Eigen::Vector3f::UnitY()));

    // Non-finite cmdDv_N or rHatPref_N should throw
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector3f cmdDv_N{1.0F, 2.0F, -3.0F};
    EXPECT_THROW(DvGuidanceConfig::create(Eigen::Vector3f{nan, 0.0F, 0.0F}, Eigen::Vector3f::UnitY()),
                 fsw::invalid_argument);
    EXPECT_THROW(DvGuidanceConfig::create(cmdDv_N, Eigen::Vector3f{nan, 0.0F, 0.0F}), fsw::invalid_argument);

    // A non-unit rHatPref_N should throw
    EXPECT_THROW(DvGuidanceConfig::create(cmdDv_N, Eigen::Vector3f{0.0F, 3.0F, 0.0F}), fsw::invalid_argument);

    // Config round-trip: cmdDv_N is stored as given, rHatPref_N is stored normalized
    const Eigen::Vector3f rHatPref_N{0.0F, 1.0005F, 0.0F};
    const auto config = DvGuidanceConfig::create(cmdDv_N, rHatPref_N);
    EXPECT_EQ(config.getCmdDv(), cmdDv_N);
    EXPECT_TRUE(config.getRHatPref().isApprox(Eigen::Vector3f::UnitY(), 1e-7F));

    // Static validators
    EXPECT_TRUE(DvGuidanceConfig::isValidCmdDv(cmdDv_N));
    EXPECT_FALSE(DvGuidanceConfig::isValidCmdDv(Eigen::Vector3f::Zero()));
    EXPECT_TRUE(DvGuidanceConfig::isValidRHatPref(cmdDv_N, rHatPref_N));
    EXPECT_FALSE(DvGuidanceConfig::isValidRHatPref(cmdDv_N, Eigen::Vector3f::Zero()));
    EXPECT_FALSE(DvGuidanceConfig::isValidRHatPref(cmdDv_N, Eigen::Vector3f{0.0F, 3.0F, 0.0F}));
}

// setConfig() installs the new configuration. Thus the module uses the new delta-V command.
TEST(DvGuidanceTest, SetConfigAppliesNewDeltaV) {
    const Eigen::Vector3f cmdDv_N = Eigen::Vector3f::UnitZ();
    const Eigen::Vector3f rHatPref_N = Eigen::Vector3f::UnitY();
    DvGuidanceAlgorithm alg{DvGuidanceConfig::create(cmdDv_N, rHatPref_N)};
    EXPECT_TRUE(alg.update().sigma_RN.isZero());

    const Eigen::Vector3f newCmdDv_N = Eigen::Vector3f::UnitX();
    alg.setConfig(DvGuidanceConfig::create(newCmdDv_N, rHatPref_N));
    const Eigen::Matrix3f dcm_RN = mrpToDcm<float>(alg.update().sigma_RN);
    EXPECT_TRUE(dcm_RN.row(2).transpose().isApprox(newCmdDv_N, 1e-5F));
    EXPECT_TRUE(dcm_RN.row(1).transpose().isApprox(rHatPref_N, 1e-5F));
}

TEST(DvGuidanceTest, ThirdAxisAlignedWithDeltaV) {
    // The burn frame's 3rd axis r3 is expected to point along the commanded delta-V direction.
    propertyThirdAxisAlignedWithDeltaV(Eigen::Vector3f{2.0F, -1.0F, 4.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
}

TEST(DvGuidanceTest, SecondAxisTowardRHatPref) {
    // The burn frame's 2nd axis r2 is expected to point toward the preferred direction (r2 . rHatPref > 0).
    propertySecondAxisTowardRHatPref(Eigen::Vector3f{2.0F, -1.0F, 4.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
}
