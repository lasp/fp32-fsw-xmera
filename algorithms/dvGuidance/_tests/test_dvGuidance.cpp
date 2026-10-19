#include "dvGuidanceTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(DvGuidanceTest, ReferenceTest) {
    // The preferred 2nd-axis direction is not orthogonal to the delta-V command.
    testDvGuidanceRegression(Eigen::Vector3f{5.0F, 5.0F, 5.0F},   // cmdDv_N
                             Eigen::Vector3f{1.0F, 0.0F, 0.0F});  // rHatPref_N
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
