/*
 Fuzz tests for StPlatformAttitudeAlgorithm
 */

#include "stPlatformAttitudeTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

namespace {
// Normalize four components to a unit quaternion (scalar-last), or give the identity for a zero vector.
Eigen::Vector4f toUnitQuaternion(const Eigen::Vector4d& v) {
    const double norm = v.stableNorm();
    if (norm < 1e-12) {
        return {0.0F, 0.0F, 0.0F, 1.0F};
    }
    return (v / norm).cast<float>();
}
}  // namespace

/*! @brief Fuzz the whole algorithm: attitude, rate and mounting together */
void fuzzStPlatformAttitude(const Eigen::Vector4f& q_PN,
                            const Eigen::Vector4f& rateQuaternion_PN,
                            const Eigen::Vector3f& sigma_PB) {
    EXPECT_NO_THROW(testStPlatformAttitudeRegression(q_PN, rateQuaternion_PN, sigma_PB));
    propertyOutputIsFinite(q_PN, rateQuaternion_PN, sigma_PB);
    propertySigmaNormBounded(q_PN, rateQuaternion_PN, sigma_PB);
}

// q_PN and rateQuaternion_PN: any unit quaternion, either scalar sign (rates up to 2π rad/s).
// sigma_PB: principal and shadow MRP sets; create() changes a shadow set to the principal set.
FUZZ_TEST(StPlatformAttitudeFuzz, fuzzStPlatformAttitude)
    .WithDomains(fuzztest::Map(toUnitQuaternion, xmera::fuzz::EigenVectorOf<double, 4>(fuzztest::InRange(-1.0, 1.0))),
                 fuzztest::Map(toUnitQuaternion, xmera::fuzz::EigenVectorOf<double, 4>(fuzztest::InRange(-1.0, 1.0))),
                 xmera::fuzz::Vector3fInRange(-2.0F, 2.0F));
