/*
 Fuzz tests for StPlatformAttitudeAlgorithm
 */

#include "stPlatformAttitudeTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>
#include <cmath>
#include <numbers>

namespace {
// The mounting range covers the principal MRP set and more, which create() changes to the principal set.
constexpr float kMrpLimit = 2.0F;

// Keep ‖ω‖ < π so the helper's ω → rate quaternion conversion stays single-valued: ‖ω‖ ≤ √3 · 0.9 ≈ 1.56 rad/s.
constexpr double kRateLimit = 0.9;

// Small rates, where the rate quaternion's scalar part is close to 1.
constexpr double kSmallRateLimit = 1e-4;

// Normalize four components to a unit quaternion (scalar-last), or give the identity for a zero vector.
Eigen::Vector4f toUnitQuaternion(const Eigen::Vector4d& v) {
    const double norm = v.stableNorm();
    if (norm < 1e-12) {
        return {0.0F, 0.0F, 0.0F, 1.0F};
    }
    return (v / norm).cast<float>();
}

// Build q_PN (scalar-last) from a rotation angle about an axis.
Eigen::Vector4f axisAngleToQuaternion(double angle, const Eigen::Vector3d& axis) {
    const Eigen::Vector3d unitAxis = (axis.stableNorm() < 1e-12) ? Eigen::Vector3d::UnitZ() : axis;
    return toScalarLast(prvToEp<double>(angle * unitAxis.stableNormalized()));
}

// q_PN: any unit quaternion (either scalar sign), near identity, or near 180 deg.
auto attitudeDomain() {
    return fuzztest::OneOf(
        fuzztest::Map(toUnitQuaternion, xmera::fuzz::EigenVectorOf<double, 4>(fuzztest::InRange(-1.0, 1.0))),
        fuzztest::Map(axisAngleToQuaternion, fuzztest::InRange(0.0, 0.01), xmera::fuzz::Vector3dInRange(-1.0, 1.0)),
        fuzztest::Map(axisAngleToQuaternion,
                      fuzztest::InRange(std::numbers::pi - 0.01, std::numbers::pi),
                      xmera::fuzz::Vector3dInRange(-1.0, 1.0)));
}

// rateQuaternion_PN: a typical rate, a small rate, any unit rate quaternion (either scalar sign), or no rate.
auto rateDomain() {
    return fuzztest::OneOf(
        fuzztest::Map(omegaToRateQuaternion, xmera::fuzz::Vector3dInRange(-kRateLimit, kRateLimit)),
        fuzztest::Map(omegaToRateQuaternion, xmera::fuzz::Vector3dInRange(-kSmallRateLimit, kSmallRateLimit)),
        fuzztest::Map(toUnitQuaternion, xmera::fuzz::EigenVectorOf<double, 4>(fuzztest::InRange(-1.0, 1.0))),
        fuzztest::Just(Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F}));
}

// sigma_PB: any mounting, principal or shadow set, or no mounting.
auto mountingDomain() {
    return fuzztest::OneOf(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit),
                           fuzztest::Just(Eigen::Vector3f{Eigen::Vector3f::Zero()}));
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

FUZZ_TEST(StPlatformAttitudeFuzz, fuzzStPlatformAttitude)
    .WithDomains(attitudeDomain(),   // q_PN
                 rateDomain(),       // rateQuaternion_PN
                 mountingDomain());  // sigma_PB
