#include "axisToGimbalAnglesTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <limits>

namespace {
constexpr float kAccuracy = 1e-5F;
constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0F;

// A mount that is rotated about two axes, thus the tests examine the mounting orientation.
Eigen::Vector3f rotatedMount() { return dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(0.087F, 0.175F, 0.0F))); }
}  // namespace

// ---------------------------------------------------------------------------
// Regression tests
// ---------------------------------------------------------------------------

// If the mount frame is aligned with the body frame, the neutral thrust axis needs no gimbal rotation.
TEST(AxisToGimbalAnglesTest, RegressionNeutralDirection) {
    regressionTestAxisToGimbalAnglesFromAngles(Eigen::Vector3f::Zero(), 0.0F, 0.0F);
}

// Each angle moves the thrust axis in one mount plane only. The module gives back the angle that made the
// direction. The two conventions agree on each axis, thus these two cases set the signs and the zero positions.
TEST(AxisToGimbalAnglesTest, RegressionPureFirstAndSecondAngle) {
    regressionTestAxisToGimbalAnglesFromAngles(Eigen::Vector3f::Zero(), 12.0F * kDegToRad, 0.0F);
    regressionTestAxisToGimbalAnglesFromAngles(Eigen::Vector3f::Zero(), 0.0F, -7.5F * kDegToRad);
}

// The module gives back the two angles that made the direction.
TEST(AxisToGimbalAnglesTest, RegressionCombinedAngles) {
    regressionTestAxisToGimbalAnglesFromAngles(Eigen::Vector3f::Zero(), -18.25F * kDegToRad, 9.75F * kDegToRad);
}

// The module applies the mounting orientation first. Thus a rotated mount changes the two angles that it gives
// for the same body-frame direction.
TEST(AxisToGimbalAnglesTest, RegressionRotatedMount) {
    regressionTestAxisToGimbalAnglesFromAngles(rotatedMount(), 6.0F * kDegToRad, -11.0F * kDegToRad);
    regressionTestAxisToGimbalAngles(rotatedMount(), {0.1F, -0.2F, -0.97F});
    regressionTestAxisToGimbalAngles(rotatedMount(), -Eigen::Vector3f::UnitZ());
}

// A large deflection, which the mechanism cannot move to, still gives the correct two angles.
TEST(AxisToGimbalAnglesTest, RegressionLargeDeflection) {
    regressionTestAxisToGimbalAnglesFromAngles(Eigen::Vector3f::Zero(), 60.0F * kDegToRad, -55.0F * kDegToRad);
}

// ---------------------------------------------------------------------------
// Property tests with selected values
// ---------------------------------------------------------------------------

TEST(AxisToGimbalAnglesTest, PropertyOutputIsUsable) {
    propertyOutputIsUsable(rotatedMount(), {0.1F, -0.2F, -0.97F});
    propertyOutputIsUsable(rotatedMount(), Eigen::Vector3f::UnitZ());
    propertyOutputIsUsable(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
}

TEST(AxisToGimbalAnglesTest, PropertyDirectionRecovered) {
    propertyDirectionRecovered(rotatedMount(), {0.1F, -0.2F, -0.97F});
    propertyDirectionRecovered(Eigen::Vector3f::Zero(), -Eigen::Vector3f::UnitZ());
}

TEST(AxisToGimbalAnglesTest, PropertyLengthHasNoEffect) {
    propertyLengthHasNoEffect(rotatedMount(), {0.1F, -0.2F, -0.97F}, 137.0F);
    propertyLengthHasNoEffect(rotatedMount(), {0.1F, -0.2F, -0.97F}, 1e-3F);
}

// ---------------------------------------------------------------------------
// Edge case tests
// ---------------------------------------------------------------------------

// The module gives two plane angles, not a sequential Euler pair. The two conventions agree when one angle is
// zero. For all other directions they are different, thus this direction separates them.
TEST(AxisToGimbalAnglesTest, PlaneAnglesAreNotSequentialEulerAngles) {
    // Build a direction with a known reading in each convention. The -x component is sin(eulerAngle), thus an
    // arcsine gives eulerAngle. The remaining length is divided by planeAngle1 in the y-z plane.
    constexpr float eulerAngle = 27.0F * kDegToRad;
    constexpr float planeAngle1 = 18.5F * kDegToRad;
    const Eigen::Vector3f request_M{-std::sin(eulerAngle),
                                    std::cos(eulerAngle) * std::sin(planeAngle1),
                                    -std::cos(eulerAngle) * std::cos(planeAngle1)};

    const AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero())};
    const AxisToGimbalAnglesOutput out = alg.update(request_M);

    // The two conventions agree on the first angle.
    EXPECT_NEAR(out.gimbalAngle1, planeAngle1, kAccuracy);

    // They are different on the second angle, where tan(plane) = tan(euler) / cos(planeAngle1).
    const float planeAngle2 = std::atan(std::tan(eulerAngle) / std::cos(planeAngle1));
    EXPECT_NEAR(out.gimbalAngle2, planeAngle2, kAccuracy);

    // Make sure that this direction separates the two conventions by more than the test accuracy.
    EXPECT_GT(planeAngle2 - eulerAngle, 1.0F * kDegToRad);
}

// At a deflection of 90 degrees the two angles become very large. The module gives the home position and does
// not give an angle at its limit.
TEST(AxisToGimbalAnglesTest, NinetyDegreeDeflectionGivesHomePosition) {
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero())};

    for (const float sign : {1.0F, -1.0F}) {
        const AxisToGimbalAnglesOutput out = alg.update(sign * Eigen::Vector3f::UnitX());
        EXPECT_NEAR(out.gimbalAngle1, 0.0F, kAccuracy);
        EXPECT_NEAR(out.gimbalAngle2, 0.0F, kAccuracy);
    }
}

// The gimbal cannot move to a direction behind the mount. The module gives the home position.
TEST(AxisToGimbalAnglesTest, DirectionBehindTheMountGivesHomePosition) {
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero())};
    const AxisToGimbalAnglesOutput out = alg.update(Eigen::Vector3f::UnitZ());

    EXPECT_NEAR(out.gimbalAngle1, 0.0F, kAccuracy);
    EXPECT_NEAR(out.gimbalAngle2, 0.0F, kAccuracy);
}

// A direction on the boundary of the half-space has a deflection of exactly 90 degrees. Its z component in
// mount-frame coordinates is zero, and the error of the float rotation is larger than that component. Thus the
// module can give the home position or two angles at the edge of the range, and both answers are correct. A
// fuzz test found this input.
TEST(AxisToGimbalAnglesTest, DirectionOnTheHalfSpaceBoundaryGivesUsableAngles) {
    const Eigen::Vector3f sigma_MB{-1.0F, 1.0F, 1.0F};
    const Eigen::Vector3f direction_B{0.0F, -0.0482057929F, 1.0F};

    // The direction is on the boundary: its mount-frame z component is smaller than the rotation error.
    ASSERT_LT(std::fabs(thrustHatUnit_M(sigma_MB, direction_B).z()), 1e-6F);

    propertyOutputIsUsable(sigma_MB, direction_B);
    regressionTestAxisToGimbalAngles(sigma_MB, direction_B);
}

// A direction vector of zero length has a zero z component. Thus it fails the same test.
TEST(AxisToGimbalAnglesTest, ZeroDirectionGivesHomePosition) {
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero())};
    const AxisToGimbalAnglesOutput out = alg.update(Eigen::Vector3f::Zero());

    EXPECT_NEAR(out.gimbalAngle1, 0.0F, kAccuracy);
    EXPECT_NEAR(out.gimbalAngle2, 0.0F, kAccuracy);
}

// The module cannot use a direction vector that contains a value that is not a number.
TEST(AxisToGimbalAnglesTest, NonFiniteDirectionGivesHomePosition) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero())};

    EXPECT_NEAR(alg.update({nan, 0.0F, -1.0F}).gimbalAngle1, 0.0F, kAccuracy);
    EXPECT_NEAR(alg.update({0.0F, 0.0F, nan}).gimbalAngle2, 0.0F, kAccuracy);
}

// ---------------------------------------------------------------------------
// Setup tests
// ---------------------------------------------------------------------------

TEST(AxisToGimbalAnglesTest, SetupTest) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();

    // The configuration accepts a finite mounting orientation.
    EXPECT_NO_THROW((void)makeConfig(Eigen::Vector3f::Zero()));
    EXPECT_NO_THROW((void)makeConfig({0.1F, -0.2F, 0.05F}));

    // The configuration rejects a mounting orientation that is not finite.
    EXPECT_THROW((void)makeConfig({nan, 0.0F, 0.0F}), fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig({0.0F, inf, 0.0F}), fsw::invalid_argument);

    // An MRP with a norm of more than one gives the same rotation. The module stores the shadow set.
    const Eigen::Vector3f shadow{0.0F, 0.0F, 2.0F};
    EXPECT_LE(makeConfig(shadow).getSigma_MB().norm(), 1.0F);
    EXPECT_TRUE(mrpToDcm(makeConfig(shadow).getSigma_MB()).isApprox(mrpToDcm(shadow), 1e-4F));
}

// setConfig() calculates the mounting orientation again. Thus the module uses the new mount for the same input.
TEST(AxisToGimbalAnglesTest, SetConfigAppliesNewMounting) {
    AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero())};

    constexpr float angle2 = 20.0F * kDegToRad;
    const Eigen::Vector3f sigma_MB = dcmToMrp(eulerAngles123ToDcm(Eigen::Vector3f(0.0F, -angle2, 0.0F)));
    alg.setConfig(makeConfig(sigma_MB));

    const AxisToGimbalAnglesOutput out = alg.update(-Eigen::Vector3f::UnitZ());
    EXPECT_NEAR(out.gimbalAngle1, 0.0F, kAccuracy);
    EXPECT_NEAR(out.gimbalAngle2, angle2, kAccuracy);
}
