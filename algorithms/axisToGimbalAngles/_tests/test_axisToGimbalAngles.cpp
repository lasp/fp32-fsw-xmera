#include "axisToGimbalAnglesTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <limits>
#include <numbers>

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
    regressionTestAxisToGimbalAnglesFromAngles(
        Eigen::Vector3f::Zero(), 60.0F * kDegToRad, -55.0F * kDegToRad, 80.0F * kDegToRad);
}

// ---------------------------------------------------------------------------
// Property tests with selected values
// ---------------------------------------------------------------------------

TEST(AxisToGimbalAnglesTest, PropertyOutputIsUsable) {
    propertyOutputIsUsable(rotatedMount(), {0.1F, -0.2F, -0.97F});
    propertyOutputIsUsable(rotatedMount(), Eigen::Vector3f::UnitZ());
    propertyOutputIsUsable(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
}

TEST(AxisToGimbalAnglesTest, PropertyAchievedDirectionIsNearestReachable) {
    propertyAchievedDirectionIsNearestReachable(rotatedMount(), {0.1F, -0.2F, -0.97F});
    propertyAchievedDirectionIsNearestReachable(rotatedMount(), {0.1F, -0.2F, 0.97F});
    propertyAchievedDirectionIsNearestReachable(Eigen::Vector3f::Zero(), -Eigen::Vector3f::UnitZ());
    propertyAchievedDirectionIsNearestReachable(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
}

TEST(AxisToGimbalAnglesTest, PropertyLimitIsIdempotent) {
    propertyLimitIsIdempotent(rotatedMount(), {0.1F, -0.2F, -0.97F});
    propertyLimitIsIdempotent(rotatedMount(), {0.1F, -0.2F, 0.97F});
    propertyLimitIsIdempotent(Eigen::Vector3f::Zero(), -Eigen::Vector3f::UnitZ());
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
    // Build a direction with a known reading in each convention. The x component is sin(eulerAngle), thus an
    // arcsine gives eulerAngle. The remaining length is divided by planeAngle1 in the y-z plane.
    constexpr float eulerAngle = 27.0F * kDegToRad;
    constexpr float planeAngle1 = 18.5F * kDegToRad;
    const Eigen::Vector3f request_M{std::sin(eulerAngle),
                                    -std::cos(eulerAngle) * std::sin(planeAngle1),
                                    std::cos(eulerAngle) * std::cos(planeAngle1)};

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

// A request outside the travel goes to the edge of the cone, in the plane that the request and the neutral axis
// span. Each expected direction below is calculated by hand from that geometry: the part along the neutral axis
// becomes cos(thetaMax), and the perpendicular part keeps its direction with the length sin(thetaMax).
TEST(AxisToGimbalAnglesTest, DeflectionBeyondTheTravelGoesToTheEdgeOfTheCone) {
    struct Case {
        Eigen::Vector3f request_M;
        float thetaMax;
        Eigen::Vector3f expected_M;
        float angle1;
        float angle2;
    };
    const float halfSqrt2 = std::sqrt(2.0F) / 2.0F;
    // atan(0.5 / (sqrt(2) / 2)) = atan(1 / sqrt(2))
    const float angleFromDiagonal = std::atan(1.0F / std::sqrt(2.0F));
    const float sin20 = std::sin(20.0F * kDegToRad);
    const float cos20 = std::cos(20.0F * kDegToRad);
    const Case cases[] = {
        // 90 degrees along +x and -x with a travel of 30 degrees: [+-sin(30), 0, cos(30)].
        {Eigen::Vector3f::UnitX(), 30.0F * kDegToRad, {0.5F, 0.0F, std::sqrt(3.0F) / 2.0F}, 0.0F, 30.0F * kDegToRad},
        {-Eigen::Vector3f::UnitX(), 30.0F * kDegToRad, {-0.5F, 0.0F, std::sqrt(3.0F) / 2.0F}, 0.0F, -30.0F * kDegToRad},
        // A request of length 5 that points away from the neutral axis, in the y-z plane.
        {{0.0F, -3.0F, -4.0F}, 45.0F * kDegToRad, {0.0F, -halfSqrt2, halfSqrt2}, 45.0F * kDegToRad, 0.0F},
        // The perpendicular part is along the x-y diagonal: [sin(45) / sqrt(2), sin(45) / sqrt(2), cos(45)].
        {{1.0F, 1.0F, -1.0F}, 45.0F * kDegToRad, {0.5F, 0.5F, halfSqrt2}, -angleFromDiagonal, angleFromDiagonal},
        // A request at 22.6 degrees, just outside a travel of 20 degrees. The perpendicular part is [3, 4] / 5.
        {Eigen::Vector3f{3.0F, 4.0F, 12.0F} / 13.0F,
         20.0F * kDegToRad,
         {0.6F * sin20, 0.8F * sin20, cos20},
         std::atan2(-0.8F * sin20, cos20),
         std::atan2(0.6F * sin20, cos20)},
    };

    for (const Case& c : cases) {
        const AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero(), c.thetaMax)};
        const AxisToGimbalAnglesOutput out = alg.update(c.request_M);
        EXPECT_TRUE(out.thrustHat_B.isApprox(c.expected_M, kAccuracy)) << out.thrustHat_B.transpose();
        EXPECT_NEAR(out.gimbalAngle1, c.angle1, kAccuracy);
        EXPECT_NEAR(out.gimbalAngle2, c.angle2, kAccuracy);
    }
}

// The travel limit also applies in mount-frame coordinates when the mount is rotated. The expected body-frame
// direction is the hand-calculated mount-frame direction, rotated back to the body frame.
TEST(AxisToGimbalAnglesTest, DeflectionBeyondTheTravelWithARotatedMount) {
    constexpr float thetaMax = 30.0F * kDegToRad;
    const Eigen::Matrix3f dcm_MB = mrpToDcm(rotatedMount());
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(rotatedMount(), thetaMax)};

    const AxisToGimbalAnglesOutput out = alg.update(dcm_MB.transpose() * Eigen::Vector3f::UnitY());

    const Eigen::Vector3f expected_M{0.0F, 0.5F, std::sqrt(3.0F) / 2.0F};
    EXPECT_TRUE(out.thrustHat_B.isApprox(dcm_MB.transpose() * expected_M, kAccuracy)) << out.thrustHat_B.transpose();
    EXPECT_NEAR(out.gimbalAngle1, -thetaMax, kAccuracy);
    EXPECT_NEAR(out.gimbalAngle2, 0.0F, kAccuracy);
}

// A request inside the travel is not changed, up to the edge of the cone. Just outside the edge, the limited
// direction is nearly the request, thus the limit has no jump at the edge.
TEST(AxisToGimbalAnglesTest, TravelLimitIsContinuousAtTheEdgeOfTheCone) {
    constexpr float thetaMax = 30.0F * kDegToRad;
    constexpr float step = 1e-3F;
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero(), thetaMax)};

    for (const float requested : {thetaMax - step, thetaMax, thetaMax + step}) {
        const Eigen::Vector3f request{std::sin(requested) * 0.6F, std::sin(requested) * -0.8F, std::cos(requested)};
        const AxisToGimbalAnglesOutput out = alg.update(request);
        const Eigen::Vector3f expected =
            (requested <= thetaMax)
                ? request
                : Eigen::Vector3f{std::sin(thetaMax) * 0.6F, std::sin(thetaMax) * -0.8F, std::cos(thetaMax)};
        EXPECT_TRUE(out.thrustHat_B.isApprox(expected, kAccuracy)) << requested;
        EXPECT_LT((out.thrustHat_B - request).norm(), 1.01F * step);
    }
}

// A request exactly opposite the neutral axis leaves no plane to move in. The module takes an arbitrary
// perpendicular direction and still gives a direction on the edge of the cone, not the neutral position.
TEST(AxisToGimbalAnglesTest, DirectionOppositeTheNeutralAxisGoesToTheEdgeOfTheCone) {
    constexpr float thetaMax = 25.0F * kDegToRad;
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero(), thetaMax)};
    const AxisToGimbalAnglesOutput out = alg.update(-Eigen::Vector3f::UnitZ());

    EXPECT_NEAR(std::acos(gimbalAxis_M(out.gimbalAngle1, out.gimbalAngle2).z()), thetaMax, kAccuracy);
    EXPECT_NEAR(std::acos(out.thrustHat_B.z()), thetaMax, kAccuracy);
}

// Nearly opposite the neutral axis, a perpendicular part that is longer than the threshold still sets the plane.
// A shorter one does not, but the direction still goes to the edge of the cone.
TEST(AxisToGimbalAnglesTest, DirectionNearlyOppositeTheNeutralAxis) {
    constexpr float thetaMax = 25.0F * kDegToRad;
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero(), thetaMax)};

    const AxisToGimbalAnglesOutput kept = alg.update({0.0F, 2.0F * static_cast<float>(kMinPerpendicular), -1.0F});
    EXPECT_TRUE(kept.thrustHat_B.isApprox(Eigen::Vector3f{0.0F, std::sin(thetaMax), std::cos(thetaMax)}, kAccuracy))
        << kept.thrustHat_B.transpose();

    const AxisToGimbalAnglesOutput lost = alg.update({0.0F, 0.5F * static_cast<float>(kMinPerpendicular), -1.0F});
    EXPECT_NEAR(std::acos(lost.thrustHat_B.z()), thetaMax, kAccuracy);
    expectWithinTravel(lost, thetaMax);
}

// A direction at a deflection of exactly 90 degrees has a zero z component in mount-frame coordinates. The
// travel limit moves it onto the cone, where the two angles are well conditioned. A fuzz test found this input
// when the module had no travel limit.
TEST(AxisToGimbalAnglesTest, NinetyDegreeDeflectionGivesUsableAngles) {
    const Eigen::Vector3f sigma_MB{-1.0F, 1.0F, 1.0F};
    const Eigen::Vector3f direction_B{0.0F, -0.0482057929F, 1.0F};

    // The deflection is 90 degrees: the mount-frame z component is smaller than the rotation error.
    ASSERT_LT(std::fabs(thrustHatUnit_M(sigma_MB, direction_B).z()), 1e-6F);

    propertyOutputIsUsable(sigma_MB, direction_B);
    regressionTestAxisToGimbalAngles(sigma_MB, direction_B);
}

// A direction vector of zero length carries no direction, thus there is nothing to limit and nothing to point
// at. The gimbal stays at its neutral position and fires along the neutral axis.
TEST(AxisToGimbalAnglesTest, ZeroDirectionGivesHomePosition) {
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(rotatedMount())};
    const AxisToGimbalAnglesOutput out = alg.update(Eigen::Vector3f::Zero());

    EXPECT_NEAR(out.gimbalAngle1, 0.0F, kAccuracy);
    EXPECT_NEAR(out.gimbalAngle2, 0.0F, kAccuracy);
    EXPECT_TRUE(out.thrustHat_B.isApprox(mrpToDcm(rotatedMount()).transpose() * Eigen::Vector3f::UnitZ(), kAccuracy));
}

// The module assumes a finite input and does not examine it. For an input that is not finite, the two angles still
// stay finite and inside the travel. They are not always the home position, and thrustHat_B can contain NaN, thus
// the test makes no claim about the two.
TEST(AxisToGimbalAnglesTest, NonFiniteDirectionGivesFiniteAngles) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();

    for (const Eigen::Vector3f& sigma_MB : {Eigen::Vector3f::Zero().eval(), rotatedMount()}) {
        for (const Eigen::Vector3f& request : {Eigen::Vector3f{nan, 0.0F, 1.0F},
                                               Eigen::Vector3f{0.0F, 0.0F, nan},
                                               Eigen::Vector3f{0.0F, inf, 1.0F},
                                               Eigen::Vector3f{-inf, 0.0F, -1.0F}}) {
            const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB)};
            const AxisToGimbalAnglesOutput out = alg.update(request);
            EXPECT_TRUE(std::isfinite(out.gimbalAngle1));
            EXPECT_TRUE(std::isfinite(out.gimbalAngle2));
            expectWithinTravel(out, kDefaultThetaMax);
        }
    }
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

    // The configuration accepts a travel inside the open interval (0, pi/2).
    EXPECT_NO_THROW((void)makeConfig(Eigen::Vector3f::Zero(), 1.0F * kDegToRad));
    EXPECT_NO_THROW((void)makeConfig(Eigen::Vector3f::Zero(), 89.0F * kDegToRad));

    // Two plane angles cannot describe a deflection of 90 degrees or more, thus the travel must stay below it.
    // A travel of zero or less leaves the gimbal with no movement at all.
    constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
    EXPECT_THROW((void)makeConfig(Eigen::Vector3f::Zero(), halfPi), fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(Eigen::Vector3f::Zero(), 2.0F), fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(Eigen::Vector3f::Zero(), 0.0F), fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(Eigen::Vector3f::Zero(), -0.1F), fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(Eigen::Vector3f::Zero(), nan), fsw::invalid_argument);
    EXPECT_THROW((void)makeConfig(Eigen::Vector3f::Zero(), inf), fsw::invalid_argument);

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

    const AxisToGimbalAnglesOutput out = alg.update(Eigen::Vector3f::UnitZ());
    EXPECT_NEAR(out.gimbalAngle1, 0.0F, kAccuracy);
    EXPECT_NEAR(out.gimbalAngle2, angle2, kAccuracy);
}

// setConfig() calculates the cosine and the sine of the travel again. Thus the module uses the new travel for the
// same input.
TEST(AxisToGimbalAnglesTest, SetConfigAppliesNewTravel) {
    AxisToGimbalAnglesAlgorithm alg{makeConfig(Eigen::Vector3f::Zero(), 60.0F * kDegToRad)};
    EXPECT_NEAR(alg.update(Eigen::Vector3f::UnitX()).gimbalAngle2, 60.0F * kDegToRad, kAccuracy);

    alg.setConfig(makeConfig(Eigen::Vector3f::Zero(), 10.0F * kDegToRad));
    EXPECT_NEAR(alg.update(Eigen::Vector3f::UnitX()).gimbalAngle2, 10.0F * kDegToRad, kAccuracy);
}
