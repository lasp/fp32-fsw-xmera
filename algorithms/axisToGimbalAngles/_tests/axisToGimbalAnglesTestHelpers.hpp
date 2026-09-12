#ifndef F32XMERA_AXIS_TO_GIMBAL_ANGLES_TEST_HELPERS_H
#define F32XMERA_AXIS_TO_GIMBAL_ANGLES_TEST_HELPERS_H

#include "axisToGimbalAnglesAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <numbers>

//! [rad] travel that the tests use when a case does not name one. It is well inside the 90 degree limit, thus the
//! two angles stay well conditioned for every request.
inline constexpr float kDefaultThetaMax = 60.0F * (std::numbers::pi_v<float> / 180.0F);

//! Must agree with kMinPerpendicular in the algorithm. Below this length the plane that holds the request and the
//! neutral axis is not defined, and the algorithm takes an arbitrary perpendicular direction.
inline constexpr double kMinPerpendicular = 1e-3;

// Build a configuration from the mounting orientation and the travel that the tests use.
inline AxisToGimbalAnglesConfig makeConfig(const Eigen::Vector3f& sigma_MB, const float thetaMax = kDefaultThetaMax) {
    return AxisToGimbalAnglesConfig::create(sigma_MB, thetaMax);
}

// The gimbal thrust axis for a pair of angles. This function does not use the algorithm, thus the tests can
// examine the mapping and do not repeat it. T(angle1, angle2) is proportional to [tan(angle2), -tan(angle1), 1].
inline Eigen::Vector3f gimbalAxis_M(const float angle1, const float angle2) {
    return Eigen::Vector3f{
        std::sin(angle2) * std::cos(angle1), -std::cos(angle2) * std::sin(angle1), std::cos(angle2) * std::cos(angle1)}
        .normalized();
}

// The input direction in mount-frame coordinates. The two angles are ratios against the mount +z axis. Thus this
// function does not change the length of the vector.
inline Eigen::Vector3f thrustDir_M(const Eigen::Vector3f& sigma_MB, const Eigen::Vector3f& thrustDirection_B) {
    return mrpToDcm(mrpSwitch(sigma_MB)) * thrustDirection_B;
}

// The input direction in mount-frame coordinates, with unit length. This function uses stableNormalized() and
// not normalized(). normalized() calculates squaredNorm(), which is too small for a very short vector and too
// large for a very long one.
inline Eigen::Vector3f thrustHatUnit_M(const Eigen::Vector3f& sigma_MB, const Eigen::Vector3f& thrustDirection_B) {
    return thrustDir_M(sigma_MB, thrustDirection_B).stableNormalized();
}

// Shows if the input carries a direction at all. A request of zero length, or one with a component that is not a
// number, leaves the gimbal at its neutral position. Every other request gives two angles, because the algorithm
// pulls a request outside the travel back onto the cone.
inline bool hasDirection(const Eigen::Vector3f& sigma_MB, const Eigen::Vector3f& thrustDirection_B) {
    const Eigen::Vector3f unitDirection = thrustHatUnit_M(sigma_MB, thrustDirection_B);
    return unitDirection.allFinite() && !unitDirection.isZero();
}

// Shows if the request is nearly opposite the neutral axis. There the plane that holds the request and the
// neutral axis is not defined, and the algorithm takes an arbitrary perpendicular direction. The float algorithm
// and a double reference can take different directions, thus the tests make no claim about the two angles there.
// The band is much larger than the error of the float rotation.
inline bool isNearlyOppositeTheNeutralAxis(const Eigen::Vector3f& unitDirection_M) {
    const float perpendicular = (unitDirection_M - (Eigen::Vector3f::UnitZ() * unitDirection_M.z())).stableNorm();
    return unitDirection_M.z() < 0.0F && perpendicular < static_cast<float>(10.0 * kMinPerpendicular);
}

// The travel limit in double precision. It must agree with limitDeflection() in the algorithm.
inline Eigen::Vector3d limitDeflectionDouble(const Eigen::Vector3d& direction, const double thetaMax) {
    const double cosThetaMax = std::cos(thetaMax);
    if (direction.z() >= cosThetaMax) {
        return direction;
    }
    const Eigen::Vector3d perpendicular = direction - (Eigen::Vector3d::UnitZ() * direction.z());
    const Eigen::Vector3d perpendicularHat =
        (perpendicular.stableNorm() > kMinPerpendicular) ? perpendicular.stableNormalized() : Eigen::Vector3d::UnitX();
    return (cosThetaMax * Eigen::Vector3d::UnitZ()) + (std::sin(thetaMax) * perpendicularHat);
}

// Double-precision reference for the two gimbal angles. The regression tests compare the float algorithm with
// this reference.
struct GimbalAnglesDouble {
    double angle1;
    double angle2;
};

inline GimbalAnglesDouble referenceUpdate(const Eigen::Vector3d& sigma_MB,
                                          const Eigen::Vector3d& thrustDirection_B,
                                          const double thetaMax) {
    const Eigen::Vector3d direction = (mrpToDcm(mrpSwitch<double>(sigma_MB)) * thrustDirection_B).stableNormalized();
    if (!direction.allFinite() || direction.isZero()) {
        return {0.0, 0.0};
    }
    const Eigen::Vector3d limited = limitDeflectionDouble(direction, thetaMax);
    return {std::atan2(-limited.y(), limited.z()), std::atan2(limited.x(), limited.z())};
}

// The largest difference that is permitted between the float angles and the double reference angles.
//
// The travel limit holds the mount-frame z component at or above cos(thetaMax), thus the two angles are always
// well conditioned and one constant is sufficient. Two effects set its size. A request outside the travel keeps
// only the direction of its perpendicular part, thus the error of the float rotation is divided by the length of
// that part. The change to the shadow set is also a boundary: near a norm of one, the float and the double
// mounting orientation can go to different sides of it, which gives the same rotation with different rounding.
// The measured worst difference is 1.5e-5 for a request outside the travel and 1.2e-6 for one inside it. The
// constant below includes margin above that.
inline constexpr float kAngleTolerance = 5e-5F;

// The travel limit holds both angles inside thetaMax. Each angle is an arctangent of a ratio whose numerator is
// at most sin(thetaMax) and whose denominator is at least cos(thetaMax).
inline void expectWithinTravel(const AxisToGimbalAnglesOutput& out,
                               const float thetaMax,
                               const float accuracy = kAngleTolerance) {
    EXPECT_LE(std::fabs(out.gimbalAngle1), thetaMax + accuracy);
    EXPECT_LE(std::fabs(out.gimbalAngle2), thetaMax + accuracy);
}

// ---------------------------------------------------------------------------
// Regression tests
// ---------------------------------------------------------------------------

// The float algorithm must agree with the double reference. The two angles must also align the gimbal thrust axis
// with the request after the travel limit.
inline void regressionTestAxisToGimbalAngles(const Eigen::Vector3f& sigma_MB,
                                             const Eigen::Vector3f& thrustDirection_B,
                                             const float thetaMax = kDefaultThetaMax) {
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB, thetaMax)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);

    EXPECT_TRUE(std::isfinite(out.gimbalAngle1));
    EXPECT_TRUE(std::isfinite(out.gimbalAngle2));

    if (!hasDirection(sigma_MB, thrustDirection_B)) {
        // The request carries no direction, thus the gimbal stays at its neutral position.
        EXPECT_NEAR(out.gimbalAngle1, 0.0F, 1e-6F);
        EXPECT_NEAR(out.gimbalAngle2, 0.0F, 1e-6F);
        return;
    }

    expectWithinTravel(out, thetaMax);

    const Eigen::Vector3f unitDirection_M = thrustHatUnit_M(sigma_MB, thrustDirection_B);
    if (isNearlyOppositeTheNeutralAxis(unitDirection_M)) {
        // The plane is not defined here, thus only the two conditions above hold.
        return;
    }

    const GimbalAnglesDouble reference =
        referenceUpdate(sigma_MB.cast<double>(), thrustDirection_B.cast<double>(), static_cast<double>(thetaMax));
    EXPECT_NEAR(out.gimbalAngle1, static_cast<float>(reference.angle1), kAngleTolerance);
    EXPECT_NEAR(out.gimbalAngle2, static_cast<float>(reference.angle2), kAngleTolerance);

    // The two angles must rebuild the direction that the travel limit gives.
    const Eigen::Vector3f limited_M =
        limitDeflectionDouble(unitDirection_M.cast<double>(), static_cast<double>(thetaMax)).cast<float>();
    EXPECT_LT((gimbalAxis_M(out.gimbalAngle1, out.gimbalAngle2) - limited_M).norm(), kAngleTolerance);
}

// Regression test for a case that a known pair of angles defines. The helper builds the direction from the two
// angles, changes it to body-frame coordinates, and makes sure that the module gives the same two angles again.
// The body-frame input also makes the module apply the mounting orientation. Both angles must be inside the
// travel, otherwise the limit changes the direction and the module cannot give them again.
inline void regressionTestAxisToGimbalAnglesFromAngles(const Eigen::Vector3f& sigma_MB,
                                                       const float angle1,
                                                       const float angle2,
                                                       const float thetaMax = kDefaultThetaMax) {
    const Eigen::Vector3f thrustDirection_B = mrpToDcm(mrpSwitch(sigma_MB)).transpose() * gimbalAxis_M(angle1, angle2);

    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB, thetaMax)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);

    EXPECT_NEAR(out.gimbalAngle1, angle1, kAngleTolerance);
    EXPECT_NEAR(out.gimbalAngle2, angle2, kAngleTolerance);

    regressionTestAxisToGimbalAngles(sigma_MB, thrustDirection_B, thetaMax);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// Every input gives two usable angles: each angle is finite and stays inside the travel of the mechanism.
inline void propertyOutputIsUsable(const Eigen::Vector3f& sigma_MB,
                                   const Eigen::Vector3f& thrustDirection_B,
                                   const float thetaMax = kDefaultThetaMax) {
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB, thetaMax)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);

    // The neutral position is zero, thus these conditions are true for every input.
    EXPECT_TRUE(std::isfinite(out.gimbalAngle1));
    EXPECT_TRUE(std::isfinite(out.gimbalAngle2));
    expectWithinTravel(out, thetaMax);
}

// The two angles must rebuild the direction that the travel limit gives. A request inside the travel is left
// alone, thus the two angles rebuild the request itself.
inline void propertyDirectionRecovered(const Eigen::Vector3f& sigma_MB,
                                       const Eigen::Vector3f& thrustDirection_B,
                                       const float thetaMax = kDefaultThetaMax) {
    const Eigen::Vector3f unitDirection_M = thrustHatUnit_M(sigma_MB, thrustDirection_B);
    if (!hasDirection(sigma_MB, thrustDirection_B) || isNearlyOppositeTheNeutralAxis(unitDirection_M)) {
        propertyOutputIsUsable(sigma_MB, thrustDirection_B, thetaMax);
        return;
    }

    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB, thetaMax)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);

    const Eigen::Vector3f limited_M =
        limitDeflectionDouble(unitDirection_M.cast<double>(), static_cast<double>(thetaMax)).cast<float>();
    EXPECT_LT((gimbalAxis_M(out.gimbalAngle1, out.gimbalAngle2) - limited_M).norm(), kAngleTolerance);
}

// Both angles are ratios against the mount +z axis. Thus a change of the length of the input direction does not
// change the two angles.
//
// A very small or very large scale is the one exception, and it is a limit of the float type and not of the
// module. Such a scale makes a component of the scaled vector too large for a float, or so small that it becomes
// zero or loses almost all of its bits. The scaled vector then holds a different direction, or no direction, and
// the module correctly gives different angles. The helper skips no input: it examines both vectors with
// propertyOutputIsUsable in that condition.
inline void propertyLengthHasNoEffect(const Eigen::Vector3f& sigma_MB,
                                      const Eigen::Vector3f& thrustDirection_B,
                                      const float scale,
                                      const float thetaMax = kDefaultThetaMax) {
    constexpr float kDirectionKept = 1e-6F;

    const Eigen::Vector3f scaledDirection_B = scale * thrustDirection_B;
    const bool scalingKeptTheDirection =
        hasDirection(sigma_MB, thrustDirection_B) && hasDirection(sigma_MB, scaledDirection_B) &&
        !isNearlyOppositeTheNeutralAxis(thrustHatUnit_M(sigma_MB, thrustDirection_B)) &&
        (thrustHatUnit_M(sigma_MB, scaledDirection_B) - thrustHatUnit_M(sigma_MB, thrustDirection_B)).norm() <
            kDirectionKept;
    if (!scalingKeptTheDirection) {
        propertyOutputIsUsable(sigma_MB, thrustDirection_B, thetaMax);
        propertyOutputIsUsable(sigma_MB, scaledDirection_B, thetaMax);
        return;
    }

    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB, thetaMax)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);
    const AxisToGimbalAnglesOutput scaledOut = alg.update(scaledDirection_B);

    EXPECT_NEAR(scaledOut.gimbalAngle1, out.gimbalAngle1, kAngleTolerance);
    EXPECT_NEAR(scaledOut.gimbalAngle2, out.gimbalAngle2, kAngleTolerance);
}

#endif  // F32XMERA_AXIS_TO_GIMBAL_ANGLES_TEST_HELPERS_H
