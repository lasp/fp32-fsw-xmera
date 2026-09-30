#ifndef F32XMERA_AXIS_TO_GIMBAL_ANGLES_TEST_HELPERS_H
#define F32XMERA_AXIS_TO_GIMBAL_ANGLES_TEST_HELPERS_H

#include "axisToGimbalAnglesAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <numbers>

//! [rad] travel that the tests use when a case does not name one. It is well inside the 90 degree limit, thus the
//! two angles stay well conditioned for every request.
inline constexpr float kDefaultThetaMax = 60.0F * (std::numbers::pi_v<float> / 180.0F);

//! Must agree with kMinPerpendicular in the algorithm. Below this length the plane that holds the request and the
//! neutral axis is not defined, and the algorithm takes an arbitrary perpendicular direction.
inline constexpr double kMinPerpendicular = 1e-3;

//! [-] largest difference between the float rotation of a unit direction into the mount frame and the double
//! rotation that the helpers use. The measured worst difference is 5.1e-7 across the fuzz domain. The constant
//! includes margin above that.
inline constexpr double kRotationError = 1e-6;

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

// The DCM from the body frame to the mount frame, in double precision.
inline Eigen::Matrix3d dcmMountFromBody(const Eigen::Vector3f& sigma_MB) {
    return mrpToDcm(mrpSwitch<double>(sigma_MB.cast<double>()));
}

// The input direction in mount-frame coordinates, with unit length. This function uses stableNormalized() and
// not normalized(). normalized() calculates squaredNorm(), which is too small for a very short vector and too
// large for a very long one.
inline Eigen::Vector3d thrustHatUnit_M(const Eigen::Vector3f& sigma_MB, const Eigen::Vector3f& thrustDirection_B) {
    return (dcmMountFromBody(sigma_MB) * thrustDirection_B.cast<double>()).stableNormalized();
}

// [rad] angle between two directions. The arctangent of the cross and dot products stays well conditioned at
// every angle, where an arccosine of the dot product does not near zero.
inline double angleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

// [rad] deflection of a direction from the mount +z axis, the neutral thrust axis.
inline double deflection(const Eigen::Vector3d& direction_M) {
    return angleBetween(direction_M, Eigen::Vector3d::UnitZ());
}

// The travel limit in spherical coordinates about the mount +z axis. It does not repeat the construction in the
// algorithm: the limited direction keeps the azimuth of the request and takes the smaller of its polar angle and
// thetaMax.
inline Eigen::Vector3d limitDeflectionReference(const Eigen::Vector3d& unitDirection_M, const double thetaMax) {
    const double azimuth = std::atan2(unitDirection_M.y(), unitDirection_M.x());
    const double polar = std::min(deflection(unitDirection_M), thetaMax);
    return {std::sin(polar) * std::cos(azimuth), std::sin(polar) * std::sin(azimuth), std::cos(polar)};
}

// Double-precision reference for the two gimbal angles. The regression tests compare the float algorithm with
// this reference.
struct GimbalAnglesDouble {
    double angle1;
    double angle2;
};

inline GimbalAnglesDouble referenceUpdate(const Eigen::Vector3f& sigma_MB,
                                          const Eigen::Vector3f& thrustDirection_B,
                                          const double thetaMax) {
    if (thrustDirection_B.stableNorm() == 0.0F) {
        return {0.0, 0.0};
    }
    const Eigen::Vector3d limited = limitDeflectionReference(thrustHatUnit_M(sigma_MB, thrustDirection_B), thetaMax);
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

// The tolerance for the two angles. At the edge of the cone, an error in the azimuth of the direction changes each
// angle by up to tan(thetaMax) times as much, because each angle is a ratio against a z component of
// cos(thetaMax). The azimuth error is largest for a request nearly opposite the neutral axis, where the error of
// the float rotation is divided by the short perpendicular part.
inline float angleTolerance(const float thetaMax) { return kAngleTolerance * std::max(1.0F, std::tan(thetaMax)); }

//! [-] perpendicular length below which a request is nearly opposite the neutral axis. The rotation error changes
//! the azimuth of the perpendicular part by up to kRotationError divided by its length. At this length that azimuth
//! error is kAngleTolerance, which angleTolerance() then scales for the edge of the cone.
inline constexpr double kNearlyOppositeBand = kRotationError / static_cast<double>(kAngleTolerance);

// The float and the double rotation must make the same kMinPerpendicular decision outside the band.
static_assert(kNearlyOppositeBand > kMinPerpendicular + kRotationError);

// Shows if the request is nearly opposite the neutral axis. There the plane that holds the request and the
// neutral axis is not defined, and the algorithm takes an arbitrary perpendicular direction. The float algorithm
// and a double reference can take different directions, thus the tests make no claim about the two angles there.
inline bool isNearlyOppositeTheNeutralAxis(const Eigen::Vector3d& unitDirection_M) {
    return unitDirection_M.z() < 0.0 && std::hypot(unitDirection_M.x(), unitDirection_M.y()) < kNearlyOppositeBand;
}

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

    if (thrustDirection_B.stableNorm() == 0.0F) {
        // The request carries no direction, thus the gimbal stays at its neutral position.
        EXPECT_NEAR(out.gimbalAngle1, 0.0F, 1e-6F);
        EXPECT_NEAR(out.gimbalAngle2, 0.0F, 1e-6F);
        return;
    }

    expectWithinTravel(out, thetaMax);

    const Eigen::Vector3d unitDirection_M = thrustHatUnit_M(sigma_MB, thrustDirection_B);
    if (isNearlyOppositeTheNeutralAxis(unitDirection_M)) {
        // The plane is not defined here, thus only the two conditions above hold.
        return;
    }

    const GimbalAnglesDouble reference = referenceUpdate(sigma_MB, thrustDirection_B, static_cast<double>(thetaMax));
    EXPECT_NEAR(out.gimbalAngle1, static_cast<float>(reference.angle1), angleTolerance(thetaMax));
    EXPECT_NEAR(out.gimbalAngle2, static_cast<float>(reference.angle2), angleTolerance(thetaMax));

    // The two angles and the achieved direction must both rebuild the direction that the travel limit gives.
    const Eigen::Vector3d limited_M = limitDeflectionReference(unitDirection_M, static_cast<double>(thetaMax));
    EXPECT_LT((gimbalAxis_M(out.gimbalAngle1, out.gimbalAngle2).cast<double>() - limited_M).norm(), kAngleTolerance);
    EXPECT_LT((out.thrustHat_B.cast<double>() - (dcmMountFromBody(sigma_MB).transpose() * limited_M)).norm(),
              kAngleTolerance);
}

// Regression test for a case that a known pair of angles defines. The helper builds the direction from the two
// angles, changes it to body-frame coordinates, and makes sure that the module gives the same two angles again.
// The body-frame input also makes the module apply the mounting orientation. Both angles must be inside the
// travel, otherwise the limit changes the direction and the module cannot give them again.
inline void regressionTestAxisToGimbalAnglesFromAngles(const Eigen::Vector3f& sigma_MB,
                                                       const float angle1,
                                                       const float angle2,
                                                       const float thetaMax = kDefaultThetaMax) {
    const Eigen::Vector3f thrustDirection_B =
        (dcmMountFromBody(sigma_MB).transpose() * gimbalAxis_M(angle1, angle2).cast<double>()).cast<float>();

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

// The achieved direction is the reachable direction nearest to the request, and the two angles point the gimbal
// along it. These conditions define the travel limit without its construction: the achieved direction has unit
// length, its deflection is the smaller of the requested deflection and thetaMax, and the angle from the request
// to it is only the part of the requested deflection beyond thetaMax. A request inside the travel is therefore
// left alone, and a request outside it goes to the edge of the cone in the plane that holds the neutral axis.
inline void propertyAchievedDirectionIsNearestReachable(const Eigen::Vector3f& sigma_MB,
                                                        const Eigen::Vector3f& thrustDirection_B,
                                                        const float thetaMax = kDefaultThetaMax) {
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB, thetaMax)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);

    const Eigen::Matrix3d dcm_MB = dcmMountFromBody(sigma_MB);
    const Eigen::Vector3d achieved_M = dcm_MB * out.thrustHat_B.cast<double>();

    EXPECT_NEAR(achieved_M.norm(), 1.0, 1e-6);
    EXPECT_LT((achieved_M - gimbalAxis_M(out.gimbalAngle1, out.gimbalAngle2).cast<double>()).norm(), kAngleTolerance);

    if (thrustDirection_B.stableNorm() == 0.0F) {
        EXPECT_LT((achieved_M - Eigen::Vector3d::UnitZ()).norm(), 1e-6);
        return;
    }

    const Eigen::Vector3d request_M = thrustHatUnit_M(sigma_MB, thrustDirection_B);
    const double thetaMaxD = static_cast<double>(thetaMax);
    EXPECT_NEAR(deflection(achieved_M), std::min(deflection(request_M), thetaMaxD), kAngleTolerance);

    // Nearly opposite the neutral axis, every direction on the edge of the cone is nearly equally far away.
    if (!isNearlyOppositeTheNeutralAxis(request_M)) {
        EXPECT_NEAR(
            angleBetween(achieved_M, request_M), std::max(deflection(request_M) - thetaMaxD, 0.0), kAngleTolerance);
    }
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
        thrustDirection_B.stableNorm() > 0.0F && scaledDirection_B.stableNorm() > 0.0F &&
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
