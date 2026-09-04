#ifndef TEST_AXIS_TO_GIMBAL_ANGLES_H
#define TEST_AXIS_TO_GIMBAL_ANGLES_H

#include "axisToGimbalAnglesAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <numbers>

// Build a configuration from the mounting orientation that the tests use.
inline AxisToGimbalAnglesConfig makeConfig(const Eigen::Vector3f& sigma_MB) {
    return AxisToGimbalAnglesConfig::create(sigma_MB);
}

// The gimbal thrust axis for a pair of angles. This function does not use the algorithm, thus the tests can
// examine the mapping and do not repeat it. T(angle1, angle2) is proportional to [-tan(angle2), tan(angle1), -1].
// The cos(angle1)cos(angle2) factor keeps the components small at a deflection of 90 degrees.
inline Eigen::Vector3f gimbalAxis_M(const float angle1, const float angle2) {
    return Eigen::Vector3f{
        -std::sin(angle2) * std::cos(angle1), std::cos(angle2) * std::sin(angle1), -std::cos(angle2) * std::cos(angle1)}
        .normalized();
}

// The input direction in mount-frame coordinates. The two angles are ratios against the mount -z axis. Thus this
// function does not change the length of the vector.
inline Eigen::Vector3f thrustDir_M(const Eigen::Vector3f& sigma_MB, const Eigen::Vector3f& thrustDirection_B) {
    return mrpToDcm(mrpSwitch(sigma_MB)) * thrustDirection_B;
}

// The input direction in mount-frame coordinates, with unit length. This function uses stableNormalized() and
// not normalized(). normalized() calculates squaredNorm(), which is too small for a very short vector and too
// large for a very long one. The module has neither problem, because it uses ratios.
inline Eigen::Vector3f thrustHatUnit_M(const Eigen::Vector3f& sigma_MB, const Eigen::Vector3f& thrustDirection_B) {
    return thrustDir_M(sigma_MB, thrustDirection_B).stableNormalized();
}

// Shows if the module can give two angles for the input direction. The direction must be in the open half-space
// into which the neutral thrust fires, on the -z side of the mount frame. Each component must also be a number:
// the safe arctangent gives zero for a component that is not a number, and a very large input can become
// infinite in the rotation to mount-frame coordinates.
inline bool isResolvable(const Eigen::Vector3f& sigma_MB, const Eigen::Vector3f& thrustDirection_B) {
    const Eigen::Vector3f unitDirection = thrustHatUnit_M(sigma_MB, thrustDirection_B);
    return unitDirection.allFinite() && -unitDirection.z() > 0.0F;
}

// Double-precision reference for the two gimbal angles. The regression tests compare the float algorithm with
// this reference.
struct GimbalAnglesDouble {
    double angle1;
    double angle2;
};

inline GimbalAnglesDouble referenceUpdate(const Eigen::Vector3d& sigma_MB, const Eigen::Vector3d& thrustDirection_B) {
    const Eigen::Vector3d thrustDir = (mrpToDcm(mrpSwitch<double>(sigma_MB)) * thrustDirection_B).stableNormalized();
    const double towardsThrust = -thrustDir.z();
    // The float algorithm uses safe arctangents, which give zero for an argument that is not a number.
    if (!thrustDir.allFinite() || !(towardsThrust > 0.0)) {
        return {0.0, 0.0};
    }
    return {std::atan2(thrustDir.y(), towardsThrust), std::atan2(-thrustDir.x(), towardsThrust)};
}

// Shows if the input direction is on the boundary of the half-space, where the deflection is 90 degrees and the
// mount-frame z component is zero. There the two angles are at the edge of their range and hold no more
// information about the direction. The error of the float rotation is also larger than the z component, thus the
// float algorithm and a double reference can put the direction on different sides of the boundary. The band is
// much larger than that error, which is approximately 4e-7 for a direction of unit length.
inline bool isOnTheHalfSpaceBoundary(const Eigen::Vector3f& unitDirection_M) {
    constexpr float kBoundaryBand = 1e-5F;
    return std::fabs(unitDirection_M.z()) < kBoundaryBand;
}

// The largest difference that is permitted between the float angles and the double reference angles.
//
// The two angles are the coordinates of the point where the thrust axis touches the plane z = -1. Thus they
// become very large as the deflection increases to 90 degrees. Near that limit the last bits of a float value
// hold all of the remaining z component, and the accuracy decreases in proportion to 1 / |z|. This is a property
// of the two angles and not a defect. The constant below includes margin above the measured behaviour. For the
// small deflections that a gimbal can move to, this bound is approximately 1e-5.
inline float angleTolerance(const Eigen::Vector3f& unitDirection_M) {
    constexpr float kScale = 2e-6F;
    return 1e-5F + (kScale / std::fmax(-unitDirection_M.z(), kScale));
}

// In the half-space, both angles are arctangents of a ratio with a denominator of more than zero. Thus both
// angles stay in the range -pi/2 to pi/2.
inline void expectPrincipalBranches(const AxisToGimbalAnglesOutput& out, const float accuracy) {
    constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
    EXPECT_LE(std::fabs(out.gimbalAngle1), halfPi + accuracy);
    EXPECT_LE(std::fabs(out.gimbalAngle2), halfPi + accuracy);
}

// ---------------------------------------------------------------------------
// Regression tests
// ---------------------------------------------------------------------------

// The float algorithm must agree with the double reference. For a direction in the half-space, the two angles
// must also align the gimbal thrust axis with that direction.
inline void regressionTestAxisToGimbalAngles(const Eigen::Vector3f& sigma_MB,
                                             const Eigen::Vector3f& thrustDirection_B) {
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);

    const Eigen::Vector3f expected_M = thrustHatUnit_M(sigma_MB, thrustDirection_B);
    const float accuracy = angleTolerance(expected_M);

    // On the boundary of the half-space, neither the home position nor an angle at the edge of the range is more
    // correct. Examine only that the two angles are usable.
    if (isOnTheHalfSpaceBoundary(expected_M)) {
        EXPECT_TRUE(std::isfinite(out.gimbalAngle1));
        EXPECT_TRUE(std::isfinite(out.gimbalAngle2));
        expectPrincipalBranches(out, 1e-5F);
        return;
    }

    const GimbalAnglesDouble reference = referenceUpdate(sigma_MB.cast<double>(), thrustDirection_B.cast<double>());
    EXPECT_NEAR(out.gimbalAngle1, static_cast<float>(reference.angle1), accuracy);
    EXPECT_NEAR(out.gimbalAngle2, static_cast<float>(reference.angle2), accuracy);

    if (!isResolvable(sigma_MB, thrustDirection_B)) {
        // The direction is not in the half-space, thus the module gives the home position.
        EXPECT_NEAR(out.gimbalAngle1, 0.0F, 1e-6F);
        EXPECT_NEAR(out.gimbalAngle2, 0.0F, 1e-6F);
        return;
    }

    expectPrincipalBranches(out, accuracy);
    EXPECT_LT((gimbalAxis_M(out.gimbalAngle1, out.gimbalAngle2) - expected_M).norm(), accuracy);
}

// Regression test for a case that a known pair of angles defines. The helper builds the direction from the two
// angles, changes it to body-frame coordinates, and makes sure that the module gives the same two angles again.
// The body-frame input also makes the module apply the mounting orientation.
inline void regressionTestAxisToGimbalAnglesFromAngles(const Eigen::Vector3f& sigma_MB,
                                                       const float angle1,
                                                       const float angle2) {
    const Eigen::Vector3f thrustDirection_B = mrpToDcm(mrpSwitch(sigma_MB)).transpose() * gimbalAxis_M(angle1, angle2);

    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);

    const float accuracy = angleTolerance(thrustHatUnit_M(sigma_MB, thrustDirection_B));
    EXPECT_NEAR(out.gimbalAngle1, angle1, accuracy);
    EXPECT_NEAR(out.gimbalAngle2, angle2, accuracy);

    regressionTestAxisToGimbalAngles(sigma_MB, thrustDirection_B);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// Each input gives two angles that a caller can use. The two angles are always numbers. For a direction in the
// half-space they stay in the range -pi/2 to pi/2. For all other directions the module gives the home position.
inline void propertyOutputIsUsable(const Eigen::Vector3f& sigma_MB, const Eigen::Vector3f& thrustDirection_B) {
    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);

    // The home position is zero, thus these two conditions are true for every input.
    EXPECT_TRUE(std::isfinite(out.gimbalAngle1));
    EXPECT_TRUE(std::isfinite(out.gimbalAngle2));
    expectPrincipalBranches(out, 1e-5F);

    // The module gives the home position for a direction that is certainly outside the half-space. Two
    // conditions are not certain. On the boundary the module can give the home position or an angle at the edge
    // of the range, and both answers are correct. For a direction with a component that is not a number, the
    // safe arctangent gives zero for one angle and can give a value for the other. This test makes no claim for
    // those two conditions.
    const Eigen::Vector3f unitDirection_M = thrustHatUnit_M(sigma_MB, thrustDirection_B);
    const bool certainlyOutside =
        unitDirection_M.allFinite() && -unitDirection_M.z() <= 0.0F && !isOnTheHalfSpaceBoundary(unitDirection_M);
    if (certainlyOutside) {
        EXPECT_NEAR(out.gimbalAngle1, 0.0F, 1e-6F);
        EXPECT_NEAR(out.gimbalAngle2, 0.0F, 1e-6F);
    }
}

// For a direction in the half-space, the two angles align the gimbal thrust axis with that direction. The helper
// skips no input. It examines a direction that is not in the half-space with propertyOutputIsUsable.
inline void propertyDirectionRecovered(const Eigen::Vector3f& sigma_MB, const Eigen::Vector3f& thrustDirection_B) {
    const Eigen::Vector3f expected_M = thrustHatUnit_M(sigma_MB, thrustDirection_B);
    if (!isResolvable(sigma_MB, thrustDirection_B) || isOnTheHalfSpaceBoundary(expected_M)) {
        propertyOutputIsUsable(sigma_MB, thrustDirection_B);
        return;
    }

    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);

    EXPECT_LT((gimbalAxis_M(out.gimbalAngle1, out.gimbalAngle2) - expected_M).norm(), angleTolerance(expected_M));
}

// Both angles are ratios against the mount -z axis. Thus a change of the length of the input direction does not
// change the two angles.
//
// A very small or very large scale is the one exception, and it is a limit of the float type and not of the
// module. Such a scale makes a component of the scaled vector too large for a float, or so small that it becomes
// zero or loses almost all of its bits. The scaled vector then holds a different direction, or no direction, and
// the module correctly gives different angles. The helper skips no input: it examines both vectors with
// propertyOutputIsUsable in that condition.
inline void propertyLengthHasNoEffect(const Eigen::Vector3f& sigma_MB,
                                      const Eigen::Vector3f& thrustDirection_B,
                                      const float scale) {
    constexpr float kDirectionKept = 1e-6F;

    const Eigen::Vector3f scaledDirection_B = scale * thrustDirection_B;
    const bool scalingKeptTheDirection =
        isResolvable(sigma_MB, thrustDirection_B) && isResolvable(sigma_MB, scaledDirection_B) &&
        !isOnTheHalfSpaceBoundary(thrustHatUnit_M(sigma_MB, thrustDirection_B)) &&
        (thrustHatUnit_M(sigma_MB, scaledDirection_B) - thrustHatUnit_M(sigma_MB, thrustDirection_B)).norm() <
            kDirectionKept;
    if (!scalingKeptTheDirection) {
        propertyOutputIsUsable(sigma_MB, thrustDirection_B);
        propertyOutputIsUsable(sigma_MB, scaledDirection_B);
        return;
    }

    const AxisToGimbalAnglesAlgorithm alg{makeConfig(sigma_MB)};
    const AxisToGimbalAnglesOutput out = alg.update(thrustDirection_B);
    const AxisToGimbalAnglesOutput scaledOut = alg.update(scaledDirection_B);

    const float accuracy = angleTolerance(thrustHatUnit_M(sigma_MB, thrustDirection_B));
    EXPECT_NEAR(scaledOut.gimbalAngle1, out.gimbalAngle1, accuracy);
    EXPECT_NEAR(scaledOut.gimbalAngle2, out.gimbalAngle2, accuracy);
}

#endif  // TEST_AXIS_TO_GIMBAL_ANGLES_H
