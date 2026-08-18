#include "../gimbalAnglesToMotorAnglesAlgorithm.h"
#include "gimbalAnglesToMotorAnglesTestHelpers.hpp"

#include <gtest/gtest.h>
#include <numbers>

// ---------------------------------------------------------------------------
// Requests outside the gimbal boundary return the gimbal home position
// ---------------------------------------------------------------------------

// No interpolation outside the gimbal boundary returns home motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, NoInterpolationOutsideBoundaryReturnsHome) {
    const float gimbalAngle1 = -12.0F * degToRad;  // [rad]
    const float gimbalAngle2 = -10.0F * degToRad;  // [rad]

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    EXPECT_NEAR(result.motorAngle1, kReferenceDefaultMotorAngle, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, kReferenceDefaultMotorAngle, 1e-6F);
}

// Linear interpolation outside the gimbal boundary returns home motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, LinearInterpolationOutsideBoundaryReturnsHome) {
    const float gimbalAngle1 = 5.0F * degToRad;
    const float gimbalAngle2 = -21.3F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    EXPECT_NEAR(result.motorAngle1, kReferenceDefaultMotorAngle, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, kReferenceDefaultMotorAngle, 1e-6F);
}

// Bilinear interpolation outside the gimbal boundary returns home motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, BilinearInterpolationOutsideBoundaryReturnsHome) {
    const float gimbalAngle1 = 8.37F * degToRad;
    const float gimbalAngle2 = 16.63F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    EXPECT_NEAR(result.motorAngle1, kReferenceDefaultMotorAngle, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, kReferenceDefaultMotorAngle, 1e-6F);
}

// --------------------------------------------------------------------------------------------------------
// Requests along the gimbal boundary return the gimbal home position for linear and bilinear interpolation
// --------------------------------------------------------------------------------------------------------

// Linear interpolation along the gimbal boundary returns home motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, LinearInterpolationAlongBoundaryReturnsHome) {
    const float gimbalAngle1 = -0.71F * degToRad;
    const float gimbalAngle2 = 25.5F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    EXPECT_NEAR(result.motorAngle1, kReferenceDefaultMotorAngle, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, kReferenceDefaultMotorAngle, 1e-6F);
}

// Bilinear interpolation outside along the gimbal boundary returns home motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, BilinearInterpolationAlongBoundaryReturnsHome) {
    const float gimbalAngle1 = 1.61F * degToRad;
    const float gimbalAngle2 = 25.81F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    EXPECT_NEAR(result.motorAngle1, kReferenceDefaultMotorAngle, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, kReferenceDefaultMotorAngle, 1e-6F);
}

// ---------------------------------------------------------------------------
// Requests inside the gimbal boundary return expected results
// ---------------------------------------------------------------------------

// No interpolation inside gimbal boundary returns exact motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, NoInterpolationInsideBoundaryReturnsExact) {
    const float gimbalAngle1 = 1.5F * degToRad;
    const float gimbalAngle2 = 4.0F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    const float motor1Expected = 84.06074422F * degToRad;
    const float motor2Expected = 72.42640668F * degToRad;
    EXPECT_NEAR(result.motorAngle1, motor1Expected, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, motor2Expected, 1e-6F);
}

// Linear interpolation along gimbal tip angle 1 inside gimbal boundary returns exact motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, LinearInterpolationInsideBoundaryAlongTipReturnsExact) {
    const float gimbalAngle1 = 2.2F * degToRad;
    const float gimbalAngle2 = 6.5F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    // Gimbal bounds
    const float gimbalAngle1LBound = 2.0F * degToRad;
    const float gimbalAngle1UBound = 2.5F * degToRad;
    const float gimbalAngle2LBound = 6.5F * degToRad;
    const float gimbalAngle2UBound = gimbalAngle2LBound;

    // Motor bounds
    const float motor1LLBound = 86.6590305F * degToRad;
    const float motor1ULBound = 85.59218715F * degToRad;
    const float motor1LUBound = motor1LLBound;
    const float motor1UUBound = motor1ULBound;
    const float motor2LLBound = 67.5806477F * degToRad;
    const float motor2ULBound = 66.39986032F * degToRad;
    const float motor2LUBound = motor2LLBound;
    const float motor2UUBound = motor2ULBound;

    // Interpolate to find expected motor angles
    const std::optional<float> motor1Expected = bilinearInterpolation(gimbalAngle1LBound,
                                                                      gimbalAngle1UBound,
                                                                      gimbalAngle2LBound,
                                                                      gimbalAngle2UBound,
                                                                      motor1LLBound,
                                                                      motor1LUBound,
                                                                      motor1ULBound,
                                                                      motor1UUBound,
                                                                      gimbalAngle1,
                                                                      gimbalAngle2);
    const std::optional<float> motor2Expected = bilinearInterpolation(gimbalAngle1LBound,
                                                                      gimbalAngle1UBound,
                                                                      gimbalAngle2LBound,
                                                                      gimbalAngle2UBound,
                                                                      motor2LLBound,
                                                                      motor2LUBound,
                                                                      motor2ULBound,
                                                                      motor2UUBound,
                                                                      gimbalAngle1,
                                                                      gimbalAngle2);
    ASSERT_TRUE(motor1Expected.has_value());
    ASSERT_TRUE(motor2Expected.has_value());
    EXPECT_NEAR(result.motorAngle1, *motor1Expected, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, *motor2Expected, 1e-6F);
}

// Linear interpolation along gimbal tilt angle 2 inside gimbal boundary returns exact motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, LinearInterpolationInsideBoundaryAlongTiltReturnsExact) {
    const float gimbalAngle1 = 0.5F * degToRad;
    const float gimbalAngle2 = -2.7F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    // Gimbal bounds
    const float gimbalAngle1LBound = 0.5F * degToRad;
    const float gimbalAngle1UBound = gimbalAngle1LBound;
    const float gimbalAngle2LBound = -3.0F * degToRad;
    const float gimbalAngle2UBound = -2.5F * degToRad;

    // Motor bounds
    const float motor1LLBound = 76.06429902F * degToRad;
    const float motor1ULBound = motor1LLBound;
    const float motor1LUBound = 76.78640286F * degToRad;
    const float motor1UUBound = motor1LUBound;
    const float motor2LLBound = 84.72342209F * degToRad;
    const float motor2ULBound = motor2LLBound;
    const float motor2LUBound = 83.99860858F * degToRad;
    const float motor2UUBound = motor2LUBound;

    // Interpolate to find expected motor angles
    const std::optional<float> motor1Expected = bilinearInterpolation(gimbalAngle1LBound,
                                                                      gimbalAngle1UBound,
                                                                      gimbalAngle2LBound,
                                                                      gimbalAngle2UBound,
                                                                      motor1LLBound,
                                                                      motor1LUBound,
                                                                      motor1ULBound,
                                                                      motor1UUBound,
                                                                      gimbalAngle1,
                                                                      gimbalAngle2);
    const std::optional<float> motor2Expected = bilinearInterpolation(gimbalAngle1LBound,
                                                                      gimbalAngle1UBound,
                                                                      gimbalAngle2LBound,
                                                                      gimbalAngle2UBound,
                                                                      motor2LLBound,
                                                                      motor2LUBound,
                                                                      motor2ULBound,
                                                                      motor2UUBound,
                                                                      gimbalAngle1,
                                                                      gimbalAngle2);
    ASSERT_TRUE(motor1Expected.has_value());
    ASSERT_TRUE(motor2Expected.has_value());
    EXPECT_NEAR(result.motorAngle1, *motor1Expected, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, *motor2Expected, 1e-6F);
}

// Bilinear interpolation inside gimbal boundary returns exact motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, BilinearInterpolationInsideBoundaryReturnsExact) {
    const float gimbalAngle1 = 1.7F * degToRad;
    const float gimbalAngle2 = -2.1F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    // Gimbal bounds
    const float gimbalAngle1LBound = 1.5F * degToRad;
    const float gimbalAngle1UBound = 2.0F * degToRad;
    const float gimbalAngle2LBound = -2.5F * degToRad;
    const float gimbalAngle2UBound = -2.0F * degToRad;

    // Motor bounds
    const float motor1LLBound = 74.61881907F * degToRad;
    const float motor1LUBound = 75.34739993F * degToRad;
    const float motor1ULBound = 73.51549152F * degToRad;
    const float motor1UUBound = 74.24682682F * degToRad;
    const float motor2LLBound = 81.87856171F * degToRad;
    const float motor2LUBound = 81.15338313F * degToRad;
    const float motor2ULBound = 80.8092383F * degToRad;
    const float motor2UUBound = 80.07973765F * degToRad;

    // Interpolate to find expected motor angles
    const std::optional<float> motor1Expected = bilinearInterpolation(gimbalAngle1LBound,
                                                                      gimbalAngle1UBound,
                                                                      gimbalAngle2LBound,
                                                                      gimbalAngle2UBound,
                                                                      motor1LLBound,
                                                                      motor1LUBound,
                                                                      motor1ULBound,
                                                                      motor1UUBound,
                                                                      gimbalAngle1,
                                                                      gimbalAngle2);
    const std::optional<float> motor2Expected = bilinearInterpolation(gimbalAngle1LBound,
                                                                      gimbalAngle1UBound,
                                                                      gimbalAngle2LBound,
                                                                      gimbalAngle2UBound,
                                                                      motor2LLBound,
                                                                      motor2LUBound,
                                                                      motor2ULBound,
                                                                      motor2UUBound,
                                                                      gimbalAngle1,
                                                                      gimbalAngle2);
    ASSERT_TRUE(motor1Expected.has_value());
    ASSERT_TRUE(motor2Expected.has_value());
    EXPECT_NEAR(result.motorAngle1, *motor1Expected, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, *motor2Expected, 1e-6F);
}

// ----------------------------------------------------------------------
// Exact requests at 4 corners of gimbal boundary return expected results
// ----- Tip allowable range is [-18.5, 18]
// ----- Tilt allowable range is [-27, 27]
// ----------------------------------------------------------------------

// Leftmost gimbal boundary angles (psi, phi) = (-18.5, 0) returns exact motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, LeftmostBoundaryRequestReturnsExact) {
    const float gimbalAngle1 = -18.5F * degToRad;
    const float gimbalAngle2 = 0.0F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    const float motor1Expected = 125.1789051F * degToRad;
    const float motor2Expected = 125.1788998F * degToRad;
    EXPECT_NEAR(result.motorAngle1, motor1Expected, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, motor2Expected, 1e-6F);
}

// Rightmost gimbal boundary angles (psi, phi) = (18, 0) returns exact motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, RightmostBoundaryRequestReturnsExact) {
    const float gimbalAngle1 = 18.0F * degToRad;
    const float gimbalAngle2 = 0.0F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    const float motor1Expected = 12.59996263F * degToRad;
    const float motor2Expected = 12.59997332F * degToRad;
    EXPECT_NEAR(result.motorAngle1, motor1Expected, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, motor2Expected, 1e-6F);
}

// Uppermost gimbal boundary angles (psi, phi) = (0.5, 27) returns exact motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, UppermostBoundaryRequestReturnsExact) {
    const float gimbalAngle1 = 0.5F * degToRad;
    const float gimbalAngle2 = 27.0F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    const float motor1Expected = 125.7651741F * degToRad;
    const float motor2Expected = 12.875392F * degToRad;
    EXPECT_NEAR(result.motorAngle1, motor1Expected, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, motor2Expected, 1e-6F);
}

// Lowermost gimbal boundary angles (psi, phi) = (0.5, -27) returns exact motor angles
TEST(GimbalAnglesToMotorAnglesReadTablesTest, LowermostBoundaryRequestReturnsExact) {
    const float gimbalAngle1 = 0.5F * degToRad;
    const float gimbalAngle2 = -27.0F * degToRad;

    GimbalAnglesToMotorAnglesAlgorithm alg(makeConfig());
    auto result = alg.update(gimbalAngle1, gimbalAngle2);

    const float motor1Expected = 12.87064698F * degToRad;
    const float motor2Expected = 125.7651701F * degToRad;
    EXPECT_NEAR(result.motorAngle1, motor1Expected, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, motor2Expected, 1e-6F);
}
