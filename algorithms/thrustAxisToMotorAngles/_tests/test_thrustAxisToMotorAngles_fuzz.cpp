#include "thrustAxisToMotorAnglesTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz
// ---------------------------------------------------------------------------
FUZZ_TEST(ThrustAxisToMotorAnglesAlgorithmFuzz, testThrustAxisToMotorAnglesRegression)
    .WithDomains(fuzztest::InRange(-90.0F * std::numbers::pi_v<float> / 180.0F,
                                   90.0F * std::numbers::pi_v<float> / 180.0F),  // gimbalAngle1 [rad]
                 fuzztest::InRange(-90.0F * std::numbers::pi_v<float> / 180.0F,
                                   90.0F * std::numbers::pi_v<float> / 180.0F),  // gimbalAngle2 [rad]
                 xmera::fuzz::Vector3fInRange(0.0F, 1.0F),                       // tableCoeffs1
                 xmera::fuzz::Vector3fInRange(0.0F, 1.0F),                       // tableCoeffs2
                 fuzztest::InRange(-(NUM_GIMBAL_TO_MOTOR_TABLE_COLS - 1),
                                   NUM_GIMBAL_TO_MOTOR_TABLE_COLS - 1),  // tipColIdxOffset
                 fuzztest::InRange(-(NUM_GIMBAL_TO_MOTOR_TABLE_ROWS - 1),
                                   NUM_GIMBAL_TO_MOTOR_TABLE_ROWS - 1),  // tiltRowIdxOffset
                 // Step angles above about 1.5 degrees are rejected by the +- 90 degree gimbal travel check
                 fuzztest::InRange(0.25F * std::numbers::pi_v<float> / 180.0F,
                                   1.5F * std::numbers::pi_v<float> / 180.0F));  // tableStepAngle

// ---------------------------------------------------------------------------
// Property fuzz
// ---------------------------------------------------------------------------
FUZZ_TEST(ThrustAxisToMotorAnglesAlgorithmFuzz, propertyOutputIsFinite)
    .WithDomains(fuzztest::InRange(-90.0F * std::numbers::pi_v<float> / 180.0F,
                                   90.0F * std::numbers::pi_v<float> / 180.0F),  // gimbalAngle1 [rad]
                 fuzztest::InRange(-90.0F * std::numbers::pi_v<float> / 180.0F,
                                   90.0F * std::numbers::pi_v<float> / 180.0F),  // gimbalAngle2 [rad]
                 xmera::fuzz::Vector3fInRange(0.0F, 1.0F),                       // tableCoeffs1
                 xmera::fuzz::Vector3fInRange(0.0F, 1.0F),                       // tableCoeffs2
                 fuzztest::InRange(-(NUM_GIMBAL_TO_MOTOR_TABLE_COLS - 1),
                                   NUM_GIMBAL_TO_MOTOR_TABLE_COLS - 1),  // tipColIdxOffset
                 fuzztest::InRange(-(NUM_GIMBAL_TO_MOTOR_TABLE_ROWS - 1),
                                   NUM_GIMBAL_TO_MOTOR_TABLE_ROWS - 1),  // tiltRowIdxOffset
                 // Step angles above about 1.5 degrees are rejected by the +- 90 degree gimbal travel check
                 fuzztest::InRange(0.25F * std::numbers::pi_v<float> / 180.0F,
                                   1.5F * std::numbers::pi_v<float> / 180.0F));  // tableStepAngle

FUZZ_TEST(ThrustAxisToMotorAnglesAlgorithmFuzz, propertyMotorAnglesBounded)
    .WithDomains(fuzztest::InRange(-90.0F * std::numbers::pi_v<float> / 180.0F,
                                   90.0F * std::numbers::pi_v<float> / 180.0F),  // gimbalAngle1 [rad]
                 fuzztest::InRange(-90.0F * std::numbers::pi_v<float> / 180.0F,
                                   90.0F * std::numbers::pi_v<float> / 180.0F),  // gimbalAngle2 [rad]
                 xmera::fuzz::Vector3fInRange(0.0F, 1.0F),                       // tableCoeffs1
                 xmera::fuzz::Vector3fInRange(0.0F, 1.0F),                       // tableCoeffs2
                 fuzztest::InRange(-(NUM_GIMBAL_TO_MOTOR_TABLE_COLS - 1),
                                   NUM_GIMBAL_TO_MOTOR_TABLE_COLS - 1),  // tipColIdxOffset
                 fuzztest::InRange(-(NUM_GIMBAL_TO_MOTOR_TABLE_ROWS - 1),
                                   NUM_GIMBAL_TO_MOTOR_TABLE_ROWS - 1),  // tiltRowIdxOffset
                 // Step angles above about 1.5 degrees are rejected by the +- 90 degree gimbal travel check
                 fuzztest::InRange(0.25F * std::numbers::pi_v<float> / 180.0F,
                                   1.5F * std::numbers::pi_v<float> / 180.0F));  // tableStepAngle
