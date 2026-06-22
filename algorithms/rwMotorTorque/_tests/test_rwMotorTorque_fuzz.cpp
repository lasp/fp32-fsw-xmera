#include "rwMotorTorqueTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// Shared input domains for the regression and most property helpers. RW speeds and the null-space feedback gain are
// bounded so tau * d stays well-scaled.
FUZZ_TEST(RwMotorTorqueAlgorithmFuzz, runRegressionCase)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::InRange(0, RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW * 3U),
                 fuzztest::InRange(0, 3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::InRange(0.0F, 1e3F));

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(RwMotorTorquePropertyFuzz, propertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::InRange(0, RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW * 3U),
                 fuzztest::InRange(0, 3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::InRange(0.0F, 1e3F));

FUZZ_TEST(RwMotorTorquePropertyFuzz, propertyExcludedWheelsZeroTorque)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::InRange(0, RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW * 3U),
                 fuzztest::InRange(0, 3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::InRange(0.0F, 1e3F));

FUZZ_TEST(RwMotorTorquePropertyFuzz, propertyNullSpaceAddsNoBodyTorque)
    .WithDomains(fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::InRange(0, RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW * 3U),
                 fuzztest::InRange(0, 3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::InRange(0.0F, 1e3F));

// No null-space-gain domain — this property forces the gain to zero internally.
FUZZ_TEST(RwMotorTorquePropertyFuzz, propertyZeroGainDisablesNullSpace)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::InRange(0, RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW * 3U),
                 fuzztest::InRange(0, 3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW));

// Control-only property: no RW speed or gain domains (the null-space term is not exercised here).
FUZZ_TEST(RwMotorTorquePropertyFuzz, propertyControlTorqueRealized)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::InRange(0, RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW * 3U),
                 fuzztest::InRange(0, 3));
