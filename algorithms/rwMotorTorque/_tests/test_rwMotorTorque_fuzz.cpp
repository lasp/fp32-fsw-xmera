#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "rwMotorTorqueTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(RwMotorTorqueAlgorithmFuzz, testRwMotorTorque)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::InRange(0, RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW * 3U),
                 fuzztest::InRange(0, 3),
                 // RW speeds, desired speeds, and despin gain bounded so tau * d stays well-scaled.
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_MOTOR_TORQUE_MAX_NUM_RW),
                 fuzztest::InRange(0.0F, 1e3F));
