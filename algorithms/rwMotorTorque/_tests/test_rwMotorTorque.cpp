#include "rwMotorTorqueTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(RwMotorTorqueTest, ReferenceTest) {
    testRwMotorTorque(Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                      Eigen::Vector3f{0.2F, -0.4F, 0.7F},
                      std::vector<bool>{false, false, true, false},
                      true,
                      true,
                      4,
                      std::vector<float>{0.4F, 0.1F, -0.3F, 1.2F, 0.4F, 0.1F, -0.3F, 1.2F, 0.4F, 0.1F, -0.3F, 1.2F},
                      3,
                      std::vector<float>{},
                      std::vector<float>{},
                      0.0F);
}

// Non-zero gain: despin must match the reference and add no net body torque.
TEST(RwMotorTorqueTest, NullSpaceDespinFourWheels) {
    // Four wheels: body x, y, z, and a skewed axis so a 1-D null space exists for 3 control axes.
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 4U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    rwConfiguration.GsMatrix_B.col(3) = Eigen::Vector3f{1.0F, 1.0F, 1.0F}.normalized();
    const RwMotorTorqueAvailability availability{};  // all wheels available

    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Identity()};
    constexpr float kOmegaGain = 0.5F;
    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, kOmegaGain)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<4>() = Eigen::Vector4f{100.0F, -50.0F, 30.0F, 80.0F};
    speeds.rwDesiredSpeeds.head<4>() = Eigen::Vector4f{10.0F, 10.0F, 10.0F, 10.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});
    const Eigen::Vector<float, kMaxNumRw> ref =
        referenceUpdate(controlAxes_B, rwConfiguration, availability, Lr_B, speeds, kOmegaGain);

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(out[i], ref[i], 1e-6);
    }

    // The despin contribution (out - controlOnly) must produce no net body torque.
    const Eigen::Vector3f despinBodyTorque = rwConfiguration.GsMatrix_B.leftCols<4>() * (out - controlOnly).head<4>();
    EXPECT_NEAR(despinBodyTorque.norm(), 0.0F, 1e-5);

    // The despin term must be non-trivial (otherwise the test would pass vacuously).
    EXPECT_GT((out - controlOnly).norm(), 1e-3);
}

// Zero gain disables despin: output equals the control-only (zero-speed) update.
TEST(RwMotorTorqueTest, NullSpaceNoOpOmegaGainZero) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 4U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    rwConfiguration.GsMatrix_B.col(3) = Eigen::Vector3f{1.0F, 1.0F, 1.0F}.normalized();
    const RwMotorTorqueAvailability availability{};

    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration, availability, 0.0F)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<4>() = Eigen::Vector4f{100.0F, -50.0F, 30.0F, 80.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> withSpeeds = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_FLOAT_EQ(withSpeeds[i], controlOnly[i]);
    }
}

// Three spanning wheels leave no null space, so despin is zero even with a non-zero gain.
TEST(RwMotorTorqueTest, NullSpaceNoOpThreeSpanningWheels) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    const RwMotorTorqueAvailability availability{};

    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration, availability, 0.5F)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<3>() = Eigen::Vector3f{100.0F, -50.0F, 30.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> withSpeeds = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_FLOAT_EQ(withSpeeds[i], controlOnly[i]);
    }
}

TEST(RwMotorTorqueTest, SetupTest) { testRwMotorTorqueSetup(); }

// Control axes may sit in any rows; only the spanned subspace matters. Controlling body x and z via
// non-contiguous rows {0, 2} must produce the same motor torques as the contiguous rows {0, 1}.
TEST(RwMotorTorqueTest, NonContiguousControlAxes) {
    // Three wheels spinning about body x, y, z so any control axis is reachable.
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    const RwMotorTorqueAvailability availability{};  // all wheels available

    Eigen::Matrix3f contiguous{Eigen::Matrix3f::Zero()};
    contiguous.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    contiguous.row(1) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    Eigen::Matrix3f nonContiguous{Eigen::Matrix3f::Zero()};
    nonContiguous.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    nonContiguous.row(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    RwMotorTorqueAlgorithm algContiguous{RwMotorTorqueConfig::create(contiguous, rwConfiguration, availability)};
    RwMotorTorqueAlgorithm algNonContiguous{RwMotorTorqueConfig::create(nonContiguous, rwConfiguration, availability)};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> outContiguous = algContiguous.update(Lr_B, RwMotorTorqueSpeeds{});
    const Eigen::Vector<float, kMaxNumRw> outNonContiguous = algNonContiguous.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(outContiguous[i], outNonContiguous[i], 1e-6);
    }
}
