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
                      3);
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
    const Eigen::Vector<float, kMaxNumRw> outContiguous = algContiguous.update(Lr_B);
    const Eigen::Vector<float, kMaxNumRw> outNonContiguous = algNonContiguous.update(Lr_B);

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(outContiguous[i], outNonContiguous[i], 1e-6);
    }
}
