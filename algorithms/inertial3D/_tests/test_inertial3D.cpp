#include "inertial3DTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <limits>

TEST(Inertial3DTest, ReferenceTest) { testInertial3D(Eigen::Vector3f{10000.03F, -0.00002F, 120310133.2F}); }

TEST(Inertial3DTest, ZeroReference) { testInertial3D(Eigen::Vector3f::Zero()); }

TEST(Inertial3DTest, ConfigRoundTripsAndRejectsNonFinite) {
    const Eigen::Vector3f sigma_RN{0.1F, -0.2F, 0.3F};
    const auto config = Inertial3DConfig::create(sigma_RN);
    EXPECT_TRUE(config.getSigmaRN().isApprox(sigma_RN, 1e-7F));

    EXPECT_TRUE(Inertial3DConfig::isValidSigmaRN(sigma_RN));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(Inertial3DConfig::isValidSigmaRN(Eigen::Vector3f{nan, 0.0F, 0.0F}));
    EXPECT_THROW(Inertial3DConfig::create(Eigen::Vector3f{nan, 0.0F, 0.0F}), fsw::invalid_argument);
}
