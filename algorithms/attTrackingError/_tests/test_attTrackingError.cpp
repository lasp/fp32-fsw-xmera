#include "attTrackingErrorTestHelpers.hpp"

#include <gtest/gtest.h>

TEST(attTrackingErrorTest, RegressionTest) {
    regressionTestAttTrackingError(Eigen::Vector3f{0.2f, -0.1f, -0.4f},
                                   Eigen::Vector3f{-0.1f, -0.4f, 0.0f},
                                   Eigen::Vector3f{0.009f, 0.007f, -0.006f},
                                   Eigen::Vector3f{0.08f, -0.001f, -0.003f},
                                   Eigen::Vector3f{0.01f, 0.02f, -0.04f});
}

TEST(attTrackingErrorTest, ZeroPropertyTest) { zeroPropertyTest({0.2f, -0.1f, -0.4f}); }

TEST(attTrackingErrorTest, IdentityPropertyTest) {
    identityPropertyTest({0.08f, -0.001f, -0.003f}, {-0.1f, -0.4f, 0.0f});
}

TEST(attTrackingErrorTest, FinitenessPropertyTest) {
    finitenessPropertyTest(Eigen::Vector3f{0.2f, -0.1f, -0.4f},
                           Eigen::Vector3f{-0.1f, -0.4f, 0.0f},
                           Eigen::Vector3f{0.009f, 0.007f, -0.006f},
                           Eigen::Vector3f{0.08f, -0.001f, -0.003f},
                           Eigen::Vector3f{0.01f, 0.02f, -0.04f});
}
