#include "utilities/fsw/validInertiaCheck.h"
#include <gtest/gtest.h>

TEST(ValidateInertiaTest, SetupTest) {
    // --- Test expected exceptions ---
    Eigen::Matrix3f badInertia = Eigen::Matrix3f::Identity();
    badInertia << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    EXPECT_FALSE(inertiaIsValid(badInertia));
    badInertia << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    EXPECT_FALSE(inertiaIsValid(badInertia));
    badInertia << 3, 0, 0, 0, 1, 0, 0, 0, 1;
    EXPECT_FALSE(inertiaIsValid(badInertia));
}
