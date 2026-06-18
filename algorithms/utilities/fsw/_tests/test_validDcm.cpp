#include "utilities/fsw/validDcmCheck.h"
#include <gtest/gtest.h>

#include "utilitiesHelpers.hpp"

TEST(ValidDcm, KnownBadDcm) {
    // --- Test expected exceptions ---
    Eigen::Matrix3f badDcm = Eigen::Matrix3f::Identity();
    badDcm << 1, 0, 0, 0, 1, 0, 0, 0, 0;  // not orthogonal, not +1 determinant
    EXPECT_FALSE(isValidDcm(badDcm));
    badDcm << 1, 0, 0, 0, 1, 0, 0, 1, 1;  // not orthogonal, +1 determinant
    EXPECT_FALSE(isValidDcm(badDcm));
    badDcm << 1, 0, 0, 0, 1, 0, 0, 0, -1;  // orthogonal, not +1 determinant
    EXPECT_FALSE(isValidDcm(badDcm));
}
