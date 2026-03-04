#include "sunlineEphemTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(SunlineEphemTest, ReferenceTest) {
    testSunlineEphem(std::vector<double>{1.0e6, 2.0e6, 3.0e6},   // sun position
                     std::vector<double>{1.0e3, -2.0e3, 0.5e3},  // spacecraft position
                     std::vector<float>{0.1f, 0.2f, -0.3f}       // non-trivial attitude
    );
}
