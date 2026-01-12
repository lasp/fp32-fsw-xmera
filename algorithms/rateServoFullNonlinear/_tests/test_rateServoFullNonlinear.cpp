#include "rateServoFullNonlinearTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(RateServoFullNonlinearTest, ReferenceTest) {
    testRateServoFullNonlinear(0.4,
                               0.1,
                               1.1,
                               std::vector<float>{0.1, -0.2, 0.3},
                               std::vector<float>{-0.4, 0.5, -0.6},
                               std::vector<float>{0.7, -0.8, 0.9},
                               std::vector<float>{-1.0, 1.1, -1.2},
                               std::vector<float>{1.3, -1.4, 1.5},
                               std::vector<float>{-1.6, 1.7, -1.8},
                               std::vector<float>{1.9, -2.0, 2.1, -2.2},
                               std::vector<bool>{false, true, false, false},
                               2,
                               std::vector<float>{2.3, -2.4, 2.5, -2.6},
                               std::vector<float>{2.7, -2.8, 2.9, -3.0},
                               std::vector<float>{0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2},
                               std::vector<float>{0.4, 0.1, -0.3, 0.4, 0.1, -0.3, 0.4, 0.1, -0.3},
                               false,
                               0.1);
}

TEST(RateServoFullNonlinearTest, SetupTest) { testRateServoFullNonlinearSetup(); }
