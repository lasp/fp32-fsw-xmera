#include "mrpPDTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(MrpPDTest, RegressionTest) {
    regressionTestMrpPD(10,
                        200,
                        std::vector<float>{0.2, -0.1, -0.4},
                        std::vector<float>{-0.1, -0.4, 0},
                        std::vector<float>{0.009, 0.007, -0.006},
                        std::vector<float>{-0.0009, -0.00005, -0.0004},
                        std::vector<float>{0.08, -0.001, -0.003});
}

TEST(MrpPDTest, PropertyTest) { propertyTestMrpPD(); }

TEST(MrpPDTest, SetupTest) { testMrpPDSetup(); }
