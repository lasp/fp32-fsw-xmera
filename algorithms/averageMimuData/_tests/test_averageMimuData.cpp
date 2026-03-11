#include "averageMimuDataTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(averageMimuDataTest, RegressionTest) {
    regressionTestaverageMimuData(0.5f,
                                  0.6f,
                                  0.2f,
                                  0.3f,
                                  Vec3Arr{0.1f, 0.3f, -0.1f},
                                  Vec3Arr{0.5f, -0.2f, 2.0f},
                                  Vec3Arr{1.1f, 0.8f, 0.7f},
                                  Vec3Arr{11.5f, -0.2f, 6.0f},
                                  Vec3Arr{-0.3f, -4.3f, -6.1f},
                                  Vec3Arr{-0.9f, -0.2f, -2.4f},
                                  Vec3Arr{7.1f, -0.9f, -0.0f},
                                  Vec3Arr{-80.5f, 0.4f, 2.8f});
}

TEST(averageMimuDataTest, PropertyKnownSolution) { testKnownSolaverageMimuData(); }

TEST(averageMimuDataTest, SetupTest) { testSetupAverageMimuData(); }
