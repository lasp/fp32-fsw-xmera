#include "ephemeridesRecenterTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(EphemeridesRecenterTest, RegressionTest) {
    const std::vector<BodyName> bodyListInOrder{
        makeBodyName("SUN"),
        makeBodyName("EARTH"),
        makeBodyName("MOON"),
        makeBodyName("SATURN"),
    };

    regressionTestEphemeridesRecenter(bodyListInOrder,
                                      0,
                                      1,
                                      // SUN
                                      {0.0, 0.0, 0.0},
                                      {0.0, 0.0, 0.0},
                                      false,
                                      // EARTH (relative to SUN)
                                      {1.0e8, 2.0e8, -3.0e8},
                                      {1.0e2, -2.0e2, 3.0e2},
                                      false,
                                      // MOON (relative to EARTH)
                                      {3.8e5, -2.0e5, 1.0e5},
                                      {1.0, 0.2, -0.1},
                                      true,
                                      // SATURN (relative to SUN)
                                      {1.4e9, -2.3e9, 0.7e9},
                                      {9.5e1, 1.1e2, -7.0e1},
                                      false);
}

TEST(EphemeridesRecenterTest, SetupTest) { testEphemeridesRecenterSetup(); }

TEST(EphemeridesRecenterTest, PropertyTest) { propertyTestEphemeridesRecenter(); }
