#include "thrFiringSchmittTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(ThrFiringSchmittTest, ReferenceTest) {
    testThrFiringSchmitt(0.7F,
                         0.3F,
                         0.01F,
                         1U,
                         2.0F,
                         4U,
                         std::vector{3.1F, 4.2F, 5.3F, 6.4F},
                         std::vector{2.1F, 1.2F, 0.3F, 10.4F},
                         0.1F);
}

TEST(ThrFiringSchmittTest, SetupTest) { testThrFiringSchmittSetup(); }
