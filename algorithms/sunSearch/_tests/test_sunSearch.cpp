#include "sunSearchAlgorithm.h"
#include "sunSearchTestHelpers.hpp"
#include "sunSearchTypes.h"
#include "utilities/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cstdint>

TEST(SunSearchTest, ReferenceTest) {
    testSunSearch(/* rotationTimes */ {5.0F, 10.0F, 7.0F, 3.0F},
                  /* rotationRates */ {0.1F, 0.2F, 0.15F, 0.05F},
                  /* rotationAxes */ {0, 1, 2, 0},
                  /* omega_BN_B  */ Eigen::Vector3f{0.01F, -0.02F, 0.03F},
                  /* dt           */ 0.1F,
                  /* numSteps     */ 300);
}

TEST(SunSearchTest, SetupTest) { testSunSearchSetup(); }
