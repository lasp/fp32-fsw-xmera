#include "flybyPointTestHelpers.hpp"
#include <gtest/gtest.h>

// Approach trajectory seeded at t=0, stepped at 0.3 s intervals.
// timeBetweenFilterData=0.5 s triggers re-seeding at steps 2 and 4 (dt=0.6 s),
// while steps 1 and 3 stay in extrapolation mode (dt=0.3 s). Both branches of
// the state machine are covered in a single run.
TEST(FlybyPointTest, RegressionTest) {
    const FlybyPointConfig cfg = FlybyPointConfig::create(0.5, 1e-3F, 1, 10.0F, 1.0F, 1e9F);
    regressionTestFlybyPoint(cfg,
                             {-5e7, 7.5e6, 5e5},  // r_BN_N [m]
                             {2e4, 0, 0},          // v_BN_N [m/s]
                             300'000'000ULL,        // stepNanos: 0.3 s
                             4);
}
