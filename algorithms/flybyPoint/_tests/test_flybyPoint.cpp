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
                             {2e4, 0, 0},         // v_BN_N [m/s]
                             300'000'000ULL,      // stepNanos: 0.3 s
                             4);
}

TEST(FlybyPointTest, SetupTest) {
    // Valid config builds without throwing.
    EXPECT_NO_THROW({
        const FlybyPointConfig cfg = FlybyPointConfig::create(1.0, 1e-3F, 1, 1.0F, 1.0F, 1e3F);
        const FlybyPointAlgorithm alg(cfg);
        (void)alg;
    });

    // timeBetweenFilterData must be > 0
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(0.0, 1e-3F, 1, 1.0F, 1.0F, 1e3F));
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(-1.0, 1e-3F, 1, 1.0F, 1.0F, 1e3F));

    // toleranceForCollinearity must be > 0
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, 0.0F, 1, 1.0F, 1.0F, 1e3F));
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, -1e-3F, 1, 1.0F, 1.0F, 1e3F));

    // signOfOrbitNormalFrameVector must be +1 or -1
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, 1e-3F, 0, 1.0F, 1.0F, 1e3F));
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, 1e-3F, 2, 1.0F, 1.0F, 1e3F));
    EXPECT_NO_THROW((void)FlybyPointConfig::create(1.0, 1e-3F, -1, 1.0F, 1.0F, 1e3F));

    // maximumRateThreshold must be > 0
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, 1e-3F, 1, 0.0F, 1.0F, 1e3F));
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, 1e-3F, 1, -1.0F, 1.0F, 1e3F));

    // maximumAccelerationThreshold must be > 0
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, 1e-3F, 1, 1.0F, 0.0F, 1e3F));
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, 1e-3F, 1, 1.0F, -1.0F, 1e3F));

    // positionKnowledgeSigma must be > 0
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, 1e-3F, 1, 1.0F, 1.0F, 0.0F));
    EXPECT_ANY_THROW((void)FlybyPointConfig::create(1.0, 1e-3F, 1, 1.0F, 1.0F, -1e3F));
}
