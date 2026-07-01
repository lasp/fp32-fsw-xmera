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

// Flipping signOfOrbitNormalFrameVector negates uh (orbit-normal); keeping the frame
// right-handed then forces ut to flip too, while ur is shared. Flipping two axes about a
// fixed third is a 180 deg rotation about that axis (ur), i.e. R2N = diag(1,-1,-1) * RtN.
// Checks that directly via each run's DCM, and confirms omega/domega are unaffected (they're
// computed from the un-flipped RtN in computeGuidanceSolution, before the sign is applied).
TEST(FlybyPointTest,
     SignOfOrbitNormalRotatesOutputBy180DegAboutRadialAxis) {  // NOLINT(readability-function-cognitive-complexity)
    const FlybyPointConfig cfgPlus = FlybyPointConfig::create(0.5, 1e-3F, 1, 10.0F, 1.0F, 1e9F);
    const FlybyPointConfig cfgMinus = FlybyPointConfig::create(0.5, 1e-3F, -1, 10.0F, 1.0F, 1e9F);
    FlybyPointAlgorithm algPlus(cfgPlus);
    FlybyPointAlgorithm algMinus(cfgMinus);

    const Eigen::Vector3d r_BN_N{-5e7, 7.5e6, 5e5};
    const Eigen::Vector3d v_BN_N{2e4, 0, 0};
    constexpr uint64_t stepNanos = 300'000'000ULL;  // 0.3 s, matches RegressionTest cadence

    Eigen::Matrix3d flipAboutRadial = Eigen::Matrix3d::Identity();
    flipAboutRadial(1, 1) = -1.0;
    flipAboutRadial(2, 2) = -1.0;

    algPlus.updateState(0U, r_BN_N, v_BN_N);
    algMinus.updateState(0U, r_BN_N, v_BN_N);

    for (int k = 1; k <= 4; ++k) {
        const uint64_t simNanos = static_cast<uint64_t>(k) * stepNanos;
        const AttGuideOutput outPlus = algPlus.updateState(simNanos, r_BN_N, v_BN_N);
        const AttGuideOutput outMinus = algMinus.updateState(simNanos, r_BN_N, v_BN_N);
        ASSERT_TRUE(outPlus.validOutput);
        ASSERT_TRUE(outMinus.validOutput);

        const Eigen::Matrix3d dcmPlus = mrpToDcm(Eigen::Vector3d(outPlus.sigma_RN.cast<double>()));
        const Eigen::Matrix3d dcmMinus = mrpToDcm(Eigen::Vector3d(outMinus.sigma_RN.cast<double>()));
        const Eigen::Matrix3d expectedDcmMinus = flipAboutRadial * dcmPlus;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                EXPECT_NEAR(dcmMinus(i, j), expectedDcmMinus(i, j), 1e-6);
            }
            EXPECT_FLOAT_EQ(outPlus.omega_RN_N[i], outMinus.omega_RN_N[i]);
            EXPECT_FLOAT_EQ(outPlus.domega_RN_N[i], outMinus.domega_RN_N[i]);
        }
    }
}

// checkValidity() rejecting a reseed doesn't invalidate the output -- it should keep
// extrapolating the last-good solution. Each test below trips exactly one trigger and checks
// (a) only that trigger fires and (b) the output matches extrapolating the original seed, not a reseed off the new
// (bad) reading.
TEST(FlybyPointTest, CollinearityRejectsReseed) {
    const Eigen::Vector3d r_BN_N{-5e7, 7.5e6, 5e5};
    const Eigen::Vector3d v_BN_N{2e4, 0, 0};
    const FlybyPointConfig cfg = FlybyPointConfig::create(0.5, 1e-3F, 1, 10.0F, 1.0F, 1e9F);

    FlybyPointAlgorithm alg(cfg);
    alg.updateState(0U, r_BN_N, v_BN_N);

    // Radial-only velocity at the same position: r and v become exactly collinear.
    const Eigen::Vector3d collinearV = r_BN_N.normalized() * v_BN_N.norm();
    const AttGuideOutput out = alg.updateState(600'000'000ULL, r_BN_N, collinearV);

    EXPECT_TRUE(out.collinearityTrigger);
    EXPECT_FALSE(out.maxRateTrigger);
    EXPECT_FALSE(out.maxAccelerationTrigger);
    EXPECT_FALSE(out.positionKnowledgeExceedTrigger);
    ASSERT_TRUE(out.validOutput);

    expectMatchesExtrapolation(
        out,
        expectedExtrapolatedOutput(
            cfg.getTimeBetweenFilterData(), r_BN_N, v_BN_N, cfg.getSignOfOrbitNormalFrameVector()));
}

TEST(FlybyPointTest, MaxRateRejectsReseed) {
    const Eigen::Vector3d r_BN_N{-5e7, 7.5e6, 5e5};
    const Eigen::Vector3d v_BN_N{2e4, 0, 0};
    // maximumRateThreshold set below small value
    // maximumAccelerationThreshold left loose so only the rate trigger fires.
    const FlybyPointConfig cfg = FlybyPointConfig::create(0.5, 1e-3F, 1, 1e-6F, 1.0F, 1e9F);

    FlybyPointAlgorithm alg(cfg);
    alg.updateState(0U, r_BN_N, v_BN_N);
    const AttGuideOutput out = alg.updateState(600'000'000ULL, r_BN_N, v_BN_N);

    EXPECT_TRUE(out.maxRateTrigger);
    EXPECT_FALSE(out.collinearityTrigger);
    EXPECT_FALSE(out.maxAccelerationTrigger);
    EXPECT_FALSE(out.positionKnowledgeExceedTrigger);
    ASSERT_TRUE(out.validOutput);

    expectMatchesExtrapolation(
        out,
        expectedExtrapolatedOutput(
            cfg.getTimeBetweenFilterData(), r_BN_N, v_BN_N, cfg.getSignOfOrbitNormalFrameVector()));
}

TEST(FlybyPointTest, MaxAccelerationRejectsReseed) {
    const Eigen::Vector3d r_BN_N{-5e7, 7.5e6, 5e5};
    const Eigen::Vector3d v_BN_N{2e4, 0, 0};
    // maximumAccelerationThreshold set below small value
    // maximumRateThreshold left loose so only the accel trigger fires.
    const FlybyPointConfig cfg = FlybyPointConfig::create(0.5, 1e-3F, 1, 10.0F, 1e-6F, 1e9F);

    FlybyPointAlgorithm alg(cfg);
    alg.updateState(0U, r_BN_N, v_BN_N);
    const AttGuideOutput out = alg.updateState(600'000'000ULL, r_BN_N, v_BN_N);

    EXPECT_TRUE(out.maxAccelerationTrigger);
    EXPECT_FALSE(out.collinearityTrigger);
    EXPECT_FALSE(out.maxRateTrigger);
    EXPECT_FALSE(out.positionKnowledgeExceedTrigger);
    ASSERT_TRUE(out.validOutput);

    expectMatchesExtrapolation(
        out,
        expectedExtrapolatedOutput(
            cfg.getTimeBetweenFilterData(), r_BN_N, v_BN_N, cfg.getSignOfOrbitNormalFrameVector()));
}

TEST(FlybyPointTest, PositionKnowledgeRejectsReseed) {
    const Eigen::Vector3d r_BN_N{-5e7, 7.5e6, 5e5};
    const Eigen::Vector3d v_BN_N{2e4, 0, 0};
    // positionKnowledgeSigma tight enough that a position far off the straight-line prediction
    // (firstNavPosition + dt*firstNavVelocity) is rejected, while direction/speed stay close
    // enough to the seed that the rate/accel/collinearity checks stay well clear.
    const FlybyPointConfig cfg = FlybyPointConfig::create(0.5, 1e-3F, 1, 10.0F, 1.0F, 1.0F);

    FlybyPointAlgorithm alg(cfg);
    alg.updateState(0U, r_BN_N, v_BN_N);

    const Eigen::Vector3d offPredictionR = r_BN_N + Eigen::Vector3d{0, 0, 2e5};
    const AttGuideOutput out = alg.updateState(600'000'000ULL, offPredictionR, v_BN_N);

    EXPECT_TRUE(out.positionKnowledgeExceedTrigger);
    EXPECT_FALSE(out.collinearityTrigger);
    EXPECT_FALSE(out.maxRateTrigger);
    EXPECT_FALSE(out.maxAccelerationTrigger);
    ASSERT_TRUE(out.validOutput);

    expectMatchesExtrapolation(
        out,
        expectedExtrapolatedOutput(
            cfg.getTimeBetweenFilterData(), r_BN_N, v_BN_N, cfg.getSignOfOrbitNormalFrameVector()));
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
