#include "mrpRotationTestHelpers.hpp"

#include <numbers>

// ---------------------------------------------------------------------------
// Regression test: drive the algorithm through several integration steps and compare
// to the reference implementation.
// ---------------------------------------------------------------------------

TEST(MrpRotationTest, RegressionStaticFrame) {
    regressionTestMrpRotation(/* sigma_RcR0 = */ Eigen::Vector3f{0.3F, 0.5F, 0.0F},
                              /* sigma_RpRc = */ Eigen::Vector3f{0.05F, 0.0F, -0.02F},
                              /* spinAxis_Rp = */ Eigen::Vector3f::UnitX(),
                              /* spinRate = */ 0.0017453F,
                              /* sigma_R0N = */ Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                              /* omega_R0N_N = */ Eigen::Vector3f::Zero(),
                              /* domega_R0N_N = */ Eigen::Vector3f::Zero(),
                              /* updateTimeSec = */ 0.5F,
                              /* numSteps = */ 4);
}

TEST(MrpRotationTest, RegressionSpinningInputFrame) {
    regressionTestMrpRotation(/* sigma_RcR0 = */ Eigen::Vector3f{0.0F, 0.0F, 0.0F},
                              /* sigma_RpRc = */ Eigen::Vector3f{0.0F, 0.1F, 0.0F},
                              /* spinAxis_Rp = */ Eigen::Vector3f{0.01F, -0.005F, 0.002F}.stableNormalized(),
                              /* spinRate = */ -0.0113578F,
                              /* sigma_R0N = */ Eigen::Vector3f{0.05F, 0.05F, 0.0F},
                              /* omega_R0N_N = */ Eigen::Vector3f{0.1F, 0.0F, 0.0F},
                              /* domega_R0N_N = */ Eigen::Vector3f::Zero(),
                              /* updateTimeSec = */ 0.25F,
                              /* numSteps = */ 6);
}

// ---------------------------------------------------------------------------
// Setup tests: Config validators and constructor behavior.
// ---------------------------------------------------------------------------

TEST(MrpRotationConfigTest, RejectsNonFiniteSigma_RcR0) {
    const Eigen::Vector3f bad{std::nanf(""), 0.0F, 0.0F};
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    EXPECT_THROW(MrpRotationConfig::create(bad, good, axis, /* spinRate = */ 0.0F, /* controlPeriod = */ 0.5F),
                 fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonFiniteSigma_RpRc) {
    const Eigen::Vector3f bad{0.0F, 0.0F, std::numeric_limits<float>::infinity()};
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    EXPECT_THROW(MrpRotationConfig::create(good, bad, axis, /* spinRate = */ 0.0F, /* controlPeriod = */ 0.5F),
                 fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonFiniteOrNonUnitSpinAxis) {
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f badAxes[] = {
        Eigen::Vector3f::Zero(),
        1.002F * Eigen::Vector3f::UnitY(),
        0.998F * Eigen::Vector3f::UnitY(),
        Eigen::Vector3f{std::nanf(""), 0.0F, 0.0F},
        Eigen::Vector3f{0.0F, std::numeric_limits<float>::infinity(), 0.0F},
    };
    for (const Eigen::Vector3f& bad : badAxes) {
        EXPECT_THROW(MrpRotationConfig::create(good, good, bad, /* spinRate = */ 0.0F, /* controlPeriod = */ 0.5F),
                     fsw::invalid_argument);
    }
}

TEST(MrpRotationConfigTest, RejectsNonFiniteSpinRate) {
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    EXPECT_THROW(MrpRotationConfig::create(good, good, axis, std::nanf(""), /* controlPeriod = */ 0.5F),
                 fsw::invalid_argument);
    EXPECT_THROW(MrpRotationConfig::create(
                     good, good, axis, -std::numeric_limits<float>::infinity(), /* controlPeriod = */ 0.5F),
                 fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonPositiveControlPeriod) {
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    EXPECT_THROW(MrpRotationConfig::create(good, good, axis, /* spinRate = */ 0.0F, /* controlPeriod = */ 0.0F),
                 fsw::invalid_argument);
    EXPECT_THROW(MrpRotationConfig::create(good, good, axis, /* spinRate = */ 0.0F, /* controlPeriod = */ -0.1F),
                 fsw::invalid_argument);
    EXPECT_THROW(
        MrpRotationConfig::create(good, good, axis, /* spinRate = */ 0.0F, /* controlPeriod = */ std::nanf("")),
        fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, AcceptsFiniteInputs) {
    EXPECT_NO_THROW(MrpRotationConfig::create(/* sigma_RcR0 = */ Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                                              /* sigma_RpRc = */ Eigen::Vector3f{-0.1F, 0.0F, 0.05F},
                                              /* spinAxis_Rp = */ Eigen::Vector3f{0.6F, 0.0F, -0.8F},
                                              /* spinRate = */ -0.02F,
                                              /* controlPeriod = */ 0.5F));
}

TEST(MrpRotationConfigTest, GettersRoundTrip) {
    const Eigen::Vector3f sigma_RcR0{0.1F, 0.2F, 0.3F};
    const Eigen::Vector3f sigma_RpRc{-0.1F, 0.0F, 0.05F};
    const Eigen::Vector3f spinAxis_Rp{0.0F, 0.6F, 0.8F};
    constexpr float spinRate = -0.03F;
    constexpr float controlPeriod = 0.25F;
    const auto cfg = MrpRotationConfig::create(sigma_RcR0, sigma_RpRc, spinAxis_Rp, spinRate, controlPeriod);

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(cfg.getSigma_RcR0()(i), sigma_RcR0(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_RpRc()(i), sigma_RpRc(i));
        EXPECT_FLOAT_EQ(cfg.getSpinAxis_Rp()(i), spinAxis_Rp(i));
    }
    EXPECT_FLOAT_EQ(cfg.getSpinRate(), spinRate);
    EXPECT_FLOAT_EQ(cfg.getControlPeriod(), controlPeriod);
}

// create() accepts a spin axis whose norm is within 1e-3 of 1 and stores it normalized.
TEST(MrpRotationConfigTest, SpinAxisStoredNormalized) {
    const Eigen::Vector3f direction = Eigen::Vector3f{1.0F, -2.0F, 2.0F}.stableNormalized();
    const Eigen::Vector3f nearUnitAxis = 1.0009F * direction;
    const auto cfg = MrpRotationConfig::create(Eigen::Vector3f::Zero(),
                                               Eigen::Vector3f::Zero(),
                                               nearUnitAxis,
                                               /* spinRate = */ 0.01F,
                                               /* controlPeriod = */ 0.5F);

    EXPECT_NEAR(cfg.getSpinAxis_Rp().stableNorm(), 1.0F, 1e-6F);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(cfg.getSpinAxis_Rp()(i), direction(i), 1e-6F);
    }
}

// create() bounds both offset MRPs to the principal set: an MRP with norm > 1 is stored as its shadow-set
// representative (-sigma / |sigma|^2), which has norm <= 1.
TEST(MrpRotationConfigTest, OffsetsSwitchedToShadowSetWhenNormExceedsOne) {
    const Eigen::Vector3f largeSigma{0.8F, 0.6F, 0.6F};  // |sigma|^2 = 1.36 > 1
    ASSERT_GT(largeSigma.stableNorm(), 1.0F) << "Test setup: MRP must exceed the norm-1 boundary";
    const Eigen::Vector3f expectedShadow = -largeSigma / largeSigma.squaredNorm();

    const auto cfg = MrpRotationConfig::create(largeSigma,
                                               largeSigma,
                                               /* spinAxis_Rp = */ Eigen::Vector3f::UnitX(),
                                               /* spinRate = */ 0.0F,
                                               /* controlPeriod = */ 0.5F);

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(cfg.getSigma_RcR0()(i), expectedShadow(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_RpRc()(i), expectedShadow(i));
    }
}

// Offset MRPs already within the principal set (norm <= 1) are stored unchanged.
TEST(MrpRotationConfigTest, OffsetsWithinBoundStoredUnchanged) {
    const Eigen::Vector3f sigma_RcR0{0.3F, -0.4F, 0.2F};
    const Eigen::Vector3f sigma_RpRc{-0.1F, 0.2F, 0.0F};
    ASSERT_LE(sigma_RcR0.stableNorm(), 1.0F);
    ASSERT_LE(sigma_RpRc.stableNorm(), 1.0F);

    const auto cfg = MrpRotationConfig::create(sigma_RcR0,
                                               sigma_RpRc,
                                               /* spinAxis_Rp = */ Eigen::Vector3f::UnitX(),
                                               /* spinRate = */ 0.0F,
                                               /* controlPeriod = */ 0.5F);
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(cfg.getSigma_RcR0()(i), sigma_RcR0(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_RpRc()(i), sigma_RpRc(i));
    }
}

TEST(MrpRotationConfigTest, IsValidValidatorsHonorContracts) {
    EXPECT_TRUE(MrpRotationConfig::isValidSigma_RcR0(Eigen::Vector3f{0.0F, 0.0F, 0.0F}));
    EXPECT_TRUE(MrpRotationConfig::isValidSigma_RcR0(Eigen::Vector3f{1.5F, -2.0F, 3.5F}));
    EXPECT_FALSE(MrpRotationConfig::isValidSigma_RcR0(Eigen::Vector3f{std::nanf(""), 0.0F, 0.0F}));

    EXPECT_TRUE(MrpRotationConfig::isValidSigma_RpRc(Eigen::Vector3f{0.0F, 0.0F, 0.0F}));
    EXPECT_TRUE(MrpRotationConfig::isValidSigma_RpRc(Eigen::Vector3f{-1.5F, 2.0F, 0.5F}));
    EXPECT_FALSE(MrpRotationConfig::isValidSigma_RpRc(Eigen::Vector3f{0.0F, std::nanf(""), 0.0F}));

    EXPECT_TRUE(MrpRotationConfig::isValidSpinAxis_Rp(Eigen::Vector3f::UnitZ()));
    EXPECT_TRUE(MrpRotationConfig::isValidSpinAxis_Rp(1.0009F * Eigen::Vector3f::UnitZ()));
    EXPECT_TRUE(MrpRotationConfig::isValidSpinAxis_Rp(0.9991F * Eigen::Vector3f::UnitZ()));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinAxis_Rp(1.0011F * Eigen::Vector3f::UnitZ()));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinAxis_Rp(0.9989F * Eigen::Vector3f::UnitZ()));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinAxis_Rp(Eigen::Vector3f::Zero()));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinAxis_Rp(Eigen::Vector3f{std::nanf(""), 0.0F, 0.0F}));
    EXPECT_FALSE(
        MrpRotationConfig::isValidSpinAxis_Rp(Eigen::Vector3f{0.0F, 0.0F, std::numeric_limits<float>::infinity()}));

    EXPECT_TRUE(MrpRotationConfig::isValidSpinRate(0.0F));
    EXPECT_TRUE(MrpRotationConfig::isValidSpinRate(0.5F));
    EXPECT_TRUE(MrpRotationConfig::isValidSpinRate(-0.5F));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinRate(std::nanf("")));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinRate(std::numeric_limits<float>::infinity()));

    EXPECT_TRUE(MrpRotationConfig::isValidControlPeriod(0.5F));
    EXPECT_TRUE(MrpRotationConfig::isValidControlPeriod(1e-6F));
    EXPECT_FALSE(MrpRotationConfig::isValidControlPeriod(0.0F));
    EXPECT_FALSE(MrpRotationConfig::isValidControlPeriod(-1.0F));
    EXPECT_FALSE(MrpRotationConfig::isValidControlPeriod(std::nanf("")));
    EXPECT_FALSE(MrpRotationConfig::isValidControlPeriod(std::numeric_limits<float>::infinity()));
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

TEST(MrpRotationTest, OutputIsFiniteSmallInputs) {
    propertyOutputIsFinite(/* sigma_RcR0 = */ {0.1F, -0.2F, 0.3F},
                           /* sigma_RpRc = */ {0.0F, 0.05F, 0.0F},
                           /* spinAxis_Rp = */ {1.0F, 0.0F, 0.0F},
                           /* spinRate = */ 0.01F);
}

TEST(MrpRotationTest, OutputIsFiniteLargeOmega) {
    propertyOutputIsFinite(/* sigma_RcR0 = */ {0.0F, 0.0F, 0.0F},
                           /* sigma_RpRc = */ {0.0F, 0.0F, 0.0F},
                           /* spinAxis_Rp = */ Eigen::Vector3f{5.0F, -3.0F, 2.0F}.stableNormalized(),
                           /* spinRate = */ 6.164414F);
}

TEST(MrpRotationTest, SigmaRNEqualsSigmaRRpWhenOffsetsAndInputRefAreIdentity) {
    propertySigmaRNEqualsSigmaRRpWhenOffsetsAndInputRefAreIdentity(
        /* spinAxis_Rp = */ Eigen::Vector3f{0.05F, -0.02F, 0.01F}.stableNormalized(), /* spinRate = */ 0.054772F);
}

TEST(MrpRotationTest, SigmaRNNormLessOrEqualToOne) {
    // The rate is large enough that the 100-step run crosses the |sigma| = 1 shadow-switch boundary several times.
    propertySigmaRNNormLessOrEqualToOne(/* sigma_RcR0 = */ {0.5F, 0.0F, 0.0F},
                                        /* sigma_RpRc = */ {0.0F, 0.1F, 0.0F},
                                        /* spinAxis_Rp = */ Eigen::Vector3f{0.3F, 0.1F, -0.2F}.stableNormalized(),
                                        /* spinRate = */ 0.374166F);
}

TEST(MrpRotationTest, OmegaRNDecomposesCorrectly) {
    propertyOmegaRNDecomposesCorrectly(/* sigma_RcR0 = */ {0.1F, 0.2F, -0.1F},
                                       /* sigma_RpRc = */ {0.0F, -0.05F, 0.02F},
                                       /* spinAxis_Rp = */ Eigen::Vector3f{0.05F, -0.03F, 0.02F}.stableNormalized(),
                                       /* spinRate = */ -0.061644F,
                                       /* sigma_R0N = */ {0.05F, 0.1F, 0.0F},
                                       /* omega_R0N_N = */ {0.02F, 0.0F, -0.01F});
}

TEST(MrpRotationTest, OutputRefEqualsInputRefWhenRotationIsZero) {
    propertyOutputRefEqualsInputRefWhenRotationIsZero(/* sigma_R0N = */ {0.1F, 0.2F, -0.3F},
                                                      /* omega_R0N_N = */ {0.05F, -0.02F, 0.01F},
                                                      /* domega_R0N_N = */ {0.0F, 0.03F, -0.01F});
}

TEST(MrpRotationTest, OutputEqualsOffsetInputRefWhenRateIsZero) {
    propertyOutputEqualsOffsetInputRefWhenRateIsZero(/* sigma_RcR0 = */ {0.3F, -0.4F, 0.2F},
                                                     /* sigma_RpRc = */ {0.0F, 0.0F, 0.0437F},
                                                     /* sigma_R0N = */ {0.1F, 0.2F, -0.3F});
}

TEST(MrpRotationTest, RotationAxisFixedInInputRef) {
    propertyRotationAxisFixedInInputRef(/* sigma_RcR0 = */ {0.3F, -0.4F, 0.2F},
                                        /* sigma_RpRc = */ {0.0437F, 0.0F, 0.0F},
                                        /* spinAxis_Rp = */ {1.0F, 0.0F, 0.0F},
                                        /* spinRate = */ 0.05F);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Zero rate: the output attitude does not change across updates.
TEST(MrpRotationTest, ZeroRateHoldsAttitude) {
    const auto cfg = MrpRotationConfig::create(/* sigma_RcR0 = */ Eigen::Vector3f{0.2F, 0.3F, 0.4F},
                                               /* sigma_RpRc = */ Eigen::Vector3f{0.0F, -0.1F, 0.0F},
                                               /* spinAxis_Rp = */ Eigen::Vector3f::UnitX(),
                                               /* spinRate = */ 0.0F,
                                               /* controlPeriod = */ 1.0F);
    MrpRotationAlgorithm alg{cfg};

    const MrpRotationAttRefInputs attRef{};

    const MrpRotationOutput out0 = alg.update(attRef);
    const MrpRotationOutput out1 = alg.update(attRef);
    const MrpRotationOutput out2 = alg.update(attRef);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out0.sigma_RN(i), out1.sigma_RN(i), tol);
        EXPECT_NEAR(out1.sigma_RN(i), out2.sigma_RN(i), tol);
    }
}

// reInitialize() restarts the rotation at identity, so a second pass after reInitialize matches the first.
TEST(MrpRotationTest, ReInitializeRestartsRotationForRepeatability) {
    const auto cfg = MrpRotationConfig::create(/* sigma_RcR0 = */ Eigen::Vector3f{0.1F, -0.2F, 0.3F},
                                               /* sigma_RpRc = */ Eigen::Vector3f{0.0F, 0.0F, 0.05F},
                                               /* spinAxis_Rp = */ Eigen::Vector3f::UnitX(),
                                               /* spinRate = */ 0.01F,
                                               /* controlPeriod = */ 0.5F);
    MrpRotationAlgorithm alg{cfg};

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.05F, 0.0F, 0.0F},
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
    };

    const MrpRotationOutput first0 = alg.update(attRef);
    const MrpRotationOutput first1 = alg.update(attRef);

    alg.reInitialize();
    const MrpRotationOutput second0 = alg.update(attRef);
    const MrpRotationOutput second1 = alg.update(attRef);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(first0.sigma_RN(i), second0.sigma_RN(i), tol);
        EXPECT_NEAR(first1.sigma_RN(i), second1.sigma_RN(i), tol);
    }
}

// After setConfig(cfgB) + reInitialize(), the algorithm behaves exactly like a new algorithm built from cfgB:
// the rotation restarts at identity and the step uses cfgB's offsets, rate and controlPeriod.
TEST(MrpRotationTest, ReInitializeAfterSetConfigMatchesFreshAlgorithm) {
    const auto cfgA = MrpRotationConfig::create(/* sigma_RcR0 = */ Eigen::Vector3f{0.1F, 0.0F, 0.0F},
                                                /* sigma_RpRc = */ Eigen::Vector3f{0.0F, 0.0F, 0.0F},
                                                /* spinAxis_Rp = */ Eigen::Vector3f::UnitX(),
                                                /* spinRate = */ 0.05F,
                                                /* controlPeriod = */ 0.5F);
    const auto cfgB = MrpRotationConfig::create(/* sigma_RcR0 = */ Eigen::Vector3f{0.0F, 0.5F, 0.0F},
                                                /* sigma_RpRc = */ Eigen::Vector3f{0.0F, 0.0F, 0.1F},
                                                /* spinAxis_Rp = */ Eigen::Vector3f::UnitY(),
                                                /* spinRate = */ -0.2F,
                                                /* controlPeriod = */ 0.25F);

    MrpRotationAlgorithm alg{cfgA};
    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.05F, -0.1F, 0.0F},
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
    };
    alg.update(attRef);
    alg.update(attRef);

    alg.setConfig(cfgB);
    alg.reInitialize();

    MrpRotationAlgorithm fresh{cfgB};
    for (int k = 0; k < 3; ++k) {
        const MrpRotationOutput a = alg.update(attRef);
        const MrpRotationOutput b = fresh.update(attRef);
        EXPECT_LT(dcmMaxAbsDiff(mrpToDcm(a.sigma_RN), mrpToDcm(b.sigma_RN)), 1e-6F);
    }
}

// setConfig() with a new pointing offset applies it at the next update and keeps the rotation phase: an
// algorithm that switches from offset 1 to offset 2 after k steps matches, from then on, an algorithm that
// ran with offset 2 from the start.
TEST(MrpRotationTest, SetConfigAppliesNewOffsetAndKeepsRotationPhase) {
    const Eigen::Vector3f sigma_RcR0{0.2F, -0.1F, 0.3F};
    const Eigen::Vector3f spinAxis_Rp = Eigen::Vector3f{2.0F, 0.0F, 1.0F}.stableNormalized();
    constexpr float kSpinRate = 0.0223607F;
    constexpr float kControlPeriod = 0.5F;
    const auto cfg1 = MrpRotationConfig::create(
        sigma_RcR0, /* sigma_RpRc = */ Eigen::Vector3f{0.0F, 0.0F, 0.0F}, spinAxis_Rp, kSpinRate, kControlPeriod);
    const auto cfg2 = MrpRotationConfig::create(
        sigma_RcR0, /* sigma_RpRc = */ Eigen::Vector3f{0.0437F, 0.0F, 0.0F}, spinAxis_Rp, kSpinRate, kControlPeriod);

    MrpRotationAlgorithm switched{cfg1};
    MrpRotationAlgorithm reference{cfg2};

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.05F, 0.0F, 0.0F},
        Eigen::Vector3f{0.001F, 0.0F, 0.0F},
        Eigen::Vector3f::Zero(),
    };

    for (int k = 0; k < 5; ++k) {
        const MrpRotationOutput a = switched.update(attRef);
        const MrpRotationOutput b = reference.update(attRef);
        EXPECT_GT(dcmMaxAbsDiff(mrpToDcm(a.sigma_RN), mrpToDcm(b.sigma_RN)), 1e-2F) << "offsets differ before switch";
    }

    switched.setConfig(cfg2);

    for (int k = 0; k < 5; ++k) {
        const MrpRotationOutput a = switched.update(attRef);
        const MrpRotationOutput b = reference.update(attRef);
        EXPECT_LT(dcmMaxAbsDiff(mrpToDcm(a.sigma_RN), mrpToDcm(b.sigma_RN)), 1e-6F);
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(a.omega_RN_N(i), b.omega_RN_N(i), 1e-6F);
            EXPECT_NEAR(a.domega_RN_N(i), b.domega_RN_N(i), 1e-6F);
        }
    }
}

// The body correction puts body -y on the target (input reference x-axis), the pointing offset tilts the
// boresight 10 deg about body x, and the rate points along the target line in Rp axes. The boresight then
// sweeps a cone around the target: constant 10 deg from it, reaching 20 deg from its start after half a turn.
TEST(MrpRotationTest, ConeAroundTargetKeepsConstantHalfAngle) {
    constexpr float halfAngle = 10.0F * std::numbers::pi_v<float> / 180.0F;
    constexpr float rate = 0.02F;
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 2000;  // more than 3 turns

    Eigen::Matrix3f dcm_RcR0;
    dcm_RcR0 << 0.0F, 1.0F, 0.0F, -1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;
    const Eigen::Vector3f sigma_RcR0 = dcmToMrp(dcm_RcR0);
    const Eigen::Vector3f sigma_RpRc = std::tan(halfAngle / 4.0F) * Eigen::Vector3f::UnitX();
    const Eigen::Vector3f boresight_B = -Eigen::Vector3f::UnitY();
    const Eigen::Vector3f spinAxis_Rp = mrpToDcm(sigma_RpRc) * boresight_B;

    const auto cfg = MrpRotationConfig::create(sigma_RcR0, sigma_RpRc, spinAxis_Rp, rate, kControlPeriod);
    MrpRotationAlgorithm alg{cfg};

    const Eigen::Vector3f target_R0 = Eigen::Vector3f::UnitX();
    const MrpRotationAttRefInputs identityRef{};

    Eigen::Vector3f initialBoresight_R0 = Eigen::Vector3f::Zero();
    float minCosFromStart = 1.0F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(identityRef);
        const Eigen::Vector3f boresight_R0 = mrpToDcm(out.sigma_RN).transpose() * boresight_B;
        if (k == 0) {
            initialBoresight_R0 = boresight_R0;
        }
        EXPECT_NEAR(std::acos(std::clamp(boresight_R0.dot(target_R0), -1.0F, 1.0F)), halfAngle, 1e-4F);
        minCosFromStart = std::min(minCosFromStart, boresight_R0.dot(initialBoresight_R0));
    }
    EXPECT_NEAR(minCosFromStart, std::cos(2.0F * halfAngle), 1e-3F);
}

// Direct test of the mrpSwitch path: one forward-Euler step from identity gives sigma_RRp = dt/4 * omega,
// which for this rate exceeds norm 1. The algorithm must land on the shadow-set representative.
TEST(MrpRotationTest, MrpShadowSwitchActivates) {
    const Eigen::Vector3f spinAxis_Rp = Eigen::Vector3f::UnitX();
    constexpr float kSpinRate = 5.0F;
    constexpr float kControlPeriod = 1.0F;

    const Eigen::Vector3f preSwitchSigma = kControlPeriod * 0.25F * kSpinRate * spinAxis_Rp;
    ASSERT_GT(preSwitchSigma.stableNorm(), 1.0F) << "Test setup: forward-Euler step must cross norm-1 boundary";
    const Eigen::Vector3f expectedShadow = -preSwitchSigma / preSwitchSigma.squaredNorm();

    const auto cfg = MrpRotationConfig::create(
        Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), spinAxis_Rp, kSpinRate, kControlPeriod);
    MrpRotationAlgorithm alg{cfg};

    // Identity offsets and input reference make sigma_RN equal to the internal (post-switch) sigma_RRp.
    const MrpRotationOutput out = alg.update(MrpRotationAttRefInputs{});

    EXPECT_LT(out.sigma_RN.stableNorm(), 1.0F);
    constexpr float tol = 1e-5F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.sigma_RN(i), expectedShadow(i), tol);
    }
}
