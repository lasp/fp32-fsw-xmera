#include "mrpRotationTestHelpers.hpp"

// ---------------------------------------------------------------------------
// Regression test: drive the algorithm through several integration steps and compare
// to the reference implementation.
// ---------------------------------------------------------------------------

TEST(MrpRotationTest, RegressionStaticFrame) {
    // Static input reference (zero rates), small constant rotation rate in R-frame.
    regressionTestMrpRotation(Eigen::Vector3f{0.3F, 0.5F, 0.0F},        // initialSigmaRR0
                              Eigen::Vector3f{0.0017453F, 0.0F, 0.0F},  // ~0.1 deg/s about x
                              Eigen::Vector3f{0.1F, 0.2F, 0.3F},        // sigma_R0N
                              Eigen::Vector3f::Zero(),                  // omega_R0N_N
                              Eigen::Vector3f::Zero(),                  // domega_R0N_N
                              0.5F,                                     // updateTimeSec
                              4);
}

TEST(MrpRotationTest, RegressionSpinningInputFrame) {
    // Spinning input reference (constant omega) plus rotation in R-frame.
    regressionTestMrpRotation(Eigen::Vector3f{0.0F, 0.0F, 0.0F},        // initialSigmaRR0
                              Eigen::Vector3f{0.01F, -0.005F, 0.002F},  // omegaRR0R
                              Eigen::Vector3f{0.05F, 0.05F, 0.0F},      // sigma_R0N
                              Eigen::Vector3f{0.1F, 0.0F, 0.0F},        // omega_R0N_N
                              Eigen::Vector3f::Zero(),                  // domega_R0N_N
                              0.25F,                                    // updateTimeSec
                              6);
}

// ---------------------------------------------------------------------------
// Setup tests: Config validators and constructor behavior.
// ---------------------------------------------------------------------------

TEST(MrpRotationConfigTest, RejectsNonFiniteInitialSigma) {
    const Eigen::Vector3f bad{std::nanf(""), 0.0F, 0.0F};
    const Eigen::Vector3f goodOmega = Eigen::Vector3f::Zero();
    EXPECT_THROW(MrpRotationConfig::create(bad, goodOmega, 0.5F), fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonFiniteOmega) {
    const Eigen::Vector3f goodSigma = Eigen::Vector3f::Zero();
    const Eigen::Vector3f bad{0.0F, std::numeric_limits<float>::infinity(), 0.0F};
    EXPECT_THROW(MrpRotationConfig::create(goodSigma, bad, 0.5F), fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonPositiveControlPeriod) {
    const Eigen::Vector3f goodSigma = Eigen::Vector3f::Zero();
    const Eigen::Vector3f goodOmega = Eigen::Vector3f::Zero();
    EXPECT_THROW(MrpRotationConfig::create(goodSigma, goodOmega, 0.0F), fsw::invalid_argument);
    EXPECT_THROW(MrpRotationConfig::create(goodSigma, goodOmega, -0.1F), fsw::invalid_argument);
    EXPECT_THROW(MrpRotationConfig::create(goodSigma, goodOmega, std::nanf("")), fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, AcceptsFiniteInputs) {
    const Eigen::Vector3f sigma{0.1F, 0.2F, 0.3F};
    const Eigen::Vector3f omega{0.01F, 0.0F, -0.02F};
    EXPECT_NO_THROW(MrpRotationConfig::create(sigma, omega, 0.5F));
}

TEST(MrpRotationConfigTest, GettersRoundTrip) {
    const Eigen::Vector3f sigma{0.1F, 0.2F, 0.3F};
    const Eigen::Vector3f omega{0.01F, 0.02F, 0.03F};
    constexpr float controlPeriod = 0.25F;
    const auto cfg = MrpRotationConfig::create(sigma, omega, controlPeriod);

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(cfg.getInitialSigmaRR0()(i), sigma(i));
        EXPECT_FLOAT_EQ(cfg.getOmegaRR0R()(i), omega(i));
    }
    EXPECT_FLOAT_EQ(cfg.getControlPeriod(), controlPeriod);
}

TEST(MrpRotationConfigTest, IsValidValidatorsHonorContracts) {
    EXPECT_TRUE(MrpRotationConfig::isValidInitialSigmaRR0(Eigen::Vector3f{0.0F, 0.0F, 0.0F}));
    EXPECT_TRUE(MrpRotationConfig::isValidInitialSigmaRR0(Eigen::Vector3f{1.5F, -2.0F, 3.5F}));
    EXPECT_FALSE(MrpRotationConfig::isValidInitialSigmaRR0(Eigen::Vector3f{std::nanf(""), 0.0F, 0.0F}));

    EXPECT_TRUE(MrpRotationConfig::isValidOmegaRR0R(Eigen::Vector3f::Zero()));
    EXPECT_FALSE(
        MrpRotationConfig::isValidOmegaRR0R(Eigen::Vector3f{0.0F, std::numeric_limits<float>::infinity(), 0.0F}));

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

TEST(MrpRotationTest, OutputIsFiniteSmallInputs) { propertyOutputIsFinite({0.1F, -0.2F, 0.3F}, {0.01F, 0.0F, 0.0F}); }

TEST(MrpRotationTest, OutputIsFiniteLargeOmega) { propertyOutputIsFinite({0.0F, 0.0F, 0.0F}, {5.0F, -3.0F, 2.0F}); }

TEST(MrpRotationTest, SigmaRNEqualsSigmaRR0WhenInputRefIsIdentity) {
    propertySigmaRNEqualsSigmaRR0WhenInputRefIsIdentity({0.3F, 0.5F, 0.0F}, {0.05F, -0.02F, 0.01F});
}

TEST(MrpRotationTest, SigmaRNNormLessOrEqualToOne) {
    // omegaRR0R chosen large enough that the 100-step run crosses the |sigma| = 1 shadow-switch
    // boundary several times.
    propertySigmaRNNormLessOrEqualToOne({0.5F, 0.0F, 0.0F}, {0.3F, 0.1F, -0.2F});
}

TEST(MrpRotationTest, OmegaRNDecomposesCorrectly) {
    propertyOmegaRNDecomposesCorrectly(
        {0.1F, 0.2F, -0.1F}, {0.05F, -0.03F, 0.02F}, {0.05F, 0.1F, 0.0F}, {0.02F, 0.0F, -0.01F});
}

TEST(MrpRotationTest, OutputRefEqualsInputRefWhenRotationIsZero) {
    propertyOutputRefEqualsInputRefWhenRotationIsZero(
        {0.1F, 0.2F, -0.3F}, {0.05F, -0.02F, 0.01F}, {0.0F, 0.03F, -0.01F});
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Zero angular velocity: sigma_RR0 should not advance across updates.
TEST(MrpRotationTest, ZeroOmegaHoldsSigmaRR0) {
    const Eigen::Vector3f initialSigmaRR0{0.2F, 0.3F, 0.4F};
    const Eigen::Vector3f omegaRR0R = Eigen::Vector3f::Zero();

    const auto cfg = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, 1.0F);
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

// setConfig() re-seeds runtime state from config: re-applying the same config returns the algorithm
// to its initial state, so a second pass after reconfiguring matches the first pass.
TEST(MrpRotationTest, SetConfigReseedsForRepeatability) {
    const Eigen::Vector3f initialSigmaRR0{0.1F, -0.2F, 0.3F};
    const Eigen::Vector3f omegaRR0R{0.01F, 0.0F, 0.0F};

    const auto cfg = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, 0.5F);
    MrpRotationAlgorithm alg{cfg};  // construction seeds runtime state from cfg

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.05F, 0.0F, 0.0F},
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
    };

    const MrpRotationOutput first0 = alg.update(attRef);
    const MrpRotationOutput first1 = alg.update(attRef);

    alg.setConfig(cfg);  // re-seed runtime state back to the configured initial values
    const MrpRotationOutput second0 = alg.update(attRef);
    const MrpRotationOutput second1 = alg.update(attRef);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(first0.sigma_RN(i), second0.sigma_RN(i), tol);
        EXPECT_NEAR(first1.sigma_RN(i), second1.sigma_RN(i), tol);
    }
}

// setConfig() re-seeds the runtime integrator state from the new configuration. To verify, run one
// step under cfgA (advancing runtime sigma_RR0 / omega_RR0_R away from its seed), swap in cfgB with
// deliberately different initial values and a different controlPeriod, then run another step. The
// output must match an independent reference that integrates from cfgB's *initial* sigma / omega
// (the prior runtime state is discarded), stepped by cfgB's controlPeriod.
TEST(MrpRotationTest, SetConfigReseedsRuntimeState) {
    const Eigen::Vector3f initialSigmaRR0_A{0.1F, 0.0F, 0.0F};
    const Eigen::Vector3f omegaRR0R_A{0.05F, 0.0F, 0.0F};
    constexpr float kPeriodA = 0.5F;
    const auto cfgA = MrpRotationConfig::create(initialSigmaRR0_A, omegaRR0R_A, kPeriodA);

    MrpRotationAlgorithm alg{cfgA};

    // sigma_R0N = 0 makes the output sigma_RN equal to the algorithm's internal sigma_RR0, so we
    // can read runtime state directly off the output.
    const MrpRotationAttRefInputs identityRef{};
    alg.update(identityRef);  // advance runtime state away from cfgA's seed

    // Swap in cfgB with very different initial sigma, different omega, and a different controlPeriod.
    const Eigen::Vector3f initialSigmaRR0_B{0.0F, 0.5F, 0.0F};
    const Eigen::Vector3f omegaRR0R_B{0.0F, 0.2F, 0.0F};
    constexpr float kPeriodB = 0.25F;
    const auto cfgB = MrpRotationConfig::create(initialSigmaRR0_B, omegaRR0R_B, kPeriodB);
    alg.setConfig(cfgB);

    const MrpRotationOutput secondOut = alg.update(identityRef);

    // Independent reference: setConfig re-seeded runtime state to cfgB's initial sigma / omega
    // (discarding the post-firstStep state), so the step integrates from there using cfgB's period.
    MrpRotationReferenceState refState{initialSigmaRR0_B, omegaRR0R_B};
    const auto refOut =
        referenceUpdate(refState, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), kPeriodB);

    constexpr float tol = 1e-5F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(secondOut.sigma_RN(i), refOut.sigma_RN(i), tol);
    }
}

// setConfig() must read its seed values from the *new* configuration. After setConfig(cfgB), the
// runtime state should be re-seeded from cfgB, not from cfgA (the original) and not from cached
// construction-time values.
TEST(MrpRotationTest, SetConfigUsesNewInitialValues) {
    const Eigen::Vector3f initialSigmaRR0_A{0.1F, 0.0F, 0.0F};
    const Eigen::Vector3f omegaRR0R_A{0.05F, 0.0F, 0.0F};
    const auto cfgA = MrpRotationConfig::create(initialSigmaRR0_A, omegaRR0R_A, 0.5F);

    MrpRotationAlgorithm alg{cfgA};
    alg.update(MrpRotationAttRefInputs{});  // advance runtime state away from initial

    // setConfig re-seeds from cfgB; cfgB has zero omega so the next step holds sigma_RR0 at cfgB's initial.
    const Eigen::Vector3f initialSigmaRR0_B{0.3F, -0.2F, 0.1F};
    const Eigen::Vector3f omegaRR0R_B = Eigen::Vector3f::Zero();
    const auto cfgB = MrpRotationConfig::create(initialSigmaRR0_B, omegaRR0R_B, 0.5F);
    alg.setConfig(cfgB);

    // With sigma_R0N = 0 and omegaRR0R = 0, sigma_RN equals the re-seeded sigma_RR0, which must
    // come from cfgB.
    const MrpRotationOutput out = alg.update(MrpRotationAttRefInputs{});

    constexpr float tol = 1e-5F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.sigma_RN(i), initialSigmaRR0_B(i), tol);
    }
}

// Direct test of the mrpSwitch path: choose sigma_RR0 near the |sigma| = 1 boundary and an
// omegaRR0R that, after one forward-Euler step, pushes the unswitched MRP past 1. Verify the
// algorithm lands on the shadow-set representative (-sigma_pre / |sigma_pre|^2).
TEST(MrpRotationTest, MrpShadowSwitchActivates) {
    const Eigen::Vector3f initialSigmaRR0{0.95F, 0.0F, 0.0F};  // close to the norm-1 boundary
    const Eigen::Vector3f omegaRR0R{1.0F, 0.0F, 0.0F};         // strong push along +x
    constexpr float kControlPeriod = 0.5F;

    // Compute the pre-mrpSwitch sigma_RR0 using the same forward-Euler formula the algorithm
    // applies, and confirm the test setup actually crosses the |sigma| = 1 boundary.
    const Eigen::Matrix3f B = bmatMrp(initialSigmaRR0);
    const Eigen::Vector3f preSwitchSigma = initialSigmaRR0 + kControlPeriod * 0.25F * B * omegaRR0R;
    ASSERT_GT(preSwitchSigma.norm(), 1.0F) << "Test setup: forward-Euler step must cross norm-1 boundary";

    const Eigen::Vector3f expectedShadow = -preSwitchSigma / preSwitchSigma.squaredNorm();

    const auto cfg = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, kControlPeriod);
    MrpRotationAlgorithm alg{cfg};

    // sigma_R0N = 0 makes sigma_RN equal to the internal (post-switch) sigma_RR0.
    const MrpRotationOutput out = alg.update(MrpRotationAttRefInputs{});

    EXPECT_LT(out.sigma_RN.norm(), 1.0F);
    constexpr float tol = 1e-5F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.sigma_RN(i), expectedShadow(i), tol);
    }
}
