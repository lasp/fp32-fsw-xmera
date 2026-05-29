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

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Zero angular velocity: sigma_RR0 should not advance across updates.
TEST(MrpRotationTest, ZeroOmegaHoldsSigmaRR0) {
    const Eigen::Vector3f initialSigmaRR0{0.2F, 0.3F, 0.4F};
    const Eigen::Vector3f omegaRR0R = Eigen::Vector3f::Zero();

    const auto cfg = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, 1.0F);
    MrpRotationAlgorithm alg{cfg};
    alg.reset();

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

// Reset re-seeds runtime state from config: a second pass after reset matches the first pass.
TEST(MrpRotationTest, ResetReseedsRuntimeState) {
    const Eigen::Vector3f initialSigmaRR0{0.1F, -0.2F, 0.3F};
    const Eigen::Vector3f omegaRR0R{0.01F, 0.0F, 0.0F};

    const auto cfg = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, 0.5F);
    MrpRotationAlgorithm alg{cfg};

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.05F, 0.0F, 0.0F},
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
    };

    alg.reset();
    const MrpRotationOutput first0 = alg.update(attRef);
    const MrpRotationOutput first1 = alg.update(attRef);

    alg.reset();
    const MrpRotationOutput second0 = alg.update(attRef);
    const MrpRotationOutput second1 = alg.update(attRef);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(first0.sigma_RN(i), second0.sigma_RN(i), tol);
        EXPECT_NEAR(first1.sigma_RN(i), second1.sigma_RN(i), tol);
    }
}
