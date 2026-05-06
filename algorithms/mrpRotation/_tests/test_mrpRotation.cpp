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
    EXPECT_THROW(MrpRotationConfig::create(bad, goodOmega, false), fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonFiniteOmega) {
    const Eigen::Vector3f goodSigma = Eigen::Vector3f::Zero();
    const Eigen::Vector3f bad{0.0F, std::numeric_limits<float>::infinity(), 0.0F};
    EXPECT_THROW(MrpRotationConfig::create(goodSigma, bad, false), fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, AcceptsFiniteInputs) {
    const Eigen::Vector3f sigma{0.1F, 0.2F, 0.3F};
    const Eigen::Vector3f omega{0.01F, 0.0F, -0.02F};
    EXPECT_NO_THROW(MrpRotationConfig::create(sigma, omega, true));
}

TEST(MrpRotationConfigTest, GettersRoundTrip) {
    const Eigen::Vector3f sigma{0.1F, 0.2F, 0.3F};
    const Eigen::Vector3f omega{0.01F, 0.02F, 0.03F};
    const auto cfg = MrpRotationConfig::create(sigma, omega, true);

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(cfg.getInitialSigmaRR0()(i), sigma(i));
        EXPECT_FLOAT_EQ(cfg.getOmegaRR0R()(i), omega(i));
    }
    EXPECT_TRUE(cfg.getDynamicReferenceEnabled());
}

TEST(MrpRotationConfigTest, IsValidValidatorsHonorContracts) {
    EXPECT_TRUE(MrpRotationConfig::isValidInitialSigmaRR0(Eigen::Vector3f{0.0F, 0.0F, 0.0F}));
    EXPECT_TRUE(MrpRotationConfig::isValidInitialSigmaRR0(Eigen::Vector3f{1.5F, -2.0F, 3.5F}));
    EXPECT_FALSE(MrpRotationConfig::isValidInitialSigmaRR0(Eigen::Vector3f{std::nanf(""), 0.0F, 0.0F}));

    EXPECT_TRUE(MrpRotationConfig::isValidOmegaRR0R(Eigen::Vector3f::Zero()));
    EXPECT_FALSE(
        MrpRotationConfig::isValidOmegaRR0R(Eigen::Vector3f{0.0F, std::numeric_limits<float>::infinity(), 0.0F}));
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

TEST(MrpRotationTest, OutputIsFiniteSmallInputs) { propertyOutputIsFinite({0.1F, -0.2F, 0.3F}, {0.01F, 0.0F, 0.0F}); }

TEST(MrpRotationTest, OutputIsFiniteLargeOmega) { propertyOutputIsFinite({0.0F, 0.0F, 0.0F}, {5.0F, -3.0F, 2.0F}); }

TEST(MrpRotationTest, FirstStepNoIntegration) {
    propertyFirstStepNoIntegration({0.3F, 0.5F, 0.0F}, {0.1F, 0.2F, 0.3F});
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Zero angular velocity: sigma_RR0 should not advance across updates.
TEST(MrpRotationTest, ZeroOmegaHoldsSigmaRR0) {
    const Eigen::Vector3f initialSigmaRR0{0.2F, 0.3F, 0.4F};
    const Eigen::Vector3f omegaRR0R = Eigen::Vector3f::Zero();

    const auto cfg = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, false);
    MrpRotationAlgorithm alg{cfg};
    alg.reset();

    const auto inputRef = buildAttRef(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    const AttStateMsgF32Payload emptyState{};

    const AttRefMsgF32Payload out0 = alg.update(0, inputRef, emptyState);
    const AttRefMsgF32Payload out1 = alg.update(static_cast<uint64_t>(1.0 * kSec2Nano), inputRef, emptyState);
    const AttRefMsgF32Payload out2 = alg.update(static_cast<uint64_t>(2.0 * kSec2Nano), inputRef, emptyState);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out0.sigma_RN[i], out1.sigma_RN[i], tol);
        EXPECT_NEAR(out1.sigma_RN[i], out2.sigma_RN[i], tol);
    }
}

// Reset re-seeds runtime state from config: a second pass after reset matches the first pass.
TEST(MrpRotationTest, ResetReseedsRuntimeState) {
    const Eigen::Vector3f initialSigmaRR0{0.1F, -0.2F, 0.3F};
    const Eigen::Vector3f omegaRR0R{0.01F, 0.0F, 0.0F};

    const auto cfg = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, false);
    MrpRotationAlgorithm alg{cfg};

    const auto inputRef =
        buildAttRef(Eigen::Vector3f{0.05F, 0.0F, 0.0F}, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    const AttStateMsgF32Payload emptyState{};

    alg.reset();
    const AttRefMsgF32Payload first0 = alg.update(0, inputRef, emptyState);
    const AttRefMsgF32Payload first1 = alg.update(static_cast<uint64_t>(0.5 * kSec2Nano), inputRef, emptyState);

    alg.reset();
    const AttRefMsgF32Payload second0 = alg.update(0, inputRef, emptyState);
    const AttRefMsgF32Payload second1 = alg.update(static_cast<uint64_t>(0.5 * kSec2Nano), inputRef, emptyState);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(first0.sigma_RN[i], second0.sigma_RN[i], tol);
        EXPECT_NEAR(first1.sigma_RN[i], second1.sigma_RN[i], tol);
    }
}

// Dynamic reference path: when enabled, the algorithm latches incoming command set and rates.
TEST(MrpRotationTest, DynamicReferenceLatchesNewCommand) {
    const Eigen::Vector3f initialSigmaRR0 = Eigen::Vector3f::Zero();
    const Eigen::Vector3f omegaRR0R = Eigen::Vector3f::Zero();

    const auto cfg = MrpRotationConfig::create(initialSigmaRR0, omegaRR0R, true);
    MrpRotationAlgorithm alg{cfg};
    alg.reset();

    const auto inputRef = buildAttRef(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());

    // First update: zero command -- algorithm sigma_RR0 stays at the initial value (zero).
    const auto cmd0 = buildAttState(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    const AttRefMsgF32Payload out0 = alg.update(0, inputRef, cmd0);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out0.sigma_RN[i], 0.0F, 1e-6F);
    }

    // Second update: a new command is provided; latched sigma_RR0 holds the new set when
    // the commanded rate is zero.
    const auto cmd1 = buildAttState(Eigen::Vector3f{0.4F, 0.0F, 0.0F}, Eigen::Vector3f::Zero());
    const AttRefMsgF32Payload out1 = alg.update(static_cast<uint64_t>(0.5 * kSec2Nano), inputRef, cmd1);

    EXPECT_NEAR(out1.sigma_RN[0], 0.4F, 1e-5F);
    EXPECT_NEAR(out1.sigma_RN[1], 0.0F, 1e-5F);
    EXPECT_NEAR(out1.sigma_RN[2], 0.0F, 1e-5F);
}
