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
