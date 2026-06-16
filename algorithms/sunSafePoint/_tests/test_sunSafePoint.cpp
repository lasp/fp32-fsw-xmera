#include "sunSafePointTestHelpers.hpp"

// ---------------------------------------------------------------------------
// Regression test
// ---------------------------------------------------------------------------

TEST(SunSafePointTest, RegressionTest) {
    regressionTestSunSafePoint({1.0F, 1.0F, 0.0F},     // sunVec
                               {0.01F, 0.50F, -0.2F},  // omega_BN_B
                               0.0F,                   // sunAxisSpinRate
                               {0.0F, 0.0F, 1.0F},     // sHatBdyCmd
                               {0.0F, 0.0F, 0.0F}      // omega_RN_B_cfg
    );
}

// ---------------------------------------------------------------------------
// Setup tests (setter validation + round-trip)
// ---------------------------------------------------------------------------

TEST(SunSafePointTest, SetupTest) {
    // The validated config exposes each pointing parameter; sHatBdyCmd is stored normalized.
    const auto cfg = makeSearchConfig(
        defaultRotations(), Eigen::Vector3f{0.6F, 0.8F, 0.0F}, 1.5F, Eigen::Vector3f{0.1F, -0.2F, 0.3F}, 7);

    EXPECT_FLOAT_EQ(cfg.getSunAxisSpinRate(), 1.5F);
    EXPECT_EQ(cfg.getObservationThreshold(), 7);

    Eigen::Vector3f const expectedOmega{0.1F, -0.2F, 0.3F};
    Eigen::Vector3f retrieved_omega = cfg.getOmega_RN_B();
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(retrieved_omega(i), expectedOmega(i));
    }

    Eigen::Vector3f retrieved_sHat = cfg.getSHatBdyCmd();
    EXPECT_NEAR(retrieved_sHat.norm(), 1.0F, 1e-6F);
    EXPECT_NEAR(retrieved_sHat(0), 0.6F, 1e-6F);
    EXPECT_NEAR(retrieved_sHat(1), 0.8F, 1e-6F);
    EXPECT_NEAR(retrieved_sHat(2), 0.0F, 1e-6F);

    // create() rejects a non-unit sHatBdyCmd.
    EXPECT_THROW((void)makeSearchConfig(defaultRotations(), Eigen::Vector3f::Zero()), fsw::invalid_argument);
    EXPECT_THROW((void)makeSearchConfig(defaultRotations(), Eigen::Vector3f{2.0F, 0.0F, 0.0F}), fsw::invalid_argument);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// sigma_BR norm is bounded by 1 (inner MRP set) for any visible sun vector.
TEST(SunSafePointTest, SigmaBrNormBounded) { propertySigmaBrNormBounded({1.0F, 1.0F, 0.0F}); }

// omega_BR_B always equals omega_BN_B - omega_RN_B.
TEST(SunSafePointTest, OmegaBrIdentity) { propertyOmegaBrIdentity({1.0F, 1.0F, 0.0F}, {0.5F, -0.3F, 0.1F}); }

// All output components are finite for valid inputs.
TEST(SunSafePointTest, OutputIsFinite) { propertyOutputIsFinite({1.0F, 0.0F, 0.0F}); }

// sigma_BR is zero when sun is not visible.
TEST(SunSafePointTest, SigmaBrZeroWhenSunNotVisible) {
    SunSafePointAlgorithm alg{makeSearchConfig(
        defaultRotations(), Eigen::Vector3f{0.0F, 0.0F, 1.0F}, 0.0F, Eigen::Vector3f{0.1F, 0.0F, 0.0F})};

    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    auto output = pointUpdate(alg, Eigen::Vector3f::Zero(), omega_BN_B);
    EXPECT_FLOAT_EQ(output.sigma_BR.norm(), 0.0F);
}

// sigma_BR is zero when sun direction is aligned with sHatBdyCmd.
TEST(SunSafePointTest, SigmaBrZeroWhenAligned) {
    SunSafePointAlgorithm alg{defaultSearchConfig()};

    // Sun direction exactly along sHatBdyCmd
    Eigen::Vector3f sunVec{0.0F, 0.0F, 5.0F};

    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    auto output = pointUpdate(alg, sunVec, omega_BN_B);
    EXPECT_NEAR(output.sigma_BR.norm(), 0.0F, 1e-6F);
}

// In the normal case, sigma_BR direction is orthogonal to both sunVec and sHatBdyCmd.
TEST(SunSafePointTest, SigmaBrOrthogonalToBothVectors) {
    const auto cfg = defaultSearchConfig();
    SunSafePointAlgorithm alg{cfg};

    // Normal case: not aligned, not opposite
    Eigen::Vector3f sunVec{1.0F, 1.0F, 0.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = pointUpdate(alg, sunVec, omega_BN_B);

    Eigen::Vector3f sHat = cfg.getSHatBdyCmd();
    Eigen::Vector3f sigmaDir = output.sigma_BR.normalized();

    EXPECT_NEAR(sigmaDir.dot(sunVec.normalized()), 0.0F, 1e-5F);
    EXPECT_NEAR(sigmaDir.dot(sHat), 0.0F, 1e-5F);
}

// When sunAxisSpinRate != 0 and sun is visible, omega_RN_B is parallel to vehSunPntBdy.
TEST(SunSafePointTest, OmegaRnParallelToSunVec) {
    SunSafePointAlgorithm alg{makeSearchConfig(defaultRotations(), Eigen::Vector3f{0.0F, 0.0F, 1.0F}, 2.0F)};

    Eigen::Vector3f sunVec{1.0F, 2.0F, 3.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = pointUpdate(alg, sunVec, omega_BN_B);

    // omega_RN_B should be parallel to sunVec: cross product should be zero
    Eigen::Vector3f cross = output.omega_RN_B.cross(sunVec);
    EXPECT_NEAR(cross.norm(), 0.0F, 1e-5F);

    // Magnitude should be |sunAxisSpinRate|
    EXPECT_NEAR(output.omega_RN_B.norm(), 2.0F, 1e-5F);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Sun exactly at 180° from sHatBdyCmd uses the unitOrthogonal() fallback axis.
TEST(SunSafePointTest, SunOpposite180) {
    const auto cfg = defaultSearchConfig();
    SunSafePointAlgorithm alg{cfg};

    Eigen::Vector3f sunVec{0.0F, 0.0F, -1.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = pointUpdate(alg, sunVec, omega_BN_B);

    // sigma_BR should be non-zero (180° rotation)
    EXPECT_GT(output.sigma_BR.norm(), 0.1F);
    EXPECT_LE(output.sigma_BR.norm(), 1.0F + 1e-6F);

    // Verify against reference
    Eigen::Vector3f sHat = cfg.getSHatBdyCmd();
    auto reference = referenceUpdate(sunVec, omega_BN_B, 0.0F, sHat, Eigen::Vector3f::Zero());
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(output.sigma_BR(i), reference.sigma_BR(i), 1e-5F);
    }
}

// Sun exactly aligned with sHatBdyCmd gives zero attitude error.
TEST(SunSafePointTest, SunExactlyAligned) {
    SunSafePointAlgorithm alg{defaultSearchConfig()};

    Eigen::Vector3f sunVec{0.0F, 0.0F, 1.0F};
    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    auto output = pointUpdate(alg, sunVec, omega_BN_B);

    EXPECT_FLOAT_EQ(output.sigma_BR(0), 0.0F);
    EXPECT_FLOAT_EQ(output.sigma_BR(1), 0.0F);
    EXPECT_FLOAT_EQ(output.sigma_BR(2), 0.0F);
}

// Very small non-zero sun vector is still visible, zero vector is not.
TEST(SunSafePointTest, SmallNonZeroSunVectorIsVisible) {
    SunSafePointAlgorithm alg{makeSearchConfig(
        defaultRotations(), Eigen::Vector3f{0.0F, 0.0F, 1.0F}, 0.0F, Eigen::Vector3f{0.5F, 0.0F, 0.0F})};

    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};

    // Very small non-zero sun vector: still visible
    Eigen::Vector3f sunSmall{1e-6F, 0.0F, 0.0F};
    auto outputSmall = pointUpdate(alg, sunSmall, omega_BN_B);
    EXPECT_GT(outputSmall.sigma_BR.norm(), 0.0F);

    // Zero vector: not visible
    auto outputZero = pointUpdate(alg, Eigen::Vector3f::Zero(), omega_BN_B);
    EXPECT_FLOAT_EQ(outputZero.sigma_BR.norm(), 0.0F);
    // omega_RN_B should be the configured fallback
    EXPECT_FLOAT_EQ(outputZero.omega_RN_B(0), 0.5F);
}

// sHatBdyCmd along x-axis with sun at 180° opposition: produces a finite, non-zero sigma_BR.
TEST(SunSafePointTest, SHatAlongXAxis180Opposition) {
    SunSafePointAlgorithm alg{makeSearchConfig(defaultRotations(), Eigen::Vector3f{1.0F, 0.0F, 0.0F})};

    // Sun opposite to sHat (along -x): exercises the unitOrthogonal() fallback branch
    Eigen::Vector3f sunVec{-1.0F, 0.0F, 0.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = pointUpdate(alg, sunVec, omega_BN_B);

    // sigma_BR should be non-zero and finite
    EXPECT_GT(output.sigma_BR.norm(), 0.1F);
    EXPECT_TRUE(std::isfinite(output.sigma_BR(0)));
    EXPECT_TRUE(std::isfinite(output.sigma_BR(1)));
    EXPECT_TRUE(std::isfinite(output.sigma_BR(2)));
}

// Large unnormalized sun vector should produce correct results.
TEST(SunSafePointTest, LargeUnnormalizedSunVector) {
    SunSafePointAlgorithm alg{defaultSearchConfig()};

    Eigen::Vector3f sunVec{1000.0F, 1000.0F, 0.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = pointUpdate(alg, sunVec, omega_BN_B);

    // Should produce same sigma_BR as unit-magnitude version (angle is the same)
    Eigen::Vector3f sunVecSmall{1.0F, 1.0F, 0.0F};
    auto outputSmall = pointUpdate(alg, sunVecSmall, omega_BN_B);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(output.sigma_BR(i), outputSmall.sigma_BR(i), 1e-5F);
    }
}

// Zero sunAxisSpinRate produces zero omega_RN_B when sun is visible.
TEST(SunSafePointTest, ZeroSpinRateZeroOmegaRn) {
    SunSafePointAlgorithm alg{defaultSearchConfig()};

    Eigen::Vector3f sunVec{1.0F, 1.0F, 0.0F};
    Eigen::Vector3f omega_BN_B{0.5F, -0.3F, 0.1F};
    auto output = pointUpdate(alg, sunVec, omega_BN_B);

    EXPECT_FLOAT_EQ(output.omega_RN_B(0), 0.0F);
    EXPECT_FLOAT_EQ(output.omega_RN_B(1), 0.0F);
    EXPECT_FLOAT_EQ(output.omega_RN_B(2), 0.0F);

    // omega_BR_B should equal omega_BN_B
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(output.omega_BR_B(i), omega_BN_B(i));
    }
}

// Negative sunAxisSpinRate produces omega_RN_B anti-parallel to sun vector.
TEST(SunSafePointTest, NegativeSpinRate) {
    SunSafePointAlgorithm alg{makeSearchConfig(defaultRotations(), Eigen::Vector3f{0.0F, 0.0F, 1.0F}, -2.0F)};

    Eigen::Vector3f sunVec{1.0F, 0.0F, 0.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = pointUpdate(alg, sunVec, omega_BN_B);

    // omega_RN_B should be anti-parallel to sunVec
    EXPECT_LT(output.omega_RN_B(0), 0.0F);
    EXPECT_NEAR(output.omega_RN_B.norm(), 2.0F, 1e-5F);
}

// ---------------------------------------------------------------------------
// Search-mechanics tests (ported verbatim from the former sunSearch module)
// ---------------------------------------------------------------------------

TEST(SunSafePointTest, ReferenceTest) {
    // numSteps trimmed (24 s) to stay inside the 25 s sequence (pure search phase).
    testSearchSequence(/* rotationTimes */ {5.0F, 10.0F, 7.0F, 3.0F},
                       /* rotationRates */ {0.1F, 0.2F, 0.15F, 0.05F},
                       /* rotationAxes */ {0, 1, 2, 0},
                       /* omega_BN_B  */ Eigen::Vector3f{0.01F, -0.02F, 0.03F},
                       /* dt           */ 0.1F,
                       /* numSteps     */ 240);
}

TEST(SunSafePointTest, SearchConfigSetupTest) { searchConfigValidationChecks(); }

TEST(SunSafePointTest, BodyRateErrorMatchesDefinition) {
    // omega_BR_B = omega_BN_B - omega_RN_B at every step, for non-trivial body rate.
    // numSteps trimmed (19 s) to stay inside the 20 s sequence.
    testSearchSequence(
        {5.0F, 5.0F, 5.0F, 5.0F}, {0.1F, 0.2F, 0.3F, 0.4F}, {0, 1, 2, 0}, Eigen::Vector3f{0.5F, -0.4F, 0.3F}, 0.5F, 38);
}

TEST(SunSafePointTest, NegativeRateProducesSignedOmegaComponent) {
    // Signed rotationRate selects rotation direction along the chosen axis.
    const auto rotations = buildRotations({10.0F, 10.0F, 10.0F, 10.0F}, {-0.5F, 0.1F, 0.1F, 0.1F}, {2, 0, 0, 0});
    SunSafePointAlgorithm alg{makeSearchConfig(rotations)};

    const uint64_t startTime = 1000U;
    (void)alg.update(startTime, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0);

    const uint64_t midSlotZero = startTime + static_cast<uint64_t>(5.0F * kSec2NanoF);
    const SunSafePointOutput out = alg.update(midSlotZero, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0);
    EXPECT_NEAR(out.omega_RN_B[2], -0.5F, 1e-6);  // b3Hat_B
    EXPECT_NEAR(out.omega_RN_B[0], 0.0F, 1e-6);
    EXPECT_NEAR(out.omega_RN_B[1], 0.0F, 1e-6);
}

TEST(SunSafePointTest, ReInitializeReArmsStartTime) {
    // After setConfig + reInitialize, the next update() must latch a fresh sequence start time so the
    // new sequence begins from elapsed = 0.
    const auto rotations1 = buildRotations({1.0F, 1.0F, 1.0F, 1.0F}, {0.1F, 0.2F, 0.3F, 0.4F}, {0, 1, 2, 0});
    SunSafePointAlgorithm alg{makeSearchConfig(rotations1)};

    (void)alg.update(1000U, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0);
    (void)alg.update(
        1000U + static_cast<uint64_t>(0.5F * kSec2NanoF), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0);

    const auto rotations2 = buildRotations({2.0F, 2.0F, 2.0F, 2.0F}, {1.0F, 1.0F, 1.0F, 1.0F}, {1, 1, 1, 1});
    alg.setConfig(makeSearchConfig(rotations2));
    alg.reInitialize();

    // Far-future absolute time; if start was re-armed, elapsed = 0 and we are in slot 0 of cfg2.
    const uint64_t newCall = 1000U + static_cast<uint64_t>(100.0F * kSec2NanoF);
    const SunSafePointOutput out = alg.update(newCall, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0);
    EXPECT_NEAR(out.omega_RN_B[1], 1.0F, 1e-6);  // slot 0 of cfg2: rate 1 about b2Hat_B
    EXPECT_NEAR(out.omega_RN_B[0], 0.0F, 1e-6);
    EXPECT_NEAR(out.omega_RN_B[2], 0.0F, 1e-6);
}

TEST(SunSafePointTest, ConfigPreservesAxisAndRateRoundTrip) {
    // The Config getter should return exactly what was supplied.
    const auto rotations = buildRotations({1.0F, 2.0F, 3.0F, 4.0F}, {0.1F, -0.2F, 0.3F, -0.4F}, {0, 1, 2, 0});
    const SunSafePointConfig cfg = makeSearchConfig(rotations);
    for (uint32_t i = 0U; i < kNumRotations; ++i) {
        EXPECT_FLOAT_EQ(cfg.getRotations().at(i).rotationDuration, rotations[i].rotationDuration);
        EXPECT_FLOAT_EQ(cfg.getRotations().at(i).rotationRate, rotations[i].rotationRate);
        EXPECT_EQ(cfg.getRotations().at(i).rotationAxis, rotations[i].rotationAxis);
    }
}

// ---------------------------------------------------------------------------
// State-machine tests (search -> point transition)
// Canonical search config: four 10 s rotations -> rotationEndTimes = {10, 20, 30, 40} s.
// ---------------------------------------------------------------------------

TEST(SunSafePointTest, NoTransitionDuringFirstRotation) {
    const auto rotations = buildRotations({10.0F, 10.0F, 10.0F, 10.0F}, {0.1F, 0.2F, 0.3F, 0.4F}, {0, 1, 2, 0});
    SunSafePointAlgorithm alg{makeSearchConfig(rotations)};

    const uint64_t startTime = 1000U;
    (void)alg.update(startTime, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0);  // latch start

    // Mid rotation 1 (5 s), observations far above threshold: must NOT transition yet.
    const uint64_t midRotation1 = startTime + static_cast<uint64_t>(5.0F * kSec2NanoF);
    const Eigen::Vector3f sun{1.0F, 1.0F, 0.0F};
    const Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    const SunSafePointOutput out = alg.update(midRotation1, sun, omega_BN_B, 100);

    // Still SEARCH: zero attitude error, omega_RN_B = rotation-1 rate (0.1 about b1), no fault.
    EXPECT_FLOAT_EQ(out.sigma_BR.norm(), 0.0F);
    EXPECT_NEAR(out.omega_RN_B[0], 0.1F, 1e-6F);
    EXPECT_NEAR(out.omega_RN_B[1], 0.0F, 1e-6F);
    EXPECT_NEAR(out.omega_RN_B[2], 0.0F, 1e-6F);
    EXPECT_FALSE(out.faultDetected);
}

TEST(SunSafePointTest, StaysSearchingBelowThreshold) {
    const auto rotations = buildRotations({10.0F, 10.0F, 10.0F, 10.0F}, {0.1F, 0.2F, 0.3F, 0.4F}, {0, 1, 2, 0});
    SunSafePointAlgorithm alg{makeSearchConfig(rotations)};

    const uint64_t startTime = 1000U;
    const Eigen::Vector3f sun{1.0F, 1.0F, 0.0F};
    (void)alg.update(startTime, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 3);  // latch start

    // Observations below the threshold (3 < 4) must NOT transition. Sample inside rotations 2, 3
    // and 4 (all before the 40 s sequence end): the gate stays closed and search advances
    // rotation-by-rotation.
    auto checkSearch = [&](float tSec, const Eigen::Vector3f& expectedOmegaRN) {
        const uint64_t callTime = startTime + static_cast<uint64_t>(tSec * kSec2NanoF);
        const SunSafePointOutput out = alg.update(callTime, sun, Eigen::Vector3f::Zero(), 3);
        EXPECT_FLOAT_EQ(out.sigma_BR.norm(), 0.0F);
        EXPECT_NEAR(out.omega_RN_B[0], expectedOmegaRN[0], 1e-6F);
        EXPECT_NEAR(out.omega_RN_B[1], expectedOmegaRN[1], 1e-6F);
        EXPECT_NEAR(out.omega_RN_B[2], expectedOmegaRN[2], 1e-6F);
    };
    checkSearch(15.0F, Eigen::Vector3f{0.0F, 0.2F, 0.0F});  // rotation 2 about b2
    checkSearch(25.0F, Eigen::Vector3f{0.0F, 0.0F, 0.3F});  // rotation 3 about b3
    checkSearch(35.0F, Eigen::Vector3f{0.4F, 0.0F, 0.0F});  // rotation 4 about b1
}

TEST(SunSafePointTest, ForcedTransitionAfterAllRotations) {
    const auto rotations = buildRotations({10.0F, 10.0F, 10.0F, 10.0F}, {0.1F, 0.2F, 0.3F, 0.4F}, {0, 1, 2, 0});
    // sHatBdyCmd {0,0,1}, spin rate 0 -> pointing omega_RN_B = 0
    const auto cfg = makeSearchConfig(rotations);
    SunSafePointAlgorithm alg{cfg};

    const uint64_t startTime = 1000U;
    const Eigen::Vector3f sun{1.0F, 1.0F, 0.0F};
    const Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    (void)alg.update(startTime, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0);  // latch start

    // Past the full 40 s sequence with observations never exceeding threshold: forced POINT.
    const uint64_t pastSequence = startTime + static_cast<uint64_t>(45.0F * kSec2NanoF);
    const SunSafePointOutput out = alg.update(pastSequence, sun, omega_BN_B, 0);

    // Pointing output (matches the reference), NOT the held last-rotation rate (0.4 about b1).
    const auto reference = referenceUpdate(sun, omega_BN_B, 0.0F, cfg.getSHatBdyCmd(), Eigen::Vector3f::Zero());
    EXPECT_GT(out.sigma_BR.norm(), 0.0F);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.sigma_BR[i], reference.sigma_BR[i], 1e-5F);
    }
    EXPECT_NEAR(out.omega_RN_B.norm(), 0.0F, 1e-6F);
    EXPECT_TRUE(out.faultDetected);  // search failed (sun never acquired) -> fault latched
}
