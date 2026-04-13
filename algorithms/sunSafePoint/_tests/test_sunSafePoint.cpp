#include "sunSafePointTestHelpers.hpp"

// ---------------------------------------------------------------------------
// Regression test
// ---------------------------------------------------------------------------

TEST(SunSafePointTest, RegressionTest) {
    regressionTestSunSafePoint(
        {1.0F, 1.0F, 0.0F},    // sunVec
        {0.01F, 0.50F, -0.2F}, // omega_BN_B
        0.01F,                  // smallAngle (~0.01 deg in rad)
        0.0F,                   // sunAxisSpinRate
        {0.0F, 0.0F, 1.0F},   // sHatBdyCmd
        {0.0F, 0.0F, 0.0F}    // omega_RN_B_cfg
    );
}

// ---------------------------------------------------------------------------
// Setup tests (setter validation + round-trip)
// ---------------------------------------------------------------------------

TEST(SunSafePointTest, SetupTest) {
    SunSafePointAlgorithm alg{};

    // smallAngle: 0 and negative should throw
    EXPECT_THROW(alg.setSmallAngle(0.0F), fsw::invalid_argument);
    EXPECT_THROW(alg.setSmallAngle(-0.01F), fsw::invalid_argument);

    // sHatBdyCmd: zero vector should throw
    EXPECT_THROW(alg.setSHatBdyCmd(Eigen::Vector3f::Zero()), fsw::invalid_argument);

    // sunAxisSpinRate and omega_RN_B: no validation, should not throw
    EXPECT_NO_THROW(alg.setSunAxisSpinRate(-5.0F));
    EXPECT_NO_THROW(alg.setSunAxisSpinRate(0.0F));
    EXPECT_NO_THROW(alg.setOmega_RN_B(Eigen::Vector3f{100.0F, -200.0F, 300.0F}));

    // Getter/setter round-trips
    alg.setSmallAngle(0.02F);
    EXPECT_FLOAT_EQ(alg.getSmallAngle(), 0.02F);

    alg.setSunAxisSpinRate(1.5F);
    EXPECT_FLOAT_EQ(alg.getSunAxisSpinRate(), 1.5F);

    Eigen::Vector3f omega{0.1F, -0.2F, 0.3F};
    alg.setOmega_RN_B(omega);
    Eigen::Vector3f retrieved_omega = alg.getOmega_RN_B();
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(retrieved_omega(i), omega(i));
    }

    // sHatBdyCmd: verify auto-normalization
    alg.setSHatBdyCmd(Eigen::Vector3f{2.0F, 0.0F, 0.0F});
    Eigen::Vector3f retrieved_sHat = alg.getSHatBdyCmd();
    EXPECT_NEAR(retrieved_sHat(0), 1.0F, 1e-6F);
    EXPECT_NEAR(retrieved_sHat(1), 0.0F, 1e-6F);
    EXPECT_NEAR(retrieved_sHat(2), 0.0F, 1e-6F);

    // sHatBdyCmd: non-trivial normalization
    alg.setSHatBdyCmd(Eigen::Vector3f{3.0F, 4.0F, 0.0F});
    retrieved_sHat = alg.getSHatBdyCmd();
    EXPECT_NEAR(retrieved_sHat.norm(), 1.0F, 1e-6F);
    EXPECT_NEAR(retrieved_sHat(0), 0.6F, 1e-6F);
    EXPECT_NEAR(retrieved_sHat(1), 0.8F, 1e-6F);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// sigma_BR norm is bounded by 1 (inner MRP set) for any visible sun vector.
TEST(SunSafePointTest, SigmaBrNormBounded) {
    propertySigmaBrNormBounded({1.0F, 1.0F, 0.0F});
}

// omega_BR_B always equals omega_BN_B - omega_RN_B.
TEST(SunSafePointTest, OmegaBrIdentity) {
    propertyOmegaBrIdentity({1.0F, 1.0F, 0.0F}, {0.5F, -0.3F, 0.1F});
}

// All output components are finite for valid inputs.
TEST(SunSafePointTest, OutputIsFinite) {
    propertyOutputIsFinite({1.0F, 0.0F, 0.0F});
}

// sigma_BR is zero when sun is not visible.
TEST(SunSafePointTest, SigmaBrZeroWhenSunNotVisible) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.01F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.setOmega_RN_B(Eigen::Vector3f{0.1F, 0.0F, 0.0F});
    alg.reset();

    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    auto output = alg.update(Eigen::Vector3f::Zero(), omega_BN_B);
    EXPECT_FLOAT_EQ(output.sigma_BR.norm(), 0.0F);
}

// sigma_BR is zero when sun direction is aligned with sHatBdyCmd.
TEST(SunSafePointTest, SigmaBrZeroWhenAligned) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.01F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.reset();

    // Sun direction exactly along sHatBdyCmd
    Eigen::Vector3f sunVec{0.0F, 0.0F, 5.0F};

    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    auto output = alg.update(sunVec, omega_BN_B);
    EXPECT_NEAR(output.sigma_BR.norm(), 0.0F, 1e-6F);
}

// In the normal case, sigma_BR direction is orthogonal to both sunVec and sHatBdyCmd.
TEST(SunSafePointTest, SigmaBrOrthogonalToBothVectors) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.001F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.reset();

    // Normal case: not aligned, not opposite
    Eigen::Vector3f sunVec{1.0F, 1.0F, 0.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = alg.update(sunVec, omega_BN_B);

    Eigen::Vector3f sHat = alg.getSHatBdyCmd();
    Eigen::Vector3f sigmaDir = output.sigma_BR.normalized();

    EXPECT_NEAR(sigmaDir.dot(sunVec.normalized()), 0.0F, 1e-5F);
    EXPECT_NEAR(sigmaDir.dot(sHat), 0.0F, 1e-5F);
}

// When sunAxisSpinRate != 0 and sun is visible, omega_RN_B is parallel to vehSunPntBdy.
TEST(SunSafePointTest, OmegaRnParallelToSunVec) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.01F);
    alg.setSunAxisSpinRate(2.0F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.reset();

    Eigen::Vector3f sunVec{1.0F, 2.0F, 3.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = alg.update(sunVec, omega_BN_B);

    // omega_RN_B should be parallel to sunVec: cross product should be zero
    Eigen::Vector3f cross = output.omega_RN_B.cross(sunVec);
    EXPECT_NEAR(cross.norm(), 0.0F, 1e-5F);

    // Magnitude should be |sunAxisSpinRate|
    EXPECT_NEAR(output.omega_RN_B.norm(), 2.0F, 1e-5F);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Sun exactly at 180° from sHatBdyCmd uses eHat180_B fallback axis.
TEST(SunSafePointTest, SunOpposite180) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.01F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.reset();

    Eigen::Vector3f sunVec{0.0F, 0.0F, -1.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = alg.update(sunVec, omega_BN_B);

    // sigma_BR should be non-zero (180° rotation)
    EXPECT_GT(output.sigma_BR.norm(), 0.1F);
    EXPECT_LE(output.sigma_BR.norm(), 1.0F + 1e-6F);

    // Verify against reference
    Eigen::Vector3f sHat = alg.getSHatBdyCmd();
    Eigen::Vector3f eHat180_B = computeEHat180(sHat);
    auto reference = referenceUpdate(sunVec, omega_BN_B, 0.01F, 0.0F, sHat, Eigen::Vector3f::Zero(), eHat180_B);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(output.sigma_BR(i), reference.sigma_BR(i), 1e-5F);
    }
}

// Sun exactly aligned with sHatBdyCmd gives zero attitude error.
TEST(SunSafePointTest, SunExactlyAligned) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.01F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.reset();

    Eigen::Vector3f sunVec{0.0F, 0.0F, 1.0F};
    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    auto output = alg.update(sunVec, omega_BN_B);

    EXPECT_FLOAT_EQ(output.sigma_BR(0), 0.0F);
    EXPECT_FLOAT_EQ(output.sigma_BR(1), 0.0F);
    EXPECT_FLOAT_EQ(output.sigma_BR(2), 0.0F);
}

// Very small non-zero sun vector is still visible, zero vector is not.
TEST(SunSafePointTest, SmallNonZeroSunVectorIsVisible) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.01F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.setOmega_RN_B(Eigen::Vector3f{0.5F, 0.0F, 0.0F});
    alg.reset();

    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};

    // Very small non-zero sun vector: still visible
    Eigen::Vector3f sunSmall{1e-6F, 0.0F, 0.0F};
    auto outputSmall = alg.update(sunSmall, omega_BN_B);
    EXPECT_GT(outputSmall.sigma_BR.norm(), 0.0F);

    // Zero vector: not visible
    auto outputZero = alg.update(Eigen::Vector3f::Zero(), omega_BN_B);
    EXPECT_FLOAT_EQ(outputZero.sigma_BR.norm(), 0.0F);
    // omega_RN_B should be the configured fallback
    EXPECT_FLOAT_EQ(outputZero.omega_RN_B(0), 0.5F);
}

// sHatBdyCmd along x-axis: tests eHat180_B fallback to cross with [0,1,0].
TEST(SunSafePointTest, SHatAlongXAxisFallback) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.01F);
    alg.setSHatBdyCmd(Eigen::Vector3f{1.0F, 0.0F, 0.0F});
    alg.reset();

    // Sun opposite to sHat (along -x): should use eHat180_B computed from cross([1,0,0], [0,1,0])
    Eigen::Vector3f sunVec{-1.0F, 0.0F, 0.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = alg.update(sunVec, omega_BN_B);

    // sigma_BR should be non-zero and finite
    EXPECT_GT(output.sigma_BR.norm(), 0.1F);
    EXPECT_TRUE(std::isfinite(output.sigma_BR(0)));
    EXPECT_TRUE(std::isfinite(output.sigma_BR(1)));
    EXPECT_TRUE(std::isfinite(output.sigma_BR(2)));

    // Verify eHat180_B is orthogonal to sHat = [1,0,0]
    Eigen::Vector3f eHat180 = computeEHat180(alg.getSHatBdyCmd());
    EXPECT_NEAR(eHat180.dot(alg.getSHatBdyCmd()), 0.0F, 1e-6F);
}

// Large unnormalized sun vector should produce correct results.
TEST(SunSafePointTest, LargeUnnormalizedSunVector) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.001F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.reset();

    Eigen::Vector3f sunVec{1000.0F, 1000.0F, 0.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = alg.update(sunVec, omega_BN_B);

    // Should produce same sigma_BR as unit-magnitude version (angle is the same)
    Eigen::Vector3f sunVecSmall{1.0F, 1.0F, 0.0F};
    auto outputSmall = alg.update(sunVecSmall, omega_BN_B);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(output.sigma_BR(i), outputSmall.sigma_BR(i), 1e-5F);
    }
}

// Zero sunAxisSpinRate produces zero omega_RN_B when sun is visible.
TEST(SunSafePointTest, ZeroSpinRateZeroOmegaRn) {
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.01F);
    alg.setSunAxisSpinRate(0.0F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.reset();

    Eigen::Vector3f sunVec{1.0F, 1.0F, 0.0F};
    Eigen::Vector3f omega_BN_B{0.5F, -0.3F, 0.1F};
    auto output = alg.update(sunVec, omega_BN_B);

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
    SunSafePointAlgorithm alg{};
    alg.setSmallAngle(0.01F);
    alg.setSunAxisSpinRate(-2.0F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.reset();

    Eigen::Vector3f sunVec{1.0F, 0.0F, 0.0F};
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    auto output = alg.update(sunVec, omega_BN_B);

    // omega_RN_B should be anti-parallel to sunVec
    EXPECT_LT(output.omega_RN_B(0), 0.0F);
    EXPECT_NEAR(output.omega_RN_B.norm(), 2.0F, 1e-5F);
}
