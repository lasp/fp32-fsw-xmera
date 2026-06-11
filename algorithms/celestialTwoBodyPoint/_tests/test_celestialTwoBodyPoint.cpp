#include "celestialTwoBodyPointTestHelpers.hpp"
#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Regression test
// ---------------------------------------------------------------------------

TEST(CelestialTwoBodyPointTest, RegressionTest) {
    const Eigen::Vector3d r_PN_N{-1e4, 0.0, 0.0};
    const Eigen::Vector3d v_PN_N{0.0, -3e4, 0.0};
    const Eigen::Vector3d r_SN_N{1e3, 0.0, 0.0};
    const Eigen::Vector3d v_SN_N{3e4, 0.0, 0.0};
    const Eigen::Vector3d r_BN_N{5e2, 5e3, 0.0};
    const Eigen::Vector3d v_BN_N{-1e4, 1e3, 0.0};
    const float celestialBodyAlignmentThreshold = 0.017F;

    testCelestialTwoBodyPointRegression(
        r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N, celestialBodyAlignmentThreshold);
}

// ---------------------------------------------------------------------------
// Setup tests (setter validation + round-trip)
// ---------------------------------------------------------------------------

TEST(CelestialTwoBodyPointTest, SetupTest) {
    // Valid config should not throw
    EXPECT_NO_THROW(CelestialTwoBodyPointConfig::create(0.017F));
    
    // Negative or zero celestialBodyAlignmentThreshold should throw
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(-1.0F), fsw::invalid_argument);
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(0.0F), fsw::invalid_argument);

    // Config round-trip
    const float celestialBodyAlignmentThreshold = 0.017F;
    auto config = CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold);
    EXPECT_EQ(config.getCelestialBodyAlignmentThreshold(), celestialBodyAlignmentThreshold);

    // Static validators
    EXPECT_TRUE(CelestialTwoBodyPointConfig::isValidCelestialBodyAlignmentThreshold(1.0F));
    EXPECT_FALSE(CelestialTwoBodyPointConfig::isValidCelestialBodyAlignmentThreshold(0.0F));
    EXPECT_FALSE(CelestialTwoBodyPointConfig::isValidCelestialBodyAlignmentThreshold(-1.0F));

    // Set config
    const auto config1 = CelestialTwoBodyPointConfig::create(0.017F);
    CelestialTwoBodyPointAlgorithm alg(config1);

    const auto config2 = CelestialTwoBodyPointConfig::create(0.02F);
    EXPECT_NO_THROW(alg.setConfig(config2));
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// All output components are finite for valid inputs.
TEST(CelestialTwoBodyPointTest, PropertyOutputIsFinite) {
    const Eigen::Vector3d r_PN_N{-1e4, 0.0, 0.0};
    const Eigen::Vector3d v_PN_N{0.0, -3e4, 0.0};
    const Eigen::Vector3d r_SN_N{1e3, 0.0, 0.0};
    const Eigen::Vector3d v_SN_N{3e4, 0.0, 0.0};
    const Eigen::Vector3d r_BN_N{5e2, 5e3, 0.0};
    const Eigen::Vector3d v_BN_N{-1e4, 1e3, 0.0};
    const float celestialBodyAlignmentThreshold = 0.017F;

    propertyOutputIsFinite(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N, celestialBodyAlignmentThreshold);
}

// sigma_RN norm is bounded by 1 (inner MRP set) for any inputs
TEST(CelestialTwoBodyPointTest, PropertySigmaNormBounded) {
    const Eigen::Vector3d r_PN_N{-1e4, 0.0, 0.0};
    const Eigen::Vector3d v_PN_N{0.0, -3e4, 0.0};
    const Eigen::Vector3d r_SN_N{1e3, 0.0, 0.0};
    const Eigen::Vector3d v_SN_N{3e4, 0.0, 0.0};
    const Eigen::Vector3d r_BN_N{5e2, 5e3, 0.0};
    const Eigen::Vector3d v_BN_N{-1e4, 1e3, 0.0};
    const float celestialBodyAlignmentThreshold = 0.017F;

    propertySigmaNormBounded(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N, celestialBodyAlignmentThreshold);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// When the relative position between the spacecraft and the primary celestial body is zero, identity reference
// attitude and zero reference rates are returned
TEST(CelestialTwoBodyPointTest, SpacecraftAtPrimaryReturnsZero) {
    const Eigen::Vector3d r_PN_N{-1e4, 0.0, 0.0};
    const Eigen::Vector3d v_PN_N{0.0, -3e4, 0.0};
    const Eigen::Vector3d r_SN_N{1e3, 0.0, 0.0};
    const Eigen::Vector3d v_SN_N{3e4, 0.0, 0.0};
    const Eigen::Vector3d r_BN_N{-1e4, 0.0, 0.0};
    const Eigen::Vector3d v_BN_N{-1e4, 1e3, 0.0};
    const float celestialBodyAlignmentThreshold = 0.017F;

    auto config = CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold);
    const CelestialTwoBodyPointAlgorithm alg(config);

    CelestialTwoBodyPointOutput result = alg.update(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.sigma_RN[i], 0.0F, tol);
        EXPECT_NEAR(result.omega_RN_N[i], 0.0F, tol);
        EXPECT_NEAR(result.domega_RN_N[i], 0.0F, tol);
    }
}

// When the relative position between the spacecraft and the secondary celestial body is zero, identity reference
// attitude and zero reference rates are returned
TEST(CelestialTwoBodyPointTest, SpacecraftAtSecondaryReturnsZero) {
    const Eigen::Vector3d r_PN_N{-1e4, 0.0, 0.0};
    const Eigen::Vector3d v_PN_N{0.0, -3e4, 0.0};
    const Eigen::Vector3d r_SN_N{1e3, 0.0, 0.0};
    const Eigen::Vector3d v_SN_N{3e4, 0.0, 0.0};
    const Eigen::Vector3d r_BN_N{1e3, 0.0, 0.0};
    const Eigen::Vector3d v_BN_N{-1e4, 1e3, 0.0};
    const float celestialBodyAlignmentThreshold = 0.017F;

    auto config = CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold);
    const CelestialTwoBodyPointAlgorithm alg(config);

    CelestialTwoBodyPointOutput result = alg.update(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.sigma_RN[i], 0.0F, tol);
        EXPECT_NEAR(result.omega_RN_N[i], 0.0F, tol);
        EXPECT_NEAR(result.domega_RN_N[i], 0.0F, tol);
    }
}

// When the celestial bodies are aligned, the primary body angular momentum vector is used as the
// secondary constraint axis.
TEST(CelestialTwoBodyPointTest, CelestialBodiesAlignedUsesFixedConstraintAxis) {
    const Eigen::Vector3d r_PN_N{-1e4, 0.0, 0.0};
    const Eigen::Vector3d v_PN_N{0.0, -3e4, 0.0};
    const Eigen::Vector3d r_SN_N{-1e4, 0.0, 0.0};
    const Eigen::Vector3d v_SN_N{3e4, 0.0, 0.0};
    const Eigen::Vector3d r_BN_N{5e2, 5e3, 0.0};
    const Eigen::Vector3d v_BN_N{-1e4, 1e3, 0.0};
    const float celestialBodyAlignmentThreshold = 0.017F;

    auto config = CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold);
    const CelestialTwoBodyPointAlgorithm alg(config);

    CelestialTwoBodyPointOutput result = alg.update(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N);
    ReferenceCelestialTwoBodyPointOutput expected =
        referenceCelestialTwoBodyPoint(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N, celestialBodyAlignmentThreshold);

    constexpr float tol = 1e-5F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.sigma_RN[i], static_cast<float>(expected.sigma_RN[i]), tol);
        EXPECT_NEAR(result.omega_RN_N[i], static_cast<float>(expected.omega_RN_N[i]), tol);
        EXPECT_NEAR(result.domega_RN_N[i], static_cast<float>(expected.domega_RN_N[i]), tol);
    }
}

// When the celestial bodies are aligned and the primary body angular momentum vector is undefined, identity reference
// attitude and zero reference rates are returned
TEST(CelestialTwoBodyPointTest, CelestialBodiesAlignedFallbackUndefinedReturnsZero) {
    const Eigen::Vector3d r_PN_N{1e4, 0.0, 0.0};
    const Eigen::Vector3d v_PN_N{1e4, 0.0, 0.0};
    const Eigen::Vector3d r_SN_N{-1e4, 0.0, 0.0};
    const Eigen::Vector3d v_SN_N{0.0, 3e4, 0.0};
    const Eigen::Vector3d r_BN_N{5e2, 0.0, 0.0};
    const Eigen::Vector3d v_BN_N{-1e4, 0.0, 0.0};
    const float celestialBodyAlignmentThreshold = 0.017F;

    auto config = CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold);
    const CelestialTwoBodyPointAlgorithm alg(config);

    CelestialTwoBodyPointOutput result = alg.update(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.sigma_RN[i], 0.0F, tol);
        EXPECT_NEAR(result.omega_RN_N[i], 0.0F, tol);
        EXPECT_NEAR(result.domega_RN_N[i], 0.0F, tol);
    }
}
