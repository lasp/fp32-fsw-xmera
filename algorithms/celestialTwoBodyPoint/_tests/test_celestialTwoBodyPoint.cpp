#include "celestialTwoBodyPointTestHelpers.hpp"
#include <gtest/gtest.h>

namespace {
constexpr double kMuEarth = 3.986004418e14;     // [m^3/s^2]
constexpr double kEarthRadius = 6.378136e6;     // [m]
constexpr float kDeg2Rad = 0.017453292519943F;  // [rad/deg]
}  // namespace

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
    const float celestialBodyAlignmentThreshold = kDeg2Rad;

    testCelestialTwoBodyPointRegression(
        r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N, celestialBodyAlignmentThreshold);
}

// ---------------------------------------------------------------------------
// Setup tests (setter validation + round-trip)
// ---------------------------------------------------------------------------

TEST(CelestialTwoBodyPointTest, SetupTest) {
    // Valid config should not throw
    EXPECT_NO_THROW(CelestialTwoBodyPointConfig::create(kDeg2Rad));

    // Negative or zero celestialBodyAlignmentThreshold should throw
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(-1.0F), fsw::invalid_argument);
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(0.0F), fsw::invalid_argument);

    // Config round-trip
    const float celestialBodyAlignmentThreshold = kDeg2Rad;
    auto config = CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold);
    EXPECT_EQ(config.getCelestialBodyAlignmentThreshold(), celestialBodyAlignmentThreshold);

    // Static validators
    EXPECT_TRUE(CelestialTwoBodyPointConfig::isValidCelestialBodyAlignmentThreshold(1.0F));
    EXPECT_FALSE(CelestialTwoBodyPointConfig::isValidCelestialBodyAlignmentThreshold(0.0F));
    EXPECT_FALSE(CelestialTwoBodyPointConfig::isValidCelestialBodyAlignmentThreshold(-1.0F));

    // Set config
    const auto config1 = CelestialTwoBodyPointConfig::create(kDeg2Rad);
    CelestialTwoBodyPointAlgorithm alg(config1);

    const auto config2 = CelestialTwoBodyPointConfig::create(0.02F);
    EXPECT_NO_THROW(alg.setConfig(config2));
}

TEST(CelestialTwoBodyPointTest, ReferenceTestWithSecondaryBody) {
    // Same primary geometry as the Python regression test, with the secondary body well away from
    // the singular (parallel / anti-parallel) configurations.
    const double a = kEarthRadius * 2.8;
    const double speed = std::sqrt(kMuEarth / a);
    const double f_rad = 60.0 * M_PI / 180.0;

    testCelestialTwoBodyPointRegression(
        Eigen::Vector3d{a * std::cos(f_rad), a * std::sin(f_rad), 0.0},           // r_PN_N
        Eigen::Vector3d{-speed * std::sin(f_rad), speed * std::cos(f_rad), 0.0},  // v_PN_N
        Eigen::Vector3d{500.0, 500.0, 500.0},                                     // r_SN_N
        Eigen::Vector3d{100.0, -10.0, 20.0},                                      // v_SN_N
        Eigen::Vector3d::Zero(),                                                  // r_BN_N
        Eigen::Vector3d::Zero(),                                                  // v_BN_N
        1.0F * kDeg2Rad);
}

TEST(CelestialTwoBodyPointTest, ReferenceTestNonZeroSpacecraftState) {
    // Spacecraft with a non-zero inertial state observing a primary body offset from the origin.
    testCelestialTwoBodyPointRegression(Eigen::Vector3d{1.5e11, 0.0, 0.0},          // r_PN_N (heliocentric Earth)
                                        Eigen::Vector3d{0.0, 2.978e4, 0.0},         // v_PN_N
                                        Eigen::Vector3d{0.0, 0.0, 0.0},             // r_SN_N (Sun at origin)
                                        Eigen::Vector3d{0.0, 0.0, 0.0},             // v_SN_N
                                        Eigen::Vector3d{1.5e11 + 7.0e6, 0.0, 0.0},  // r_BN_N (LEO offset)
                                        Eigen::Vector3d{0.0, 7.7e3, 0.0},           // v_BN_N
                                        1.0F * kDeg2Rad);
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
    const float celestialBodyAlignmentThreshold = kDeg2Rad;

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
    const float celestialBodyAlignmentThreshold = kDeg2Rad;

    propertySigmaNormBounded(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N, celestialBodyAlignmentThreshold);
}
