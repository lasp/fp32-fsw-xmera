#include "celestialTwoBodyPointTestHelpers.hpp"
#include <gtest/gtest.h>

namespace {
constexpr double kMuEarth = 3.986004418e14;     // [m^3/s^2]
constexpr double kEarthRadius = 6.378136e6;     // [m]
constexpr float kDeg2Rad = 0.017453292519943F;  // [rad/deg]
}  // namespace

TEST(CelestialTwoBodyPointTest, Setup) { testCelestialTwoBodyPointSetup(); }

TEST(CelestialTwoBodyPointTest, ReferenceTestWithSecondaryBody) {
    // Same primary geometry as the Python regression test, with the secondary body well away from
    // the singular (parallel / anti-parallel) configurations.
    const double a = kEarthRadius * 2.8;
    const double speed = std::sqrt(kMuEarth / a);
    const double f_rad = 60.0 * M_PI / 180.0;

    testCelestialTwoBodyPoint(Eigen::Vector3d{a * std::cos(f_rad), a * std::sin(f_rad), 0.0},           // r_PN_N
                              Eigen::Vector3d{-speed * std::sin(f_rad), speed * std::cos(f_rad), 0.0},  // v_PN_N
                              Eigen::Vector3d{500.0, 500.0, 500.0},                                     // r_SN_N
                              Eigen::Vector3d{100.0, -10.0, 20.0},                                      // v_SN_N
                              Eigen::Vector3d::Zero(),                                                  // r_BN_N
                              Eigen::Vector3d::Zero(),                                                  // v_BN_N
                              1.0F * kDeg2Rad);
}

TEST(CelestialTwoBodyPointTest, ReferenceTestNonZeroSpacecraftState) {
    // Spacecraft with a non-zero inertial state observing a primary body offset from the origin.
    testCelestialTwoBodyPoint(Eigen::Vector3d{1.5e11, 0.0, 0.0},          // r_PN_N (heliocentric Earth)
                              Eigen::Vector3d{0.0, 2.978e4, 0.0},         // v_PN_N
                              Eigen::Vector3d{0.0, 0.0, 0.0},             // r_SN_N (Sun at origin)
                              Eigen::Vector3d{0.0, 0.0, 0.0},             // v_SN_N
                              Eigen::Vector3d{1.5e11 + 7.0e6, 0.0, 0.0},  // r_BN_N (LEO offset)
                              Eigen::Vector3d{0.0, 7.7e3, 0.0},           // v_BN_N
                              1.0F * kDeg2Rad);
}

TEST(CelestialTwoBodyPointTest, ConfigValidCreation) {
    EXPECT_NO_THROW(CelestialTwoBodyPointConfig::create(0.0F));
    EXPECT_NO_THROW(CelestialTwoBodyPointConfig::create(0.5F));
}

TEST(CelestialTwoBodyPointTest, ConfigInvalidSingularityThreshold) {
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(-1.0F), fsw::invalid_argument);
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(-1e-7F), fsw::invalid_argument);
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(std::nanf("")), fsw::invalid_argument);
}

TEST(CelestialTwoBodyPointTest, ConfigRoundTrip) {
    const auto config = CelestialTwoBodyPointConfig::create(0.25F);
    EXPECT_FLOAT_EQ(config.getSingularityThreshold(), 0.25F);
}

TEST(CelestialTwoBodyPointTest, ConfigStaticValidators) {
    EXPECT_TRUE(CelestialTwoBodyPointConfig::isValidSingularityThreshold(0.0F));
    EXPECT_TRUE(CelestialTwoBodyPointConfig::isValidSingularityThreshold(1.0F));
    EXPECT_FALSE(CelestialTwoBodyPointConfig::isValidSingularityThreshold(-0.1F));
}

TEST(CelestialTwoBodyPointTest, AlgorithmSetConfig) {
    const auto config1 = CelestialTwoBodyPointConfig::create(0.1F);
    CelestialTwoBodyPointAlgorithm alg(config1);

    const auto config2 = CelestialTwoBodyPointConfig::create(0.3F);
    EXPECT_NO_THROW(alg.setConfig(config2));
}

TEST(CelestialTwoBodyPointTest, PropertyOutputIsFinite) {
    propertyOutputIsFinite({1.0e7, 2.0e6, 3.0e5},
                           {-1.0e3, 5.0e3, 2.0e2},
                           {500.0, 500.0, 500.0},
                           {100.0, -10.0, 20.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           1.0F * kDeg2Rad);
    propertyOutputIsFinite({1.5e11, 0.0, 0.0},
                           {0.0, 2.978e4, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           {1.5e11 + 7.0e6, 0.0, 0.0},
                           {0.0, 7.7e3, 0.0},
                           1.0F * kDeg2Rad);
    propertyOutputIsFinite({1.0e7, 0.0, 0.0},
                           {0.0, 5.0e3, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           1.0F * kDeg2Rad);
}

TEST(CelestialTwoBodyPointTest, PropertySigmaNormBounded) {
    propertySigmaNormBounded({1.0e7, 2.0e6, 3.0e5},
                             {-1.0e3, 5.0e3, 2.0e2},
                             {500.0, 500.0, 500.0},
                             {100.0, -10.0, 20.0},
                             {0.0, 0.0, 0.0},
                             {0.0, 0.0, 0.0},
                             1.0F * kDeg2Rad);
    propertySigmaNormBounded({-1.0e7, -2.0e6, 3.0e5},
                             {1.0e3, -5.0e3, 2.0e2},
                             {0.0, 0.0, 0.0},
                             {0.0, 0.0, 0.0},
                             {0.0, 0.0, 0.0},
                             {0.0, 0.0, 0.0},
                             1.0F * kDeg2Rad);
}
