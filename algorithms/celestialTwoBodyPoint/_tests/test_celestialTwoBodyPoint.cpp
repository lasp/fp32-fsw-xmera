#include "celestialTwoBodyPointTestHelpers.hpp"
#include <gtest/gtest.h>

namespace {
constexpr double kMuEarth = 3.986004418e14;     // [m^3/s^2]
constexpr double kEarthRadius = 6.378136e6;     // [m]
constexpr float kDeg2Rad = 0.017453292519943F;  // [rad/deg]
}  // namespace

TEST(CelestialTwoBodyPointTest, Setup) { testCelestialTwoBodyPointSetup(); }

TEST(CelestialTwoBodyPointTest, ReferenceTestNoSecondaryBody) {
    // Circular equatorial orbit at a = 2.8 R_E, true anomaly 60 deg; matches the Python regression
    // test geometry. Without a secondary body the constraint axis comes from the orbit normal.
    const double a = kEarthRadius * 2.8;
    const double speed = std::sqrt(kMuEarth / a);
    const double f_rad = 60.0 * M_PI / 180.0;

    testCelestialTwoBodyPoint(Eigen::Vector3d{a * std::cos(f_rad), a * std::sin(f_rad), 0.0},  // r_celBody_N
                              Eigen::Vector3d{-speed * std::sin(f_rad), speed * std::cos(f_rad), 0.0},
                              Eigen::Vector3d::Zero(),  // r_secCelBody_N (unused)
                              Eigen::Vector3d::Zero(),  // v_secCelBody_N (unused)
                              Eigen::Vector3d::Zero(),  // r_BN_N
                              Eigen::Vector3d::Zero(),  // v_BN_N
                              1.0F * kDeg2Rad,
                              10.0F * kDeg2Rad,
                              false);
}

TEST(CelestialTwoBodyPointTest, ReferenceTestWithSecondaryBody) {
    // Same primary geometry as the Python regression test, with the secondary body well away from
    // the singular (parallel / anti-parallel) configurations.
    const double a = kEarthRadius * 2.8;
    const double speed = std::sqrt(kMuEarth / a);
    const double f_rad = 60.0 * M_PI / 180.0;

    testCelestialTwoBodyPoint(Eigen::Vector3d{a * std::cos(f_rad), a * std::sin(f_rad), 0.0},  // r_celBody_N
                              Eigen::Vector3d{-speed * std::sin(f_rad), speed * std::cos(f_rad), 0.0},
                              Eigen::Vector3d{500.0, 500.0, 500.0},  // r_secCelBody_N
                              Eigen::Vector3d{100.0, -10.0, 20.0},   // v_secCelBody_N
                              Eigen::Vector3d::Zero(),               // r_BN_N
                              Eigen::Vector3d::Zero(),               // v_BN_N
                              1.0F * kDeg2Rad,
                              10.0F * kDeg2Rad,
                              true);
}

TEST(CelestialTwoBodyPointTest, ReferenceTestNonZeroSpacecraftState) {
    // Spacecraft with a non-zero inertial state observing a primary body offset from the origin.
    testCelestialTwoBodyPoint(Eigen::Vector3d{1.5e11, 0.0, 0.0},          // r_celBody_N (heliocentric Earth)
                              Eigen::Vector3d{0.0, 2.978e4, 0.0},         // v_celBody_N
                              Eigen::Vector3d{0.0, 0.0, 0.0},             // r_secCelBody_N (Sun at origin)
                              Eigen::Vector3d{0.0, 0.0, 0.0},             // v_secCelBody_N
                              Eigen::Vector3d{1.5e11 + 7.0e6, 0.0, 0.0},  // r_BN_N (LEO offset)
                              Eigen::Vector3d{0.0, 7.7e3, 0.0},           // v_BN_N
                              1.0F * kDeg2Rad,
                              10.0F * kDeg2Rad,
                              true);
}

TEST(CelestialTwoBodyPointTest, CircularEquatorialOrbitTruthValues) {
    // Body on a circular equatorial orbit (a = 2.8 R_E, true anomaly 60 deg) seen from a spacecraft
    // at the origin. Without a secondary body the constraint axis is the angular momentum
    // direction, giving the analytic reference frame:
    //   r1 = (cos f, sin f, 0)   (toward the primary body)
    //   r3 = (sin f, -cos f, 0)  (normal of the r1 / h plane)
    //   r2 = r3 x r1 = (0, 0, 1)
    // The frame tracks the body direction, so omega_RN_N = n * z_hat with n = speed / a, and the
    // angular acceleration is zero for a circular orbit.
    const double a = kEarthRadius * 2.8;
    const double speed = std::sqrt(kMuEarth / a);
    const double f_rad = 60.0 * M_PI / 180.0;

    const CelestialTwoBodyPointAlgorithm alg(
        CelestialTwoBodyPointConfig::create(1.0F * kDeg2Rad, 10.0F * kDeg2Rad, false));

    CelestialTwoBodyPointOutput out;
    EXPECT_NO_THROW(out = alg.update(Eigen::Vector3d{a * std::cos(f_rad), a * std::sin(f_rad), 0.0},
                                     Eigen::Vector3d{-speed * std::sin(f_rad), speed * std::cos(f_rad), 0.0},
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero()));

    Eigen::Matrix3d dcm_RN_expected;
    dcm_RN_expected.row(0) = Eigen::Vector3d{std::cos(f_rad), std::sin(f_rad), 0.0};
    dcm_RN_expected.row(1) = Eigen::Vector3d{0.0, 0.0, 1.0};
    dcm_RN_expected.row(2) = Eigen::Vector3d{std::sin(f_rad), -std::cos(f_rad), 0.0};
    const Eigen::Vector3d sigma_expected = dcmToMrp(dcm_RN_expected);

    constexpr float tol = 1e-5F;
    EXPECT_NEAR(out.sigma_RN[0], static_cast<float>(sigma_expected[0]), tol);
    EXPECT_NEAR(out.sigma_RN[1], static_cast<float>(sigma_expected[1]), tol);
    EXPECT_NEAR(out.sigma_RN[2], static_cast<float>(sigma_expected[2]), tol);

    const auto meanMotion = static_cast<float>(speed / a);
    EXPECT_NEAR(out.omega_RN_N[0], 0.0F, tol);
    EXPECT_NEAR(out.omega_RN_N[1], 0.0F, tol);
    EXPECT_NEAR(out.omega_RN_N[2], meanMotion, tol);

    // In a circular orbit the angular acceleration is zero (radial velocity is zero by construction).
    EXPECT_NEAR(out.domega_RN_N[0], 0.0F, tol);
    EXPECT_NEAR(out.domega_RN_N[1], 0.0F, tol);
    EXPECT_NEAR(out.domega_RN_N[2], 0.0F, tol);
}

TEST(CelestialTwoBodyPointTest, ConfigValidCreation) {
    EXPECT_NO_THROW(CelestialTwoBodyPointConfig::create(0.0F, 0.0F, false));
    EXPECT_NO_THROW(CelestialTwoBodyPointConfig::create(0.5F, 0.1F, true));
}

TEST(CelestialTwoBodyPointTest, ConfigInvalidSingularityThreshold) {
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(-1.0F, 0.1F, false), fsw::invalid_argument);
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(-1e-7F, 0.1F, false), fsw::invalid_argument);
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(std::nanf(""), 0.1F, false), fsw::invalid_argument);
}

TEST(CelestialTwoBodyPointTest, ConfigInvalidRateThreshold) {
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(0.1F, -1.0F, false), fsw::invalid_argument);
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(0.1F, -1e-7F, false), fsw::invalid_argument);
    EXPECT_THROW(CelestialTwoBodyPointConfig::create(0.1F, std::nanf(""), false), fsw::invalid_argument);
}

TEST(CelestialTwoBodyPointTest, ConfigRoundTrip) {
    const auto config = CelestialTwoBodyPointConfig::create(0.25F, 0.125F, true);
    EXPECT_FLOAT_EQ(config.getSingularityThreshold(), 0.25F);
    EXPECT_FLOAT_EQ(config.getRateThreshold(), 0.125F);
    EXPECT_TRUE(config.getSecCelBodyIsLinked());
}

TEST(CelestialTwoBodyPointTest, ConfigStaticValidators) {
    EXPECT_TRUE(CelestialTwoBodyPointConfig::isValidSingularityThreshold(0.0F));
    EXPECT_TRUE(CelestialTwoBodyPointConfig::isValidSingularityThreshold(1.0F));
    EXPECT_FALSE(CelestialTwoBodyPointConfig::isValidSingularityThreshold(-0.1F));
    EXPECT_TRUE(CelestialTwoBodyPointConfig::isValidRateThreshold(0.0F));
    EXPECT_TRUE(CelestialTwoBodyPointConfig::isValidRateThreshold(1.0F));
    EXPECT_FALSE(CelestialTwoBodyPointConfig::isValidRateThreshold(-0.1F));
}

TEST(CelestialTwoBodyPointTest, AlgorithmSetConfig) {
    const auto config1 = CelestialTwoBodyPointConfig::create(0.1F, 0.2F, false);
    CelestialTwoBodyPointAlgorithm alg(config1);

    const auto config2 = CelestialTwoBodyPointConfig::create(0.3F, 0.4F, true);
    EXPECT_NO_THROW(alg.setConfig(config2));
}

TEST(CelestialTwoBodyPointTest, SecondaryAlignedWithPrimaryFallsBack) {
    // When the secondary body is aligned with the primary as seen by the spacecraft, the
    // secondary constraint is invalid and the output must match the no-secondary-body result.
    const Eigen::Vector3d r_celBody_N{1.0e7, 2.0e6, 3.0e5};
    const Eigen::Vector3d v_celBody_N{-1.0e3, 5.0e3, 2.0e2};
    const Eigen::Vector3d r_secCelBody_N = 2.5 * r_celBody_N;  // same direction, farther away
    const Eigen::Vector3d v_secCelBody_N{10.0, -5.0, 2.0};

    const CelestialTwoBodyPointAlgorithm algWithSecondary(
        CelestialTwoBodyPointConfig::create(1.0F * kDeg2Rad, 10.0F * kDeg2Rad, true));
    const CelestialTwoBodyPointAlgorithm algNoSecondary(
        CelestialTwoBodyPointConfig::create(1.0F * kDeg2Rad, 10.0F * kDeg2Rad, false));

    const CelestialTwoBodyPointOutput outWith = algWithSecondary.update(
        r_celBody_N, v_celBody_N, r_secCelBody_N, v_secCelBody_N, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    const CelestialTwoBodyPointOutput outWithout = algNoSecondary.update(r_celBody_N,
                                                                         v_celBody_N,
                                                                         Eigen::Vector3d::Zero(),
                                                                         Eigen::Vector3d::Zero(),
                                                                         Eigen::Vector3d::Zero(),
                                                                         Eigen::Vector3d::Zero());

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(outWith.sigma_RN[i], outWithout.sigma_RN[i]);
        EXPECT_FLOAT_EQ(outWith.omega_RN_N[i], outWithout.omega_RN_N[i]);
        EXPECT_FLOAT_EQ(outWith.domega_RN_N[i], outWithout.domega_RN_N[i]);
    }
}

TEST(CelestialTwoBodyPointTest, SecondaryAntiAlignedWithPrimaryFallsBack) {
    // Anti-aligned secondary body (angle ~ pi) is also an invalid constraint configuration.
    const Eigen::Vector3d r_celBody_N{1.0e7, 2.0e6, 3.0e5};
    const Eigen::Vector3d v_celBody_N{-1.0e3, 5.0e3, 2.0e2};
    const Eigen::Vector3d r_secCelBody_N = -3.0 * r_celBody_N;  // opposite direction
    const Eigen::Vector3d v_secCelBody_N{10.0, -5.0, 2.0};

    const CelestialTwoBodyPointAlgorithm algWithSecondary(
        CelestialTwoBodyPointConfig::create(1.0F * kDeg2Rad, 10.0F * kDeg2Rad, true));
    const CelestialTwoBodyPointAlgorithm algNoSecondary(
        CelestialTwoBodyPointConfig::create(1.0F * kDeg2Rad, 10.0F * kDeg2Rad, false));

    const CelestialTwoBodyPointOutput outWith = algWithSecondary.update(
        r_celBody_N, v_celBody_N, r_secCelBody_N, v_secCelBody_N, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    const CelestialTwoBodyPointOutput outWithout = algNoSecondary.update(r_celBody_N,
                                                                         v_celBody_N,
                                                                         Eigen::Vector3d::Zero(),
                                                                         Eigen::Vector3d::Zero(),
                                                                         Eigen::Vector3d::Zero(),
                                                                         Eigen::Vector3d::Zero());

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(outWith.sigma_RN[i], outWithout.sigma_RN[i]);
        EXPECT_FLOAT_EQ(outWith.omega_RN_N[i], outWithout.omega_RN_N[i]);
        EXPECT_FLOAT_EQ(outWith.domega_RN_N[i], outWithout.domega_RN_N[i]);
    }
}

TEST(CelestialTwoBodyPointTest, HighReferenceRateFallsBack) {
    // With a near-zero rate threshold, any non-zero reference rate from the secondary constraint
    // exceeds the threshold and the algorithm must fall back to the no-secondary-body solution.
    const Eigen::Vector3d r_celBody_N{1.0e7, 2.0e6, 3.0e5};
    const Eigen::Vector3d v_celBody_N{-1.0e3, 5.0e3, 2.0e2};
    const Eigen::Vector3d r_secCelBody_N{500.0, 500.0, 500.0};
    const Eigen::Vector3d v_secCelBody_N{100.0, -10.0, 20.0};

    const CelestialTwoBodyPointAlgorithm algTightRate(
        CelestialTwoBodyPointConfig::create(1.0F * kDeg2Rad, 1e-12F, true));
    const CelestialTwoBodyPointAlgorithm algNoSecondary(
        CelestialTwoBodyPointConfig::create(1.0F * kDeg2Rad, 1e-12F, false));

    const CelestialTwoBodyPointOutput outWith = algTightRate.update(
        r_celBody_N, v_celBody_N, r_secCelBody_N, v_secCelBody_N, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    const CelestialTwoBodyPointOutput outWithout = algNoSecondary.update(r_celBody_N,
                                                                         v_celBody_N,
                                                                         Eigen::Vector3d::Zero(),
                                                                         Eigen::Vector3d::Zero(),
                                                                         Eigen::Vector3d::Zero(),
                                                                         Eigen::Vector3d::Zero());

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(outWith.sigma_RN[i], outWithout.sigma_RN[i]);
        EXPECT_FLOAT_EQ(outWith.omega_RN_N[i], outWithout.omega_RN_N[i]);
        EXPECT_FLOAT_EQ(outWith.domega_RN_N[i], outWithout.domega_RN_N[i]);
    }
}

TEST(CelestialTwoBodyPointTest, PropertyOutputIsFinite) {
    propertyOutputIsFinite({1.0e7, 2.0e6, 3.0e5},
                           {-1.0e3, 5.0e3, 2.0e2},
                           {500.0, 500.0, 500.0},
                           {100.0, -10.0, 20.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           1.0F * kDeg2Rad,
                           10.0F * kDeg2Rad,
                           true);
    propertyOutputIsFinite({1.5e11, 0.0, 0.0},
                           {0.0, 2.978e4, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           {1.5e11 + 7.0e6, 0.0, 0.0},
                           {0.0, 7.7e3, 0.0},
                           1.0F * kDeg2Rad,
                           10.0F * kDeg2Rad,
                           true);
    propertyOutputIsFinite({1.0e7, 0.0, 0.0},
                           {0.0, 5.0e3, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           1.0F * kDeg2Rad,
                           10.0F * kDeg2Rad,
                           false);
}

TEST(CelestialTwoBodyPointTest, PropertySigmaNormBounded) {
    propertySigmaNormBounded({1.0e7, 2.0e6, 3.0e5},
                             {-1.0e3, 5.0e3, 2.0e2},
                             {500.0, 500.0, 500.0},
                             {100.0, -10.0, 20.0},
                             {0.0, 0.0, 0.0},
                             {0.0, 0.0, 0.0},
                             1.0F * kDeg2Rad,
                             10.0F * kDeg2Rad,
                             true);
    propertySigmaNormBounded({-1.0e7, -2.0e6, 3.0e5},
                             {1.0e3, -5.0e3, 2.0e2},
                             {0.0, 0.0, 0.0},
                             {0.0, 0.0, 0.0},
                             {0.0, 0.0, 0.0},
                             {0.0, 0.0, 0.0},
                             1.0F * kDeg2Rad,
                             10.0F * kDeg2Rad,
                             false);
}
