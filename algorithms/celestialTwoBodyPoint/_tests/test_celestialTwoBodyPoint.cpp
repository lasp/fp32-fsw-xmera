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
