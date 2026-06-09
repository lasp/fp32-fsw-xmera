/*
 Shared test helper for OEStateEphemAlgorithm unit and fuzz tests.
 */

#ifndef TEST_OE_STATE_EPHEM_HELPERS_H
#define TEST_OE_STATE_EPHEM_HELPERS_H

#include "oeStateEphemAlgorithm.h"
#include "utilities/fsw/orbitalMotion.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <cmath>

using orbitalMotion::CartesianState;

// Test constants
constexpr double EARTH_MU = 3.986004418e14;  // m^3/s^2
constexpr double MOON_MU = 4.902800118e12;   // m^3/s^2
constexpr double TEST_TOLERANCE = 1e-6;
constexpr double TEST_TOLERANCE_POSITION = 1.0;   // meters
constexpr double TEST_TOLERANCE_VELOCITY = 1e-3;  // m/s
constexpr double NEWTON_RAPHSON_TOLERANCE = 1e-8;
constexpr double PARABOLIC_TOLERANCE = 1e-10;

// ---------------------------------------------------------------------------
// Double-precision orbital mechanics helpers
// ---------------------------------------------------------------------------

inline double meanToEccentricAnomalyDouble(double M, double e) {
    double E = M;
    for (int i = 0; i < 200; ++i) {
        double dE = (E - e * std::sin(E) - M) / (1.0 - e * std::cos(E));
        E -= dE;
        if (std::abs(dE) < NEWTON_RAPHSON_TOLERANCE) break;
    }
    return E;
}

inline double eccentricToTrueAnomalyDouble(double E, double e) {
    return 2.0 * std::atan2(std::sqrt(1.0 + e) * std::sin(E / 2.0), std::sqrt(1.0 - e) * std::cos(E / 2.0));
}

inline double meanToTrueAnomalyDouble(double M, double e) {
    double E = meanToEccentricAnomalyDouble(M, e);
    return eccentricToTrueAnomalyDouble(E, e);
}

inline double meanToHyperbolicAnomalyDouble(double N, double e) {
    double H = std::abs(N) > 7.0 ? 7.0 * (N > 0 ? 1 : -1) : N;
    for (int i = 0; i < 200; ++i) {
        double dH = (e * std::sinh(H) - H - N) / (e * std::cosh(H) - 1.0);
        H -= dH;
        if (std::abs(dH) < NEWTON_RAPHSON_TOLERANCE) break;
    }
    return H;
}

inline double hyperbolicToTrueAnomalyDouble(double H, double e) {
    return 2.0 * std::atan(std::sqrt((e + 1.0) / (e - 1.0)) * std::tanh(H / 2.0));
}

inline CartesianState
elementsToCartesianStateDouble(double mu, double a, double e, double i, double Omega, double omega, double f) {
    double p = a * (1.0 - e * e);
    double r = p / (1.0 + e * std::cos(f));
    double h = std::sqrt(mu * p);

    double cos_O = std::cos(Omega);
    double sin_O = std::sin(Omega);
    double cos_o = std::cos(omega);
    double sin_o = std::sin(omega);
    double cos_i = std::cos(i);
    double sin_i = std::sin(i);
    double cos_f = std::cos(f);
    double sin_f = std::sin(f);

    double cos_theta = cos_o * cos_f - sin_o * sin_f;
    double sin_theta = sin_o * cos_f + cos_o * sin_f;

    CartesianState state{};
    state.position(0) = r * (cos_O * cos_theta - sin_O * sin_theta * cos_i);
    state.position(1) = r * (sin_O * cos_theta + cos_O * sin_theta * cos_i);
    state.position(2) = r * (sin_theta * sin_i);

    state.velocity(0) = -mu / h * (cos_O * (sin_theta + e * sin_o) + sin_O * (cos_theta + e * cos_o) * cos_i);
    state.velocity(1) = -mu / h * (sin_O * (sin_theta + e * sin_o) - cos_O * (cos_theta + e * cos_o) * cos_i);
    state.velocity(2) = mu / h * (cos_theta + e * cos_o) * sin_i;

    return state;
}

// ---------------------------------------------------------------------------
// Shared test helper
// ---------------------------------------------------------------------------

/*! @brief Test and validate OEStateEphemAlgorithm
 *
 *  This function sets up the algorithm with constant orbital elements (using only the first
 *  Chebyshev coefficient) and validates that the computed state matches an all-double
 *  reference to within float precision (4 ULPs by default, or explicit tolerances for fuzz).
 *
 *  @param mu Gravitational parameter [m^3/s^2]
 *  @param r_p_m Radius of periapsis [m]
 *  @param eccentricity Orbital eccentricity [-]
 *  @param inclination Orbital inclination [rad]
 *  @param arg_periapsis Argument of periapsis [rad]
 *  @param raan Right ascension of ascending node [rad]
 *  @param anomaly_angle Anomaly angle [rad]
 *  @param anomaly_type Anomaly type (true or mean)
 *  @param call_time_ns Call time [ns]
 *  @param ephemeris_time Ephemeris time offset [s]
 *  @param vehicle_time Vehicle time offset [s]
 *  @param arc_radius_time Arc radius time [s]
 */
inline void testOEStateEphemUpdate(double mu,
                                   double r_p_m,
                                   double eccentricity,
                                   double inclination,
                                   double arg_periapsis,
                                   double raan,
                                   double anomaly_angle,
                                   AnomalyType anomaly_type = AnomalyType::TRUE_ANOMALY,
                                   uint64_t call_time_ns = 0,
                                   double ephemeris_time = 0.0,
                                   double vehicle_time = 0.0,
                                   double arc_radius_time = 1000.0) {
    OEStateEphemAlgorithm algorithm;

    algorithm.setCentralBodyGravitationalParameter(mu);
    algorithm.setEphemerisTimeJ2000(ephemeris_time);
    algorithm.setVehicleTimeOffset(vehicle_time);

    const double arcSpacing = 2.0 * arc_radius_time;

    // Replicate the algorithm's clamp of currentEphTime to >= 0
    const double testEphTime = std::max(ephemeris_time - vehicle_time + call_time_ns * kNano2Sec, 0.0);

    // Determine which arc index the current eph time falls into
    const double arcIndex = std::floor(testEphTime / arcSpacing);
    unsigned int arcToPopulate = arcIndex < kMaxOeRecords ? static_cast<unsigned int>(arcIndex) : kMaxOeRecords - 1;

    double const arcMiddleTimeToUse = arcToPopulate * arcSpacing + arc_radius_time / 4;

    // Setup relevant arc with constant coefficients (only first coefficient non-zero)
    algorithm.setNumberOfArcs(arcToPopulate + 1);
    for (auto arcIdx = 0; arcIdx < arcToPopulate + 1; arcIdx++) {
        algorithm.setArcNumberOfCoefficients(arcIdx, 1);
        algorithm.setArcMiddleTime(arcIdx, arcMiddleTimeToUse);
        algorithm.setArcRadiusTime(arcIdx, arc_radius_time);
        algorithm.setArcAnomalyFlag(arcIdx, anomaly_type);

        // Set constant coefficients (first one) of each orbital element
        std::array<double, kMaxOeCoeff> rpCoeffs{};
        rpCoeffs[0] = r_p_m;
        algorithm.setArcRadiusPeriapsisCoefficients(arcIdx, rpCoeffs);

        std::array<double, kMaxOeCoeff> eCoeffs{};
        eCoeffs[0] = eccentricity;
        algorithm.setArcEccentricityCoefficients(arcIdx, eCoeffs);

        std::array<double, kMaxOeCoeff> iCoeffs{};
        iCoeffs[0] = inclination;
        algorithm.setArcInclinationCoefficients(arcIdx, iCoeffs);

        std::array<double, kMaxOeCoeff> omegaCoeffs{};
        omegaCoeffs[0] = arg_periapsis;
        algorithm.setArcArgPeriapsisCoefficients(arcIdx, omegaCoeffs);

        std::array<double, kMaxOeCoeff> raanCoeffs{};
        raanCoeffs[0] = raan;
        algorithm.setArcRaanCoefficients(arcIdx, raanCoeffs);

        std::array<double, kMaxOeCoeff> nuCoeffs{};
        nuCoeffs[0] = anomaly_angle;
        algorithm.setArcTrueAnomalyCoefficients(arcIdx, nuCoeffs);
    }
    // Call the update function
    CartesianState computed_state = algorithm.update(testEphTime / kNano2Sec);

    // Reference uses full double-precision inputs — no float truncation
    double a_d = 0.0;
    if (std::abs(eccentricity - 1.0) > PARABOLIC_TOLERANCE) {
        a_d = r_p_m / (1.0 - eccentricity);
    }

    double f_d = 0.0;
    if (anomaly_type == AnomalyType::TRUE_ANOMALY) {
        f_d = anomaly_angle;
    } else if (eccentricity < 1.0) {
        f_d = meanToTrueAnomalyDouble(anomaly_angle, eccentricity);
    } else {
        f_d = hyperbolicToTrueAnomalyDouble(meanToHyperbolicAnomalyDouble(anomaly_angle, eccentricity), eccentricity);
    }

    CartesianState expected_state =
        elementsToCartesianStateDouble(mu, a_d, eccentricity, inclination, raan, arg_periapsis, f_d);

    // Compare to 6 significant digits (relative tolerance 1e-6)
    constexpr double relTol = 1e-14;
    constexpr double absFloor = 1e-14;
    constexpr double angleTol = 1e-6;

    double posTol = std::max(absFloor, expected_state.position.norm() * relTol);
    double velTol = std::max(absFloor, expected_state.velocity.norm() * relTol);
    for (int idx = 0; idx < 3; ++idx) {
        EXPECT_TRUE(std::isfinite(computed_state.position[idx]));
        EXPECT_TRUE(std::isfinite(computed_state.velocity[idx]));

        EXPECT_NEAR(computed_state.position[idx], expected_state.position[idx], posTol);
        EXPECT_NEAR(computed_state.velocity[idx], expected_state.velocity[idx], velTol);
    }
    EXPECT_LT(safeAcos(computed_state.position.normalized().dot(expected_state.position.normalized())), angleTol);
}

#endif  // TEST_OE_STATE_EPHEM_HELPERS_H
