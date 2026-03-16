/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Fuzz tests for OEStateEphemAlgorithm class
 */

#include "test_oeStateEphem_helpers.h"
#include <fuzztest/fuzztest.h>
#include <cmath>

/*! @brief Main fuzz test for OEStateEphemAlgorithm
 *
 *  Tests the algorithm across a wide range of valid orbital parameters.
 *  Uses the shared testOEStateEphemUpdate() function for validation.
 */
void fuzzOEStateEphemUpdate(double mu,
                            double r_p_m,
                            double eccentricity,
                            double inclination,
                            double arg_periapsis,
                            double raan,
                            double true_anomaly,
                            uint64_t call_time_ns,
                            double ephemeris_time,
                            double offset) {
    // Call the shared test function
    EXPECT_NO_THROW(testOEStateEphemUpdate(mu,
                                           r_p_m,
                                           eccentricity,
                                           inclination,
                                           arg_periapsis,
                                           raan,
                                           true_anomaly,
                                           AnomalyType::TRUE_ANOMALY,
                                           call_time_ns,
                                           ephemeris_time,
                                           offset));
}

FUZZ_TEST(OEStateEphemFuzz, fuzzOEStateEphemUpdate)
    .WithDomains(fuzztest::InRange(1.0, 1e14),
                 fuzztest::InRange(1.0, 1e14),
                 fuzztest::InRange(0.0, 0.99),
                 fuzztest::InRange(0.0, M_PI),
                 fuzztest::InRange(0.0, 2.0 * M_PI),
                 fuzztest::InRange(0.0, 2.0 * M_PI),
                 fuzztest::InRange(0.0, 2.0 * M_PI),
                 fuzztest::InRange(static_cast<uint64_t>(0), static_cast<uint64_t>(1e19)),
                 fuzztest::InRange(1.0, 1e14),
                 fuzztest::InRange(1.0, 1e14));

/*! @brief Fuzz test for edge cases and boundary conditions
 *
 *  Focuses on potentially problematic cases like near-circular orbits,
 *  near-parabolic orbits, and special inclinations.
 */
void fuzzOEStateEphemUpdateEdgeCases(double mu,
                                     double r_p_m,
                                     double eccentricity,
                                     double inclination,
                                     double true_anomaly) {
    // Use the shared test function with default time values
    EXPECT_NO_THROW(testOEStateEphemUpdate(mu, r_p_m, eccentricity, inclination, 0.0, 0.0, true_anomaly));
}

FUZZ_TEST(OEStateEphemFuzz, fuzzOEStateEphemUpdateEdgeCases)
    .WithDomains(fuzztest::InRange(1.0, 1e14),
                 fuzztest::InRange(1.0, 1e14),
                 fuzztest::OneOf(fuzztest::InRange(0.0, 0.01),    // Near-circular
                                 fuzztest::InRange(0.95, 0.999),  // Near-parabolic
                                 fuzztest::InRange(0.4, 0.6)      // Moderate
                                 ),
                 fuzztest::OneOf(fuzztest::InRange(0.0, 0.01),  // Near-equatorial
                                 fuzztest::InRange(M_PI / 2 - 0.01,
                                                   M_PI / 2 + 0.01),   // Near-polar
                                 fuzztest::InRange(M_PI - 0.01, M_PI)  // Near-retrograde
                                 ),
                 fuzztest::InRange(0.0, 2.0 * M_PI));

/*! @brief Fuzz test for time-related edge cases
 *
 *  Tests the algorithm's time offset calculations and arc selection with
 *  various time configurations including very large values and different
 *  arc widths.
 */
void fuzzOEStateEphemUpdateTimeEdgeCases(uint64_t call_time_ns,
                                         double ephemeris_time,
                                         double offset,
                                         double arc_radius_time) {
    const double mu = 3.986004418e14;
    const double r_p_m = 7000000.0;

    // Use the shared test function
    EXPECT_NO_THROW(testOEStateEphemUpdate(mu,
                                           r_p_m,
                                           0.0,
                                           0.0,
                                           0.0,
                                           0.0,
                                           0.0,
                                           AnomalyType::TRUE_ANOMALY,
                                           call_time_ns,
                                           ephemeris_time,
                                           offset,
                                           arc_radius_time));
}

FUZZ_TEST(OEStateEphemFuzz, fuzzOEStateEphemUpdateTimeEdgeCases)
    .WithDomains(fuzztest::InRange(static_cast<uint64_t>(0), static_cast<uint64_t>(1e19)),
                 fuzztest::InRange(0.0, 1e14),
                 fuzztest::InRange(0.0, 1e14),
                 fuzztest::OneOf(fuzztest::InRange(1.0, 100.0),      // Very narrow arc
                                 fuzztest::InRange(100.0, 10000.0),  // Normal arc
                                 fuzztest::InRange(10000.0, 1e6)     // Very wide arc
                                 ));
