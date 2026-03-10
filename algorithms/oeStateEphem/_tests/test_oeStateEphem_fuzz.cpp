/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Fuzz tests for OEStateEphemAlgorithm class
 */

#include "../../utilities/orbitalMotion.hpp"
#include "../freestandingInvalidArgument.h"
#include "gtest/gtest.h"
#include "test_oeStateEphem.cpp"
#include <fuzztest/fuzztest.h>

#include <cmath>

// Fuzz-specific tolerances (more relaxed than unit tests)
constexpr double FUZZ_TOLERANCE_POSITION = 10.0;  // meters
constexpr double FUZZ_TOLERANCE_VELOCITY = 0.01;  // m/s

/*! @brief Main fuzz test for OEStateEphemAlgorithm
 *
 *  Tests the algorithm across a wide range of valid orbital parameters.
 *  Uses the shared testOEStateEphemUpdate() function for validation.
 */
void fuzzOEStateEphemUpdate(double mu,
                            double r_p_km,
                            float eccentricity,
                            float inclination,
                            float arg_periapsis,
                            float raan,
                            float true_anomaly,
                            uint64_t call_time_ns,
                            double ephemeris_time,
                            double offset) {
    // Call the shared test function with fuzz tolerances
    EXPECT_NO_THROW(testOEStateEphemUpdate(mu,
                                           r_p_km,
                                           eccentricity,
                                           inclination,
                                           arg_periapsis,
                                           raan,
                                           true_anomaly,
                                           call_time_ns,
                                           ephemeris_time,
                                           offset,
                                           1000.0,  // arc_radius_time
                                           FUZZ_TOLERANCE_POSITION,
                                           FUZZ_TOLERANCE_VELOCITY));
}

FUZZ_TEST(OEStateEphemFuzz, fuzzOEStateEphemUpdate)
    .WithDomains(fuzztest::InRange(1.0, 1e14),
                 fuzztest::InRange(1.0, 1e14),
                 fuzztest::InRange(0.0f, 0.99f),
                 fuzztest::InRange(0.0f, static_cast<float>(M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2.0 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2.0 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2.0 * M_PI)),
                 fuzztest::InRange(static_cast<uint64_t>(0), static_cast<uint64_t>(1e19)),
                 fuzztest::InRange(0.0, 1e14),
                 fuzztest::InRange(0.0, 1e14));

/*! @brief Fuzz test for edge cases and boundary conditions
 *
 *  Focuses on potentially problematic cases like near-circular orbits,
 *  near-parabolic orbits, and special inclinations.
 */
void fuzzOEStateEphemUpdateEdgeCases(double mu,
                                     double r_p_km,
                                     float eccentricity,
                                     float inclination,
                                     float true_anomaly) {
    // Use the shared test function with default time values and fuzz tolerances
    EXPECT_NO_THROW(testOEStateEphemUpdate(mu,
                                           r_p_km,
                                           eccentricity,
                                           inclination,
                                           0.0f,  // arg_periapsis
                                           0.0f,  // raan
                                           true_anomaly,
                                           0,       // call_time_ns
                                           0.0,     // ephemeris_time
                                           0.0,     // vehicle_time
                                           1000.0,  // arc_radius_time
                                           FUZZ_TOLERANCE_POSITION,
                                           FUZZ_TOLERANCE_VELOCITY));
}

FUZZ_TEST(OEStateEphemFuzz, fuzzOEStateEphemUpdateEdgeCases)
    .WithDomains(fuzztest::InRange(1.0, 1e14),
                 fuzztest::InRange(1.0, 1e14),
                 fuzztest::OneOf(fuzztest::InRange(0.0f, 0.01f),    // Near-circular
                                 fuzztest::InRange(0.95f, 0.999f),  // Near-parabolic
                                 fuzztest::InRange(0.4f, 0.6f)      // Moderate
                                 ),
                 fuzztest::OneOf(fuzztest::InRange(0.0f, 0.01f),  // Near-equatorial
                                 fuzztest::InRange(static_cast<float>(M_PI / 2 - 0.01),
                                                   static_cast<float>(M_PI / 2 + 0.01)),  // Near-polar
                                 fuzztest::InRange(static_cast<float>(M_PI - 0.01),
                                                   static_cast<float>(M_PI))  // Near-retrograde
                                 ),
                 fuzztest::InRange(0.0f, static_cast<float>(2.0 * M_PI)));

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
    const double r_p_km = 7000.0;

    // Use the shared test function
    EXPECT_NO_THROW(testOEStateEphemUpdate(mu,
                                           r_p_km,
                                           0.0f,  // eccentricity (circular)
                                           0.0f,  // inclination
                                           0.0f,  // arg_periapsis
                                           0.0f,  // raan
                                           0.0f,  // true_anomaly
                                           call_time_ns,
                                           ephemeris_time,
                                           offset,
                                           arc_radius_time,
                                           FUZZ_TOLERANCE_POSITION,
                                           FUZZ_TOLERANCE_VELOCITY));
}

FUZZ_TEST(OEStateEphemFuzz, fuzzOEStateEphemUpdateTimeEdgeCases)
    .WithDomains(fuzztest::InRange(static_cast<uint64_t>(0), static_cast<uint64_t>(1e19)),
                 fuzztest::InRange(0.0, 1e14),
                 fuzztest::InRange(0.0, 1e14),
                 fuzztest::OneOf(fuzztest::InRange(1.0, 100.0),      // Very narrow arc
                                 fuzztest::InRange(100.0, 10000.0),  // Normal arc
                                 fuzztest::InRange(10000.0, 1e6)     // Very wide arc
                                 ));
