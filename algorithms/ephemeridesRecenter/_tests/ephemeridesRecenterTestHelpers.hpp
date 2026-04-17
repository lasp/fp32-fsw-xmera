#ifndef TEST_EPHEMERIDESRECENTER_H
#define TEST_EPHEMERIDESRECENTER_H

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "ephemeridesRecenterAlgorithm.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <numbers>
#include <vector>

constexpr int SUN_SPICE_ID = 10;
constexpr int EARTH_SPICE_ID = 399;
constexpr int MOON_SPICE_ID = 301;
constexpr int SATURN_SPICE_ID = 699;
constexpr int TITAN_SPICE_ID = 606;
constexpr int MARS_SPICE_ID = 499;

// Reference computation for update
std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> referenceUpdate(
    const EphemeridesRecenterAlgorithm& alg,
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies) {
    // Pull required "configuration state" from the algorithm via getters
    const int newCentralBodyId = alg.getNewZeroBase();
    const int previousCentralBodyId = alg.getPreviousCommonZeroBase();
    const size_t celestialBodyCount = alg.getNumberOfBodies();
    const auto bodyIds = alg.getAllIds();

    if (celestialBodyCount == 0U) {
        FS_THROW_INVALID_ARGUMENT("The current celestial body count is 0");
    }

    // Find the new central body index using the returned bodyIds ordering
    const auto it = std::ranges::find(bodyIds, newCentralBodyId);
    if (it == bodyIds.end()) {
        FS_THROW_INVALID_ARGUMENT("New zero base body was not in the list of existing bodies");
    }
    const size_t newCentralIndex = static_cast<size_t>(std::distance(bodyIds.begin(), it));

    // Local copy of "celestialBodies" used by the algorithm
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> celestialBodies = newBodies;

    // Central body payload
    const auto newCentralBody = celestialBodies[newCentralIndex];
    Eigen::Vector3d newCentral_input_r = newCentralBody.input_r;
    Eigen::Vector3d newCentral_input_v = newCentralBody.input_v;

    /* - If the new central body is a moon (its original central body is not the common central body but another body in
     * the list) first re-center the moon around the common central body so that every body is relative to the common
     * center*/
    if (newCentralBody.originalCentralBodyId != previousCentralBodyId) {
        size_t moonCentralBodyIndex = 0U;
        for (size_t k = 0U; k < celestialBodyCount; ++k) {
            if (bodyIds[k] == newCentralBody.originalCentralBodyId) {
                moonCentralBodyIndex = k;
                break;
            }
        }
        Eigen::Vector3d moonCentral_input_r = celestialBodies[moonCentralBodyIndex].input_r;
        Eigen::Vector3d moonCentral_input_v = celestialBodies[moonCentralBodyIndex].input_v;
        newCentral_input_r = newCentral_input_r + moonCentral_input_r;
        newCentral_input_v = newCentral_input_v + moonCentral_input_v;
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> recenteredBodies{};
    for (size_t i = 0U; i < celestialBodyCount; ++i) {
        if (recenteredBodies[i].isMoon) {
            continue;
        }

        recenteredBodies[i] = BodyEphemerisPayload{};
        Eigen::Vector3d newEphemerisToRecenter_input_r = newBodies[i].input_r;
        Eigen::Vector3d newEphemerisToRecenter_input_v = newBodies[i].input_v;

        if (celestialBodies[i].originalCentralBodyId == previousCentralBodyId) {
            Eigen::Vector3d const relativePosition = newEphemerisToRecenter_input_r - newCentral_input_r;

            Eigen::Vector3d const relativeVelocity = newEphemerisToRecenter_input_v - newCentral_input_v;

            // implement private function findMoonOfBody(celestialBodies[i]) in the algorithm
            size_t moonIndex = 0U;
            bool moonFound = false;
            for (size_t j = 0U; j < celestialBodyCount; ++j) {
                if (celestialBodies[j].originalCentralBodyId == celestialBodies[i].bodySpiceId) {
                    moonIndex = j;
                    moonFound = true;
                    break;
                }
            }

            if (moonFound && celestialBodies[i].bodySpiceId != previousCentralBodyId) {
                Eigen::Vector3d moonOfBody_input_r = celestialBodies[moonIndex].input_r;
                Eigen::Vector3d moonOfBody_input_v = celestialBodies[moonIndex].input_v;
                Eigen::Vector3d const moonRelativePosition = relativePosition + moonOfBody_input_r;

                Eigen::Vector3d const moonRelativeVelocity = relativeVelocity + moonOfBody_input_v;

                recenteredBodies[moonIndex].bodySpiceId = celestialBodies[moonIndex].bodySpiceId;
                recenteredBodies[moonIndex].isMoon = true;
                recenteredBodies[moonIndex].originalCentralBodyId = celestialBodies[moonIndex].originalCentralBodyId;
                recenteredBodies[moonIndex].output_r = moonRelativePosition;
                recenteredBodies[moonIndex].output_v = moonRelativeVelocity;
            }

            recenteredBodies[i] = newBodies[i];
            recenteredBodies[i].output_r = relativePosition;
            recenteredBodies[i].output_v = relativeVelocity;
        }
    }

    return recenteredBodies;
}

// Assume a fixed body list SUN/EARTH/MOON/SATURN
inline void regressionTestEphemeridesRecenter(
    const std::vector<int>& bodyListInOrder,  // must be {SUN, EARTH, MOON, SATURN}
    int previousCommonIdx,                    // in {0,1,2,3}
    int newZeroIdx,                           // in {0,1,2,3}
    // Inputs aligned with bodyListInOrder indices: 0=SUN, 1=EARTH, 2=MOON, 3=SATURN
    const std::array<double, 3>& r0,
    const std::array<double, 3>& v0,
    bool isMoon0,
    const std::array<double, 3>& r1,
    const std::array<double, 3>& v1,
    bool isMoon1,
    const std::array<double, 3>& r2,
    const std::array<double, 3>& v2,
    bool isMoon2,
    const std::array<double, 3>& r3,
    const std::array<double, 3>& v3,
    bool isMoon3) {
    // --- Basic validation: bodies must exist in list ---
    ASSERT_EQ(bodyListInOrder.size(), 4U);
    ASSERT_GE(previousCommonIdx, 0);
    ASSERT_LE(previousCommonIdx, 3);
    ASSERT_GE(newZeroIdx, 0);
    ASSERT_LE(newZeroIdx, 3);
    const int previousCommonZeroBaseId = bodyListInOrder[static_cast<size_t>(previousCommonIdx)];
    const int newZeroBaseId = bodyListInOrder[static_cast<size_t>(newZeroIdx)];
    auto containsId = [&](int id) -> bool {
        return std::find(bodyListInOrder.begin(), bodyListInOrder.end(), id) != bodyListInOrder.end();
    };
    ASSERT_TRUE(containsId(previousCommonZeroBaseId));
    ASSERT_TRUE(containsId(newZeroBaseId));
    ASSERT_EQ(bodyListInOrder.size(), 4U);  // SUN/EARTH/MOON/SATURN only

    // --- Build newBodies from the inputs ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    for (auto& b : newBodies) {
        b = BodyEphemerisPayload{};
    }

    auto fillBody = [&](size_t idx,
                        int spiceId,
                        int originalCentralId,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies[idx].bodySpiceId = spiceId;
        newBodies[idx].originalCentralBodyId = originalCentralId;
        newBodies[idx].isMoon = isMoonFlag;
        for (int k = 0; k < 3; ++k) {
            newBodies[idx].input_r[k] = r[k];
            newBodies[idx].input_v[k] = v[k];
        }
    };
    fillBody(0, bodyListInOrder[0], bodyListInOrder[previousCommonIdx], r0, v0, isMoon0);
    fillBody(1, bodyListInOrder[1], bodyListInOrder[previousCommonIdx], r1, v1, isMoon1);
    if (isMoon2) {
        fillBody(2, bodyListInOrder[2], bodyListInOrder[1], r2, v2, isMoon2);
    } else {
        fillBody(2, bodyListInOrder[2], bodyListInOrder[previousCommonIdx], r2, v2, isMoon2);
    }
    fillBody(3, bodyListInOrder[3], bodyListInOrder[previousCommonIdx], r3, v3, isMoon3);

    // --- Configure algorithm ---
    EphemeridesRecenterAlgorithm alg{};
    const int previousCommonId = bodyListInOrder[static_cast<size_t>(previousCommonIdx)];
    const std::array<int, 4> originalCentralIds = {
        previousCommonId,
        previousCommonId,
        isMoon2 ? bodyListInOrder[1] : previousCommonId,
        previousCommonId,
    };
    for (size_t idx = 0; idx < bodyListInOrder.size(); ++idx) {
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[idx], originalCentralIds[idx]}));
    }
    EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(previousCommonZeroBaseId));
    EXPECT_NO_THROW(alg.setNewZeroBaseId(newZeroBaseId));
    EXPECT_NO_THROW(alg.reset());

    // --- Run algorithm + reference ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> out{};
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> ref{};

    EXPECT_NO_THROW(out = alg.updateState(newBodies));
    EXPECT_NO_THROW(ref = referenceUpdate(alg, newBodies));

    // --- Compare ---
    const size_t N = alg.getNumberOfBodies();
    for (size_t i = 0U; i < N; ++i) {
        for (int k = 0; k < 3; ++k) {
            EXPECT_NEAR(out[i].output_r[k], ref[i].output_r[k], 1e-6);
            EXPECT_NEAR(out[i].output_v[k], ref[i].output_v[k], 1e-6);

            EXPECT_TRUE(std::isfinite(out[i].output_r[k]));
            EXPECT_TRUE(std::isfinite(out[i].output_v[k]));
        }
        EXPECT_EQ(out[i].isMoon, ref[i].isMoon);
        EXPECT_EQ(out[i].bodySpiceId, ref[i].bodySpiceId);
        EXPECT_EQ(out[i].originalCentralBodyId, ref[i].originalCentralBodyId);
    }
}

inline void testEphemeridesRecenterSetup() {
    const std::vector<int> bodyListInOrder{
        SUN_SPICE_ID,
        EARTH_SPICE_ID,
        MOON_SPICE_ID,
        SATURN_SPICE_ID,
    };

    EphemeridesRecenterAlgorithm alg{};
    // Setting previous common zero base when body list is empty should throw
    EXPECT_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]), fs::invalid_argument);
    // Requesting info when empty should throw
    EXPECT_THROW(alg.getAllIds(), fs::invalid_argument);

    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[0], bodyListInOrder[0]}));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[1], bodyListInOrder[0]}));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[2], bodyListInOrder[1]}));

    // Duplicate body is allowed (same body from a different oeStateEphem message)
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[0], bodyListInOrder[0]}));

    // Same bodySpiceId with different centralBodyId is also allowed
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[2], bodyListInOrder[0]}));

    // Basic getters after adding (5 bodies: Sun, Earth, Moon, Sun-dup, Moon-diff-central)
    EXPECT_EQ(alg.getNumberOfBodies(), 5U);

    const auto ids = alg.getAllIds();
    EXPECT_EQ(ids[0], bodyListInOrder[0]);  // Sun
    EXPECT_EQ(ids[1], bodyListInOrder[1]);  // Earth
    EXPECT_EQ(ids[2], bodyListInOrder[2]);  // Moon (central=Earth)
    EXPECT_EQ(ids[3], bodyListInOrder[0]);  // Sun (duplicate)
    EXPECT_EQ(ids[4], bodyListInOrder[2]);  // Moon (central=Sun)

    // Set and get the previous common zero base
    EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
    EXPECT_EQ(alg.getPreviousCommonZeroBase(), bodyListInOrder[0]);

    // setNewZeroBaseId should not throw for any ID, but reset() should validate it exists in list
    EXPECT_NO_THROW(alg.setNewZeroBaseId(bodyListInOrder[1]));
    EXPECT_EQ(alg.getNewZeroBase(), bodyListInOrder[1]);

    // test reset()
    EXPECT_NO_THROW(alg.reset());

    // If you set a zero base that is NOT in the list, reset() must throw
    EXPECT_NO_THROW(alg.setNewZeroBaseId(bodyListInOrder[3]));
    EXPECT_THROW(alg.reset(), fs::invalid_argument);

    // Clear should reset internal list; then getters that require non-empty should throw again
    EXPECT_NO_THROW(alg.clearAllBodies());
    EXPECT_EQ(alg.getNumberOfBodies(), 0U);
    EXPECT_THROW(alg.getAllIds(), fs::invalid_argument);

    // Add exactly MAX_NUM_CHANGE_BODIES bodies (unique IDs)
    for (std::size_t i = 0; i < MAX_NUM_CHANGE_BODIES; ++i) {
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({100 + static_cast<int>(i), 0}));
    }
    // One more should exceed the limit and throw
    EXPECT_THROW(alg.addBodyEphemerisToRecenter({999, 0}), fs::invalid_argument);
}

inline void testRecenterEphemeridesRecenter() {
    const std::vector<int> bodyListInOrder{
        SUN_SPICE_ID,
        EARTH_SPICE_ID,
        MOON_SPICE_ID,
        SATURN_SPICE_ID,
        TITAN_SPICE_ID,
    };

    // --- Build newBodies from the inputs ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    for (auto& b : newBodies) {
        b = BodyEphemerisPayload{};
    }

    auto fillBody = [&](size_t idx,
                        int spiceId,
                        int originalCentralId,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies[idx].bodySpiceId = spiceId;
        newBodies[idx].originalCentralBodyId = originalCentralId;
        newBodies[idx].isMoon = isMoonFlag;
        for (int k = 0; k < 3; ++k) {
            newBodies[idx].input_r[k] = r[k];
            newBodies[idx].input_v[k] = v[k];
        }
    };
    const std::array<double, 3> r_sun = {0.0, 0.0, 0.0};
    const std::array<double, 3> v_sun = {0.0, 0.0, 0.0};
    const std::array<double, 3> r_earth = {10.0, -2.0, 3.0};
    const std::array<double, 3> v_earth = {1.0, 0.5, -0.25};
    const std::array<double, 3> r_moon_to_earth = {2.0, 1.0, 0.0};
    const std::array<double, 3> v_moon_to_earth = {0.1, -0.2, 0.3};
    const std::array<double, 3> r_saturn = {-7.0, 4.0, 9.0};
    const std::array<double, 3> v_saturn = {0.0, 2.0, 1.0};
    const std::array<double, 3> r_titan = {-7.0, 4.0, 9.0};
    const std::array<double, 3> v_titan = {0.3, -0.2, 11.0};
    fillBody(0, bodyListInOrder[0], bodyListInOrder[0], r_sun, v_sun, false);
    fillBody(1, bodyListInOrder[1], bodyListInOrder[0], r_earth, v_earth, false);
    fillBody(2, bodyListInOrder[2], bodyListInOrder[1], r_moon_to_earth, v_moon_to_earth, true);
    fillBody(3, bodyListInOrder[3], bodyListInOrder[0], r_saturn, v_saturn, false);
    fillBody(4, bodyListInOrder[4], bodyListInOrder[3], r_titan, v_titan, true);

    // ------------------------------------------------------------
    // newZeroBase = EARTH (planet)
    // ------------------------------------------------------------
    {
        EphemeridesRecenterAlgorithm alg{};
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[0], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[1], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[2], bodyListInOrder[1]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[3], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[4], bodyListInOrder[3]}));

        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseId(bodyListInOrder[1]));
        EXPECT_NO_THROW(alg.reset());

        auto out = alg.updateState(newBodies);

        // Sun'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[0].output_r[i], r_sun[i] - r_earth[i], 1e-6);
            EXPECT_NEAR(out[0].output_v[i], v_sun[i] - v_earth[i], 1e-6);
        }
        // Earth'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[1].output_r[i], 0.0, 1e-6);
            EXPECT_NEAR(out[1].output_v[i], 0.0, 1e-6);
        }
        // Moon'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[2].output_r[i], r_moon_to_earth[i], 1e-6);
            EXPECT_NEAR(out[2].output_v[i], v_moon_to_earth[i], 1e-6);
        }
        // SATURN'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[3].output_r[i], r_saturn[i] - r_earth[i], 1e-6);
            EXPECT_NEAR(out[3].output_v[i], v_saturn[i] - v_earth[i], 1e-6);
        }
        // TITAN'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[4].output_r[i], r_titan[i] + (r_saturn[i] - r_earth[i]), 1e-6);
            EXPECT_NEAR(out[4].output_v[i], v_titan[i] + (v_saturn[i] - v_earth[i]), 1e-6);
        }
    }
}

inline void testRecenterMoonEphemeridesRecenter() {
    const std::vector<int> bodyListInOrder{
        SUN_SPICE_ID,
        EARTH_SPICE_ID,
        MOON_SPICE_ID,
        SATURN_SPICE_ID,
        TITAN_SPICE_ID,
    };

    // --- Build newBodies from the inputs ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    for (auto& b : newBodies) {
        b = BodyEphemerisPayload{};
    }

    auto fillBody = [&](size_t idx,
                        int spiceId,
                        int originalCentralId,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies[idx].bodySpiceId = spiceId;
        newBodies[idx].originalCentralBodyId = originalCentralId;
        newBodies[idx].isMoon = isMoonFlag;
        for (int k = 0; k < 3; ++k) {
            newBodies[idx].input_r[k] = r[k];
            newBodies[idx].input_v[k] = v[k];
        }
    };
    const std::array<double, 3> r_sun = {0.0, 0.0, 0.0};
    const std::array<double, 3> v_sun = {0.0, 0.0, 0.0};
    const std::array<double, 3> r_earth = {10.0, -2.0, 3.0};
    const std::array<double, 3> v_earth = {1.0, 0.5, -0.25};
    const std::array<double, 3> r_moon_to_earth = {2.0, 1.0, 0.0};
    const std::array<double, 3> v_moon_to_earth = {0.1, -0.2, 0.3};
    const std::array<double, 3> r_saturn = {-7.0, 4.0, 9.0};
    const std::array<double, 3> v_saturn = {0.0, 2.0, 1.0};
    const std::array<double, 3> r_titan = {-7.0, 4.0, 9.0};
    const std::array<double, 3> v_titan = {0.3, -0.2, 11.0};
    fillBody(0, bodyListInOrder[0], bodyListInOrder[0], r_sun, v_sun, false);
    fillBody(1, bodyListInOrder[1], bodyListInOrder[0], r_earth, v_earth, false);
    fillBody(2, bodyListInOrder[2], bodyListInOrder[1], r_moon_to_earth, v_moon_to_earth, true);
    fillBody(3, bodyListInOrder[3], bodyListInOrder[0], r_saturn, v_saturn, false);
    fillBody(4, bodyListInOrder[4], bodyListInOrder[3], r_titan, v_titan, true);

    // ------------------------------------------------------------
    // newZeroBase = Moon
    // ------------------------------------------------------------
    {
        EphemeridesRecenterAlgorithm alg{};
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[0], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[1], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[2], bodyListInOrder[1]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[3], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[4], bodyListInOrder[3]}));

        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseId(bodyListInOrder[2]));
        EXPECT_NO_THROW(alg.reset());

        auto out = alg.updateState(newBodies);

        // Sun'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[0].output_r[i], r_sun[i] - (r_moon_to_earth[i] + r_earth[i]), 1e-6);
            EXPECT_NEAR(out[0].output_v[i], v_sun[i] - (v_moon_to_earth[i] + v_earth[i]), 1e-6);
        }
        // Earth'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[1].output_r[i], r_earth[i] - (r_moon_to_earth[i] + r_earth[i]), 1e-6);
            EXPECT_NEAR(out[1].output_v[i], v_earth[i] - (v_moon_to_earth[i] + v_earth[i]), 1e-6);
        }
        // Moon'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[2].output_r[i], 0.0, 1e-6);
            EXPECT_NEAR(out[2].output_v[i], 0.0, 1e-6);
        }
        // SATURN'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[3].output_r[i], r_saturn[i] - (r_moon_to_earth[i] + r_earth[i]), 1e-6);
            EXPECT_NEAR(out[3].output_v[i], v_saturn[i] - (v_moon_to_earth[i] + v_earth[i]), 1e-6);
        }
        // TITAN'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[4].output_r[i], r_titan[i] + (r_saturn[i] - (r_moon_to_earth[i] + r_earth[i])), 1e-6);
            EXPECT_NEAR(out[4].output_v[i], v_titan[i] + (v_saturn[i] - (v_moon_to_earth[i] + v_earth[i])), 1e-6);
        }
    }
}

inline void testRecenterPreCommonEphemeridesRecenter() {
    const std::vector<int> bodyListInOrder{
        SUN_SPICE_ID,
        EARTH_SPICE_ID,
        MOON_SPICE_ID,
        SATURN_SPICE_ID,
        TITAN_SPICE_ID,
    };

    // --- Build newBodies from the inputs ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    for (auto& b : newBodies) {
        b = BodyEphemerisPayload{};
    }

    auto fillBody = [&](size_t idx,
                        int spiceId,
                        int originalCentralId,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies[idx].bodySpiceId = spiceId;
        newBodies[idx].originalCentralBodyId = originalCentralId;
        newBodies[idx].isMoon = isMoonFlag;
        for (int k = 0; k < 3; ++k) {
            newBodies[idx].input_r[k] = r[k];
            newBodies[idx].input_v[k] = v[k];
        }
    };
    const std::array<double, 3> r_sun = {0.0, 0.0, 0.0};
    const std::array<double, 3> v_sun = {0.0, 0.0, 0.0};
    const std::array<double, 3> r_earth = {10.0, -2.0, 3.0};
    const std::array<double, 3> v_earth = {1.0, 0.5, -0.25};
    const std::array<double, 3> r_moon_to_earth = {2.0, 1.0, 0.0};
    const std::array<double, 3> v_moon_to_earth = {0.1, -0.2, 0.3};
    const std::array<double, 3> r_saturn = {-7.0, 4.0, 9.0};
    const std::array<double, 3> v_saturn = {0.0, 2.0, 1.0};
    const std::array<double, 3> r_titan = {-7.0, 4.0, 9.0};
    const std::array<double, 3> v_titan = {0.3, -0.2, 11.0};
    fillBody(0, bodyListInOrder[0], bodyListInOrder[0], r_sun, v_sun, false);
    fillBody(1, bodyListInOrder[1], bodyListInOrder[0], r_earth, v_earth, false);
    fillBody(2, bodyListInOrder[2], bodyListInOrder[1], r_moon_to_earth, v_moon_to_earth, true);
    fillBody(3, bodyListInOrder[3], bodyListInOrder[0], r_saturn, v_saturn, false);
    fillBody(4, bodyListInOrder[4], bodyListInOrder[3], r_titan, v_titan, true);

    // ------------------------------------------------------------
    // newZeroBase = previousCommon
    // ------------------------------------------------------------
    {
        EphemeridesRecenterAlgorithm alg{};
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[0], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[1], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[2], bodyListInOrder[1]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[3], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[4], bodyListInOrder[3]}));

        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseId(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.reset());

        auto out = alg.updateState(newBodies);

        // Sun'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[0].output_r[i], r_sun[i], 1e-6);
            EXPECT_NEAR(out[0].output_v[i], v_sun[i], 1e-6);
        }
        // Earth'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[1].output_r[i], r_earth[i], 1e-6);
            EXPECT_NEAR(out[1].output_v[i], v_earth[i], 1e-6);
        }
        // Moon'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[2].output_r[i], r_moon_to_earth[i] + r_earth[i], 1e-6);
            EXPECT_NEAR(out[2].output_v[i], v_moon_to_earth[i] + v_earth[i], 1e-6);
        }
        // SATURN'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[3].output_r[i], r_saturn[i], 1e-6);
            EXPECT_NEAR(out[3].output_v[i], v_saturn[i], 1e-6);
        }
        // TITAN'
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[4].output_r[i], r_titan[i] + r_saturn[i], 1e-6);
            EXPECT_NEAR(out[4].output_v[i], v_titan[i] + v_saturn[i], 1e-6);
        }
    }
}

inline void testMultiMoonsRecenter() {
    const std::vector<int> bodyListInOrder{
        SUN_SPICE_ID,
        EARTH_SPICE_ID,
        MOON_SPICE_ID,
        SATURN_SPICE_ID,
        TITAN_SPICE_ID,
    };

    // --- Build newBodies from the inputs ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    for (auto& b : newBodies) {
        b = BodyEphemerisPayload{};
    }

    auto fillBody = [&](size_t idx,
                        int spiceId,
                        int originalCentralId,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies[idx].bodySpiceId = spiceId;
        newBodies[idx].originalCentralBodyId = originalCentralId;
        newBodies[idx].isMoon = isMoonFlag;
        for (int k = 0; k < 3; ++k) {
            newBodies[idx].input_r[k] = r[k];
            newBodies[idx].input_v[k] = v[k];
        }
    };
    const std::array<double, 3> r_sun = {0.0, 0.0, 0.0};
    const std::array<double, 3> v_sun = {0.0, 0.0, 0.0};
    const std::array<double, 3> r_earth = {10.0, -2.0, 3.0};
    const std::array<double, 3> v_earth = {1.0, 0.5, -0.25};
    const std::array<double, 3> r_moon_to_earth = {2.0, 1.0, 0.0};
    const std::array<double, 3> v_moon_to_earth = {0.1, -0.2, 0.3};
    const std::array<double, 3> r_saturn = {-7.0, 4.0, 9.0};
    const std::array<double, 3> v_saturn = {0.0, 2.0, 1.0};
    const std::array<double, 3> r_titan = {-7.0, 4.0, 9.0};
    const std::array<double, 3> v_titan = {0.3, -0.2, 11.0};
    fillBody(0, bodyListInOrder[0], bodyListInOrder[0], r_sun, v_sun, false);
    fillBody(1, bodyListInOrder[1], bodyListInOrder[0], r_earth, v_earth, false);
    fillBody(2, bodyListInOrder[2], bodyListInOrder[1], r_moon_to_earth, v_moon_to_earth, true);
    fillBody(3, bodyListInOrder[3], bodyListInOrder[0], r_saturn, v_saturn, false);
    fillBody(4, bodyListInOrder[4], bodyListInOrder[1], r_titan, v_titan, true);

    // ------------------------------------------------------------
    // newZeroBase = previousCommon
    // ------------------------------------------------------------
    {
        EphemeridesRecenterAlgorithm alg{};
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[0], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[1], bodyListInOrder[0]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[2], bodyListInOrder[1]}));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({bodyListInOrder[3], bodyListInOrder[0]}));
        EXPECT_NO_THROW(
            alg.addBodyEphemerisToRecenter({bodyListInOrder[4], bodyListInOrder[1]}));  // Titan→Earth (duplicate moon)

        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseId(bodyListInOrder[0]));
        EXPECT_THROW(alg.reset(), fs::invalid_argument);  // multiple moons caught at configuration time
    }
}

inline void testMoonOfMoonRecenter() {
    // A "sub-moon" whose parent is itself a moon should be rejected at reset()
    constexpr int SUB_MOON_SPICE_ID = 30101;

    EphemeridesRecenterAlgorithm alg{};
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({SUN_SPICE_ID, SUN_SPICE_ID}));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({EARTH_SPICE_ID, SUN_SPICE_ID}));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({MOON_SPICE_ID, EARTH_SPICE_ID}));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({SUB_MOON_SPICE_ID, MOON_SPICE_ID}));  // moon-of-moon

    EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(SUN_SPICE_ID));
    EXPECT_NO_THROW(alg.setNewZeroBaseId(EARTH_SPICE_ID));

    EXPECT_THROW(alg.reset(), fs::invalid_argument);  // "moon-of-moon not supported"
}

inline void testOrphanMoonRecenter() {
    // A moon whose parent is NOT in the body list should be rejected at reset()
    EphemeridesRecenterAlgorithm alg{};
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({SUN_SPICE_ID, SUN_SPICE_ID}));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({EARTH_SPICE_ID, SUN_SPICE_ID}));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({MOON_SPICE_ID, MARS_SPICE_ID}));  // parent=Mars, not in list

    EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(SUN_SPICE_ID));
    EXPECT_NO_THROW(alg.setNewZeroBaseId(EARTH_SPICE_ID));

    EXPECT_THROW(alg.reset(), fs::invalid_argument);  // "body is not found" from findBodyIndex
}

inline void testDuplicateBodyRecenter() {
    // Two Sun entries (exact duplicates), Earth as new central
    EphemeridesRecenterAlgorithm alg{};
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({SUN_SPICE_ID, SUN_SPICE_ID}));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({EARTH_SPICE_ID, SUN_SPICE_ID}));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter({SUN_SPICE_ID, SUN_SPICE_ID}));  // duplicate Sun

    EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(SUN_SPICE_ID));
    EXPECT_NO_THROW(alg.setNewZeroBaseId(EARTH_SPICE_ID));
    EXPECT_NO_THROW(alg.reset());

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    newBodies[0].bodySpiceId = SUN_SPICE_ID;
    newBodies[0].originalCentralBodyId = SUN_SPICE_ID;
    newBodies[0].input_r = {0.0, 0.0, 0.0};
    newBodies[1].bodySpiceId = EARTH_SPICE_ID;
    newBodies[1].originalCentralBodyId = SUN_SPICE_ID;
    newBodies[1].input_r = {10.0, -2.0, 3.0};
    newBodies[2].bodySpiceId = SUN_SPICE_ID;  // duplicate
    newBodies[2].originalCentralBodyId = SUN_SPICE_ID;
    newBodies[2].input_r = {0.0, 0.0, 0.0};

    auto out = alg.updateState(newBodies);

    // Both Sun entries should be recentered to Earth
    for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(out[0].output_r[k], -newBodies[1].input_r[k], 1e-6);
        EXPECT_NEAR(out[2].output_r[k], -newBodies[1].input_r[k], 1e-6);  // duplicate matches
    }
    // Earth is the new center — output should be zero
    for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(out[1].output_r[k], 0.0, 1e-6);
    }
}

#endif  // TEST_EPHEMERIDESRECENTER_H
