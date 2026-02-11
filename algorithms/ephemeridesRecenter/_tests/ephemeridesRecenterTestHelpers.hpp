#ifndef TEST_EPHEMERIDESRECENTER_H
#define TEST_EPHEMERIDESRECENTER_H

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "ephemeridesRecenterAlgorithm.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <cstring>
#include <numbers>
#include <vector>

static BodyName makeBodyName(const std::string& bodyName) {
    BodyName newBodyName{};
    std::ranges::copy(bodyName.begin(), bodyName.end(), newBodyName.data());
    return newBodyName;
}

// Reference computation for update
std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> referenceUpdate(
    const EphemeridesRecenterAlgorithm& alg,
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies) {
    // Pull required "configuration state" from the algorithm via getters
    const BodyName newCentralBodyName = alg.getNewZeroBase();
    const BodyName previousCentralBodyName = alg.getPreviousCommonZeroBase();
    const size_t celestialBodyCount = alg.getNumberOfBodies();
    const auto bodyNames = alg.getAllNames();

    if (celestialBodyCount == 0U) {
        FS_THROW_INVALID_ARGUMENT("The current celestial body count is 0");
    }

    // findNewZeroBaseIndex(newCentralBodyName) using the returned bodyNames ordering
    auto* it = std::ranges::find(bodyNames, newCentralBodyName);
    if (it == bodyNames.end()) {
        FS_THROW_INVALID_ARGUMENT("New zero base body was not in the list of existing bodies");
    }
    const size_t newCentralIndex = static_cast<size_t>(std::distance(bodyNames.begin(), it));

    // Local copy of "celestialBodies" used by the algorithm
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> celestialBodies = newBodies;

    // Central body payload
    const auto newCentralBody = celestialBodies[newCentralIndex];
    Eigen::Vector3d newCentral_input_r = newCentralBody.input_r;
    Eigen::Vector3d newCentral_input_v = newCentralBody.input_v;

    /* - If the new central body is a moon (its original central body is not the common central body but another body in
     * the list) first re-center the moon around the common central body so that every body is relative to the common
     * center*/
    if (newCentralBody.originalCentralBodyName != previousCentralBodyName) {
        const auto moonCentralBodyIndex = alg.getBodyIndexFromName(newCentralBody.originalCentralBodyName);
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

        if (celestialBodies[i].originalCentralBodyName == previousCentralBodyName) {
            Eigen::Vector3d const relativePosition = newEphemerisToRecenter_input_r - newCentral_input_r;

            Eigen::Vector3d const relativeVelocity = newEphemerisToRecenter_input_v - newCentral_input_v;

            // implement private function findMoonOfBody(celestialBodies[i]) in the algorithm
            size_t moonIndex = 0U;
            bool moonFound = false;
            for (size_t j = 0U; j < celestialBodyCount; ++j) {
                if (celestialBodies[j].originalCentralBodyName == celestialBodies[i].bodySpiceName) {
                    moonIndex = j;
                    moonFound = true;
                    break;
                }
            }

            if (moonFound && celestialBodies[i].bodySpiceName != previousCentralBodyName) {
                Eigen::Vector3d moonOfBody_input_r = celestialBodies[moonIndex].input_r;
                Eigen::Vector3d moonOfBody_input_v = celestialBodies[moonIndex].input_v;
                Eigen::Vector3d const moonRelativePosition = relativePosition + moonOfBody_input_r;

                Eigen::Vector3d const moonRelativeVelocity = relativeVelocity + moonOfBody_input_v;

                recenteredBodies[moonIndex].bodySpiceName = celestialBodies[moonIndex].bodySpiceName;
                recenteredBodies[moonIndex].isMoon = true;
                recenteredBodies[moonIndex].originalCentralBodyName =
                    celestialBodies[moonIndex].originalCentralBodyName;
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
    const std::vector<BodyName>& bodyListInOrder,  // must be {"SUN","EARTH","MOON","SATURN"}
    int previousCommonIdx,                         // in {0,1,2,3}
    int newZeroIdx,                                // in {0,1,2,3}
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
    const BodyName previousCommonZeroBaseName = bodyListInOrder[static_cast<size_t>(previousCommonIdx)];
    const BodyName newZeroBaseName = bodyListInOrder[static_cast<size_t>(newZeroIdx)];
    auto containsName = [&](const BodyName& name) -> bool {
        return std::find(bodyListInOrder.begin(), bodyListInOrder.end(), name) != bodyListInOrder.end();
    };
    ASSERT_TRUE(containsName(previousCommonZeroBaseName));
    ASSERT_TRUE(containsName(newZeroBaseName));
    ASSERT_EQ(bodyListInOrder.size(), 4U);  // SUN/EARTH/MOON/SATURN only

    // --- Build newBodies from the inputs ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    for (auto& b : newBodies) {
        b = BodyEphemerisPayload{};
    }

    auto fillBody = [&](size_t idx,
                        const BodyName& spiceName,
                        const BodyName& originalCentral,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies[idx].bodySpiceName = spiceName;
        newBodies[idx].originalCentralBodyName = originalCentral;
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
    for (const auto& name : bodyListInOrder) {
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(name));
    }
    EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(previousCommonZeroBaseName));
    EXPECT_NO_THROW(alg.setNewZeroBaseName(newZeroBaseName));
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
        EXPECT_EQ(out[i].bodySpiceName, ref[i].bodySpiceName);
        EXPECT_EQ(out[i].originalCentralBodyName, ref[i].originalCentralBodyName);
    }
}

inline void testEphemeridesRecenterSetup() {
    const std::vector<BodyName> bodyListInOrder{
        makeBodyName("SUN"),
        makeBodyName("EARTH"),
        makeBodyName("MOON"),
        makeBodyName("SATURN"),
    };

    EphemeridesRecenterAlgorithm alg{};
    // Setting previous common zero base when body list is empty should throw
    EXPECT_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]), fs::invalid_argument);
    // Requesting info when empty should throw (per your getAllNames() / getBodyIndexFromName guards)
    EXPECT_THROW(alg.getAllNames(), fs::invalid_argument);

    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[0]));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[1]));
    EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[2]));

    // Duplicate should throw
    EXPECT_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[2]), fs::invalid_argument);

    // Basic getters after adding
    EXPECT_EQ(alg.getNumberOfBodies(), 3U);

    const auto names = alg.getAllNames();
    EXPECT_EQ(names[0], bodyListInOrder[0]);
    EXPECT_EQ(names[1], bodyListInOrder[1]);
    EXPECT_EQ(names[2], bodyListInOrder[2]);

    // Set and get the previous common zero base
    EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
    EXPECT_EQ(alg.getPreviousCommonZeroBase(), bodyListInOrder[0]);

    // setNewZeroBaseName should not throw for any name, but reset() should validate it exists in list
    EXPECT_NO_THROW(alg.setNewZeroBaseName(bodyListInOrder[1]));
    EXPECT_EQ(alg.getNewZeroBase(), bodyListInOrder[1]);

    // test reset()
    EXPECT_NO_THROW(alg.reset());

    // If you set a zero base that is NOT in the list, reset() must throw
    EXPECT_NO_THROW(alg.setNewZeroBaseName(bodyListInOrder[3]));
    EXPECT_THROW(alg.reset(), fs::invalid_argument);

    // Clear should reset internal list; then getters that require non-empty should throw again
    EXPECT_NO_THROW(alg.clearAllBodies());
    EXPECT_EQ(alg.getNumberOfBodies(), 0U);
    EXPECT_THROW(alg.getAllNames(), fs::invalid_argument);

    // Add exactly MAX_NUM_CHANGE_BODIES bodies (unique names)
    for (std::size_t i = 0; i < MAX_NUM_CHANGE_BODIES; ++i) {
        // Make a unique name (fits BodyName max len)
        const std::string name = "B" + std::to_string(i);
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(makeBodyName(name.c_str())));
    }
    // One more should exceed the limit and throw
    EXPECT_THROW(alg.addBodyEphemerisToRecenter(makeBodyName("B_TOO_MANY")), fs::invalid_argument);
}

inline void testRecenterEphemeridesRecenter() {
    const std::vector<BodyName> bodyListInOrder{
        makeBodyName("SUN"),
        makeBodyName("EARTH"),
        makeBodyName("MOON"),
        makeBodyName("SATURN"),
        makeBodyName("TITAN"),
    };

    // --- Build newBodies from the inputs ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    for (auto& b : newBodies) {
        b = BodyEphemerisPayload{};
    }

    auto fillBody = [&](size_t idx,
                        const BodyName& spiceName,
                        const BodyName& originalCentral,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies[idx].bodySpiceName = spiceName;
        newBodies[idx].originalCentralBodyName = originalCentral;
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
        // empty body shall throw
        EXPECT_THROW(alg.updateState(newBodies), fs::invalid_argument);
        EXPECT_THROW(alg.getBodyIndexFromName(bodyListInOrder[0]), fs::invalid_argument);
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[1]));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[2]));

        // get body that has not been added shall throw
        EXPECT_THROW(alg.getBodyIndexFromName(bodyListInOrder[3]), fs::invalid_argument);
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[3]));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[4]));

        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseName(bodyListInOrder[1]));
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
    const std::vector<BodyName> bodyListInOrder{
        makeBodyName("SUN"),
        makeBodyName("EARTH"),
        makeBodyName("MOON"),
        makeBodyName("SATURN"),
        makeBodyName("TITAN"),
    };

    // --- Build newBodies from the inputs ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    for (auto& b : newBodies) {
        b = BodyEphemerisPayload{};
    }

    auto fillBody = [&](size_t idx,
                        const BodyName& spiceName,
                        const BodyName& originalCentral,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies[idx].bodySpiceName = spiceName;
        newBodies[idx].originalCentralBodyName = originalCentral;
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
        // empty body shall throw
        EXPECT_THROW(alg.updateState(newBodies), fs::invalid_argument);
        EXPECT_THROW(alg.getBodyIndexFromName(bodyListInOrder[0]), fs::invalid_argument);
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[1]));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[2]));

        // get body that has not been added shall throw
        EXPECT_THROW(alg.getBodyIndexFromName(bodyListInOrder[3]), fs::invalid_argument);
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[3]));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[4]));

        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseName(bodyListInOrder[2]));
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
    const std::vector<BodyName> bodyListInOrder{
        makeBodyName("SUN"),
        makeBodyName("EARTH"),
        makeBodyName("MOON"),
        makeBodyName("SATURN"),
        makeBodyName("TITAN"),
    };

    // --- Build newBodies from the inputs ---
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    for (auto& b : newBodies) {
        b = BodyEphemerisPayload{};
    }

    auto fillBody = [&](size_t idx,
                        const BodyName& spiceName,
                        const BodyName& originalCentral,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies[idx].bodySpiceName = spiceName;
        newBodies[idx].originalCentralBodyName = originalCentral;
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
        // empty body shall throw
        EXPECT_THROW(alg.updateState(newBodies), fs::invalid_argument);
        EXPECT_THROW(alg.getBodyIndexFromName(bodyListInOrder[0]), fs::invalid_argument);
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[1]));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[2]));

        // get body that has not been added shall throw
        EXPECT_THROW(alg.getBodyIndexFromName(bodyListInOrder[3]), fs::invalid_argument);
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[3]));
        EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(bodyListInOrder[4]));

        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseName(bodyListInOrder[0]));
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

#endif  // TEST_EPHEMERIDESRECENTER_H
