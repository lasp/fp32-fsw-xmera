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
#include <stdexcept>
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
    EphemerisMsgF32Payload newCentralBodyPayload = newCentralBody.inputEphemerisPayload;

    /* - If the new central body is a moon (its original central body is not the common central body but another body in
     * the list) first re-center the moon around the common central body so that every body is relative to the common
     * center*/
    if (newCentralBody.originalCentralBodyName != previousCentralBodyName) {
        const auto moonCentralBodyIndex = alg.getBodyIndexFromName(newCentralBody.originalCentralBodyName);
        auto moonCentralBodyInput = celestialBodies[moonCentralBodyIndex].inputEphemerisPayload;
        Eigen::Vector3d const relativePosition = cArrayToEigenVector3(newCentralBodyPayload.r_BdyZero_N) +
                                                 cArrayToEigenVector3(moonCentralBodyInput.r_BdyZero_N);
        eigenVectorToCArray(relativePosition, newCentralBodyPayload.r_BdyZero_N);

        Eigen::Vector3d const relativeVelocity = cArrayToEigenVector3(newCentralBodyPayload.v_BdyZero_N) +
                                                 cArrayToEigenVector3(moonCentralBodyInput.v_BdyZero_N);
        eigenVectorToCArray(relativeVelocity, newCentralBodyPayload.v_BdyZero_N);
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> recenteredBodies{};
    for (size_t i = 0U; i < celestialBodyCount; ++i) {
        if (recenteredBodies[i].isMoon) {
            continue;
        }

        recenteredBodies[i] = BodyEphemerisPayload{};
        EphemerisMsgF32Payload newEphemerisToRecenterPayload = newBodies[i].inputEphemerisPayload;

        if (celestialBodies[i].originalCentralBodyName == previousCentralBodyName) {
            Eigen::Vector3d const relativePosition = cArrayToEigenVector3(newEphemerisToRecenterPayload.r_BdyZero_N) -
                                                     cArrayToEigenVector3(newCentralBodyPayload.r_BdyZero_N);
            eigenVectorToCArray(relativePosition, newEphemerisToRecenterPayload.r_BdyZero_N);

            Eigen::Vector3d const relativeVelocity = cArrayToEigenVector3(newEphemerisToRecenterPayload.v_BdyZero_N) -
                                                     cArrayToEigenVector3(newCentralBodyPayload.v_BdyZero_N);
            eigenVectorToCArray(relativeVelocity, newEphemerisToRecenterPayload.v_BdyZero_N);

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
                EphemerisMsgF32Payload moonOfBodyPayload = celestialBodies[moonIndex].inputEphemerisPayload;

                Eigen::Vector3d const moonRelativePosition =
                    cArrayToEigenVector3(newEphemerisToRecenterPayload.r_BdyZero_N) +
                    cArrayToEigenVector3(moonOfBodyPayload.r_BdyZero_N);
                eigenVectorToCArray(moonRelativePosition, moonOfBodyPayload.r_BdyZero_N);

                Eigen::Vector3d const moonRelativeVelocity =
                    cArrayToEigenVector3(newEphemerisToRecenterPayload.v_BdyZero_N) +
                    cArrayToEigenVector3(moonOfBodyPayload.v_BdyZero_N);
                eigenVectorToCArray(moonRelativeVelocity, moonOfBodyPayload.v_BdyZero_N);

                recenteredBodies[moonIndex].bodySpiceName = celestialBodies[moonIndex].bodySpiceName;
                recenteredBodies[moonIndex].isMoon = true;
                recenteredBodies[moonIndex].originalCentralBodyName =
                    celestialBodies[moonIndex].originalCentralBodyName;
                recenteredBodies[moonIndex].inputEphemerisPayload = celestialBodies[moonIndex].inputEphemerisPayload;
                recenteredBodies[moonIndex].outputEphemerisPayload = moonOfBodyPayload;
            }

            recenteredBodies[i] = newBodies[i];
            recenteredBodies[i].outputEphemerisPayload = newEphemerisToRecenterPayload;
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
            newBodies[idx].inputEphemerisPayload.r_BdyZero_N[k] = r[k];
            newBodies[idx].inputEphemerisPayload.v_BdyZero_N[k] = v[k];
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
            EXPECT_NEAR(
                out[i].outputEphemerisPayload.r_BdyZero_N[k], ref[i].outputEphemerisPayload.r_BdyZero_N[k], 1e-6);
            EXPECT_NEAR(
                out[i].outputEphemerisPayload.v_BdyZero_N[k], ref[i].outputEphemerisPayload.v_BdyZero_N[k], 1e-6);

            EXPECT_TRUE(std::isfinite(out[i].outputEphemerisPayload.r_BdyZero_N[k]));
            EXPECT_TRUE(std::isfinite(out[i].outputEphemerisPayload.v_BdyZero_N[k]));
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

inline void propertyTestEphemeridesRecenter() {
    const std::vector<BodyName> bodyListInOrder{
        makeBodyName("SUN"),
        makeBodyName("EARTH"),
        makeBodyName("MOON"),
        makeBodyName("SATURN"),
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
            newBodies[idx].inputEphemerisPayload.r_BdyZero_N[k] = r[k];
            newBodies[idx].inputEphemerisPayload.v_BdyZero_N[k] = v[k];
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
    fillBody(0, bodyListInOrder[0], bodyListInOrder[0], r_sun, v_sun, false);
    fillBody(1, bodyListInOrder[1], bodyListInOrder[0], r_earth, v_earth, false);
    fillBody(2, bodyListInOrder[2], bodyListInOrder[1], r_moon_to_earth, v_moon_to_earth, true);
    fillBody(3, bodyListInOrder[3], bodyListInOrder[0], r_saturn, v_saturn, false);

    // ------------------------------------------------------------
    // Property 1: newZeroBase = EARTH (planet)
    // Expect SATURN' = SATURN - EARTH (since SATURN originalCentral == SUN and previousCommon == SUN)
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

        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseName(bodyListInOrder[1]));
        EXPECT_NO_THROW(alg.reset());

        auto out = alg.updateState(newBodies);

        // SATURN' = SATURN - EARTH
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[3].outputEphemerisPayload.r_BdyZero_N[i], r_saturn[i] - r_earth[i], 1e-6);
            EXPECT_NEAR(out[3].outputEphemerisPayload.v_BdyZero_N[i], v_saturn[i] - v_earth[i], 1e-6);
        }
    }

    // ------------------------------------------------------------
    // Property 1.1: newZeroBase = previous common base
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

        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseName(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.reset());

        auto out = alg.updateState(newBodies);

        // SATURN' = SATURN
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(out[3].outputEphemerisPayload.r_BdyZero_N[i], r_saturn[i], 1e-6);
            EXPECT_NEAR(out[3].outputEphemerisPayload.v_BdyZero_N[i], v_saturn[i], 1e-6);
        }
    }

    // ------------------------------------------------------------
    // Property 2: newZeroBase = MOON (moon)
    // First, MOON is converted to common base (SUN) by: moon_wrt_sun = moon_wrt_earth + earth_wrt_sun
    // Then SATURN' = SATURN - moon_wrt_sun
    // ------------------------------------------------------------
    {
        EphemeridesRecenterAlgorithm alg{};
        for (const auto& n : bodyListInOrder) EXPECT_NO_THROW(alg.addBodyEphemerisToRecenter(n));
        EXPECT_NO_THROW(alg.setPreviousCommonZeroBase(bodyListInOrder[0]));
        EXPECT_NO_THROW(alg.setNewZeroBaseName(bodyListInOrder[2]));
        EXPECT_NO_THROW(alg.reset());

        auto out = alg.updateState(newBodies);

        // SATURN' = SATURN - moon_to_sun, moon_to_sun = moon_to_earth + earth_to_sun
        for (size_t i = 0U; i < 3U; ++i) {
            EXPECT_NEAR(
                out[3].outputEphemerisPayload.r_BdyZero_N[i], r_saturn[i] - (r_moon_to_earth[i] + r_earth[i]), 1e-6);
            EXPECT_NEAR(
                out[3].outputEphemerisPayload.v_BdyZero_N[i], v_saturn[i] - (v_moon_to_earth[i] + v_earth[i]), 1e-6);
        }
    }
}

#endif  // TEST_EPHEMERIDESRECENTER_H
