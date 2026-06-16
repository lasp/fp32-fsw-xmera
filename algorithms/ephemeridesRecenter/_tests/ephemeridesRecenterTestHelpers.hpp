#ifndef TEST_EPHEMERIDESRECENTER_H
#define TEST_EPHEMERIDESRECENTER_H

#include "ephemeridesRecenterAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <vector>

constexpr int SUN_SPICE_ID = 10;
constexpr int EARTH_SPICE_ID = 399;
constexpr int MOON_SPICE_ID = 301;
constexpr int SATURN_SPICE_ID = 699;
constexpr int TITAN_SPICE_ID = 606;
constexpr int MARS_SPICE_ID = 499;

//! Build a validated config from same-length body / original-central ID lists.
inline EphemeridesRecenterConfig makeConfig(const std::vector<int>& bodyIds,
                                            const std::vector<int>& originalCentralBodyIds,
                                            int previousCommonZeroBaseId,
                                            int newZeroBaseId) {
    std::array<int, MAX_NUM_CHANGE_BODIES> ids{};
    std::array<int, MAX_NUM_CHANGE_BODIES> origIds{};
    for (std::size_t i = 0U; i < bodyIds.size() && i < MAX_NUM_CHANGE_BODIES; ++i) {
        ids.at(i) = bodyIds[i];
        origIds.at(i) = originalCentralBodyIds[i];
    }
    return EphemeridesRecenterConfig::create(newZeroBaseId, previousCommonZeroBaseId, ids, origIds, bodyIds.size());
}

// Reference computation for update, independent of the algorithm's internal precompute.
inline std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> referenceUpdate(
    const EphemeridesRecenterConfig& cfg,
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies) {
    const int newCentralBodyId = cfg.getNewCentralBodyId();
    const int previousCentralBodyId = cfg.getPreviousCentralBodyId();
    const size_t celestialBodyCount = cfg.getBodyCount();
    const std::array<int, MAX_NUM_CHANGE_BODIES>& bodyIds = cfg.getBodyIds();

    size_t newCentralIndex = 0U;
    for (size_t k = 0U; k < celestialBodyCount; ++k) {
        if (bodyIds.at(k) == newCentralBodyId) {
            newCentralIndex = k;
            break;
        }
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> celestialBodies = newBodies;

    const BodyEphemerisPayload newCentralBody = celestialBodies.at(newCentralIndex);
    Eigen::Vector3d newCentral_input_r = newCentralBody.input_r;
    Eigen::Vector3d newCentral_input_v = newCentralBody.input_v;

    // If the new central body is a moon, first re-center it around the common central body.
    if (newCentralBody.originalCentralBodyId != previousCentralBodyId) {
        size_t moonCentralBodyIndex = 0U;
        for (size_t k = 0U; k < celestialBodyCount; ++k) {
            if (bodyIds.at(k) == newCentralBody.originalCentralBodyId) {
                moonCentralBodyIndex = k;
                break;
            }
        }
        newCentral_input_r += celestialBodies.at(moonCentralBodyIndex).input_r;
        newCentral_input_v += celestialBodies.at(moonCentralBodyIndex).input_v;
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> recenteredBodies{};
    for (size_t i = 0U; i < celestialBodyCount; ++i) {
        if (recenteredBodies.at(i).isMoon) {
            continue;
        }

        recenteredBodies.at(i) = BodyEphemerisPayload{};
        if (celestialBodies.at(i).originalCentralBodyId == previousCentralBodyId) {
            Eigen::Vector3d const relativePosition = newBodies.at(i).input_r - newCentral_input_r;
            Eigen::Vector3d const relativeVelocity = newBodies.at(i).input_v - newCentral_input_v;

            size_t moonIndex = 0U;
            bool moonFound = false;
            for (size_t j = 0U; j < celestialBodyCount; ++j) {
                if (celestialBodies.at(j).originalCentralBodyId == celestialBodies.at(i).bodySpiceId) {
                    moonIndex = j;
                    moonFound = true;
                    break;
                }
            }

            if (moonFound && celestialBodies.at(i).bodySpiceId != previousCentralBodyId) {
                recenteredBodies.at(moonIndex).bodySpiceId = celestialBodies.at(moonIndex).bodySpiceId;
                recenteredBodies.at(moonIndex).isMoon = true;
                recenteredBodies.at(moonIndex).originalCentralBodyId =
                    celestialBodies.at(moonIndex).originalCentralBodyId;
                recenteredBodies.at(moonIndex).output_r = relativePosition + celestialBodies.at(moonIndex).input_r;
                recenteredBodies.at(moonIndex).output_v = relativeVelocity + celestialBodies.at(moonIndex).input_v;
            }

            recenteredBodies.at(i) = newBodies.at(i);
            recenteredBodies.at(i).output_r = relativePosition;
            recenteredBodies.at(i).output_v = relativeVelocity;
        }
    }

    return recenteredBodies;
}

// Fixed body list SUN/EARTH/MOON/SATURN regression check.
inline void regressionTestEphemeridesRecenter(const std::vector<int>& bodyListInOrder,
                                              int previousCommonIdx,
                                              int newZeroIdx,
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
    ASSERT_EQ(bodyListInOrder.size(), 4U);
    ASSERT_GE(previousCommonIdx, 0);
    ASSERT_LE(previousCommonIdx, 3);
    ASSERT_GE(newZeroIdx, 0);
    ASSERT_LE(newZeroIdx, 3);
    const int previousCommonZeroBaseId = bodyListInOrder[static_cast<size_t>(previousCommonIdx)];
    const int newZeroBaseId = bodyListInOrder[static_cast<size_t>(newZeroIdx)];

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    auto fillBody = [&](size_t idx,
                        int spiceId,
                        int originalCentralId,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies.at(idx).bodySpiceId = spiceId;
        newBodies.at(idx).originalCentralBodyId = originalCentralId;
        newBodies.at(idx).isMoon = isMoonFlag;
        for (int k = 0; k < 3; ++k) {
            newBodies.at(idx).input_r[k] = r[k];
            newBodies.at(idx).input_v[k] = v[k];
        }
    };
    const int previousCommonId = bodyListInOrder[static_cast<size_t>(previousCommonIdx)];
    const std::vector<int> originalCentralIds = {
        previousCommonId, previousCommonId, isMoon2 ? bodyListInOrder[1] : previousCommonId, previousCommonId};
    fillBody(0, bodyListInOrder[0], originalCentralIds[0], r0, v0, isMoon0);
    fillBody(1, bodyListInOrder[1], originalCentralIds[1], r1, v1, isMoon1);
    fillBody(2, bodyListInOrder[2], originalCentralIds[2], r2, v2, isMoon2);
    fillBody(3, bodyListInOrder[3], originalCentralIds[3], r3, v3, isMoon3);

    const EphemeridesRecenterConfig cfg =
        makeConfig(bodyListInOrder, originalCentralIds, previousCommonZeroBaseId, newZeroBaseId);
    EphemeridesRecenterAlgorithm alg{cfg};

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> out{};
    EXPECT_NO_THROW(out = alg.updateState(newBodies));
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> ref = referenceUpdate(cfg, newBodies);

    for (size_t i = 0U; i < cfg.getBodyCount(); ++i) {
        for (int k = 0; k < 3; ++k) {
            EXPECT_NEAR(out.at(i).output_r[k], ref.at(i).output_r[k], 1e-6);
            EXPECT_NEAR(out.at(i).output_v[k], ref.at(i).output_v[k], 1e-6);
            EXPECT_TRUE(std::isfinite(out.at(i).output_r[k]));
            EXPECT_TRUE(std::isfinite(out.at(i).output_v[k]));
        }
        EXPECT_EQ(out.at(i).isMoon, ref.at(i).isMoon);
        EXPECT_EQ(out.at(i).bodySpiceId, ref.at(i).bodySpiceId);
        EXPECT_EQ(out.at(i).originalCentralBodyId, ref.at(i).originalCentralBodyId);
    }
}

inline void testEphemeridesRecenterSetup() {
    const std::vector<int> bodyIds{SUN_SPICE_ID, EARTH_SPICE_ID, MOON_SPICE_ID, SATURN_SPICE_ID};
    const std::vector<int> originalCentralIds{SUN_SPICE_ID, SUN_SPICE_ID, EARTH_SPICE_ID, SUN_SPICE_ID};

    // Valid configuration round-trips its values.
    const EphemeridesRecenterConfig cfg = makeConfig(bodyIds, originalCentralIds, SUN_SPICE_ID, EARTH_SPICE_ID);
    EXPECT_EQ(cfg.getNewCentralBodyId(), EARTH_SPICE_ID);
    EXPECT_EQ(cfg.getPreviousCentralBodyId(), SUN_SPICE_ID);
    EXPECT_EQ(cfg.getBodyCount(), 4U);
    EXPECT_EQ(cfg.getBodyIds().at(2), MOON_SPICE_ID);

    // New central body must be in the list.
    EXPECT_FALSE(EphemeridesRecenterConfig::isValidNewCentralBody(MARS_SPICE_ID, cfg.getBodyIds(), cfg.getBodyCount()));
    EXPECT_THROW(makeConfig(bodyIds, originalCentralIds, SUN_SPICE_ID, MARS_SPICE_ID), fsw::invalid_argument);

    // Body count must not exceed the maximum.
    EXPECT_TRUE(EphemeridesRecenterConfig::isValidBodyCount(MAX_NUM_CHANGE_BODIES));
    EXPECT_FALSE(EphemeridesRecenterConfig::isValidBodyCount(MAX_NUM_CHANGE_BODIES + 1U));
    {
        std::array<int, MAX_NUM_CHANGE_BODIES> ids{};
        std::array<int, MAX_NUM_CHANGE_BODIES> origIds{};
        EXPECT_THROW(EphemeridesRecenterConfig::create(0, 0, ids, origIds, MAX_NUM_CHANGE_BODIES + 1U),
                     fsw::invalid_argument);
    }

    // A valid topology passes the predicate.
    EXPECT_TRUE(EphemeridesRecenterConfig::isValidMoonTopology(
        SUN_SPICE_ID, cfg.getBodyIds(), cfg.getOriginalCentralBodyIds(), cfg.getBodyCount()));
}

inline void testRecenterEphemeridesRecenter() {
    const std::vector<int> bodyIds{SUN_SPICE_ID, EARTH_SPICE_ID, MOON_SPICE_ID, SATURN_SPICE_ID, TITAN_SPICE_ID};
    const std::vector<int> originalCentralIds{
        SUN_SPICE_ID, SUN_SPICE_ID, EARTH_SPICE_ID, SUN_SPICE_ID, SATURN_SPICE_ID};

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    auto fillBody = [&](size_t idx,
                        int spiceId,
                        int originalCentralId,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies.at(idx).bodySpiceId = spiceId;
        newBodies.at(idx).originalCentralBodyId = originalCentralId;
        newBodies.at(idx).isMoon = isMoonFlag;
        for (int k = 0; k < 3; ++k) {
            newBodies.at(idx).input_r[k] = r[k];
            newBodies.at(idx).input_v[k] = v[k];
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
    fillBody(0, bodyIds[0], originalCentralIds[0], r_sun, v_sun, false);
    fillBody(1, bodyIds[1], originalCentralIds[1], r_earth, v_earth, false);
    fillBody(2, bodyIds[2], originalCentralIds[2], r_moon_to_earth, v_moon_to_earth, true);
    fillBody(3, bodyIds[3], originalCentralIds[3], r_saturn, v_saturn, false);
    fillBody(4, bodyIds[4], originalCentralIds[4], r_titan, v_titan, true);

    // newZeroBase = EARTH (planet)
    EphemeridesRecenterAlgorithm alg{makeConfig(bodyIds, originalCentralIds, SUN_SPICE_ID, EARTH_SPICE_ID)};
    auto out = alg.updateState(newBodies);

    for (size_t i = 0U; i < 3U; ++i) {
        EXPECT_NEAR(out[0].output_r[i], r_sun[i] - r_earth[i], 1e-6);
        EXPECT_NEAR(out[0].output_v[i], v_sun[i] - v_earth[i], 1e-6);
        EXPECT_NEAR(out[1].output_r[i], 0.0, 1e-6);
        EXPECT_NEAR(out[1].output_v[i], 0.0, 1e-6);
        EXPECT_NEAR(out[2].output_r[i], r_moon_to_earth[i], 1e-6);
        EXPECT_NEAR(out[2].output_v[i], v_moon_to_earth[i], 1e-6);
        EXPECT_NEAR(out[3].output_r[i], r_saturn[i] - r_earth[i], 1e-6);
        EXPECT_NEAR(out[3].output_v[i], v_saturn[i] - v_earth[i], 1e-6);
        EXPECT_NEAR(out[4].output_r[i], r_titan[i] + (r_saturn[i] - r_earth[i]), 1e-6);
        EXPECT_NEAR(out[4].output_v[i], v_titan[i] + (v_saturn[i] - v_earth[i]), 1e-6);
    }
}

inline void testRecenterMoonEphemeridesRecenter() {
    const std::vector<int> bodyIds{SUN_SPICE_ID, EARTH_SPICE_ID, MOON_SPICE_ID, SATURN_SPICE_ID, TITAN_SPICE_ID};
    const std::vector<int> originalCentralIds{
        SUN_SPICE_ID, SUN_SPICE_ID, EARTH_SPICE_ID, SUN_SPICE_ID, SATURN_SPICE_ID};

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    auto fillBody = [&](size_t idx,
                        int spiceId,
                        int originalCentralId,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies.at(idx).bodySpiceId = spiceId;
        newBodies.at(idx).originalCentralBodyId = originalCentralId;
        newBodies.at(idx).isMoon = isMoonFlag;
        for (int k = 0; k < 3; ++k) {
            newBodies.at(idx).input_r[k] = r[k];
            newBodies.at(idx).input_v[k] = v[k];
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
    fillBody(0, bodyIds[0], originalCentralIds[0], r_sun, v_sun, false);
    fillBody(1, bodyIds[1], originalCentralIds[1], r_earth, v_earth, false);
    fillBody(2, bodyIds[2], originalCentralIds[2], r_moon_to_earth, v_moon_to_earth, true);
    fillBody(3, bodyIds[3], originalCentralIds[3], r_saturn, v_saturn, false);
    fillBody(4, bodyIds[4], originalCentralIds[4], r_titan, v_titan, true);

    // newZeroBase = Moon
    EphemeridesRecenterAlgorithm alg{makeConfig(bodyIds, originalCentralIds, SUN_SPICE_ID, MOON_SPICE_ID)};
    auto out = alg.updateState(newBodies);

    for (size_t i = 0U; i < 3U; ++i) {
        EXPECT_NEAR(out[0].output_r[i], r_sun[i] - (r_moon_to_earth[i] + r_earth[i]), 1e-6);
        EXPECT_NEAR(out[0].output_v[i], v_sun[i] - (v_moon_to_earth[i] + v_earth[i]), 1e-6);
        EXPECT_NEAR(out[1].output_r[i], r_earth[i] - (r_moon_to_earth[i] + r_earth[i]), 1e-6);
        EXPECT_NEAR(out[1].output_v[i], v_earth[i] - (v_moon_to_earth[i] + v_earth[i]), 1e-6);
        EXPECT_NEAR(out[2].output_r[i], 0.0, 1e-6);
        EXPECT_NEAR(out[2].output_v[i], 0.0, 1e-6);
        EXPECT_NEAR(out[3].output_r[i], r_saturn[i] - (r_moon_to_earth[i] + r_earth[i]), 1e-6);
        EXPECT_NEAR(out[3].output_v[i], v_saturn[i] - (v_moon_to_earth[i] + v_earth[i]), 1e-6);
        EXPECT_NEAR(out[4].output_r[i], r_titan[i] + (r_saturn[i] - (r_moon_to_earth[i] + r_earth[i])), 1e-6);
        EXPECT_NEAR(out[4].output_v[i], v_titan[i] + (v_saturn[i] - (v_moon_to_earth[i] + v_earth[i])), 1e-6);
    }
}

inline void testRecenterPreCommonEphemeridesRecenter() {
    const std::vector<int> bodyIds{SUN_SPICE_ID, EARTH_SPICE_ID, MOON_SPICE_ID, SATURN_SPICE_ID, TITAN_SPICE_ID};
    const std::vector<int> originalCentralIds{
        SUN_SPICE_ID, SUN_SPICE_ID, EARTH_SPICE_ID, SUN_SPICE_ID, SATURN_SPICE_ID};

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> newBodies{};
    auto fillBody = [&](size_t idx,
                        int spiceId,
                        int originalCentralId,
                        const std::array<double, 3>& r,
                        const std::array<double, 3>& v,
                        bool isMoonFlag) {
        newBodies.at(idx).bodySpiceId = spiceId;
        newBodies.at(idx).originalCentralBodyId = originalCentralId;
        newBodies.at(idx).isMoon = isMoonFlag;
        for (int k = 0; k < 3; ++k) {
            newBodies.at(idx).input_r[k] = r[k];
            newBodies.at(idx).input_v[k] = v[k];
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
    fillBody(0, bodyIds[0], originalCentralIds[0], r_sun, v_sun, false);
    fillBody(1, bodyIds[1], originalCentralIds[1], r_earth, v_earth, false);
    fillBody(2, bodyIds[2], originalCentralIds[2], r_moon_to_earth, v_moon_to_earth, true);
    fillBody(3, bodyIds[3], originalCentralIds[3], r_saturn, v_saturn, false);
    fillBody(4, bodyIds[4], originalCentralIds[4], r_titan, v_titan, true);

    // newZeroBase = previousCommon (Sun)
    EphemeridesRecenterAlgorithm alg{makeConfig(bodyIds, originalCentralIds, SUN_SPICE_ID, SUN_SPICE_ID)};
    auto out = alg.updateState(newBodies);

    for (size_t i = 0U; i < 3U; ++i) {
        EXPECT_NEAR(out[0].output_r[i], r_sun[i], 1e-6);
        EXPECT_NEAR(out[0].output_v[i], v_sun[i], 1e-6);
        EXPECT_NEAR(out[1].output_r[i], r_earth[i], 1e-6);
        EXPECT_NEAR(out[1].output_v[i], v_earth[i], 1e-6);
        EXPECT_NEAR(out[2].output_r[i], r_moon_to_earth[i] + r_earth[i], 1e-6);
        EXPECT_NEAR(out[2].output_v[i], v_moon_to_earth[i] + v_earth[i], 1e-6);
        EXPECT_NEAR(out[3].output_r[i], r_saturn[i], 1e-6);
        EXPECT_NEAR(out[3].output_v[i], v_saturn[i], 1e-6);
        EXPECT_NEAR(out[4].output_r[i], r_titan[i] + r_saturn[i], 1e-6);
        EXPECT_NEAR(out[4].output_v[i], v_titan[i] + v_saturn[i], 1e-6);
    }
}

inline void testMultiMoonsRecenter() {
    // Titan and Moon are both moons of Earth -> multiple moons per parent, rejected at config time.
    const std::vector<int> bodyIds{SUN_SPICE_ID, EARTH_SPICE_ID, MOON_SPICE_ID, SATURN_SPICE_ID, TITAN_SPICE_ID};
    const std::vector<int> originalCentralIds{SUN_SPICE_ID, SUN_SPICE_ID, EARTH_SPICE_ID, SUN_SPICE_ID, EARTH_SPICE_ID};
    EXPECT_THROW(makeConfig(bodyIds, originalCentralIds, SUN_SPICE_ID, SUN_SPICE_ID), fsw::invalid_argument);
}

inline void testMoonOfMoonRecenter() {
    // A sub-moon whose parent is itself a moon should be rejected at config time.
    constexpr int SUB_MOON_SPICE_ID = 30101;
    const std::vector<int> bodyIds{SUN_SPICE_ID, EARTH_SPICE_ID, MOON_SPICE_ID, SUB_MOON_SPICE_ID};
    const std::vector<int> originalCentralIds{SUN_SPICE_ID, SUN_SPICE_ID, EARTH_SPICE_ID, MOON_SPICE_ID};
    EXPECT_THROW(makeConfig(bodyIds, originalCentralIds, SUN_SPICE_ID, EARTH_SPICE_ID), fsw::invalid_argument);
}

inline void testOrphanMoonRecenter() {
    // A moon whose parent is NOT in the body list should be rejected at config time.
    const std::vector<int> bodyIds{SUN_SPICE_ID, EARTH_SPICE_ID, MOON_SPICE_ID};
    const std::vector<int> originalCentralIds{SUN_SPICE_ID, SUN_SPICE_ID, MARS_SPICE_ID};  // Moon's parent Mars absent
    EXPECT_THROW(makeConfig(bodyIds, originalCentralIds, SUN_SPICE_ID, EARTH_SPICE_ID), fsw::invalid_argument);
}

inline void testDuplicateBodyRecenter() {
    // Two Sun entries (exact duplicates), Earth as new central.
    const std::vector<int> bodyIds{SUN_SPICE_ID, EARTH_SPICE_ID, SUN_SPICE_ID};
    const std::vector<int> originalCentralIds{SUN_SPICE_ID, SUN_SPICE_ID, SUN_SPICE_ID};
    EphemeridesRecenterAlgorithm alg{makeConfig(bodyIds, originalCentralIds, SUN_SPICE_ID, EARTH_SPICE_ID)};

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

    for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(out[0].output_r[k], -newBodies[1].input_r[k], 1e-6);
        EXPECT_NEAR(out[2].output_r[k], -newBodies[1].input_r[k], 1e-6);  // duplicate matches
        EXPECT_NEAR(out[1].output_r[k], 0.0, 1e-6);                       // Earth is the new center
    }
}

#endif  // TEST_EPHEMERIDESRECENTER_H
