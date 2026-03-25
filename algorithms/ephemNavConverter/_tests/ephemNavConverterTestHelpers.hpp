#ifndef TEST_EPHEM_NAV_CONVERTER_H
#define TEST_EPHEM_NAV_CONVERTER_H

#include "ephemNavConverterAlgorithm.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

// ---------------------------------------------------------------------------
// Property test helper functions
// ---------------------------------------------------------------------------

// Output position exactly equals input position.
inline void propertyPositionPassthrough(double timeTag, std::vector<double> rVec, std::vector<double> vVec) {
    InputEphemerisData input{};
    input.timeTag = timeTag;
    input.r_BdyZero_N = Eigen::Map<Eigen::Vector3d>(rVec.data());
    input.v_BdyZero_N = Eigen::Map<Eigen::Vector3d>(vVec.data());

    auto out = EphemNavConverterAlgorithm::update(input);

    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(out.r_BN_N[i], rVec[i]);
    }
}

// Output velocity exactly equals input velocity.
inline void propertyVelocityPassthrough(double timeTag, std::vector<double> rVec, std::vector<double> vVec) {
    InputEphemerisData input{};
    input.timeTag = timeTag;
    input.r_BdyZero_N = Eigen::Map<Eigen::Vector3d>(rVec.data());
    input.v_BdyZero_N = Eigen::Map<Eigen::Vector3d>(vVec.data());

    auto out = EphemNavConverterAlgorithm::update(input);

    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(out.v_BN_N[i], vVec[i]);
    }
}

// Output timeTag exactly equals input timeTag.
inline void propertyTimeTagPassthrough(double timeTag, std::vector<double> rVec, std::vector<double> vVec) {
    InputEphemerisData input{};
    input.timeTag = timeTag;
    input.r_BdyZero_N = Eigen::Map<Eigen::Vector3d>(rVec.data());
    input.v_BdyZero_N = Eigen::Map<Eigen::Vector3d>(vVec.data());

    auto out = EphemNavConverterAlgorithm::update(input);

    EXPECT_EQ(out.timeTag, timeTag);
}

// All output components are finite for finite inputs.
inline void propertyOutputIsFinite(double timeTag, std::vector<double> rVec, std::vector<double> vVec) {
    InputEphemerisData input{};
    input.timeTag = timeTag;
    input.r_BdyZero_N = Eigen::Map<Eigen::Vector3d>(rVec.data());
    input.v_BdyZero_N = Eigen::Map<Eigen::Vector3d>(vVec.data());

    auto out = EphemNavConverterAlgorithm::update(input);

    EXPECT_TRUE(std::isfinite(out.timeTag));
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.r_BN_N[i]));
        EXPECT_TRUE(std::isfinite(out.v_BN_N[i]));
    }
}

#endif  // TEST_EPHEM_NAV_CONVERTER_H
