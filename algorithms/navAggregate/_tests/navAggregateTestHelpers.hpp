#ifndef TEST_NAV_AGGREGATE_H
#define TEST_NAV_AGGREGATE_H

#include "../freestandingInvalidArgument.h"
#include "navAggregateAlgorithm.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>

using namespace f32;

inline void testNavAggregate(double attTimeTag,
                             const Eigen::Vector3f& sigma_BN,
                             const Eigen::Vector3f& omega_BN_B,
                             const Eigen::Vector3f& vehSunPntBdy,
                             double transTimeTag,
                             const Eigen::Vector3d& r_BN_N,
                             const Eigen::Vector3d& v_BN_N,
                             const Eigen::Vector3f& vehAccumDV,
                             uint32_t attTimeIndex,
                             uint32_t transTimeIndex,
                             uint32_t attIdx,
                             uint32_t rateIdx,
                             uint32_t posIdx,
                             uint32_t velIdx,
                             uint32_t dvIdx,
                             uint32_t sunIdx,
                             uint32_t attMsgCount,
                             uint32_t transMsgCount) {
    NavAggregateAlgorithm alg{};

    // Set up module
    alg.setAttTimeIdx(attTimeIndex);
    alg.setTransTimeIdx(transTimeIndex);
    alg.setAttIdx(attIdx);
    alg.setRateIdx(rateIdx);
    alg.setPosIdx(posIdx);
    alg.setVelIdx(velIdx);
    alg.setDvIdx(dvIdx);
    alg.setSunIdx(sunIdx);
    alg.setAttMsgCount(attMsgCount);
    alg.setTransMsgCount(transMsgCount);

    // Populate messages
    std::array<InputNavAttData, MAX_AGG_NAV_MSG> attInputs{};
    for (uint32_t i = 0U; i < MAX_AGG_NAV_MSG; ++i) {
        InputNavAttData attInputData{};
        if (i == attTimeIndex) {
            attInputData.timeTag = attTimeTag;
        }
        if (i == attIdx) {
            attInputData.sigma_BN = sigma_BN;
        }
        if (i == rateIdx) {
            attInputData.omega_BN_B = omega_BN_B;
        }
        if (i == sunIdx) {
            attInputData.vehSunPntBdy = vehSunPntBdy;
        }
        attInputs.at(i) = attInputData;
    }

    std::array<InputNavTransData, MAX_AGG_NAV_MSG> transInputs{};
    for (uint32_t i = 0U; i < MAX_AGG_NAV_MSG; ++i) {
        InputNavTransData transInputData{};
        if (i == transTimeIndex) {
            transInputData.timeTag = transTimeTag;
        }
        if (i == posIdx) {
            transInputData.r_BN_N = r_BN_N;
        }
        if (i == velIdx) {
            transInputData.v_BN_N = v_BN_N;
        }
        if (i == dvIdx) {
            transInputData.vehAccumDV = vehAccumDV;
        }
        transInputs.at(i) = transInputData;
    }

    // Reference
    AggregateOutput out{};
    EXPECT_NO_THROW(out = alg.update(attInputs, transInputs));

    for (uint32_t i = 0U; i < 3U; ++i) {
        // --- General tests ---

        // Finiteness
        EXPECT_TRUE(std::isfinite(out.navAttOut.sigma_BN[i]));
        EXPECT_TRUE(std::isfinite(out.navAttOut.omega_BN_B[i]));
        EXPECT_TRUE(std::isfinite(out.navAttOut.vehSunPntBdy[i]));
        EXPECT_TRUE(std::isfinite(out.navTransOut.r_BN_N[i]));
        EXPECT_TRUE(std::isfinite(out.navTransOut.v_BN_N[i]));
        EXPECT_TRUE(std::isfinite(out.navTransOut.vehAccumDV[i]));

        // --- Module specific tests ---

        // Because this module simply copies data from one of the input messages to the output message, the output of
        // the module should be equal to the corresponding input, as long as the message count is greater than zero
        if (attMsgCount > 0U) {
            EXPECT_EQ(out.navAttOut.sigma_BN[i], sigma_BN[i]);
            EXPECT_EQ(out.navAttOut.omega_BN_B[i], omega_BN_B[i]);
            EXPECT_EQ(out.navAttOut.vehSunPntBdy[i], vehSunPntBdy[i]);
        }
        if (transMsgCount > 0U) {
            EXPECT_EQ(out.navTransOut.r_BN_N[i], r_BN_N[i]);
            EXPECT_EQ(out.navTransOut.v_BN_N[i], v_BN_N[i]);
            EXPECT_EQ(out.navTransOut.vehAccumDV[i], vehAccumDV[i]);
        }
    }
    if (attMsgCount > 0U) {
        EXPECT_EQ(out.navAttOut.timeTag, attTimeTag);
    }
    if (transMsgCount > 0U) {
        EXPECT_EQ(out.navTransOut.timeTag, transTimeTag);
    }
}

#endif  // TEST_NAV_AGGREGATE_H
