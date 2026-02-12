#ifndef TEST_EPHEM_NAV_CONVERTER_H
#define TEST_EPHEM_NAV_CONVERTER_H

#include "../freestandingInvalidArgument.h"
#include "ephemNavConverterAlgorithm.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

// Reference computation for update
OutputNavTransData referenceUpdate(const EphemNavConverterAlgorithm& alg, const InputEphemerisData& ephemerisInput) {
    OutputNavTransData navTransOutput{};

    // Map timeTag, position and velocity vector to translational navigation output struct
    navTransOutput.timeTag = ephemerisInput.timeTag;
    navTransOutput.r_BN_N = ephemerisInput.r_BdyZero_N;
    navTransOutput.v_BN_N = ephemerisInput.v_BdyZero_N;

    return navTransOutput;
}

inline void testEphemNavConverter(double timeTag, std::vector<double> r_BdyZero_N, std::vector<double> v_BdyZero_N) {
    EphemNavConverterAlgorithm alg{};

    // Populate algorithm input struct
    InputEphemerisData ephemerisInput{};
    ephemerisInput.timeTag = timeTag;
    ephemerisInput.r_BdyZero_N = Eigen::Map<Eigen::Vector3d>(r_BdyZero_N.data());
    ephemerisInput.v_BdyZero_N = Eigen::Map<Eigen::Vector3d>(v_BdyZero_N.data());

    // Reference
    OutputNavTransData out{};
    OutputNavTransData ref{};
    EXPECT_NO_THROW(out = alg.update(ephemerisInput));
    EXPECT_NO_THROW(ref = referenceUpdate(alg, ephemerisInput));

    for (uint32_t i = 0U; i < 3U; ++i) {
        // --- General tests ---

        // Reference correctness
        EXPECT_NEAR(out.r_BN_N[i], ref.r_BN_N[i], 1e-6);
        EXPECT_NEAR(out.v_BN_N[i], ref.v_BN_N[i], 1e-6);

        // Finiteness
        EXPECT_TRUE(std::isfinite(out.r_BN_N[i]));
        EXPECT_TRUE(std::isfinite(out.v_BN_N[i]));

        // --- Module specific tests ---

        // Because this module simply copies data from one message to the other, the output of the module should be
        // equal to the input
        EXPECT_EQ(out.r_BN_N[i], r_BdyZero_N[i]);
        EXPECT_EQ(out.v_BN_N[i], v_BdyZero_N[i]);
    }
    EXPECT_NEAR(out.timeTag, ref.timeTag, 1e-6);
    EXPECT_EQ(out.timeTag, timeTag);
}

#endif  // TEST_EPHEM_NAV_CONVERTER_H
