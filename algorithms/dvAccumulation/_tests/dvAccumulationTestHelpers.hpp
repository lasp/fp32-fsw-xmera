#ifndef TEST_DV_ACCUMULATION_HELPERS_H
#define TEST_DV_ACCUMULATION_HELPERS_H

#include "dvAccumulation/dvAccumulationAlgorithm.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

/*! @brief Reference algorithm state, mirroring DvAccumulationAlgorithm's private members. */
struct ReferenceState {
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};
    bool firstCall{true};
};

/*! @brief Reference reInitialize: zero the accumulator and restart the accumulation window. */
inline void referenceReInitialize(ReferenceState& s) {
    s.vehAccumDV_B = Eigen::Vector3f::Zero();
    s.firstCall = true;
}

/*! @brief Reference update: the first call starts the window; every later call integrates
 *         controlPeriod * accel. Returns the accumulator. */
inline Eigen::Vector3f referenceUpdate(ReferenceState& s, float controlPeriod, const Eigen::Vector3f& accel_B) {
    if (s.firstCall) {
        s.firstCall = false;
    } else {
        s.vehAccumDV_B += controlPeriod * accel_B;
    }

    return s.vehAccumDV_B;
}

/*! @brief Drive the algorithm through a sequence of acceleration samples at a fixed control period
 *         and compare to the reference at every step. */
inline void testDvAccumulation(float controlPeriod, const std::vector<Eigen::Vector3f>& accels) {
    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(controlPeriod)};

    ReferenceState ref{};
    referenceReInitialize(ref);

    for (const Eigen::Vector3f& accel_B : accels) {
        Eigen::Vector3f algOut = Eigen::Vector3f::Zero();
        EXPECT_NO_THROW(algOut = alg.update(accel_B));
        const Eigen::Vector3f refOut = referenceUpdate(ref, controlPeriod, accel_B);

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut[i], refOut[i], 1e-6F);
            EXPECT_TRUE(std::isfinite(algOut[i]));
        }
    }
}

/*! @brief Fuzz-friendly driver: drive the algorithm through a sequence of acceleration samples at a
 *         generated control period and compare to the reference step-by-step. */
inline void testDvAccumulationFuzz(float controlPeriod, const std::vector<Eigen::Vector3f>& accels) {
    if (!DvAccumulationConfig::isValidControlPeriod(controlPeriod)) {
        return;  // fuzz domain may produce a rejected control period; construction would throw
    }

    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(controlPeriod)};
    ReferenceState ref{};
    referenceReInitialize(ref);

    for (const Eigen::Vector3f& accel_B : accels) {
        Eigen::Vector3f algOut = Eigen::Vector3f::Zero();
        EXPECT_NO_THROW(algOut = alg.update(accel_B));
        const Eigen::Vector3f refOut = referenceUpdate(ref, controlPeriod, accel_B);

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut[i], refOut[i], 1e-5F);
            EXPECT_TRUE(std::isfinite(algOut[i]));
        }
    }
}

/*! @brief Construction exercise: a valid configuration constructs without throwing. */
inline void testDvAccumulationSetup() {
    EXPECT_NO_THROW({
        const DvAccumulationAlgorithm alg{DvAccumulationConfig::create(0.2F)};
        (void)alg;
    });
}

#endif  // TEST_DV_ACCUMULATION_HELPERS_H
