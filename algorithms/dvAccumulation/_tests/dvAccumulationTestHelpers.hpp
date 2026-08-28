#ifndef TEST_DV_ACCUMULATION_HELPERS_H
#define TEST_DV_ACCUMULATION_HELPERS_H

#include "dvAccumulation/dvAccumulationAlgorithm.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

/*! @brief Reference algorithm state. It is the same as the private members of
 *         DvAccumulationAlgorithm. */
struct ReferenceState {
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};
    bool firstCall{true};
};

/*! @brief Reference reInitialize(). It sets the accumulator to zero and starts a new accumulation
 *         window. */
inline void referenceReInitialize(ReferenceState& s) {
    s.vehAccumDV_B = Eigen::Vector3f::Zero();
    s.firstCall = true;
}

/*! @brief Reference oracle for DvAccumulationAlgorithm::update(). It gives the accumulator. */
inline Eigen::Vector3f referenceUpdate(ReferenceState& s,
                                       float controlPeriod,
                                       const Eigen::Vector3f& accel_B,
                                       const Eigen::Vector3f& accelBias_B) {
    if (s.firstCall) {
        s.firstCall = false;
    } else {
        s.vehAccumDV_B += controlPeriod * (accel_B - accelBias_B);
    }

    return s.vehAccumDV_B;
}

/*! @brief Uses the algorithm on a sequence of acceleration samples at a constant control period.
 *         It compares each step to the reference. */
inline void testDvAccumulation(float controlPeriod, const std::vector<Eigen::Vector3f>& accels) {
    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(controlPeriod)};

    ReferenceState ref{};
    referenceReInitialize(ref);

    for (const Eigen::Vector3f& accel_B : accels) {
        Eigen::Vector3f algOut = Eigen::Vector3f::Zero();
        EXPECT_NO_THROW(algOut = alg.update(accel_B, Eigen::Vector3f::Zero()));
        const Eigen::Vector3f refOut = referenceUpdate(ref, controlPeriod, accel_B, Eigen::Vector3f::Zero());

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut[i], refOut[i], 1e-6F);
            EXPECT_TRUE(std::isfinite(algOut[i]));
        }
    }
}

/*! @brief testDvAccumulation for a fuzz test. It ignores a control period that the validator
 *         rejects, it subtracts a generated bias, and it uses a larger tolerance. */
inline void testDvAccumulationFuzz(float controlPeriod,
                                   const std::vector<Eigen::Vector3f>& accels,
                                   const Eigen::Vector3f& accelBias_B) {
    if (!DvAccumulationConfig::isValidControlPeriod(controlPeriod)) {
        return;  // the fuzz domain can give a rejected control period. Construction throws on one
    }

    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(controlPeriod)};
    ReferenceState ref{};
    referenceReInitialize(ref);

    for (const Eigen::Vector3f& accel_B : accels) {
        Eigen::Vector3f algOut = Eigen::Vector3f::Zero();
        EXPECT_NO_THROW(algOut = alg.update(accel_B, accelBias_B));
        const Eigen::Vector3f refOut = referenceUpdate(ref, controlPeriod, accel_B, accelBias_B);

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut[i], refOut[i], 1e-5F);
            EXPECT_TRUE(std::isfinite(algOut[i]));
        }
    }
}

/*! @brief Construction test: a valid configuration constructs and does not throw. */
inline void testDvAccumulationSetup() {
    EXPECT_NO_THROW({
        const DvAccumulationAlgorithm alg{DvAccumulationConfig::create(0.2F)};
        (void)alg;
    });
}

#endif  // TEST_DV_ACCUMULATION_HELPERS_H
