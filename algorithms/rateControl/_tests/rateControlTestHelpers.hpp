#ifndef TEST_RATECONTROL_H
#define TEST_RATECONTROL_H

#include "../freestandingInvalidArgument.h"
#include "../utilities/_tests/utilitiesHelpers.hpp"
#include "architecture/utilities/eigenSupport.h"
#include "rateControlAlgorithm.h"
#include <architecture/utilities/macroDefinitions.h>
#include <gtest/gtest.h>

Eigen::Vector3f referenceUpdate(const Eigen::Matrix3f& spacecraftInertia,
                                const float derivativeGainP,
                                const Eigen::Vector3f& knownTorquePntB_B,
                                const Eigen::Vector3f omega_BR_B,
                                const Eigen::Vector3f domega_RN_B) {
    const auto ISCPntB_B = spacecraftInertia;
    const Eigen::Vector3f Lr = -derivativeGainP * omega_BR_B + ISCPntB_B * domega_RN_B - knownTorquePntB_B;  // [Nm]
    return Lr;
}

inline void regressionTestRateControl(const Eigen::Matrix3f& spacecraftInertia,
                                      const float derivativeGainP,
                                      const Eigen::Vector3f& knownTorquePntB_B,
                                      const Eigen::Vector3f& omega_BR_B,
                                      const Eigen::Vector3f& domega_RN_B) {
    RateControlAlgorithm alg;
    alg.setSpacecraftInertia(spacecraftInertia);
    alg.setDerivativeGainP(derivativeGainP);
    alg.setKnownTorquePntB_B(knownTorquePntB_B);
    const Eigen::Vector3f out_alg = alg.update(omega_BR_B, domega_RN_B);
    const Eigen::Vector3f out_ref =
        referenceUpdate(spacecraftInertia, derivativeGainP, knownTorquePntB_B, omega_BR_B, domega_RN_B);
    EXPECT_EQ(out_alg, out_ref);
}

#endif
