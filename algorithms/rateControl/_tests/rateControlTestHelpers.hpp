#ifndef TEST_RATECONTROL_H
#define TEST_RATECONTROL_H

#include "../utilities/_tests/utilitiesHelpers.hpp"
#include "architecture/utilities/eigenSupport.h"
#include "rateControlAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <architecture/utilities/macroDefinitions.h>
#include <gtest/gtest.h>

Eigen::Vector3d referenceUpdate(const Eigen::Matrix3d& spacecraftInertia,
                                const double derivativeGainP,
                                const Eigen::Vector3d& knownTorquePntB_B,
                                const Eigen::Vector3d omega_BR_B,
                                const Eigen::Vector3d domega_RN_B) {
    const auto ISCPntB_B = spacecraftInertia;
    const Eigen::Vector3d Lr = -derivativeGainP * omega_BR_B + ISCPntB_B * domega_RN_B - knownTorquePntB_B;  // [Nm]
    return Lr;
}

inline void regressionTestRateControl(const Eigen::Matrix3f& spacecraftInertia_f,
                                      const float derivativeGainP_f,
                                      const Eigen::Vector3f& knownTorquePntB_B_f,
                                      const Eigen::Vector3f& omega_BR_B_f,
                                      const Eigen::Vector3f& domega_RN_B_f) {
    // Run algorithm in float.
    RateControlAlgorithm alg;
    alg.setSpacecraftInertia(spacecraftInertia_f);
    alg.setDerivativeGainP(derivativeGainP_f);
    alg.setKnownTorquePntB_B(knownTorquePntB_B_f);
    const Eigen::Vector3f out_alg = alg.update(omega_BR_B_f, domega_RN_B_f);

    // Run reference in double on the exact same float-quantized inputs.
    const Eigen::Matrix3d spacecraftInertia_d = spacecraftInertia_f.cast<double>();
    const double derivativeGainP_d = static_cast<double>(derivativeGainP_f);
    const Eigen::Vector3d knownTorquePntB_B_d = knownTorquePntB_B_f.cast<double>();
    const Eigen::Vector3d omega_BR_B_d = omega_BR_B_f.cast<double>();
    const Eigen::Vector3d domega_RN_B_d = domega_RN_B_f.cast<double>();

    const Eigen::Vector3d out_ref_d =
        referenceUpdate(spacecraftInertia_d, derivativeGainP_d, knownTorquePntB_B_d, omega_BR_B_d, domega_RN_B_d);

    for (int i = 0; i < 3; i++) {
        const float sum_abs_terms =
            std::abs(derivativeGainP_f * omega_BR_B_f[i]) + std::abs(spacecraftInertia_f(i, 0) * domega_RN_B_f[0]) +
            std::abs(spacecraftInertia_f(i, 1) * domega_RN_B_f[1]) +
            std::abs(spacecraftInertia_f(i, 2) * domega_RN_B_f[2]) + std::abs(knownTorquePntB_B_f[i]);

        const float tol =
            8.0f * sum_abs_terms * std::numeric_limits<float>::epsilon() + std::numeric_limits<float>::min();

        EXPECT_NEAR(out_alg[i], static_cast<float>(out_ref_d[i]), tol);
    }
}

#endif
