#ifndef TEST_RATECONTROL_H
#define TEST_RATECONTROL_H

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "rateControlAlgorithm.h"
#include <architecture/utilities/macroDefinitions.h>
#include <gtest/gtest.h>

Eigen::Vector3f referenceUpdate(const Eigen::Matrix3f& spacecraftInertia, const InputGuidanceData& attGuidIn, const RateControlAlgorithm& alg){
    // Compute required attitude control torque vector
    const Eigen::Vector3f omega_BR_B = attGuidIn.omega_BR_B;
    const Eigen::Vector3f omega_RN_B = attGuidIn.omega_RN_B;
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;
    const Eigen::Vector3f domega_RN_B = attGuidIn.domega_RN_B;
    const auto P = alg.getDerivativeGainP();
    const auto ISCPntB_B = spacecraftInertia;
    const auto knownTorquePntB_B = alg.getKnownTorquePntB_B();
    const Eigen::Vector3f Lr = -P * omega_BR_B + omega_RN_B.cross(ISCPntB_B * omega_BN_B) +
                               ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                               knownTorquePntB_B;  // [Nm]
    return Lr;
}

inline void regressionTestrateControl(
        const Eigen::Matrix3f& spacecraftInertia,
        const float DrivativeGainP,
        const Eigen::Vector3f& knownTorquePntB_B,
        const Eigen::Vector3f& omega_BR_B,
        const Eigen::Vector3f& omega_RN_B,
        const Eigen::Vector3f& domega_RN_B){
    RateControlAlgorithm alg;

    alg.setSpacecraftInertia(spacecraftInertia);
    alg.setDerivativeGainP(DrivativeGainP);
    alg.setKnownTorquePntB_B(knownTorquePntB_B);

    InputGuidanceData in{};
    in.omega_BR_B = omega_BR_B;
    in.omega_RN_B = omega_RN_B;
    in.domega_RN_B = domega_RN_B;
    const Eigen::Vector3f out_alg = alg.update(in);
    const Eigen::Vector3f out_ref = referenceUpdate(spacecraftInertia, in, alg);
    EXPECT_EQ(out_alg, out_ref);
}

static Eigen::Matrix3f makeSPDInertia(const std::array<float,6>& p) {
    // Build a symmetric matrix then make it SPD-ish by adding diag > 0
    // p = [a00,a01,a02,a11,a12,a22]
    Eigen::Matrix3f M;
    M << p[0], p[1], p[2],
         p[1], p[3], p[4],
         p[2], p[4], p[5];
    M.diagonal().array() += 5.0f;  // ensure positive diagonal / avoid singular
    return M;
}

inline void fuzzAdapter_rateControl(const std::array<float,6>& inertiaParams,
                                   float DrivativeGainP,
                                   const std::array<float,3>& knownTorque_a,
                                   const std::array<float,3>& omega_BR_a,
                                   const std::array<float,3>& omega_RN_a,
                                   const std::array<float,3>& domega_RN_a)
{
    const Eigen::Matrix3f spacecraftInertia = makeSPDInertia(inertiaParams);
    const Eigen::Vector3f knownTorquePntB_B(knownTorque_a[0], knownTorque_a[1], knownTorque_a[2]);
    const Eigen::Vector3f omega_BR_B(omega_BR_a[0], omega_BR_a[1], omega_BR_a[2]);
    const Eigen::Vector3f omega_RN_B(omega_RN_a[0], omega_RN_a[1], omega_RN_a[2]);
    const Eigen::Vector3f domega_RN_B(domega_RN_a[0], domega_RN_a[1], domega_RN_a[2]);

    // Optional: drop NaN/Inf cases early
    if (!spacecraftInertia.allFinite() ||
        !knownTorquePntB_B.allFinite() ||
        !omega_BR_B.allFinite() || !omega_RN_B.allFinite() || !domega_RN_B.allFinite()) {
        return;
    }

    regressionTestrateControl(spacecraftInertia,
                              DrivativeGainP,
                              knownTorquePntB_B,
                              omega_BR_B,
                              omega_RN_B,
                              domega_RN_B);
}

#endif