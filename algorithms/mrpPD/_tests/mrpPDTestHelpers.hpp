// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef TEST_MRPPD_H
#define TEST_MRPPD_H

#include "architecture/utilities/eigenSupport.h"
#include "mrpPDAlgorithm.h"
#include "mrpPDTypes.h"
#include "utilities/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <numbers>

inline Eigen::Vector3f referenceUpdate(const MrpPDConfig& cfg,
                                       const Eigen::Vector3f& sigma_BR,
                                       const Eigen::Vector3f& omega_BR_B,
                                       const Eigen::Vector3f& domega_RN_B) {
    return -cfg.getProportionalGainK() * sigma_BR - cfg.getDerivativeGainP() * omega_BR_B +
           cfg.getSpacecraftInertia() * domega_RN_B - cfg.getKnownTorquePntB_B();
}

inline void regressionTestMrpPD(float K,
                                float P,
                                const Eigen::Vector3f& torque,
                                const Eigen::Vector3f& sigma_BR,
                                const Eigen::Vector3f& omega_BR_B,
                                const Eigen::Vector3f& domega_RN_B) {
    const Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
    const MrpPDConfig cfg = MrpPDConfig::create(K, P, torque, inertia);
    const MrpPDAlgorithm alg{cfg};

    Eigen::Vector3f outputTorque = Eigen::Vector3f::Zero();
    Eigen::Vector3f referenceTorque = Eigen::Vector3f::Zero();
    EXPECT_NO_THROW(outputTorque = alg.update(sigma_BR, omega_BR_B, domega_RN_B));
    EXPECT_NO_THROW(referenceTorque = referenceUpdate(cfg, sigma_BR, omega_BR_B, domega_RN_B));

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(outputTorque[i], referenceTorque[i], 1e-6);
        EXPECT_TRUE(std::isfinite(outputTorque[i]));
    }
}

#endif  // TEST_MRPPD_H
