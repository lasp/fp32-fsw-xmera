#ifndef TEST_MRPPD_H
#define TEST_MRPPD_H

#include "mrpPDAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>

// Reference computation for update, derived directly from the configuration values.
inline Eigen::Vector3f referenceUpdate(const MrpPDConfig& cfg,
                                       const Eigen::Vector3f& sigma_BR,
                                       const Eigen::Vector3f& omega_BR_B,
                                       const Eigen::Vector3f& domega_RN_B) {
    const Eigen::Vector3f Lr = -cfg.getProportionalGainK() * sigma_BR - cfg.getDerivativeGainP() * omega_BR_B +
                               cfg.getInertia() * domega_RN_B - cfg.getKnownTorquePntB_B();
    return Lr;
}

inline void regressionTestMrpPD(float K,
                                float P,
                                const Eigen::Vector3f& torque,
                                const Eigen::Vector3f& sigma_BR,
                                const Eigen::Vector3f& omega_BR_B,
                                const Eigen::Vector3f& domega_RN_B) {
    // Identity inertia (the default vehicle configuration used by these regression scenarios).
    const MrpPDConfig cfg = MrpPDConfig::create(K, P, torque, Eigen::Matrix3f::Identity());
    MrpPDAlgorithm alg{cfg};

    Eigen::Vector3f outputTorque = Eigen::Vector3f::Zero();
    EXPECT_NO_THROW(outputTorque = alg.update(sigma_BR, omega_BR_B, domega_RN_B));
    const Eigen::Vector3f referenceTorque = referenceUpdate(cfg, sigma_BR, omega_BR_B, domega_RN_B);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(outputTorque[i], referenceTorque[i], 1e-6);
        EXPECT_TRUE(std::isfinite(outputTorque[i]));
    }
}

#endif  // TEST_MRPPD_H
