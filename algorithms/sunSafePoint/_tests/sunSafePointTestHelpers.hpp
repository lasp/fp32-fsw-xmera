#ifndef TEST_SUNSAFEPOINT_H
#define TEST_SUNSAFEPOINT_H

#include "sunSafePointAlgorithm.h"
#include "sunSafePointTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

// Reference computation that independently reimplements the sunSafePoint update logic
inline SunSafePointOutput referenceUpdate(const Eigen::Vector3f& vehSunPntBdy,
                                          const Eigen::Vector3f& omega_BN_B,
                                          float sunAxisSpinRate,
                                          const Eigen::Vector3f& sHatBdyCmd,
                                          const Eigen::Vector3f& omega_RN_B_cfg) {
    SunSafePointOutput output{};

    Eigen::Vector3f rHat_SB_B = vehSunPntBdy.stableNormalized();
    if (rHat_SB_B.stableNorm() > 0.0F) {
        // Compute sun angle error
        float cosAngle = sHatBdyCmd.dot(rHat_SB_B);
        cosAngle = std::clamp(cosAngle, -1.0f, 1.0f);
        float sunAngleErr = std::acos(cosAngle);

        Eigen::Vector3f e_hat{};
        constexpr float kSmallAngle = 1e-3F;
        if (static_cast<float>(M_PI) - sunAngleErr < kSmallAngle) {
            e_hat = sHatBdyCmd.unitOrthogonal();
        } else {
            e_hat = rHat_SB_B.cross(sHatBdyCmd);
        }
        Eigen::Vector3f sunMnvrVec = e_hat.stableNormalized();
        Eigen::Vector3f sigma_BR = std::tan(sunAngleErr * 0.25f) * sunMnvrVec;
        sigma_BR = mrpSwitch(sigma_BR);

        output.sigma_BR = sigma_BR;
        output.omega_RN_B = sunAxisSpinRate * rHat_SB_B;
    } else {
        output.sigma_BR = Eigen::Vector3f::Zero();
        output.omega_RN_B = omega_RN_B_cfg;
    }

    output.omega_BR_B = omega_BN_B - output.omega_RN_B;

    return output;
}

// ---------------------------------------------------------------------------
// Regression test helper function
// ---------------------------------------------------------------------------

inline void regressionTestSunSafePoint(std::vector<float> sunVector,
                                       std::vector<float> omega_BN_B_Vec,
                                       float sunAxisSpinRate,
                                       std::vector<float> sHatBdyCmdVec,
                                       std::vector<float> omega_RN_B_cfgVec) {
    // The setter requires a (near-)unit vector; normalize the fuzz-generated input first.
    Eigen::Vector3f sHatBdyCmd(sHatBdyCmdVec[0], sHatBdyCmdVec[1], sHatBdyCmdVec[2]);
    if (sHatBdyCmd.norm() < 1e-3f) {
        return;
    }
    Eigen::Vector3f normalizedSHat = sHatBdyCmd.normalized();

    Eigen::Vector3f sunVec(sunVector[0], sunVector[1], sunVector[2]);
    Eigen::Vector3f omega_BN_B(omega_BN_B_Vec[0], omega_BN_B_Vec[1], omega_BN_B_Vec[2]);
    Eigen::Vector3f omega_RN_B_cfg(omega_RN_B_cfgVec[0], omega_RN_B_cfgVec[1], omega_RN_B_cfgVec[2]);

    SunSafePointAlgorithm alg{};
    alg.setSunAxisSpinRate(sunAxisSpinRate);
    alg.setSHatBdyCmd(normalizedSHat);
    alg.setOmega_RN_B(omega_RN_B_cfg);

    Eigen::Vector3f algSHat = alg.getSHatBdyCmd();

    SunSafePointOutput output{};
    EXPECT_NO_THROW(output = alg.update(sunVec, omega_BN_B));

    auto reference = referenceUpdate(sunVec, omega_BN_B, sunAxisSpinRate, algSHat, omega_RN_B_cfg);

    // Compare MRPs nominal and shadow set
    Eigen::Vector3f sigmaOut = output.sigma_BR;
    Eigen::Vector3f sigmaRef = reference.sigma_BR;
    Eigen::Vector3f sigmaRefShadow = sigmaRef;

    if (sigmaRef.squaredNorm() > 1e-12F) {
        sigmaRefShadow = -sigmaRef / sigmaRef.squaredNorm();
    }

    float errorNorm = (sigmaOut - sigmaRef).norm();
    float errorShadow = (sigmaOut - sigmaRefShadow).norm();

    EXPECT_TRUE(errorNorm < 1e-5F || errorShadow < 1e-5F);

    Eigen::Vector3f sigmaCompared = sigmaRef;
    if (errorShadow < errorNorm) {
        sigmaCompared = sigmaRefShadow;
    }

    constexpr float tol = 1e-5F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(output.sigma_BR(i), sigmaCompared(i), tol);
        EXPECT_NEAR(output.omega_BR_B(i), reference.omega_BR_B(i), tol);
        EXPECT_NEAR(output.omega_RN_B(i), reference.omega_RN_B(i), tol);
        EXPECT_TRUE(std::isfinite(output.sigma_BR(i)));
        EXPECT_TRUE(std::isfinite(output.omega_BR_B(i)));
        EXPECT_TRUE(std::isfinite(output.omega_RN_B(i)));
    }
}

// ---------------------------------------------------------------------------
// Property test helper functions
// ---------------------------------------------------------------------------

// sigma_BR norm is bounded by 1 (inner MRP set) for any visible sun vector.
inline void propertySigmaBrNormBounded(std::vector<float> sunVector) {
    Eigen::Vector3f sunVec(sunVector[0], sunVector[1], sunVector[2]);

    SunSafePointAlgorithm alg{};
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});

    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    auto output = alg.update(sunVec, omega_BN_B);
    EXPECT_LE(output.sigma_BR.norm(), 1.0F + 1e-6F);
}

// omega_BR_B always equals omega_BN_B - omega_RN_B.
inline void propertyOmegaBrIdentity(std::vector<float> sunVector, std::vector<float> omega_BN_B_Vec) {
    Eigen::Vector3f sunVec(sunVector[0], sunVector[1], sunVector[2]);
    Eigen::Vector3f omega_BN_B(omega_BN_B_Vec[0], omega_BN_B_Vec[1], omega_BN_B_Vec[2]);

    SunSafePointAlgorithm alg{};
    alg.setSunAxisSpinRate(0.5F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.setOmega_RN_B(Eigen::Vector3f{0.1F, -0.2F, 0.3F});

    auto output = alg.update(sunVec, omega_BN_B);
    Eigen::Vector3f expected = omega_BN_B - output.omega_RN_B;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(output.omega_BR_B(i), expected(i), 1e-6F);
    }
}

// All output components are finite for valid inputs.
inline void propertyOutputIsFinite(std::vector<float> sunVector) {
    Eigen::Vector3f sunVec(sunVector[0], sunVector[1], sunVector[2]);

    SunSafePointAlgorithm alg{};
    alg.setSunAxisSpinRate(1.0F);
    alg.setSHatBdyCmd(Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    alg.setOmega_RN_B(Eigen::Vector3f{0.1F, 0.2F, 0.3F});

    Eigen::Vector3f omega_BN_B{5.0F, -3.0F, 1.0F};
    auto output = alg.update(sunVec, omega_BN_B);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(output.sigma_BR(i)));
        EXPECT_TRUE(std::isfinite(output.omega_BR_B(i)));
        EXPECT_TRUE(std::isfinite(output.omega_RN_B(i)));
    }
}

#endif  // TEST_SUNSAFEPOINT_H
