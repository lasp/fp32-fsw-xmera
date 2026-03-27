#ifndef TEST_ATT_TRACKING_ERROR_H
#define TEST_ATT_TRACKING_ERROR_H

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "attTrackingErrorAlgorithm.h"
#include "attTrackingErrorTypes.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <numbers>
#include <vector>

// Create a struct for the double output
struct AttGuidOutputDouble {
    Eigen::Vector3d sigma_BR;
    Eigen::Vector3d omega_BR_B;
    Eigen::Vector3d omega_RN_B;
    Eigen::Vector3d domega_RN_B;
};

// Reference computation for update
inline AttGuidOutputDouble referenceUpdate(const AttNavInput& navIn, const AttRefInput& refIn) {
    AttGuidOutputDouble out{};

    // Convert float inputs to double
    Eigen::Vector3d sigma_BN_d = navIn.sigma_BN.cast<double>();
    Eigen::Vector3d omega_BN_B_d = navIn.omega_BN_B.cast<double>();
    Eigen::Vector3d sigma_RN_d = refIn.sigma_RN.cast<double>();
    Eigen::Vector3d omega_RN_N_d = refIn.omega_RN_N.cast<double>();
    Eigen::Vector3d domega_RN_N_d = refIn.domega_RN_N.cast<double>();

    // Compute attitude tracking error sigma_BR
    Eigen::Vector3d sigma_BR_d = subMrp(sigma_BN_d, sigma_RN_d);

    // Transform reference angular velocity from inertial to body frame
    const Eigen::Matrix3d dcm_BN_d = mrpToDcm(sigma_BN_d);
    Eigen::Vector3d omega_RN_B_d = dcm_BN_d * omega_RN_N_d;

    // Compute angular velocity tracking error omega_BR_B
    Eigen::Vector3d omega_BR_B_d = omega_BN_B_d - omega_RN_B_d;

    // Transform reference angular acceleration from inertial to body frame
    Eigen::Vector3d domega_RN_B_d = dcm_BN_d * domega_RN_N_d;

    // Get outputs in doubles
    out.sigma_BR = sigma_BR_d;
    out.omega_BR_B = omega_BR_B_d;
    out.omega_RN_B = omega_RN_B_d;
    out.domega_RN_B = domega_RN_B_d;

    return out;
}

inline void regressionTestAttTrackingError(const std::vector<float>& sigma_BN,
                                           const std::vector<float>& omega_BN_B,
                                           const std::vector<float>& sigma_RN,
                                           const std::vector<float>& omega_RN_N,
                                           const std::vector<float>& domega_RN_N) {
    // --- Regression test using expected update algorithm ---
    AttTrackingErrorAlgorithm alg{};

    AttNavInput navIn{};
    AttRefInput refIn{};

    navIn.sigma_BN = Eigen::Map<const Eigen::Vector3f>(sigma_BN.data());
    navIn.omega_BN_B = Eigen::Map<const Eigen::Vector3f>(omega_BN_B.data());
    refIn.sigma_RN = Eigen::Map<const Eigen::Vector3f>(sigma_RN.data());
    refIn.omega_RN_N = Eigen::Map<const Eigen::Vector3f>(omega_RN_N.data());
    refIn.domega_RN_N = Eigen::Map<const Eigen::Vector3f>(domega_RN_N.data());

    AttGuidOutput outputGuide{};
    AttGuidOutputDouble referenceGuide{};

    // Reference
    EXPECT_NO_THROW(outputGuide = alg.update(navIn, refIn));
    EXPECT_NO_THROW(referenceGuide = referenceUpdate(navIn, refIn));

    // Compare MRPs nominal and shadow set; since they are not unique
    Eigen::Vector3d sigma_out = outputGuide.sigma_BR.cast<double>();
    Eigen::Vector3d sigma_ref = referenceGuide.sigma_BR;
    Eigen::Vector3d sigma_ref_shadow = sigma_ref;

    if (sigma_ref.squaredNorm() > 1e-12) {
        sigma_ref_shadow = -sigma_ref / sigma_ref.squaredNorm();
    }

    double error_norm = (sigma_out - sigma_ref).norm();
    double error_shadow = (sigma_out - sigma_ref_shadow).norm();

    EXPECT_TRUE(error_norm < 1e-6 || error_shadow < 1e-6);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(outputGuide.omega_BR_B[i], referenceGuide.omega_BR_B[i], 1e-5);
        EXPECT_NEAR(outputGuide.omega_RN_B[i], referenceGuide.omega_RN_B[i], 1e-5);
        EXPECT_NEAR(outputGuide.domega_RN_B[i], referenceGuide.domega_RN_B[i], 1e-5);

        EXPECT_TRUE(std::isfinite(outputGuide.sigma_BR[i]));
        EXPECT_TRUE(std::isfinite(outputGuide.omega_BR_B[i]));
        EXPECT_TRUE(std::isfinite(outputGuide.omega_RN_B[i]));
        EXPECT_TRUE(std::isfinite(outputGuide.domega_RN_B[i]));
    }
}

#endif  // TEST_ATT_TRACKING_ERROR_H
