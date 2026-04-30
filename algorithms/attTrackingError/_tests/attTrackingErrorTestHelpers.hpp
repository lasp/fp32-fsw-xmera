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

// Create a struct for the double output
struct AttGuidOutputDouble {
    Eigen::Vector3d sigma_BR;
    Eigen::Vector3d omega_BR_B;
    Eigen::Vector3d omega_RN_B;
    Eigen::Vector3d domega_RN_B;
};

// Reference computation for update
inline AttGuidOutputDouble referenceUpdate(const Eigen::Vector3d& sigma_BN,
                                           const Eigen::Vector3d& omega_BN_B,
                                           const Eigen::Vector3d& sigma_RN,
                                           const Eigen::Vector3d& omega_RN_N,
                                           const Eigen::Vector3d& domega_RN_N) {
    AttGuidOutputDouble out{};

    // Compute attitude tracking error sigma_BR
    out.sigma_BR = subMrp(sigma_BN, sigma_RN);

    // Transform reference angular velocity from inertial to body frame
    const Eigen::Matrix3d dcm_BN = mrpToDcm(sigma_BN);
    out.omega_RN_B = dcm_BN * omega_RN_N;

    // Compute angular velocity tracking error omega_BR_B
    out.omega_BR_B = omega_BN_B - out.omega_RN_B;

    // Transform reference angular acceleration from inertial to body frame
    out.domega_RN_B = dcm_BN * domega_RN_N;

    return out;
}

// -----------------------------------------
// Regression Test
// -----------------------------------------
// The regression test compares the algorithm float outputs against double precision reference implementation
// for a given set of input vectors.

inline void regressionTestAttTrackingError(const Eigen::Vector3f& sigma_BN,
                                           const Eigen::Vector3f& omega_BN_B,
                                           const Eigen::Vector3f& sigma_RN,
                                           const Eigen::Vector3f& omega_RN_N,
                                           const Eigen::Vector3f& domega_RN_N) {
    // --- Regression test using expected update algorithm ---
    AttTrackingErrorAlgorithm alg{};

    AttNavInput navIn{};
    AttRefInput refIn{};

    // Populate algorithm inputs
    navIn.sigma_BN = sigma_BN;
    navIn.omega_BN_B = omega_BN_B;
    refIn.sigma_RN = sigma_RN;
    refIn.omega_RN_N = omega_RN_N;
    refIn.domega_RN_N = domega_RN_N;

    AttGuidOutput outputGuide{};
    AttGuidOutputDouble referenceGuide{};

    // Compute algorithm output and double-precision reference
    outputGuide = alg.update(navIn, refIn);
    referenceGuide = referenceUpdate(sigma_BN.cast<double>(),
                                     omega_BN_B.cast<double>(),
                                     sigma_RN.cast<double>(),
                                     omega_RN_N.cast<double>(),
                                     domega_RN_N.cast<double>());

    // Compare MRPs using nominal and shadow representations
    Eigen::Vector3d sigma_out = outputGuide.sigma_BR.cast<double>();
    Eigen::Vector3d sigma_ref = referenceGuide.sigma_BR;
    Eigen::Vector3d sigma_ref_shadow = sigma_ref;

    if (sigma_ref.squaredNorm() > 1e-12) {
        sigma_ref_shadow = -sigma_ref / sigma_ref.squaredNorm();
    }

    double error_norm = (sigma_out - sigma_ref).norm();
    double error_shadow = (sigma_out - sigma_ref_shadow).norm();

    EXPECT_TRUE(error_norm < 1e-6 || error_shadow < 1e-6);

    Eigen::Vector3d sigma_compared = sigma_ref;
    if (error_shadow < error_norm) {
        sigma_compared = sigma_ref_shadow;
    }

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(sigma_out[i], sigma_compared[i], 1e-6);
        EXPECT_NEAR(outputGuide.omega_BR_B[i], referenceGuide.omega_BR_B[i], 1e-5);
        EXPECT_NEAR(outputGuide.omega_RN_B[i], referenceGuide.omega_RN_B[i], 1e-5);
        EXPECT_NEAR(outputGuide.domega_RN_B[i], referenceGuide.domega_RN_B[i], 1e-5);
    }
}

// -----------------------------------------
// Zero Property Test
// -----------------------------------------
// The zero property test verifies that when the current attitude equals the reference attitude and all
// angular rates are zero, the attitude and angular velocity errors are also zero.

inline void zeroPropertyTest(const Eigen::Vector3f& sigma_BN) {
    AttTrackingErrorAlgorithm alg{};

    AttNavInput navIn{};
    AttRefInput refIn{};

    navIn.sigma_BN = sigma_BN;
    refIn.sigma_RN = navIn.sigma_BN;

    navIn.omega_BN_B << 0.0, 0.0, 0.0;
    refIn.omega_RN_N << 0.0, 0.0, 0.0;
    refIn.domega_RN_N << 0.0, 0.0, 0.0;

    AttGuidOutput out = alg.update(navIn, refIn);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(out.sigma_BR[i], 0.0, 1e-6);
        EXPECT_NEAR(out.omega_BR_B[i], 0.0, 1e-6);
    }
}

// -----------------------------------------
// Identity Property Test
// -----------------------------------------
// The identity property test verifies that when sigma_BN = 0, its DCM representation [BN] is an identity matrix.
// So, transforming reference angular velocity and acceleration to the body frame should yield identical values.

inline void identityPropertyTest(const Eigen::Vector3f& omega_RN_N, const Eigen::Vector3f& domega_RN_N) {
    AttTrackingErrorAlgorithm alg{};

    AttNavInput navIn{};
    AttRefInput refIn{};

    refIn.omega_RN_N = omega_RN_N;
    refIn.domega_RN_N = domega_RN_N;

    navIn.sigma_BN << 0.0, 0.0, 0.0;

    AttGuidOutput out = alg.update(navIn, refIn);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(out.omega_RN_B[i], refIn.omega_RN_N[i], 1e-6);
        EXPECT_NEAR(out.domega_RN_B[i], refIn.domega_RN_N[i], 1e-6);
    }
}

// -----------------------------------------
// Finiteness Property Test
// -----------------------------------------
// The finiteness property test verifies that for any input, all output parameters remain finite.

inline void finitenessPropertyTest(const Eigen::Vector3f& sigma_BN,
                                   const Eigen::Vector3f& omega_BN_B,
                                   const Eigen::Vector3f& sigma_RN,
                                   const Eigen::Vector3f& omega_RN_N,
                                   const Eigen::Vector3f& domega_RN_N) {
    AttTrackingErrorAlgorithm alg{};

    AttNavInput navIn{};
    AttRefInput refIn{};

    navIn.sigma_BN = sigma_BN;
    navIn.omega_BN_B = omega_BN_B;
    refIn.sigma_RN = sigma_RN;
    refIn.omega_RN_N = omega_RN_N;
    refIn.domega_RN_N = domega_RN_N;

    AttGuidOutput out = alg.update(navIn, refIn);

    for (int i = 0; i < 3; i++) {
        EXPECT_TRUE(std::isfinite(out.sigma_BR[i]));
        EXPECT_TRUE(std::isfinite(out.omega_BR_B[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_B[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_B[i]));
    }
}
#endif  // TEST_ATT_TRACKING_ERROR_H
