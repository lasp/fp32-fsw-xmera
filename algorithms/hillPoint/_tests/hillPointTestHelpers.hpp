#ifndef TEST_HILL_POINT_HELPERS_H
#define TEST_HILL_POINT_HELPERS_H

#include "hillPointAlgorithm.h"
#include "hillPointTypes.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

struct ReferenceHillPointOutput {
    Eigen::Vector3d sigma_RN;
    Eigen::Vector3d omega_RN_N;
    Eigen::Vector3d domega_RN_N;
};

// Independent reference implementation kept in pure double precision, used to verify the
// algorithm's mixed-precision FP32 output to within float tolerance.
inline ReferenceHillPointOutput referenceHillPoint(const Eigen::Vector3d& r_BN_N,
                                                   const Eigen::Vector3d& v_BN_N,
                                                   const Eigen::Vector3d& r_PN_N,
                                                   const Eigen::Vector3d& v_PN_N) {
    const Eigen::Vector3d r_BP_N = r_BN_N - r_PN_N;
    const Eigen::Vector3d v_BP_N = v_BN_N - v_PN_N;
    const double r_norm = r_BP_N.stableNorm();
    const Eigen::Vector3d h = r_BP_N.cross(v_BP_N);

    ReferenceHillPointOutput out{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    if (r_norm > HillPointAlgorithm::kMinOrbitRadius && v_BP_N.squaredNorm() > 0.0) {
        const double dotProduct = r_BP_N.stableNormalized().dot(v_BP_N.stableNormalized());
        const double posVelSeparationAngle = safeAcos(std::abs(dotProduct));

        if (posVelSeparationAngle >= HillPointAlgorithm::kSmallAngleThreshold) {
            const Eigen::Vector3d i_r = r_BP_N.stableNormalized();
            const Eigen::Vector3d i_h = h.stableNormalized();
            const Eigen::Vector3d i_theta = i_h.cross(i_r);

            Eigen::Matrix3d dcm_RN;
            dcm_RN.row(0) = i_r;
            dcm_RN.row(1) = i_theta;
            dcm_RN.row(2) = i_h;

            const double dfdt = h.stableNorm() / (r_norm * r_norm);
            const double ddfdt2 = -2.0 * v_BP_N.dot(i_r) / r_norm * dfdt;

            const Eigen::Vector3d omega_RN_R{0.0, 0.0, dfdt};
            const Eigen::Vector3d domega_RN_R{0.0, 0.0, ddfdt2};

            out = {
                dcmToMrp(dcm_RN),
                dcm_RN.transpose() * omega_RN_R,
                dcm_RN.transpose() * domega_RN_R,
            };
        }
    }
    return out;
}

inline void testHillPoint(const Eigen::Vector3d& r_BN_N,
                          const Eigen::Vector3d& v_BN_N,
                          const Eigen::Vector3d& r_PN_N,
                          const Eigen::Vector3d& v_PN_N) {
    HillPointAlgorithm alg;

    HillPointOutput out;
    EXPECT_NO_THROW(out = alg.update(r_BN_N, v_BN_N, r_PN_N, v_PN_N));

    ReferenceHillPointOutput ref;
    EXPECT_NO_THROW(ref = referenceHillPoint(r_BN_N, v_BN_N, r_PN_N, v_PN_N));

    // Compare attitudes as DCMs rather than MRP components: dcmToMrp can return either MRP
    // shadow-set representative near |sigma| = 1 (the 180-deg boundary). The DCM is unique through 180 deg.
    constexpr float tol = 1e-5F;
    const Eigen::Matrix3f dcmOut = mrpToDcm(out.sigma_RN);
    const Eigen::Vector3f sigmaRefFloat = ref.sigma_RN.cast<float>();
    const Eigen::Matrix3f dcmRef = mrpToDcm(sigmaRefFloat);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(dcmOut(r, c), dcmRef(r, c), tol);
        }
    }

    // Use a combined absolute + relative tolerance. The absolute floor handles near-zero outputs,
    // while the relative term scales the allowed error with the expected magnitude: a fixed
    // absolute tolerance is unachievable for large-magnitude outputs because a single float32
    // ULP can exceed it.
    constexpr float absTol = 1e-5F;
    constexpr float relTol = 1e-5F;
    const auto tolFor = [&](float expectedVal) { return std::max(absTol, relTol * std::abs(expectedVal)); };
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.omega_RN_N[i], static_cast<float>(ref.omega_RN_N[i]), tolFor(ref.omega_RN_N[i]));
        EXPECT_NEAR(out.domega_RN_N[i], static_cast<float>(ref.domega_RN_N[i]), tolFor(ref.domega_RN_N[i]));
    }
}

// ---------------------------------------------------------------------------
// Property test helper functions
// ---------------------------------------------------------------------------

// All output components are finite for valid inputs.
inline void propertyOutputIsFinite(const Eigen::Vector3d& r_BN_N,
                                   const Eigen::Vector3d& v_BN_N,
                                   const Eigen::Vector3d& r_PN_N,
                                   const Eigen::Vector3d& v_PN_N) {
    HillPointAlgorithm alg;
    const HillPointOutput out = alg.update(r_BN_N, v_BN_N, r_PN_N, v_PN_N);

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
    }
}

inline void testHillPointSetup() {
    EXPECT_NO_THROW({
        const HillPointAlgorithm alg;
        (void)alg;
    });
}

#endif  // TEST_HILL_POINT_HELPERS_H
