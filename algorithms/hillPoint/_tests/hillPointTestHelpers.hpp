#ifndef TEST_HILL_POINT_HELPERS_H
#define TEST_HILL_POINT_HELPERS_H

#include "hillPointAlgorithm.h"
#include "hillPointTypes.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
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
    const Eigen::Vector3d i_r = r_BP_N.normalized();
    const Eigen::Vector3d h = r_BP_N.cross(v_BP_N);
    const Eigen::Vector3d i_h = h.normalized();
    const Eigen::Vector3d i_theta = i_h.cross(i_r);

    Eigen::Matrix3d dcm_RN;
    dcm_RN.row(0) = i_r;
    dcm_RN.row(1) = i_theta;
    dcm_RN.row(2) = i_h;

    const double r_norm = r_BP_N.norm();
    double dfdt = 0.0;
    double ddfdt2 = 0.0;
    if (r_norm > 1.0) {
        dfdt = h.norm() / (r_norm * r_norm);
        ddfdt2 = -2.0 * v_BP_N.dot(i_r) / r_norm * dfdt;
    }

    const Eigen::Vector3d omega_RN_R{0.0, 0.0, dfdt};
    const Eigen::Vector3d domega_RN_R{0.0, 0.0, ddfdt2};

    return {
        dcmToMrp(dcm_RN),
        dcm_RN.transpose() * omega_RN_R,
        dcm_RN.transpose() * domega_RN_R,
    };
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

    // dcmToMrp can pick either MRP shadow-set representative when |sigma| is near 1 (180-deg
    // rotation boundary). Pick whichever representative is closer to the algorithm output before
    // the per-component comparison.
    const Eigen::Vector3d sigma_out = out.sigma_RN.cast<double>();
    Eigen::Vector3d sigma_ref = ref.sigma_RN;
    if (sigma_ref.squaredNorm() > 1e-12) {
        const Eigen::Vector3d sigma_ref_shadow = -sigma_ref / sigma_ref.squaredNorm();
        if ((sigma_out - sigma_ref_shadow).squaredNorm() < (sigma_out - sigma_ref).squaredNorm()) {
            sigma_ref = sigma_ref_shadow;
        }
    }

    constexpr float tol = 1e-5F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.sigma_RN[i], static_cast<float>(sigma_ref[i]), tol);
        EXPECT_NEAR(out.omega_RN_N[i], static_cast<float>(ref.omega_RN_N[i]), tol);
        EXPECT_NEAR(out.domega_RN_N[i], static_cast<float>(ref.domega_RN_N[i]), tol);

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
