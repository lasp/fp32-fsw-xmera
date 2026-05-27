#ifndef TEST_HILL_POINT_HELPERS_H
#define TEST_HILL_POINT_HELPERS_H

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "hillPointAlgorithm.h"
#include "hillPointTypes.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
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
                                                   const Eigen::Vector3d& r_planet_N,
                                                   const Eigen::Vector3d& v_planet_N) {
    const Eigen::Vector3d rel_r = r_BN_N - r_planet_N;
    const Eigen::Vector3d rel_v = v_BN_N - v_planet_N;
    const Eigen::Vector3d i_r = rel_r.normalized();
    const Eigen::Vector3d h = rel_r.cross(rel_v);
    const Eigen::Vector3d i_h = h.normalized();
    const Eigen::Vector3d i_theta = i_h.cross(i_r);

    Eigen::Matrix3d dcm_RN;
    dcm_RN.row(0) = i_r;
    dcm_RN.row(1) = i_theta;
    dcm_RN.row(2) = i_h;

    const double r_norm = rel_r.norm();
    double dfdt = 0.0;
    double ddfdt2 = 0.0;
    if (r_norm > 1.0) {
        dfdt = h.norm() / (r_norm * r_norm);
        ddfdt2 = -2.0 * rel_v.dot(i_r) / r_norm * dfdt;
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
                          const Eigen::Vector3d& r_planet_N,
                          const Eigen::Vector3d& v_planet_N) {
    HillPointAlgorithm alg(HillPointConfig::create());

    HillPointOutput out;
    EXPECT_NO_THROW(out = alg.update(r_BN_N, v_BN_N, r_planet_N, v_planet_N));

    ReferenceHillPointOutput ref;
    EXPECT_NO_THROW(ref = referenceHillPoint(r_BN_N, v_BN_N, r_planet_N, v_planet_N));

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
        const HillPointConfig cfg = HillPointConfig::create();
        const HillPointAlgorithm alg(cfg);
        (void)alg;
    });
}

#endif  // TEST_HILL_POINT_HELPERS_H
