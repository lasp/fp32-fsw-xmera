#ifndef TEST_CELESTIAL_TWO_BODY_POINT_HELPERS_H
#define TEST_CELESTIAL_TWO_BODY_POINT_HELPERS_H

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "celestialTwoBodyPointAlgorithm.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>

struct ReferenceCelestialTwoBodyPointOutput {
    Eigen::Vector3d sigma_RN;
    Eigen::Vector3d omega_RN_N;
    Eigen::Vector3d domega_RN_N;
};

// Independent double-precision reference for the rate/acceleration kernel, mirroring the original
// Xmera math. Used to verify the algorithm's mixed-precision FP32 output to within float tolerance.
inline ReferenceCelestialTwoBodyPointOutput referenceRateAndAccelCalc(const Eigen::Vector3d& r_PB_N,
                                                                      const Eigen::Vector3d& v_PB_N,
                                                                      const Eigen::Vector3d& r_SB_N,
                                                                      const Eigen::Vector3d& v_SB_N) {
    const Eigen::Vector3d R_n = r_PB_N.cross(r_SB_N);
    const Eigen::Vector3d v_n = v_PB_N.cross(r_SB_N) + r_PB_N.cross(v_SB_N);
    const Eigen::Vector3d a_n = 2.0 * v_PB_N.cross(v_SB_N);

    const Eigen::Vector3d r1_hat = r_PB_N.normalized();
    const Eigen::Vector3d r3_hat = R_n.normalized();
    const Eigen::Vector3d r2_hat = (r3_hat.cross(r1_hat)).normalized();
    Eigen::Matrix3d dcm_RN;
    dcm_RN.row(0) = r1_hat;
    dcm_RN.row(1) = r2_hat;
    dcm_RN.row(2) = r3_hat;

    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d dr1_hat = (identity - r1_hat * r1_hat.transpose()) * (v_PB_N / r_PB_N.norm());
    const Eigen::Vector3d dr3_hat = (identity - r3_hat * r3_hat.transpose()) * (v_n / R_n.norm());
    const Eigen::Vector3d dr2_hat = dr3_hat.cross(r1_hat) + r3_hat.cross(dr1_hat);

    Eigen::Vector3d omega_RN_R;
    omega_RN_R[0] = r3_hat.dot(dr2_hat);
    omega_RN_R[1] = r1_hat.dot(dr3_hat);
    omega_RN_R[2] = r2_hat.dot(dr1_hat);

    const Eigen::Vector3d ddr1_hat =
        -(2.0 * dr1_hat * r1_hat.transpose() + r1_hat * dr1_hat.transpose()) * (v_PB_N / r_PB_N.norm());
    const Eigen::Vector3d ddr3_hat = ((identity - r3_hat * r3_hat.transpose()) * a_n -
                                      (2.0 * dr3_hat * r3_hat.transpose() + r3_hat * dr3_hat.transpose()) * v_n) /
                                     R_n.norm();
    const Eigen::Vector3d ddr2_hat = ddr3_hat.cross(r1_hat) + r3_hat.cross(ddr1_hat) + 2.0 * dr3_hat.cross(dr1_hat);

    Eigen::Vector3d domega_RN_R;
    domega_RN_R[0] = dr3_hat.dot(dr2_hat) + r3_hat.dot(ddr2_hat) - omega_RN_R.dot(dr1_hat);
    domega_RN_R[1] = dr1_hat.dot(dr3_hat) + r1_hat.dot(ddr3_hat) - omega_RN_R.dot(dr2_hat);
    domega_RN_R[2] = dr2_hat.dot(dr1_hat) + r2_hat.dot(ddr1_hat) - omega_RN_R.dot(dr3_hat);

    return {
        dcmToMrp(dcm_RN),
        dcm_RN.transpose() * omega_RN_R,
        dcm_RN.transpose() * domega_RN_R,
    };
}

// Full double-precision reference for the algorithm, including the secondary-constraint
// validity logic. The thresholds and link flag must match the algorithm's configuration.
inline ReferenceCelestialTwoBodyPointOutput referenceCelestialTwoBodyPoint(const Eigen::Vector3d& r_celBody_N,
                                                                           const Eigen::Vector3d& v_celBody_N,
                                                                           const Eigen::Vector3d& r_secCelBody_N,
                                                                           const Eigen::Vector3d& v_secCelBody_N,
                                                                           const Eigen::Vector3d& r_BN_N,
                                                                           const Eigen::Vector3d& v_BN_N,
                                                                           const float singularityThreshold,
                                                                           const float rateThreshold,
                                                                           const bool secCelBodyIsLinked) {
    const Eigen::Vector3d r_PB_N = r_celBody_N - r_BN_N;
    const Eigen::Vector3d v_PB_N = v_celBody_N - v_BN_N;

    Eigen::Vector3d r_SB_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_SB_N = Eigen::Vector3d::Zero();

    double platAngDiff = 0.0;
    if (secCelBodyIsLinked) {
        r_SB_N = r_secCelBody_N - r_BN_N;
        v_SB_N = v_secCelBody_N - v_BN_N;
        platAngDiff = std::acos(r_SB_N.normalized().dot(r_PB_N.normalized()));
    }

    ReferenceCelestialTwoBodyPointOutput out = referenceRateAndAccelCalc(r_PB_N, v_PB_N, r_SB_N, v_SB_N);

    if (!secCelBodyIsLinked || out.omega_RN_N.norm() > rateThreshold || std::abs(platAngDiff) < singularityThreshold ||
        std::abs(platAngDiff) > M_PI - singularityThreshold) {
        r_SB_N = r_PB_N.cross(v_PB_N);
        v_SB_N = Eigen::Vector3d::Zero();
        out = referenceRateAndAccelCalc(r_PB_N, v_PB_N, r_SB_N, v_SB_N);
    }

    return out;
}

// Compare the FP32 algorithm output against the double-precision reference. The inputs should be
// chosen so that the constraint-validity branch decision is unambiguous (not at the threshold
// boundary), otherwise the float and double implementations may legitimately pick different
// branches.
inline void testCelestialTwoBodyPoint(const Eigen::Vector3d& r_celBody_N,
                                      const Eigen::Vector3d& v_celBody_N,
                                      const Eigen::Vector3d& r_secCelBody_N,
                                      const Eigen::Vector3d& v_secCelBody_N,
                                      const Eigen::Vector3d& r_BN_N,
                                      const Eigen::Vector3d& v_BN_N,
                                      const float singularityThreshold,
                                      const float rateThreshold,
                                      const bool secCelBodyIsLinked) {
    const CelestialTwoBodyPointAlgorithm alg(
        CelestialTwoBodyPointConfig::create(singularityThreshold, rateThreshold, secCelBodyIsLinked));

    CelestialTwoBodyPointOutput out;
    EXPECT_NO_THROW(out = alg.update(r_celBody_N, v_celBody_N, r_secCelBody_N, v_secCelBody_N, r_BN_N, v_BN_N));

    ReferenceCelestialTwoBodyPointOutput ref;
    EXPECT_NO_THROW(ref = referenceCelestialTwoBodyPoint(r_celBody_N,
                                                         v_celBody_N,
                                                         r_secCelBody_N,
                                                         v_secCelBody_N,
                                                         r_BN_N,
                                                         v_BN_N,
                                                         singularityThreshold,
                                                         rateThreshold,
                                                         secCelBodyIsLinked));

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

// Property: every output component is finite for well-posed inputs.
inline void propertyOutputIsFinite(const Eigen::Vector3d& r_celBody_N,
                                   const Eigen::Vector3d& v_celBody_N,
                                   const Eigen::Vector3d& r_secCelBody_N,
                                   const Eigen::Vector3d& v_secCelBody_N,
                                   const Eigen::Vector3d& r_BN_N,
                                   const Eigen::Vector3d& v_BN_N,
                                   const float singularityThreshold,
                                   const float rateThreshold,
                                   const bool secCelBodyIsLinked) {
    const CelestialTwoBodyPointAlgorithm alg(
        CelestialTwoBodyPointConfig::create(singularityThreshold, rateThreshold, secCelBodyIsLinked));

    CelestialTwoBodyPointOutput out;
    EXPECT_NO_THROW(out = alg.update(r_celBody_N, v_celBody_N, r_secCelBody_N, v_secCelBody_N, r_BN_N, v_BN_N));

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
    }
}

// Property: dcmToMrp always returns the short-rotation MRP set, so |sigma_RN| <= 1.
inline void propertySigmaNormBounded(const Eigen::Vector3d& r_celBody_N,
                                     const Eigen::Vector3d& v_celBody_N,
                                     const Eigen::Vector3d& r_secCelBody_N,
                                     const Eigen::Vector3d& v_secCelBody_N,
                                     const Eigen::Vector3d& r_BN_N,
                                     const Eigen::Vector3d& v_BN_N,
                                     const float singularityThreshold,
                                     const float rateThreshold,
                                     const bool secCelBodyIsLinked) {
    const CelestialTwoBodyPointAlgorithm alg(
        CelestialTwoBodyPointConfig::create(singularityThreshold, rateThreshold, secCelBodyIsLinked));

    CelestialTwoBodyPointOutput out;
    EXPECT_NO_THROW(out = alg.update(r_celBody_N, v_celBody_N, r_secCelBody_N, v_secCelBody_N, r_BN_N, v_BN_N));

    EXPECT_LE(out.sigma_RN.norm(), 1.0F + 1e-6F);
}

inline void testCelestialTwoBodyPointSetup() {
    EXPECT_NO_THROW({
        const CelestialTwoBodyPointConfig cfg = CelestialTwoBodyPointConfig::create(0.1F, 0.2F, true);
        const CelestialTwoBodyPointAlgorithm alg(cfg);
        (void)alg;
    });
}

#endif  // TEST_CELESTIAL_TWO_BODY_POINT_HELPERS_H
