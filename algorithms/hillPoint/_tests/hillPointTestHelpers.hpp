#ifndef TEST_HILL_POINT_HELPERS_H
#define TEST_HILL_POINT_HELPERS_H

#include "hillPointAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <numbers>

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

// ---------------------------------------------------------------------------
// Regression test helper functions
// ---------------------------------------------------------------------------

inline void testHillPointRegression(const Eigen::Vector3d& r_BN_N,
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
// Conic-orbit test helper functions
// ---------------------------------------------------------------------------

// Check that the generated conic state is physically valid & satisfies HillPoint's
// minimum position/velocity separation angle before running the analytical conic test
inline bool isConicOrbitGeometryValid(const double eccentricity, const double trueAnomaly) {
    const double denom = 1.0 + eccentricity * std::cos(trueAnomaly);
    if (denom <= 0.0) {
        return false;  // true anomaly lies outside the physical branch of the conic
    }
    const double posVelSeparationAngle = std::atan2(denom, std::abs(eccentricity * std::sin(trueAnomaly)));
    return posVelSeparationAngle >= HillPointAlgorithm::kSmallAngleThreshold;
}

inline void testHillPointConicOrbit(const double eccentricity,
                                    const double mu,               // [m^3/s^2]
                                    const double semiLatusRectum,  // [m]
                                    const double trueAnomaly) {    // [rad]

    // Construct the conic orbit from p, e, and true anomaly
    const double r = semiLatusRectum / (1.0 + eccentricity * std::cos(trueAnomaly));  // radius [m]
    const double h = std::sqrt(mu * semiLatusRectum);  // specific angular momentum [m^2/s]

    // Radial and tangential velocity components
    const double v_r = (mu / h) * eccentricity * std::sin(trueAnomaly);
    const double v_theta = h / r;

    // Convert orbital state to cartesian components
    const Eigen::Vector3d r_BN_N{r * std::cos(trueAnomaly), r * std::sin(trueAnomaly), 0.0};
    const Eigen::Vector3d v_BN_N{v_r * std::cos(trueAnomaly) - v_theta * std::sin(trueAnomaly),
                                 v_r * std::sin(trueAnomaly) + v_theta * std::cos(trueAnomaly),
                                 0.0};

    HillPointAlgorithm alg;
    HillPointOutput out;
    EXPECT_NO_THROW(out = alg.update(r_BN_N, v_BN_N, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));

    // Derive the angular velocity and acceleration from conic-orbit relations
    const double dfdt = h / (r * r);
    const double ddfdt2 = -2.0 * v_r / r * dfdt;

    constexpr float absTol = 1e-5F;
    constexpr float relTol = 1e-5F;
    const auto tolFor = [&](float expectedVal) { return std::max(absTol, relTol * std::abs(expectedVal)); };

    EXPECT_NEAR(out.sigma_RN[0], 0.0F, absTol);
    EXPECT_NEAR(out.sigma_RN[1], 0.0F, absTol);
    const float sigmaZExpected = static_cast<float>(std::tan(trueAnomaly / 4.0));
    EXPECT_NEAR(out.sigma_RN[2], sigmaZExpected, tolFor(sigmaZExpected));

    EXPECT_NEAR(out.omega_RN_N[0], 0.0F, absTol);
    EXPECT_NEAR(out.omega_RN_N[1], 0.0F, absTol);
    const float dfdtExpected = static_cast<float>(dfdt);
    EXPECT_NEAR(out.omega_RN_N[2], dfdtExpected, tolFor(dfdtExpected));

    EXPECT_NEAR(out.domega_RN_N[0], 0.0F, absTol);
    EXPECT_NEAR(out.domega_RN_N[1], 0.0F, absTol);
    const float ddfdt2Expected = static_cast<float>(ddfdt2);
    EXPECT_NEAR(out.domega_RN_N[2], ddfdt2Expected, tolFor(ddfdt2Expected));
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

// sigma_RN norm is bounded by 1 (inner MRP set) for any inputs
inline void propertySigmaNormBounded(const Eigen::Vector3d& r_BN_N,
                                     const Eigen::Vector3d& v_BN_N,
                                     const Eigen::Vector3d& r_PN_N,
                                     const Eigen::Vector3d& v_PN_N) {
    HillPointAlgorithm alg;
    const HillPointOutput out = alg.update(r_BN_N, v_BN_N, r_PN_N, v_PN_N);
    EXPECT_LE(out.sigma_RN.stableNorm(), 1.0F + 1e-6F);
}

inline void testHillPointSetup() {
    EXPECT_NO_THROW({
        const HillPointAlgorithm alg;
        (void)alg;
    });
}

#endif  // TEST_HILL_POINT_HELPERS_H
