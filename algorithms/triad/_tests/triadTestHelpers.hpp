#ifndef TEST_TRIAD_H
#define TEST_TRIAD_H

#include "triadAlgorithm.h"
#include "utilities/fsw/safeMath.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <architecture/utilities/rigidBodyKinematics.hpp>
#include <cmath>

// Reference implementation of the TRIAD algorithm matching the Python true_triad()
inline Eigen::Vector3f referenceTriad(const Eigen::Vector3f& sigma_BN,
                                      const Eigen::Vector3f& rHat_SB_B,
                                      const Eigen::Vector3f& hReqHat_N,
                                      const Eigen::Vector3f& a1Hat_B,
                                      const Eigen::Vector3f& hRefHat_B,
                                      const float signOfZHat_N) {
    /*! Compute angle between solar array drive axis and thrust direction */
    const float sadaAxisToThrustAngle = safeAcosf(fabsf(a1Hat_B.dot(hRefHat_B)));

    /*! Return current attitude if solar array drive axis and thrust direction are nearly parallel or if either
     * thrustHat_B or rHat_SB_B are zero */
    if (sadaAxisToThrustAngle < kParallelThresholdRad || hRefHat_B.stableNorm() == 0.0F ||
        rHat_SB_B.stableNorm() == 0.0F) {
        return mrpSwitch(sigma_BN, 1.0F);
    }

    /*! Triad (D frame) basis vectors in hub reference frame */
    Eigen::Matrix3f RD;
    const Eigen::Vector3f r2 = hRefHat_B.normalized();
    const Eigen::Vector3f r3 = a1Hat_B.cross(hRefHat_B).normalized();
    const Eigen::Vector3f r1 = r2.cross(r3).normalized();
    RD.col(0) = r1;
    RD.col(1) = r2;
    RD.col(2) = r3;

    /*! Compute angle between sun direction and thrust inertial reference direction */
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3f rHat_SB_N = (dcm_BN.transpose() * rHat_SB_B).normalized();
    const float SPE = safeAcosf(fabsf(rHat_SB_N.dot(hReqHat_N)));

    /*! Triad (D Frame) basis vectors in inertial frame */
    const Eigen::Vector3f n2 = hReqHat_N;
    Eigen::Vector3f n1 = Eigen::Vector3f::Zero();
    Eigen::Vector3f n3 = Eigen::Vector3f::Zero();

    /*! If sun direction and thrust inertial reference are nearly parallel, cross the second triad axis instead with the
     * configured inertial z-axis */
    if (fabsf(SPE) < kParallelThresholdRad) {
        const Eigen::Vector3f zHat_N = (signOfZHat_N * Eigen::Vector3f::UnitZ()).normalized();
        n3 = zHat_N.cross(n2).normalized();
        n1 = n2.cross(n3);
    } else {
        // Normal triad otherwise
        n1 = rHat_SB_N.cross(n2).normalized();
        n3 = n1.cross(n2);
    }
    Eigen::Matrix3f ND;
    ND.col(0) = n1;
    ND.col(1) = n2;
    ND.col(2) = n3;

    const Eigen::Matrix3f RN = RD * ND.transpose();

    return dcmToMrp(RN);
}

inline void testTriadRegression(const Eigen::Vector3f& sigma_BN,
                                const Eigen::Vector3f& a1Hat_B,
                                const Eigen::Vector3f& h1Hat_B,
                                const Eigen::Vector3f& rHat_SB_B,
                                const Eigen::Vector3f& hReqHat_N,
                                const float signOfZHat_N) {
    auto config = TriadConfig::create(a1Hat_B, hReqHat_N, signOfZHat_N);
    TriadAlgorithm alg(config);

    const Eigen::Vector3f result = alg.update(sigma_BN, rHat_SB_B, h1Hat_B);
    const Eigen::Vector3f expected = referenceTriad(sigma_BN, rHat_SB_B, hReqHat_N, a1Hat_B, h1Hat_B, signOfZHat_N);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result[i], expected[i], 1e-5F) << "Component " << i;
        EXPECT_TRUE(std::isfinite(result[i])) << "Component " << i << " is not finite";
    }
}

inline void testTriadSetup() {
    // Valid config should not throw
    EXPECT_NO_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), 1.0F));

    // Zero a1Hat_B should throw
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitX(), -1.0F), fsw::invalid_argument);

    // Zero hReqHat_N should throw
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::Zero(), 2.0F), fsw::invalid_argument);

    // Zero signOfZHat_N should throw
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), 0.0F), fsw::invalid_argument);

    // Config round-trip
    const Eigen::Vector3f a1 = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
    const Eigen::Vector3f hN = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
    const float signOfZHat_N = -1.0F;
    auto config = TriadConfig::create(a1, hN, signOfZHat_N);
    EXPECT_EQ(config.getA1Hat_B(), a1);
    EXPECT_EQ(config.getHHat_N(), hN);
    EXPECT_EQ(config.getSignOfZHat_N(), signOfZHat_N);

    // Static validators
    EXPECT_TRUE(TriadConfig::isValidA1Hat_B(Eigen::Vector3f::UnitX()));
    EXPECT_FALSE(TriadConfig::isValidA1Hat_B(Eigen::Vector3f::Zero()));
    EXPECT_TRUE(TriadConfig::isValidHHat_N(Eigen::Vector3f::UnitX()));
    EXPECT_FALSE(TriadConfig::isValidHHat_N(Eigen::Vector3f::Zero()));
    EXPECT_TRUE(TriadConfig::isValidSignOfZHat_N(-2.0F));
    EXPECT_FALSE(TriadConfig::isValidSignOfZHat_N(0.0F));
}

#endif  // TEST_TRIAD_H
