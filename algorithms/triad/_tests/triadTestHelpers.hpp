#ifndef TEST_TRIAD_H
#define TEST_TRIAD_H

#include "triadAlgorithm.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <architecture/utilities/rigidBodyKinematics.hpp>
#include <cmath>

// Reference implementation of the TRIAD algorithm matching the Python true_triad()
inline Eigen::Vector3f referenceTriad(const Eigen::Vector3f& sigma_BN,
                                      const Eigen::Vector3f& rHat_SB_B,
                                      const Eigen::Vector3f& hReqHat_N,
                                      const Eigen::Vector3f& a1Hat_B,
                                      const Eigen::Vector3f& hRefHat_B) {
    const Eigen::Vector3f r2 = hRefHat_B;
    const Eigen::Vector3f r3 = a1Hat_B.cross(hRefHat_B).normalized();
    const Eigen::Vector3f r1 = r2.cross(r3);

    const Eigen::Vector3f n2 = hReqHat_N;

    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3f rHat_SB_N = (dcm_BN.transpose() * rHat_SB_B).normalized();
    const Eigen::Vector3f n1 = rHat_SB_N.cross(hReqHat_N).normalized();
    const Eigen::Vector3f n3 = n1.cross(n2);

    Eigen::Matrix3f RD;
    RD.col(0) = r1;
    RD.col(1) = r2;
    RD.col(2) = r3;

    Eigen::Matrix3f BD;
    BD.col(0) = n1;
    BD.col(1) = n2;
    BD.col(2) = n3;

    const Eigen::Matrix3f RN = RD * BD.transpose();

    return dcmToMrp(RN);
}

inline void testTriadRegression(const Eigen::Vector3f& sigma_BN,
                                const Eigen::Vector3f& a1Hat_B,
                                const Eigen::Vector3f& h1Hat_B,
                                const Eigen::Vector3f& rHat_SB_B,
                                const Eigen::Vector3f& hReqHat_N) {
    auto config = TriadConfig::create(a1Hat_B, hReqHat_N);
    TriadAlgorithm alg(config);

    const Eigen::Vector3f result = alg.update(sigma_BN, rHat_SB_B, h1Hat_B);
    const Eigen::Vector3f expected = referenceTriad(sigma_BN, rHat_SB_B, hReqHat_N, a1Hat_B, h1Hat_B);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result[i], expected[i], 1e-5F) << "Component " << i;
        EXPECT_TRUE(std::isfinite(result[i])) << "Component " << i << " is not finite";
    }
}

inline void testTriadSetup() {
    // Valid config should not throw
    EXPECT_NO_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY()));

    // Zero a1Hat_B should throw
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitX()), fsw::invalid_argument);

    // Zero hReqHat_N should throw
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::Zero()), fsw::invalid_argument);

    // Config round-trip
    const Eigen::Vector3f a1 = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
    const Eigen::Vector3f hN = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
    auto config = TriadConfig::create(a1, hN);
    EXPECT_EQ(config.getA1Hat_B(), a1);
    EXPECT_EQ(config.getHHat_N(), hN);

    // Static validators
    EXPECT_TRUE(TriadConfig::isValidA1Hat_B(Eigen::Vector3f::UnitX()));
    EXPECT_FALSE(TriadConfig::isValidA1Hat_B(Eigen::Vector3f::Zero()));
    EXPECT_TRUE(TriadConfig::isValidHHat_N(Eigen::Vector3f::UnitX()));
    EXPECT_FALSE(TriadConfig::isValidHHat_N(Eigen::Vector3f::Zero()));
}

#endif  // TEST_TRIAD_H
