#include "triadTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

void fuzzTriadRegression(std::vector<float> sigma_BNVec,
                         std::vector<float> sadaHat_BVec,
                         std::vector<float> thrustHat_BVec,
                         std::vector<float> rHat_SB_BVec,
                         std::vector<float> thrustReqHat_NVec,
                         const float signOfZHat_N) {
    const Eigen::Vector3f sigma_BN(sigma_BNVec[0], sigma_BNVec[1], sigma_BNVec[2]);
    const Eigen::Vector3f sadaHat_B(sadaHat_BVec[0], sadaHat_BVec[1], sadaHat_BVec[2]);
    const Eigen::Vector3f thrustHat_B(thrustHat_BVec[0], thrustHat_BVec[1], thrustHat_BVec[2]);
    const Eigen::Vector3f rHat_SB_B(rHat_SB_BVec[0], rHat_SB_BVec[1], rHat_SB_BVec[2]);
    const Eigen::Vector3f thrustReqHat_N(thrustReqHat_NVec[0], thrustReqHat_NVec[1], thrustReqHat_NVec[2]);

    // Filter: signOfZHat_N must be non-zero
    if (signOfZHat_N == 0.0F) { return; }

    testTriadRegression(sigma_BN, sadaHat_B.stableNormalized(), thrustHat_B.stableNormalized(), rHat_SB_B.stableNormalized(), thrustReqHat_N.stableNormalized(), signOfZHat_N);
}

FUZZ_TEST(TriadAlgorithmFuzz, fuzzTriadRegression)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),   // sigma_BN
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),   // sadaHat_B
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),   // thrustHat_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),   // rHat_SB_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),  // thrustReqHat_N
                 fuzztest::InRange(-1e6F, 1e6F));  // signOfZHat_N
