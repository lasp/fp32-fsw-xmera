#include "triadTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

void fuzzTriadRegression(std::vector<float> sigma_BNVec,
                         std::vector<float> a1Vec,
                         std::vector<float> h1Vec,
                         std::vector<float> sunVec,
                         std::vector<float> earthVec,
                         const float signOfZHat_N) {
    const Eigen::Vector3f sigma_BN(sigma_BNVec[0], sigma_BNVec[1], sigma_BNVec[2]);
    const Eigen::Vector3f a1(a1Vec[0], a1Vec[1], a1Vec[2]);
    const Eigen::Vector3f h1(h1Vec[0], h1Vec[1], h1Vec[2]);
    const Eigen::Vector3f sun(sunVec[0], sunVec[1], sunVec[2]);
    const Eigen::Vector3f earth(earthVec[0], earthVec[1], earthVec[2]);

    // Filter: signOfZHat_N must be non-zero
    if (signOfZHat_N == 0.0F) { return; }

    testTriadRegression(sigma_BNVec, a1.stableNormalized(), h1.stableNormalized(), sun.stableNormalized(), earth.stableNormalized(), signOfZHat_N);
}

FUZZ_TEST(TriadAlgorithmFuzz, fuzzTriadRegression)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),   // sigma_BN
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),   // a1Hat_B
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),   // h1Hat_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),   // sunDir
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),  // earthDir
                 fuzztest::InRange(-1e6F, 1e6F));  // signOfZHat_N
