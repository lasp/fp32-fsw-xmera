#include "timeClosestApproachTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

inline void fuzzAdapterTimeClosestApproach(
                                   const Eigen::Vector3f r_BN_N,
                                   const Eigen::Vector3f v_BN_N,
                                   const Eigen::MatrixXf filterCovariance) {
    testTimeClosestApproach(r_BN_N, v_BN_N, filterCovariance);
}

FUZZ_TEST(TimeClosestApproachFuzz, fuzzAdapterTimeClosestApproach)
    .WithDomains(fuzztest::Filter([](const Eigen::Vector3f& v) { return v.norm() >= 1.0e-3F; },
                         xmera::fuzz::Vector3fInRange(-1.0e6F, 1.0e6F)),
                 fuzztest::Filter([](const Eigen::Vector3f& v) { return v.norm() >= 1.0e-3F; },
                         xmera::fuzz::Vector3fInRange(-1.0e6F, 1.0e6F)),
                 fuzztest::Just(Eigen::MatrixXf::Identity(6, 6)));
