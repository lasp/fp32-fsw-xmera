#include "timeClosestApproachTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// Generates an n×n positive definite matrix via A^T*A + 1e-3*I,
// where each element of A is drawn from [-1.0e3F, 1.0e3F].
inline auto PositiveDefiniteMatrixDomain(int n) {
    return fuzztest::Map(
        [n](const std::vector<float>& elems) -> Eigen::MatrixXf {
            Eigen::MatrixXf A = Eigen::Map<const Eigen::MatrixXf>(elems.data(), n, n);
            return A.transpose() * A + 1.0e-3F * Eigen::MatrixXf::Identity(n, n);
        },
        fuzztest::VectorOf(fuzztest::InRange(-1.0e3F, 1.0e3F)).WithSize(n * n));
}

inline void fuzzAdapterTimeClosestApproach(const Eigen::Vector3d r_BN_N,
                                           const Eigen::Vector3d v_BN_N,
                                           const Eigen::MatrixXf filterCovariance) {
    testTimeClosestApproach(r_BN_N, v_BN_N, filterCovariance);
}

FUZZ_TEST(TimeClosestApproachFuzz, fuzzAdapterTimeClosestApproach)
    .WithDomains(fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e-3; },
                                  xmera::fuzz::Vector3dInRange(-1.0e6, 1.0e6)),
                 fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e-3; },
                                  xmera::fuzz::Vector3dInRange(-1.0e6, 1.0e6)),
                 fuzztest::OneOf(PositiveDefiniteMatrixDomain(3), PositiveDefiniteMatrixDomain(6)));
