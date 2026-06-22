#include "timeClosestApproachTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// Generates a 6×6 positive definite matrix via A^T*A + 1e-3*I,
// where each element of A is drawn from physical range.
inline auto PositiveDefiniteMatrix6x6Domain() {
    return fuzztest::Map(
        [](const std::vector<double>& elems) -> Eigen::Matrix<double, 6, 6> {
            Eigen::Matrix<double, 6, 6> A = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(elems.data());
            return A.transpose() * A + 1.0e-3 * Eigen::Matrix<double, 6, 6>::Identity();
        },
        fuzztest::VectorOf(fuzztest::InRange(-1.0e14, 1.0e14)).WithSize(36));
}

inline void fuzzAdapterTimeClosestApproach(const Eigen::Vector3d r_BN_N,
                                           const Eigen::Vector3d v_BN_N,
                                           const Eigen::Matrix<double, 6, 6> filterCovariance) {
    testTimeClosestApproach(r_BN_N, v_BN_N, filterCovariance);
}

FUZZ_TEST(TimeClosestApproachFuzz, fuzzAdapterTimeClosestApproach)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1.0e14, 1.0e14),  // phyiscal range of relative position [m]
                 xmera::fuzz::Vector3dInRange(-1.0e14, 1.0e14),  // phyiscal range of relative velocity [m/s]
                 PositiveDefiniteMatrix6x6Domain());
