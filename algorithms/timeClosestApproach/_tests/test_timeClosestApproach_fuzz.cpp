#include "timeClosestApproachTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// Generates a 6×6 covariance P = S C S: S = diag(s) holds per-state standard deviations over a wide physical range,
// and C = I + B^T B / 6 (B entries in [-1, 1]) is a correlation-like matrix with cond(C) <= 7. The quadratic form
// g' P g = (S g)' C (S g) then amplifies relative errors in the gradient g by at most cond(C), as for a real filter
// covariance; a P built as A^T A with entries of +-1e14 next to +-1 amplifies them by ~1e10, past what the double
// reference itself resolves at the test tolerance.
inline auto PositiveDefiniteMatrix6x6Domain() {
    return fuzztest::Map(
        [](const std::vector<double>& sigmas, const std::vector<double>& elems) -> Eigen::Matrix<double, 6, 6> {
            Eigen::Matrix<double, 6, 6> const B = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(elems.data());
            Eigen::Matrix<double, 6, 6> const C = Eigen::Matrix<double, 6, 6>::Identity() + (B.transpose() * B / 6.0);
            auto const S = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(sigmas.data()).asDiagonal();
            return S * C * S;
        },
        fuzztest::VectorOf(fuzztest::InRange(1.0e-3, 1.0e7)).WithSize(6),  // per-state standard deviation
        fuzztest::VectorOf(fuzztest::InRange(-1.0, 1.0)).WithSize(36));
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
