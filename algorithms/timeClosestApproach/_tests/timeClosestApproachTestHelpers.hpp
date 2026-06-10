#ifndef TEST_TIME_CA_HELPERS_H
#define TEST_TIME_CA_HELPERS_H

#include "timeClosestApproachAlgorithm.h"
#include "utilities/fsw/safeMath.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <cstdint>
#include <limits>

struct ReferenceTimeClosestApproachOutput {
    double tCA{};
    double sigmaTca{};
};

inline ReferenceTimeClosestApproachOutput referenceTimeClosestApproach(
    const Eigen::Vector3d& r_BN_N,
    const Eigen::Vector3d& v_BN_N,
    const Eigen::Matrix<double, 6, 6>& filterCovariance) {
    double const ratio = v_BN_N.norm() / r_BN_N.norm();

    Eigen::Vector3d const r_BN_N_hat = r_BN_N.normalized();
    Eigen::Vector3d const v_BN_N_hat = v_BN_N.normalized();

    double sinFPA = r_BN_N_hat.dot(v_BN_N_hat);
    sinFPA = std::max(-1.0, std::min(1.0, sinFPA));

    ReferenceTimeClosestApproachOutput ref_output{};
    ref_output.tCA = -sinFPA / ratio;

    Eigen::Matrix<double, 6, 1> covariance_map_to_tca;

    covariance_map_to_tca.head(3) = v_BN_N_hat / r_BN_N.norm();
    covariance_map_to_tca.tail(3) = (r_BN_N_hat - sinFPA * v_BN_N_hat) / v_BN_N.norm();
    const double mappedCovariance = covariance_map_to_tca.transpose() * filterCovariance * covariance_map_to_tca;
    const double tCA_covariance = mappedCovariance / (ratio * ratio);

    ref_output.sigmaTca = safeSqrt(tCA_covariance);

    return ref_output;
}

inline void testTimeClosestApproach(const Eigen::Vector3d& r_BN_N,
                                    const Eigen::Vector3d& v_BN_N,
                                    const Eigen::Matrix<double, 6, 6>& filterCovariance) {
    TimeClosestApproachAlgorithm const alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out;
    EXPECT_NO_THROW(out = alg.update(r_BN_N, v_BN_N, filterCovariance));

    ReferenceTimeClosestApproachOutput const ref = referenceTimeClosestApproach(r_BN_N, v_BN_N, filterCovariance);
    const auto tCA_ref = static_cast<float>(ref.tCA);
    const auto sigma_ref = static_cast<float>(ref.sigmaTca);

    // tCA: double arithmetic narrowed to float at output — 1 op × ε_float32 × 10× margin.
    EXPECT_NEAR(out.tCA, tCA_ref, std::max(1e-7F, std::abs(tCA_ref) * 1e-6F));

    // sigmaTca: covariance path is now fully double; only the final double→float narrowing
    // and the float sqrt contribute error — 1e-6 relative, 1e-7 absolute floor.
    EXPECT_NEAR(out.sigmaTca, sigma_ref, std::max(1e-7F, std::abs(sigma_ref) * 1e-6F));

    ASSERT_TRUE(std::isfinite(out.tCA));
    ASSERT_TRUE(std::isfinite(out.sigmaTca));
    EXPECT_GE(out.sigmaTca, 0.0F);
}

#endif
