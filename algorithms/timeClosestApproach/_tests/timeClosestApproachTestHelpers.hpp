#ifndef TEST_TIME_CA_HELPERS_H
#define TEST_TIME_CA_HELPERS_H

#include "timeClosestApproachAlgorithm.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/safeMath.h"

#include <gtest/gtest.h>
#include <cmath>

struct ReferenceTimeClosestApproachOutput {
    double tCA{};
    double sigmaTca{};
};

inline ReferenceTimeClosestApproachOutput referenceTimeClosestApproach(
    const Eigen::Vector3d& r_BN_N,
    const Eigen::Vector3d& v_BN_N,
    const Eigen::Matrix<double, 6, 6>& filterCovariance) {
    ReferenceTimeClosestApproachOutput ref_output{.tCA = 0.0, .sigmaTca = 0.0};
    double const r_BN_N_norm = r_BN_N.stableNorm();
    double const v_BN_N_norm = v_BN_N.stableNorm();
    if (r_BN_N_norm >= kMinVectorNorm && v_BN_N_norm >= kMinVectorNorm) {
        double const ratio = v_BN_N_norm / r_BN_N_norm;
        Eigen::Vector3d const r_BN_N_hat = r_BN_N / r_BN_N_norm;
        Eigen::Vector3d const v_BN_N_hat = v_BN_N / v_BN_N_norm;
        double sinFPA = r_BN_N_hat.dot(v_BN_N_hat);
        sinFPA = std::max(-1.0, std::min(1.0, sinFPA));
        const auto tCA_predict = static_cast<float>(-sinFPA / ratio);

        if (fsw::is_finite(tCA_predict)) {
            ref_output.tCA = tCA_predict;

            Eigen::Matrix<double, 6, 1> covariance_map_to_tca;
            covariance_map_to_tca.head(3) = (v_BN_N_hat / r_BN_N_norm);
            covariance_map_to_tca.tail(3) = ((r_BN_N_hat - sinFPA * v_BN_N_hat) / v_BN_N_norm);
            const double mappedCovariance =
                covariance_map_to_tca.transpose() * filterCovariance * covariance_map_to_tca;
            const auto tCA_covariance = static_cast<float>(mappedCovariance / (ratio * ratio));
            if (fsw::is_finite(tCA_covariance)) {
                ref_output.sigmaTca = safeSqrtf(tCA_covariance);
            }
        }
    }
    return ref_output;
}

inline void testTimeClosestApproach(const Eigen::Vector3d& r_BN_N,
                                    const Eigen::Vector3d& v_BN_N,
                                    const Eigen::Matrix<double, 6, 6>& filterCovariance) {
    TimeClosestApproachAlgorithm const alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out{};
    EXPECT_NO_THROW(out = alg.update(r_BN_N, v_BN_N, filterCovariance));

    ReferenceTimeClosestApproachOutput const ref = referenceTimeClosestApproach(r_BN_N, v_BN_N, filterCovariance);
    const auto tCA_ref = static_cast<float>(ref.tCA);
    const auto sigma_ref = static_cast<float>(ref.sigmaTca);

    // tCA: double arithmetic narrowed to float at output — 1 op × ε_float32 × 10× margin.
    EXPECT_NEAR(out.tCA, tCA_ref, std::max(1e-7F, std::abs(tCA_ref) * 1e-6F));

    // sigmaTca: covariance path is now fully double; only the final double→float narrowing
    // and the float sqrt contribute error — 1e-6 relative, 1e-7 absolute floor.
    EXPECT_NEAR(out.sigmaTca, sigma_ref, std::max(1e-7F, std::abs(sigma_ref) * 1e-6F));

    ASSERT_TRUE(fsw::is_finite(out.tCA));
    ASSERT_TRUE(fsw::is_finite(out.sigmaTca));
    EXPECT_GE(out.sigmaTca, 0.0F);
}

#endif
