#ifndef TEST_TIME_CA_HELPERS_H
#define TEST_TIME_CA_HELPERS_H

#include "timeClosestApproachAlgorithm.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>

struct ReferenceTimeClosestApproachOutput {
    float tCA{};
    float sigmaTca{};
};

inline ReferenceTimeClosestApproachOutput referenceTimeClosestApproach(const Eigen::Vector3f& r_BN_N,
                                                                       const Eigen::Vector3f& v_BN_N,
                                                                       const Eigen::MatrixXf& filterCovariance) {
    float const ratio = v_BN_N.norm() / r_BN_N.norm();

    Eigen::Vector3f const r_BN_N_hat = r_BN_N.normalized();
    Eigen::Vector3f const v_BN_N_hat = v_BN_N.normalized();

    // sin(flightPathAngle) == r_hat·v_hat  (identity: sin(acos(-d) - π/2) = d)
    // Using the dot product directly avoids catastrophic cancellation in float32
    // when computing theta = acos(-d) and then subtracting π/2.
    float sinFPA = r_BN_N_hat.dot(v_BN_N_hat);
    sinFPA = std::max(-1.0F, std::min(1.0F, sinFPA));

    ReferenceTimeClosestApproachOutput ref_output{};
    ref_output.tCA = -sinFPA / ratio;

    const auto numberOfStates = static_cast<std::size_t>(filterCovariance.rows());
    Eigen::VectorXf covariance_map_to_tca(numberOfStates);

    covariance_map_to_tca.head(3) = v_BN_N_hat / r_BN_N.norm();
    if (numberOfStates == 6) {
        covariance_map_to_tca.tail(3) = (r_BN_N_hat - sinFPA * v_BN_N_hat) / v_BN_N.norm();
    }
    const float mappedCovariance = covariance_map_to_tca.transpose() * filterCovariance * covariance_map_to_tca;
    const float tCA_covariance = mappedCovariance / (ratio * ratio);

    ref_output.sigmaTca = std::sqrt(tCA_covariance);

    return ref_output;
}

inline void testTimeClosestApproach(const Eigen::Vector3f& r_BN_N,
                                    const Eigen::Vector3f& v_BN_N,
                                    const Eigen::MatrixXf& filterCovariance) {
    TimeClosestApproachAlgorithm const alg;
    TimeClosestApproachOutput out;
    EXPECT_NO_THROW(out = alg.update(r_BN_N, v_BN_N, filterCovariance));

    ReferenceTimeClosestApproachOutput const ref =
        referenceTimeClosestApproach(r_BN_N.cast<float>(), v_BN_N.cast<float>(), filterCovariance.cast<float>());
    const auto tCA_ref = static_cast<float>(ref.tCA);
    const auto sigma_ref = static_cast<float>(ref.sigmaTca);
    EXPECT_NEAR(out.tCA, tCA_ref, std::max(1e-5F, std::abs(tCA_ref) * 1e-4F));
    EXPECT_NEAR(out.sigmaTca, sigma_ref, std::max(1e-5F, std::abs(sigma_ref) * 1e-4F));
    EXPECT_GE(out.sigmaTca, 0.0F);
}

#endif
