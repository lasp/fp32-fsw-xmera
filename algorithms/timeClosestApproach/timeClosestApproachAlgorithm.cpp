// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "timeClosestApproachAlgorithm.h"

TimeClosestApproachOutput TimeClosestApproachAlgorithm::update(const Eigen::Vector3f& r_BN_N,
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

    TimeClosestApproachOutput algo_output{};
    algo_output.tCA = -sinFPA / ratio;

    const auto numberOfStates = static_cast<std::size_t>(filterCovariance.rows());
    Eigen::VectorXf covariance_map_to_tca(numberOfStates);

    covariance_map_to_tca.head(3) = v_BN_N_hat / r_BN_N.norm();
    if (numberOfStates == 6) {
        covariance_map_to_tca.tail(3) = (r_BN_N_hat - sinFPA * v_BN_N_hat) / v_BN_N.norm();
    }
    const float mappedCovariance = covariance_map_to_tca.transpose() * filterCovariance * covariance_map_to_tca;
    const float tCA_covariance = mappedCovariance / (ratio * ratio);

    algo_output.sigmaTca = std::sqrt(tCA_covariance);

    return algo_output;
}
