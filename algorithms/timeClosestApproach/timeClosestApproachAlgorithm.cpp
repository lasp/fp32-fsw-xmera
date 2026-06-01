// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "timeClosestApproachAlgorithm.h"

TimeClosestApproachOutput TimeClosestApproachAlgorithm::update(int numberOfStates,
                                                               const Eigen::Vector3f& r_BN_N,
                                                               const Eigen::Vector3f& v_BN_N,
                                                               const Eigen::MatrixXf& filterCovariance) {
    //    float flightPathAngle = -M_PI / 2;  //!< flight path angle of the spacecraft at time of read [rad]
    //    float ratio = 0;                    //!< ratio between relative velocity and position norms at time of read
    //    [Hz]

    /*! - compute velocity/radius ratio at time of read */
    float const ratio = v_BN_N.norm() / r_BN_N.norm();

    // compute an angle at the time of read
    Eigen::Vector3f const r_BN_N_hat = r_BN_N.normalized();
    Eigen::Vector3f const v_BN_N_hat = v_BN_N.normalized();

    float product = -r_BN_N_hat.dot(v_BN_N_hat);
    product = std::max(-1.0F, std::min(1.0F, product));
    const float theta = std::acos(product);

    // compute flight path angle at the time of read
    const float flightPathAngle = theta - static_cast<float>(M_PI) / 2.0F;

    TimeClosestApproachOutput algo_output{};
    algo_output.tCA = -std::sin(flightPathAngle) / ratio;

    // Calculate covariance_map_to_tca
    Eigen::VectorXf covariance_map_to_tca(numberOfStates);

    covariance_map_to_tca.head(3) = v_BN_N_hat / r_BN_N.norm();
    if (numberOfStates == 6) {
        covariance_map_to_tca.tail(3) = 1.0F / v_BN_N.norm() * (r_BN_N_hat - std::sin(flightPathAngle) * v_BN_N_hat);
    }
    const float mappedCovariance = covariance_map_to_tca.transpose() * filterCovariance * covariance_map_to_tca;
    const float tCA_covariance = (1.0F / std::pow(ratio, 2.0F)) * mappedCovariance;

    algo_output.sigmaTca = std::sqrt(tCA_covariance);

    return algo_output;
}
