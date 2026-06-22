// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "timeClosestApproachAlgorithm.h"

TimeClosestApproachOutput TimeClosestApproachAlgorithm::update(int numberOfStates,
                                                               const Eigen::Vector3d& r_BN_N,
                                                               const Eigen::Vector3d& v_BN_N,
                                                               const Eigen::MatrixXd& filterCovariance) {
    //    double flightPathAngle = -M_PI / 2;  //!< flight path angle of the spacecraft at time of read [rad]
    //    double ratio = 0;                    //!< ratio between relative velocity and position norms at time of read
    //    [Hz]

    /*! - compute velocity/radius ratio at time of read */
    double const ratio = v_BN_N.norm() / r_BN_N.norm();

    // compute an angle at the time of read
    Eigen::Vector3d const r_BN_N_hat = r_BN_N.normalized();
    Eigen::Vector3d const v_BN_N_hat = v_BN_N.normalized();

    double product = -r_BN_N_hat.dot(v_BN_N_hat);
    product = std::max(-1.0, std::min(1.0, product));
    const double theta = std::acos(product);

    // compute flight path angle at the time of read
    double const flightPathAngle = theta - M_PI / 2.0;

    TimeClosestApproachOutput algo_output{};
    algo_output.tCA = -std::sin(flightPathAngle) / ratio;

    // Calculate covariance_map_to_tca
    Eigen::VectorXd covariance_map_to_tca(numberOfStates);

    covariance_map_to_tca.head(3) = v_BN_N_hat / r_BN_N.norm();
    if (numberOfStates == 6) {
        covariance_map_to_tca.tail(3) = 1.0 / v_BN_N.norm() * (r_BN_N_hat - std::sin(flightPathAngle) * v_BN_N_hat);
    }
    const double mappedCovariance = covariance_map_to_tca.transpose() * filterCovariance * covariance_map_to_tca;
    const double tCA_covariance = (1.0 / std::pow(ratio, 2)) * mappedCovariance;

    algo_output.sigmaTca = std::sqrt(tCA_covariance);

    return algo_output;
}
