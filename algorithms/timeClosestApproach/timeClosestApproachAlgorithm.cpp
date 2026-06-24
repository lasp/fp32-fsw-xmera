// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "timeClosestApproachAlgorithm.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/safeMath.h"

TimeClosestApproachAlgorithm::TimeClosestApproachAlgorithm(const TimeClosestApproachConfig& config) : cfg(config) {}

void TimeClosestApproachAlgorithm::setConfig(const TimeClosestApproachConfig& config) { this->cfg = config; }

/*! Computes time of closest approach estimation during a rectilinear flyby
 @param r_BN_N spacecraft position estimate in inertial coordinates [m]
 @param v_BN_N spacecraft velocity estimate in inertial coordinates [m/s]
 @param filterCovariance filter covariance
 @return the predicted time of closest approach [s] and its standard deviation [s]
*/
// NOLINTBEGIN(readability-convert-member-functions-to-static)
TimeClosestApproachOutput TimeClosestApproachAlgorithm::update(
    const Eigen::Vector3d& r_BN_N,
    const Eigen::Vector3d& v_BN_N,
    const Eigen::Matrix<double, 6, 6>& filterCovariance) const {
    TimeClosestApproachOutput algo_output{.tCA = 0.0F, .sigmaTca = 0.0F};

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
            algo_output.tCA = tCA_predict;

            Eigen::Matrix<double, 6, 1> covariance_map_to_tca;
            covariance_map_to_tca.head(3) = (v_BN_N_hat / r_BN_N_norm);
            covariance_map_to_tca.tail(3) = ((r_BN_N_hat - sinFPA * v_BN_N_hat) / v_BN_N_norm);
            const double mappedCovariance =
                covariance_map_to_tca.transpose() * filterCovariance * covariance_map_to_tca;
            const auto tCA_covariance = static_cast<float>(mappedCovariance / (ratio * ratio));
            if (fsw::is_finite(tCA_covariance)) {
                algo_output.sigmaTca = safeSqrtf(tCA_covariance);
            }
        }
    }

    return algo_output;
}
// NOLINTEND(readability-convert-member-functions-to-static)
