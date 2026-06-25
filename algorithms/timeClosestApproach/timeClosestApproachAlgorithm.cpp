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
TimeClosestApproachOutput TimeClosestApproachAlgorithm::update(const Eigen::Vector3d& r_BN_N,
                                                               const Eigen::Vector3d& v_BN_N,
                                                               const Eigen::Matrix<double, 6, 6>& filterCovariance) {
    TimeClosestApproachOutput algoOutput{.tCA = 0.0F, .sigmaTca = 0.0F};

    double const r_BN_N_norm = r_BN_N.stableNorm();
    double const v_BN_N_norm = v_BN_N.stableNorm();
    if (r_BN_N_norm >= kMinVectorNorm && v_BN_N_norm >= kMinVectorNorm) {
        double const ratio = v_BN_N_norm / r_BN_N_norm;
        Eigen::Vector3d const r_BN_N_hat = r_BN_N / r_BN_N_norm;
        Eigen::Vector3d const v_BN_N_hat = v_BN_N / v_BN_N_norm;
        double sinFPA = r_BN_N_hat.dot(v_BN_N_hat);
        sinFPA = std::max(-1.0, std::min(1.0, sinFPA));
        const auto tCABuffer = static_cast<float>(-sinFPA / ratio);

        if (fsw::is_finite(tCABuffer)) {
            algoOutput.tCA = tCABuffer;

            Eigen::Matrix<double, 6, 1> covarianceMapToTca;
            covarianceMapToTca.head(3) = (v_BN_N_hat / r_BN_N_norm);
            covarianceMapToTca.tail(3) = ((r_BN_N_hat - sinFPA * v_BN_N_hat) / v_BN_N_norm);
            const double mappedCovariance = covarianceMapToTca.transpose() * filterCovariance * covarianceMapToTca;
            const auto tCA_covariance = static_cast<float>(mappedCovariance / (ratio * ratio));
            if (fsw::is_finite(tCA_covariance)) {
                algoOutput.sigmaTca = safeSqrtf(tCA_covariance);
            }
        }
    }

    return algoOutput;
}
