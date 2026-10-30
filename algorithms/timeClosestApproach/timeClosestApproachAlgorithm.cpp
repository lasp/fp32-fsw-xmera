#include "timeClosestApproachAlgorithm.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/safeMath.h"

/*! Computes time of closest approach estimation during a rectilinear flyby. With the flight path angle sine
 *  s = r_hat . v_hat and ratio = |v| / |r|, the estimate is tCA = -s / ratio, and its variance maps the filter
 *  covariance through g = [v_hat / |r|, (r_hat - s v_hat) / |v|] / ratio.
 *
 *  Plain float arithmetic: its rounding moves tCA and sigmaTca about as much as rounding the filter state to float
 *  does, both far below sigmaTca. The square root is taken before dividing by the ratio, so that sigmaTca stays in
 *  float range whenever it is representable.
 @param r_BN_N spacecraft position estimate in inertial coordinates [m]
 @param v_BN_N spacecraft velocity estimate in inertial coordinates [m/s]
 @param filterCovariance filter covariance
 @return the predicted time of closest approach [s] and its standard deviation [s]
*/
TimeClosestApproachOutput TimeClosestApproachAlgorithm::update(const Eigen::Vector3f& r_BN_N,
                                                               const Eigen::Vector3f& v_BN_N,
                                                               const Eigen::Matrix<float, 6, 6>& filterCovariance) {
    TimeClosestApproachOutput algoOutput{.tCA = 0.0F, .sigmaTca = 0.0F};

    float const r_BN_N_norm = r_BN_N.stableNorm();
    float const v_BN_N_norm = v_BN_N.stableNorm();
    if (r_BN_N_norm >= kMinVectorNorm && v_BN_N_norm >= kMinVectorNorm) {
        float const ratio = v_BN_N_norm / r_BN_N_norm;
        Eigen::Vector3f const r_BN_N_hat = r_BN_N / r_BN_N_norm;
        Eigen::Vector3f const v_BN_N_hat = v_BN_N / v_BN_N_norm;
        float sinFPA = r_BN_N_hat.dot(v_BN_N_hat);
        sinFPA = std::max(-1.0F, std::min(1.0F, sinFPA));
        float const tCABuffer = -sinFPA / ratio;

        if (fsw::is_finite(tCABuffer)) {
            algoOutput.tCA = tCABuffer;

            Eigen::Matrix<float, 6, 1> covarianceMapToTca;
            covarianceMapToTca.head(3) = (v_BN_N_hat / r_BN_N_norm);
            covarianceMapToTca.tail(3) = ((r_BN_N_hat - sinFPA * v_BN_N_hat) / v_BN_N_norm);
            float const mappedCovariance = covarianceMapToTca.transpose() * filterCovariance * covarianceMapToTca;
            float const sigmaTca = safeSqrtf(mappedCovariance) / ratio;
            if (fsw::is_finite(mappedCovariance) && fsw::is_finite(sigmaTca)) {
                algoOutput.sigmaTca = sigmaTca;
            }
        }
    }

    return algoOutput;
}
