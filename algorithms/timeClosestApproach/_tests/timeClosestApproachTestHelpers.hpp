#ifndef TEST_TIME_CA_HELPERS_H
#define TEST_TIME_CA_HELPERS_H

#include "timeClosestApproachAlgorithm.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>

struct ReferenceTimeClosestApproachOutput {
    double tCA{};
    double sigmaTca{};
};

inline ReferenceTimeClosestApproachOutput referenceTimeClosestApproach(const Eigen::Vector3d& r_BN_N,
                                                                       const Eigen::Vector3d& v_BN_N,
                                                                       const Eigen::MatrixXd& filterCovariance) {
    /*! - compute velocity/radius ratio at time of read */
    double const ratio = v_BN_N.norm() / r_BN_N.norm();

    // compute an angle at the time of read
    Eigen::Vector3d const r_BN_N_hat = r_BN_N.normalized();
    Eigen::Vector3d const v_BN_N_hat = v_BN_N.normalized();

    double product = -r_BN_N_hat.dot(v_BN_N_hat);
    product = std::max(-1.0, std::min(1.0, product));
    const double theta = std::acos(product);

    // compute flight path angle at the time of read
    const double flightPathAngle = theta - M_PI / 2.0;

    ReferenceTimeClosestApproachOutput algo_output{};
    algo_output.tCA = -std::sin(flightPathAngle) / ratio;

    // Calculate covariance_map_to_tca
    int numberOfStates = filterCovariance.rows();
    Eigen::VectorXd covariance_map_to_tca(numberOfStates);

    covariance_map_to_tca.head(3) = v_BN_N_hat / r_BN_N.norm();
    if (numberOfStates == 6) {
        covariance_map_to_tca.tail(3) = 1.0 / v_BN_N.norm() * (r_BN_N_hat - std::sin(flightPathAngle) * v_BN_N_hat);
    }
    const double mappedCovariance = covariance_map_to_tca.transpose() * filterCovariance * covariance_map_to_tca;
    const double tCA_covariance = (1.0 / std::pow(ratio, 2.0)) * mappedCovariance;

    algo_output.sigmaTca = std::sqrt(tCA_covariance);

    return algo_output;
}

inline void testTimeClosestApproach(const Eigen::Vector3f& r_BN_N,
                                    const Eigen::Vector3f& v_BN_N,
                                    const Eigen::MatrixXf& filterCovariance) {
    TimeClosestApproachAlgorithm alg;
    TimeClosestApproachOutput out;
    EXPECT_NO_THROW(out = alg.update(r_BN_N, v_BN_N, filterCovariance));

    ReferenceTimeClosestApproachOutput ref =
        referenceTimeClosestApproach(r_BN_N.cast<double>(), v_BN_N.cast<double>(), filterCovariance.cast<double>());
    constexpr float tol = 1e-5F;
    EXPECT_NEAR(out.tCA, static_cast<float>(ref.tCA), tol);
    EXPECT_NEAR(out.sigmaTca, static_cast<float>(ref.sigmaTca), tol);
    EXPECT_GE(out.sigmaTca, 0.0F);
}

#endif
