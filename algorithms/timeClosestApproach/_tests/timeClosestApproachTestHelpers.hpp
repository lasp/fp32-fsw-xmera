#ifndef TEST_TIME_CA_HELPERS_H
#define TEST_TIME_CA_HELPERS_H

#include "timeClosestApproachAlgorithm.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

struct ReferenceTimeClosestApproachOutput {
    double tCA{};
    double sigmaTca{};
};

// The algorithm's formulation evaluated in double: tCA = -s / ratio with s = r_hat . v_hat and ratio = |v| / |r|, and
// sigmaTca = sqrt(g' P g) / ratio with g = [v_hat / |r|, (r_hat - s v_hat) / |v|].
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
        ref_output.tCA = -sinFPA / ratio;

        Eigen::Matrix<double, 6, 1> covariance_map_to_tca;
        covariance_map_to_tca.head(3) = (v_BN_N_hat / r_BN_N_norm);
        covariance_map_to_tca.tail(3) = ((r_BN_N_hat - sinFPA * v_BN_N_hat) / v_BN_N_norm);
        double const mappedCovariance = covariance_map_to_tca.transpose() * filterCovariance * covariance_map_to_tca;
        ref_output.sigmaTca = std::sqrt(std::max(mappedCovariance, 0.0)) / ratio;
    }
    return ref_output;
}

// Compares the float algorithm, fed the float-rounded inputs, with the double reference fed the double inputs. The
// tolerances bound the float error, from the rounding of both the inputs and the arithmetic (u = float unit roundoff,
// c a small constant for the count of rounded operations):
//  - tCA: s = r_hat . v_hat is off by c u, so tCA = -s |r| / |v| is off by c u (|r| / |v| + |tCA|).
//  - sigmaTca: g / ratio is off componentwise by c u G, G = [1 / |v|, |r| / |v|^2] (|r_hat - s v_hat| <= 1), which
//    moves sigmaTca by at most c u sqrt(Q), Q = G' |P| G; rounding P and the quadratic form moves sigmaTca^2 by c u Q,
//    so sigmaTca by at most sqrt(sigmaTca^2 + c u Q) - sigmaTca; the square root and the division add c u sigmaTca.
inline void testTimeClosestApproach(const Eigen::Vector3d& r_BN_N,
                                    const Eigen::Vector3d& v_BN_N,
                                    const Eigen::Matrix<double, 6, 6>& filterCovariance) {
    TimeClosestApproachAlgorithm const alg;
    TimeClosestApproachOutput out{};
    EXPECT_NO_THROW(out = alg.update(r_BN_N.cast<float>(), v_BN_N.cast<float>(), filterCovariance.cast<float>()));

    ASSERT_TRUE(fsw::is_finite(out.tCA));
    ASSERT_TRUE(fsw::is_finite(out.sigmaTca));
    EXPECT_GE(out.sigmaTca, 0.0F);

    ReferenceTimeClosestApproachOutput const ref = referenceTimeClosestApproach(r_BN_N, v_BN_N, filterCovariance);
    double const r_norm = r_BN_N.stableNorm();
    double const v_norm = v_BN_N.stableNorm();
    if (r_norm < kMinVectorNorm || v_norm < kMinVectorNorm) {
        return;  // the norm guard, exercised by the unit tests: float rounding may flip it right at the threshold
    }

    constexpr double cU = 8.0 * std::numeric_limits<float>::epsilon() / 2.0;  // c u
    EXPECT_NEAR(out.tCA, ref.tCA, cU * ((r_norm / v_norm) + std::abs(ref.tCA)));

    Eigen::Matrix<double, 6, 1> G;
    G.head(3).setConstant(1.0 / v_norm);
    G.tail(3).setConstant(r_norm / (v_norm * v_norm));
    double const Q = G.transpose() * filterCovariance.cwiseAbs() * G;
    double const sigma = ref.sigmaTca;
    EXPECT_NEAR(out.sigmaTca, sigma, (cU * (std::sqrt(Q) + sigma)) + (std::sqrt((sigma * sigma) + (cU * Q)) - sigma));
}

#endif
