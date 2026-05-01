// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
// SPDX-License-Identifier: MIT

#include "hillPointAlgorithm.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

HillPointOutput HillPointAlgorithm::update(const Eigen::Vector3d& r_BN_N,
                                           const Eigen::Vector3d& v_BN_N,
                                           const Eigen::Vector3d& r_planet_N,
                                           const Eigen::Vector3d& v_planet_N) const {
    const Eigen::Vector3d relPosVector = r_BN_N - r_planet_N;
    const Eigen::Vector3d relVelVector = v_BN_N - v_planet_N;

    // DCM from inertial frame N to Hill reference frame R
    Eigen::Matrix3d dcm_RN;
    // first row i_r: radial unit vector
    dcm_RN.row(0) = relPosVector.normalized();
    // third row i_h: orbit angular momentum unit vector
    const Eigen::Vector3d orbitAngMomentum = relPosVector.cross(relVelVector);
    dcm_RN.row(2) = orbitAngMomentum.normalized();
    // second row i_theta = i_h x i_r completes the right-handed Hill frame
    dcm_RN.row(1) = dcm_RN.row(2).cross(dcm_RN.row(0));

    const double orbitRadius = relPosVector.norm();

    // Robustness threshold against divide-by-near-zero. Note the original Xmera comment claimed
    // "1 km" but the value is 1.0 in the same units as r_BN_N, which is meters.
    constexpr double minOrbitRadius_m = 1.0;

    double dfdt = 0.0;    // true anomaly rate
    double ddfdt2 = 0.0;  // true anomaly acceleration
    if (orbitRadius > minOrbitRadius_m) {
        dfdt = orbitAngMomentum.norm() / (orbitRadius * orbitRadius);
        ddfdt2 = -2.0 * relVelVector.dot(dcm_RN.row(0)) / orbitRadius * dfdt;
    }
    // else: degenerate geometry (radius below threshold) -- leave rates at zero rather than divide by ~0

    const Eigen::Vector3d omega_RN_R = {0.0, 0.0, dfdt};
    const Eigen::Vector3d domega_RN_R = {0.0, 0.0, ddfdt2};

    HillPointOutput out;
    out.sigma_RN = dcmToMrp(dcm_RN);
    out.omega_RN_N = dcm_RN.transpose() * omega_RN_R;
    out.domega_RN_N = dcm_RN.transpose() * domega_RN_R;
    return out;
}
