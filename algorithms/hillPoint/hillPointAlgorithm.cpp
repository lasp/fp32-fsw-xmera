#include "hillPointAlgorithm.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

HillPointAlgorithm::HillPointAlgorithm(const HillPointConfig& config) : cfg(config) {}

void HillPointAlgorithm::setConfig(const HillPointConfig& config) { this->cfg = config; }

// NOLINTBEGIN(readability-convert-member-functions-to-static, bugprone-easily-swappable-parameters)
// readability-convert-member-functions-to-static: HillPointConfig is intentionally empty for this
// algorithm; the cfg member is held for API consistency with the standard two-phase-init pattern.
// bugprone-easily-swappable-parameters: the Vector3d position/velocity inputs are documented in
// the header and follow the standard (sc, planet) ordering.
HillPointOutput HillPointAlgorithm::update(const Eigen::Vector3d& r_BN_N,
                                           const Eigen::Vector3d& v_BN_N,
                                           const Eigen::Vector3d& r_planet_N,
                                           const Eigen::Vector3d& v_planet_N) const {
    // Position/velocity scale work stays in double to avoid losing precision in
    // difference-of-large-numbers (e.g. heliocentric vectors) and large products
    // like orbitRadius^2.
    const Eigen::Vector3d relPosVector = r_BN_N - r_planet_N;
    const Eigen::Vector3d relVelVector = v_BN_N - v_planet_N;

    // Hill-frame unit vectors -- magnitude 1 by construction, so float is fine.
    const Eigen::Vector3d i_r_d = relPosVector.normalized();
    const Eigen::Vector3d orbitAngMomentum = relPosVector.cross(relVelVector);
    const Eigen::Vector3d i_h_d = orbitAngMomentum.normalized();
    const Eigen::Vector3d i_theta_d = i_h_d.cross(i_r_d);

    // DCM from inertial frame N to Hill reference frame R, stored in float.
    Eigen::Matrix3f dcm_RN;
    dcm_RN.row(0) = i_r_d.cast<float>();
    dcm_RN.row(1) = i_theta_d.cast<float>();
    dcm_RN.row(2) = i_h_d.cast<float>();

    const double orbitRadius = relPosVector.norm();

    // Robustness threshold against divide-by-near-zero. Note the original Xmera comment claimed
    // "1 km" but the value is 1.0 in the same units as r_BN_N, which is meters.
    constexpr double minOrbitRadius_m = 1.0;

    double dfdt = 0.0;    // true anomaly rate
    double ddfdt2 = 0.0;  // true anomaly acceleration
    if (orbitRadius > minOrbitRadius_m) {
        dfdt = orbitAngMomentum.norm() / (orbitRadius * orbitRadius);
        ddfdt2 = -2.0 * relVelVector.dot(i_r_d) / orbitRadius * dfdt;
    }
    // else: degenerate geometry (radius below threshold) -- leave rates at zero rather than divide by ~0

    const Eigen::Vector3f omega_RN_R{0.0F, 0.0F, static_cast<float>(dfdt)};
    const Eigen::Vector3f domega_RN_R{0.0F, 0.0F, static_cast<float>(ddfdt2)};

    HillPointOutput out;
    out.sigma_RN = dcmToMrp(dcm_RN);
    out.omega_RN_N = dcm_RN.transpose() * omega_RN_R;
    out.domega_RN_N = dcm_RN.transpose() * domega_RN_R;
    return out;
}
// NOLINTEND(readability-convert-member-functions-to-static, bugprone-easily-swappable-parameters)
