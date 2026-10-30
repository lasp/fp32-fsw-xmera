#include "stPlatformAttitudeAlgorithm.h"

StPlatformAttitudeAlgorithm::StPlatformAttitudeAlgorithm(const StPlatformAttitudeConfig& config) : cfg(config) {
    setConfig(config);
}

void StPlatformAttitudeAlgorithm::setConfig(const StPlatformAttitudeConfig& config) {
    this->cfg = config;
    this->sigma_BP = -this->cfg.getSigma_PB();
    this->dcm_BP = mrpToDcm(this->sigma_BP);
}

StAttitudeOutput StPlatformAttitudeAlgorithm::update(const Eigen::Vector4f& q_PN,
                                                     const Eigen::Vector4f& rateQuaternion_PN) const {
    // The sensor sends quaternions scalar-last; rigidBodyKinematics.hpp expects scalar-first.
    const auto toScalarFirst = [](const Eigen::Vector4f& q) { return Eigen::Vector4f{q[3], q[0], q[1], q[2]}; };

    const Eigen::Vector3f sigma_PN = epToMrp(toScalarFirst(q_PN));
    const Eigen::Vector3f sigma_BN = addMrp(sigma_PN, this->sigma_BP);

    // The rate quaternion is a rotation whose PRV is the angular velocity, without time integration.
    const Eigen::Vector3f omega_PN_P = epToPrv(toScalarFirst(rateQuaternion_PN));
    // Rigid mount (ω_BP_B = 0), so ω_BN_B = ω_PN_B; rotating ω_PN_P into body components gives ω_BN_B
    const Eigen::Vector3f omega_BN_B = this->dcm_BP * omega_PN_P;

    return {.sigma_BN = sigma_BN, .omega_BN_B = omega_BN_B};
}
