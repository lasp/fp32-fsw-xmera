#include "stPlatformAttitudeAlgorithm.h"

#include "utilities/fsw/safeMath.h"

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
    // q_PN is scalar-last; epToMrp expects scalar-first.
    const Eigen::Vector4f ep_PN{q_PN[3], q_PN[0], q_PN[1], q_PN[2]};
    const Eigen::Vector3f sigma_PN = epToMrp(ep_PN);
    const Eigen::Vector3f sigma_BN = addMrp(sigma_PN, this->sigma_BP);

    // rateQuaternion_PN angle element is the rate in rad/s; atan2 instead of acos keeps float32 precision
    const float rateQuaternionVecNorm =
        safeSqrtf((rateQuaternion_PN[0] * rateQuaternion_PN[0]) + (rateQuaternion_PN[1] * rateQuaternion_PN[1]) +
                  (rateQuaternion_PN[2] * rateQuaternion_PN[2]));
    const float omegaScale =
        (rateQuaternionVecNorm > 0.0F)
            ? 2.0F * safeAtan2f(rateQuaternionVecNorm, rateQuaternion_PN[3]) / rateQuaternionVecNorm
            : 0.0F;
    const Eigen::Vector3f omega_PN_P = {
        rateQuaternion_PN[0] * omegaScale, rateQuaternion_PN[1] * omegaScale, rateQuaternion_PN[2] * omegaScale};
    // Rigid mount (ω_BP_B = 0), so ω_BN_B = ω_PN_B; rotating ω_PN_P into body components gives ω_BN_B
    const Eigen::Vector3f omega_BN_B = this->dcm_BP * omega_PN_P;

    return {.sigma_BN = sigma_BN, .omega_BN_B = omega_BN_B};
}
