#include "convertStPlatformToBodyAlgorithm.h"

#include "utilities/fsw/safeMath.h"

ConvertStPlatformToBodyAlgorithm::ConvertStPlatformToBodyAlgorithm(const ConvertStPlatformToBodyConfig& config)
    : cfg(config) {
    setConfig(config);
}

void ConvertStPlatformToBodyAlgorithm::setConfig(const ConvertStPlatformToBodyConfig& config) {
    this->cfg = config;
    this->sigma_BP = -this->cfg.getSigma_PB();
    this->dcm_BP = mrpToDcm(this->sigma_BP);
}

StAttitudeOutput ConvertStPlatformToBodyAlgorithm::update(const Eigen::Vector4f& q_PN,
                                                          const Eigen::Vector4f& dq_PN) const {
    // q_PN is scalar-last; epToMrp expects scalar-first.
    const Eigen::Vector4f ep_PN{q_PN[3], q_PN[0], q_PN[1], q_PN[2]};
    const Eigen::Vector3f sigma_PN = epToMrp(ep_PN);
    const Eigen::Vector3f sigma_BN = addMrp(sigma_PN, this->sigma_BP);

    // dq_PN angle element is the rate in rad/s; atan2 instead of acos keeps float32 precision
    const float dqVecNorm = safeSqrtf((dq_PN[0] * dq_PN[0]) + (dq_PN[1] * dq_PN[1]) + (dq_PN[2] * dq_PN[2]));
    const float omegaScale = (dqVecNorm > 0.0F) ? 2.0F * safeAtan2f(dqVecNorm, dq_PN[3]) / dqVecNorm : 0.0F;
    const Eigen::Vector3f omega_PN_P = {dq_PN[0] * omegaScale, dq_PN[1] * omegaScale, dq_PN[2] * omegaScale};
    // Rigid mount (ω_BP_B = 0), so ω_BN_B = ω_PN_B; rotating ω_PN_P into body components gives ω_BN_B
    const Eigen::Vector3f omega_BN_B = this->dcm_BP * omega_PN_P;

    StAttitudeOutput stAttOut{};
    stAttOut.sigma_BN = sigma_BN;
    stAttOut.omega_BN_B = omega_BN_B;
    return stAttOut;
}
