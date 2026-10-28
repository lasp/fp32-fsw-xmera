#include "convertStPlatformToBodyAlgorithm.h"

#include "utilities/fsw/safeMath.h"

ConvertStPlatformToBodyAlgorithm::ConvertStPlatformToBodyAlgorithm(const ConvertStPlatformToBodyConfig& config)
    : cfg(config) {
    setConfig(config);
}

void ConvertStPlatformToBodyAlgorithm::setConfig(const ConvertStPlatformToBodyConfig& config) {
    this->cfg = config;
    this->sigma_BC = -this->cfg.getSigma_CB();
    this->dcm_BC = mrpToDcm(this->sigma_BC);
}

StAttitudeOutput ConvertStPlatformToBodyAlgorithm::update(const Eigen::Vector4f& q_CN,
                                                          const Eigen::Vector4f& dq_CN) const {
    // q_CN is scalar-last; epToMrp expects scalar-first.
    const Eigen::Vector4f ep_CN{q_CN[3], q_CN[0], q_CN[1], q_CN[2]};
    const Eigen::Vector3f sigma_CN = epToMrp(ep_CN);
    const Eigen::Vector3f sigma_BN = addMrp(sigma_CN, this->sigma_BC);

    // dq_CN angle element is the rate in rad/s; atan2 instead of acos keeps float32 precision
    const float dqVecNorm = safeSqrtf((dq_CN[0] * dq_CN[0]) + (dq_CN[1] * dq_CN[1]) + (dq_CN[2] * dq_CN[2]));
    const float omegaScale = (dqVecNorm > 0.0F) ? 2.0F * safeAtan2f(dqVecNorm, dq_CN[3]) / dqVecNorm : 0.0F;
    const Eigen::Vector3f omega_CN_C = {dq_CN[0] * omegaScale, dq_CN[1] * omegaScale, dq_CN[2] * omegaScale};
    // Rigid mount (ω_BC_B = 0), so ω_BN_B = ω_CN_B; rotating ω_CN_C into body components gives ω_BN_B
    const Eigen::Vector3f omega_BN_B = this->dcm_BC * omega_CN_C;

    StAttitudeOutput stAttOut{};
    stAttOut.sigma_BN = sigma_BN;
    stAttOut.omega_BN_B = omega_BN_B;
    return stAttOut;
}
