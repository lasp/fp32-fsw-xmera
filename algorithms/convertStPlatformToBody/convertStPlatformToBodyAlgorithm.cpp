#include "convertStPlatformToBodyAlgorithm.h"

#include "../utilities/fsw/safeMath.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

ConvertStPlatformToBodyAlgorithm::ConvertStPlatformToBodyAlgorithm(const ConvertStPlatformToBodyConfig& config)
    : cfg(config) {
    setConfig(config);
}

void ConvertStPlatformToBodyAlgorithm::setConfig(const ConvertStPlatformToBodyConfig& config) { this->cfg = config; }

StAttitudeOutput ConvertStPlatformToBodyAlgorithm::update(const Eigen::Vector4f& q_CN,
                                                          const Eigen::Vector4f& dq_CN) const {
    const Eigen::Matrix3f dcm_CB = this->cfg.getDcmCB();

    // Convert the star tracker inertial attitude from quaternion to MRP, then offset by the mounting DCM
    const Eigen::Vector3f sigma_CN = epToMrp(q_CN);
    const Eigen::Vector3f sigma_BC = dcmToMrp<float>(dcm_CB.transpose());
    const Eigen::Vector3f sigma_BN = addMrp(sigma_CN, sigma_BC);

    // Recover case-frame angular velocity from the incoming unit delta quaternion.
    // dq_CN = [sin(θ/2)·axis, cos(θ/2)] in scalar-last form. Using atan2 in place of
    // acos avoids the derivative blow-up of acos near ±1, preserving float32 relative
    // precision in the near-identity regime (cos(θ/2) ≈ 1).
    const float dqVecNorm = safeSqrtf((dq_CN[0] * dq_CN[0]) + (dq_CN[1] * dq_CN[1]) + (dq_CN[2] * dq_CN[2]));
    const float omegaScale = (dqVecNorm > 0.0F) ? 2.0F * safeAtan2f(dqVecNorm, dq_CN[3]) / dqVecNorm : 0.0F;
    const Eigen::Vector3f omega_CN_C = {dq_CN[0] * omegaScale, dq_CN[1] * omegaScale, dq_CN[2] * omegaScale};
    const Eigen::Vector3f omega_BN_B = dcm_CB.transpose() * omega_CN_C;

    StAttitudeOutput stAttOut{};
    stAttOut.sigma_BN = sigma_BN;
    stAttOut.omega_BN_B = omega_BN_B;
    return stAttOut;
}
