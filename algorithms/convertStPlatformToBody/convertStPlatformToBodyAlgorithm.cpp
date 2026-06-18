#include "convertStPlatformToBodyAlgorithm.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"

StAttitudeOutput ConvertStPlatformToBodyAlgorithm::update(const PlatformAttitude& platformAttitude,
                                                          const PlatformAngularVelocity& platformAngularRate) const {
    // Convert the star tracker inertial attitude from quaternion to MRP
    const Eigen::Vector4f ep_CN = cArrayToEigenVector(platformAttitude.q_CN);
    const Eigen::Vector3f sigma_CN = epToMrp(ep_CN);

    // Compute hub inertial attitude using specified dcm_CB
    const Eigen::Vector3f sigma_BC = dcmToMrp<float>(this->dcm_CB.transpose());
    const Eigen::Vector3f sigma_BN = addMrp(sigma_CN, sigma_BC);

    // Recover case-frame angular velocity from the incoming unit delta quaternion.
    // dq_CN = [sin(θ/2)·axis, cos(θ/2)] in scalar-last form. Using atan2 in place of
    // acos avoids the derivative blow-up of acos near ±1, preserving float32 relative
    // precision in the near-identity regime (cos(θ/2) ≈ 1).
    const float dqVecNorm = safeSqrtf((platformAngularRate.dq_CN[0] * platformAngularRate.dq_CN[0]) +
                                      (platformAngularRate.dq_CN[1] * platformAngularRate.dq_CN[1]) +
                                      (platformAngularRate.dq_CN[2] * platformAngularRate.dq_CN[2]));
    const float omegaScale =
        (dqVecNorm > 0.0F) ? 2.0F * safeAtan2f(dqVecNorm, platformAngularRate.dq_CN[3]) / dqVecNorm : 0.0F;
    const Eigen::Vector3f omega_CN_C = {platformAngularRate.dq_CN[0] * omegaScale,
                                        platformAngularRate.dq_CN[1] * omegaScale,
                                        platformAngularRate.dq_CN[2] * omegaScale};
    const Eigen::Vector3f omega_BN_B = this->dcm_CB.transpose() * omega_CN_C;

    StAttitudeOutput stAttOut{};
    stAttOut.timeTag = platformAttitude.timeTag;
    eigenVectorToCArray(sigma_BN, stAttOut.sigma_BN);
    eigenVectorToCArray(omega_BN_B, stAttOut.omega_BN_B);
    return stAttOut;
}

void ConvertStPlatformToBodyAlgorithm::setDcmCB(const Eigen::Matrix3f& dcm_CB) { this->dcm_CB = dcm_CB; }

const Eigen::Matrix3f& ConvertStPlatformToBodyAlgorithm::getDcmCB() const { return this->dcm_CB; }
