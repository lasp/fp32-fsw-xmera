#include "thrusterPlatformReferenceAlgorithm.h"

#include <math.h>
#include <numbers>

#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"

namespace {
constexpr float kZeroTolerance = 1e-6F;  // module tolerance for treating a quantity as zero
constexpr float kSmallAngle = 1e-3F;     // small angle tolerance [rad]

/*! Compute the platform rotation [FM] aligning the thruster line of action with the system center of mass. */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- the vectors are distinct by frame and documented.
Eigen::Matrix3f alignThrustLine(const Eigen::Vector3f& r_CM_M,
                                const Eigen::Vector3f& r_TM_F,
                                const Eigen::Vector3f& tHat_F) {
    Eigen::Vector3f prv_FM = Eigen::Vector3f::Zero();  // zero rotation when the center of mass coincides with the joint

    const float b = r_CM_M.norm();
    if (b >= kZeroTolerance) {
        // thrust-ray / center-of-mass-sphere intersection (positive square-root branch)
        const float rt = r_TM_F.dot(tHat_F);
        const float ct = -rt + safeSqrtf((rt * rt) - r_TM_F.squaredNorm() + (b * b));
        const Eigen::Vector3f r_CtM_F = r_TM_F + (ct * tHat_F);

        // rotate r_CM_M onto the intersection point about the cross-product axis
        const Eigen::Vector3f rHat_CM_M = r_CM_M.stableNormalized();
        const Eigen::Vector3f rHat_CtM_F = r_CtM_F.stableNormalized();
        const float angle = safeAcosf(rHat_CM_M.dot(rHat_CtM_F));

        const Eigen::Vector3f e_axis = (std::numbers::pi_v<float> - angle < kSmallAngle)
                                           ? rHat_CM_M.unitOrthogonal()  // nearly opposite: any orthogonal axis
                                           : rHat_CtM_F.cross(rHat_CM_M);
        prv_FM = angle * e_axis.stableNormalized();
    }

    return prvToDcm(prv_FM);
}
}  // namespace

/*! @brief Construct the algorithm with a validated configuration and seed the runtime integrator state.
 @param config Validated configuration (geometry, gains, angle bounds, RW configuration).
*/
ThrusterPlatformReferenceAlgorithm::ThrusterPlatformReferenceAlgorithm(const ThrusterPlatformReferenceConfig& config)
    : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

/*! @brief Replace the stored configuration at runtime without disturbing the runtime integrator state.
 @param config New validated configuration to apply.
*/
void ThrusterPlatformReferenceAlgorithm::setConfig(const ThrusterPlatformReferenceConfig& config) {
    this->cfg = config;
}

/*! @brief Re-seed the runtime integrator state (RW momentum integral, prior sample) to its initial values. */
void ThrusterPlatformReferenceAlgorithm::reInitialize() {
    this->hsInt_M.setZero();
    this->priorHs_M.setZero();
}

/*! This method computes the platform reference orientation that aligns the thruster line of action with the system
 center of mass (optionally offset to dump reaction wheel momentum) and the associated body-heading, thruster-torque
 and thruster-configuration quantities.
 @return ThrusterPlatformReferenceOutput derived body-frame thruster quantities
 @param in per-cycle inputs read from the input messages
*/
ThrusterPlatformReferenceOutput ThrusterPlatformReferenceAlgorithm::update(const ThrusterPlatformReferenceInputs& in) {
    ThrusterPlatformReferenceOutput out{};

    const Eigen::Matrix3f dcm_MB = mrpToDcm(this->cfg.getSigma_MB());  // B to M DCM
    const Eigen::Vector3f r_CB_M = dcm_MB * in.r_CB_B;                 // position of C w.r.t. B in M-frame coordinates
    const Eigen::Vector3f r_CM_M = r_CB_M + this->cfg.getR_BM_M();     // position of C w.r.t. M in M-frame coordinates
    const Eigen::Vector3f r_TM_F = this->cfg.getR_FM_F() + in.r_TF_F;  // position of T w.r.t. M, F coordinates
    const Eigen::Vector3f thrust_F = in.thrust * in.tHat_F;            // thrust vector in F-frame coordinates
    const Eigen::Vector3f tHat_F = thrust_F.normalized();              // thrust unit direction, F frame

    Eigen::Matrix3f dcm_FM = alignThrustLine(r_CM_M, r_TM_F, tHat_F);

    if (this->cfg.getMomentumDumping()) {
        const ThrusterPlatformReferenceRwArrayConfiguration& rwConfig = this->cfg.getRwConfig();

        // compute net RW momentum in body frame
        Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
        for (uint32_t i = 0; i < rwConfig.numRW; ++i) {
            hs_B += rwConfig.JsList(i) * in.wheelSpeeds(i) * rwConfig.GsMatrix_B.col(i);
        }
        const Eigen::Vector3f hs_M = dcm_MB * hs_B;

        // update the trapezoidal integral of the RW momentum using the fixed control period as the time step
        const float dt = this->cfg.getControlPeriod();
        this->hsInt_M += 0.5F * dt * (this->priorHs_M + hs_M);
        this->priorHs_M = hs_M;

        // compute the offset vector that shifts the effective CM to produce the desired dumping torque
        const Eigen::Vector3f thrust_M = dcm_FM.transpose() * thrust_F;
        const Eigen::Vector3f H_M = (this->cfg.getK() * hs_M) + (this->cfg.getKi() * this->hsInt_M);
        const Eigen::Vector3f d_M = -1.0F / thrust_M.dot(thrust_M) * thrust_M.cross(H_M);

        // recompute the platform rotation about the offset CM
        const Eigen::Vector3f r_CdM_M = r_CM_M + d_M;
        dcm_FM = alignThrustLine(r_CdM_M, r_TM_F, tHat_F);
    }

    // mapping between the final platform frame and the body frame
    const Eigen::Matrix3f dcm_FB = dcm_FM * dcm_MB;

    // thruster torque on the system in body frame coordinates
    const Eigen::Vector3f r_CM_F = dcm_FM * r_CM_M;
    const Eigen::Vector3f r_TC_F = r_TM_F - r_CM_F;
    const Eigen::Vector3f Lreq_F = thrust_F.cross(r_TC_F);
    out.Lreq_B = dcm_FB.transpose() * Lreq_F;

    // thruster configuration in body frame coordinates
    const Eigen::Vector3f r_TC_B = dcm_FB.transpose() * r_TC_F;
    out.r_TB_B = in.r_CB_B + r_TC_B;
    out.tHat_B = (dcm_FB.transpose() * thrust_F).normalized();
    out.thrust = thrust_F.norm();

    return out;
}
