#include "thrusterPlatformReferenceAlgorithm.h"

#include <math.h>
#include <numbers>

#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"

namespace {
constexpr float kZeroTolerance = 1e-6F;  // module tolerance for treating a quantity as zero
constexpr float kSmallAngle = 1e-3F;     // small angle tolerance [rad]

/*! Compute the platform rotation [FM] that points the thruster so it produces the requested torque Lreq_F (the
 thruster torque about the system center of mass, r_TC x thrust) on the body. A zero requested torque aligns the
 thruster line of action through the center of mass. */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- the vectors are distinct by frame and documented.
Eigen::Matrix3f computeThrusterPointing(const Eigen::Vector3f& r_CM_M,
                                        const Eigen::Vector3f& r_TM_F,
                                        const Eigen::Vector3f& thrust_F,
                                        const Eigen::Vector3f& Lreq_F) {
    Eigen::Vector3f prv_FM = Eigen::Vector3f::Zero();  // zero rotation when the center of mass coincides with the joint

    const float b = r_CM_M.norm();  // distance from the joint M to the center of mass (unchanged by any rotation)
    if (b >= kZeroTolerance) {
        const Eigen::Vector3f tHat_F = thrust_F.normalized();

        // perpendicular arm (in F frame) of the requested-torque line: the CoM positions (in F frame) that give Lreq_F
        const Eigen::Vector3f r_lineM_F = thrust_F.cross(r_TM_F.cross(thrust_F) - Lreq_F) / thrust_F.squaredNorm();
        // r_CtM_F: intersect that line with the distance-b locus (arm r_lineM_F plus an offset along the thrust)
        const float distAlongThrust = safeSqrtf((b * b) - r_lineM_F.squaredNorm());
        const Eigen::Vector3f r_CtM_F = r_lineM_F + (distAlongThrust * tHat_F);

        // shortest rotation carrying r_CM_M onto r_CtM_F, about their cross-product axis
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

/*! Clamp the reference rotation so the thrust deflection from its neutral direction stays within the cone of
 half-angle thetaMax. */
Eigen::Matrix3f clampThrustDeflection(const Eigen::Matrix3f& dcm_FM, const Eigen::Vector3f& tHat_F, float thetaMax) {
    const Eigen::Vector3f& tHatNeutral_M = tHat_F;                  // neutral thrust direction (F == M), M coordinates
    const Eigen::Vector3f tHatRef_M = dcm_FM.transpose() * tHat_F;  // reference thrust direction, M coordinates
    const float deflection = safeAcosf(tHatNeutral_M.dot(tHatRef_M));

    Eigen::Matrix3f dcm_FcM = dcm_FM;  // clamped platform reference frame Fc; equals [FM] while within the cone
    if (deflection > thetaMax) {
        // rotate the platform reference from the aligned frame F to the clamped frame Fc, removing the excess
        // deflection (deflection - thetaMax) so the thrust lands on the cone.
        Eigen::Vector3f prvAxis_F = dcm_FM * (tHatRef_M.cross(tHatNeutral_M));
        if (std::numbers::pi_v<float> - deflection < kSmallAngle) {
            prvAxis_F = tHat_F.unitOrthogonal();  // nearly opposite: any orthogonal axis
        }
        const Eigen::Vector3f prv_FcF_F = (deflection - thetaMax) * prvAxis_F.stableNormalized();
        dcm_FcM = prvToDcm(prv_FcF_F) * dcm_FM;
    }
    return dcm_FcM;
}
}  // namespace

/*! @brief Advance the RW momentum integrator and return the momentum-dumping torque request in the platform frame.
 The desired thruster torque opposes the accumulated reaction-wheel momentum. It is converted from the body frame to
 the platform frame using the previous cycle's pointing, seeded once with the nominal pointing on the first cycle.
 @return requested thruster torque, platform-frame coordinates
*/
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- the vectors are distinct by frame and documented.
Eigen::Vector3f ThrusterPlatformReferenceAlgorithm::computeDumpingTorque(const ThrusterPlatformReferenceInputs& in,
                                                                         const Eigen::Vector3f& r_CM_M,
                                                                         const Eigen::Vector3f& r_TM_F,
                                                                         const Eigen::Vector3f& thrust_F,
                                                                         const Eigen::Matrix3f& dcm_MB) {
    const ThrusterPlatformReferenceRwArrayConfiguration& rwConfig = this->cfg.getRwConfig();

    // compute net RW momentum in body frame
    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
    for (uint32_t i = 0; i < rwConfig.numRW; ++i) {
        hs_B += rwConfig.JsList(i) * in.wheelSpeeds(i) * rwConfig.GsMatrix_B.col(i);
    }

    // update the trapezoidal integral of the RW momentum using the fixed control period as the time step
    const float dt = this->cfg.getControlPeriod();
    this->hsInt_B += 0.5F * dt * (this->priorHs_B + hs_B);
    this->priorHs_B = hs_B;

    // desired thruster torque about the center of mass: oppose the accumulated wheel momentum to dump it.
    const Eigen::Vector3f Lreq_B = -this->cfg.getK() * hs_B - this->cfg.getKi() * this->hsInt_B;

    // Torque is converted from the body frame into the platform frame using the previous cycle's pointing; on the
    // first cycle there is no prior, so seed it once with the nominal zero-torque pointing.
    if (this->priorDcm_FM.isZero()) {
        this->priorDcm_FM = computeThrusterPointing(r_CM_M, r_TM_F, thrust_F, Eigen::Vector3f::Zero());
    }
    return this->priorDcm_FM * dcm_MB * Lreq_B;
}

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

/*! @brief Re-seed the runtime state (RW momentum integral, prior sample, prior pointing DCM) to its initial values. */
void ThrusterPlatformReferenceAlgorithm::reInitialize() {
    this->hsInt_B.setZero();
    this->priorHs_B.setZero();
    this->priorDcm_FM.setZero();
}

/*! This method computes the platform reference orientation that points the thruster line of action through the
 system center of mass (or produces a torque to dump reaction-wheel momentum) and the associated body-heading,
 thruster-torque and thruster-configuration quantities.
 @return ThrusterPlatformReferenceOutput derived body-frame thruster quantities
 @param in per-cycle inputs read from the input messages
*/
ThrusterPlatformReferenceOutput ThrusterPlatformReferenceAlgorithm::update(const ThrusterPlatformReferenceInputs& in) {
    ThrusterPlatformReferenceOutput out{};

    const Eigen::Matrix3f dcm_MB = mrpToDcm(this->cfg.getSigma_MB());  // B to M DCM
    const Eigen::Vector3f r_CM_B = in.r_CB_B - this->cfg.getR_MB_B();  // position of C w.r.t. M, B coordinates
    const Eigen::Vector3f r_CM_M = dcm_MB * r_CM_B;                    // position of C w.r.t. M, M coordinates
    const Eigen::Vector3f r_TM_F = this->cfg.getR_FM_F() + in.r_TF_F;  // position of T w.r.t. M, F coordinates
    const Eigen::Vector3f thrust_F = in.thrust * in.tHat_F;            // thrust vector in F-frame coordinates

    // requested torque, platform frame (zero -> point through CM; non-zero when dumping reaction-wheel momentum)
    const Eigen::Vector3f Lreq_F = this->computeDumpingTorque(in, r_CM_M, r_TM_F, thrust_F, dcm_MB);

    Eigen::Matrix3f dcm_FM = computeThrusterPointing(r_CM_M, r_TM_F, thrust_F, Lreq_F);

    // limit the thrust deflection to the configured cone about its neutral direction
    dcm_FM = clampThrustDeflection(dcm_FM, in.tHat_F, this->cfg.getThetaMax());
    this->priorDcm_FM = dcm_FM;  // save for the next cycle's torque conversion

    // mapping between the final platform frame and the body frame
    const Eigen::Matrix3f dcm_FB = dcm_FM * dcm_MB;

    // thruster configuration in body frame coordinates
    const Eigen::Vector3f r_CM_F = dcm_FM * r_CM_M;
    const Eigen::Vector3f r_TC_F = r_TM_F - r_CM_F;
    const Eigen::Vector3f r_TC_B = dcm_FB.transpose() * r_TC_F;
    out.r_TB_B = in.r_CB_B + r_TC_B;
    out.tHat_B = (dcm_FB.transpose() * thrust_F).normalized();
    out.thrust = thrust_F.norm();

    return out;
}
