#include "thrusterPlatformReferenceAlgorithm.h"

#include <math.h>
#include <numbers>

#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"

namespace {
constexpr float kZeroTolerance = 1e-6F;  // module tolerance for treating a quantity as zero

/*! Compute the first rotation that makes the thrust direction parallel to the center-of-mass direction. */
Eigen::Matrix3f tprComputeFirstRotation(const Eigen::Vector3f& tHat_F, const Eigen::Vector3f& rHat_CM_F) {
    float phi = safeAcosf(tHat_F.dot(rHat_CM_F));
    Eigen::Vector3f e_phi = tHat_F.cross(rHat_CM_F);
    // if phi = pi, e_phi can be any vector perpendicular to tHat_F
    if (fabsf(phi - std::numbers::pi_v<float>) < kZeroTolerance) {
        phi = std::numbers::pi_v<float>;
        if (fabsf(tHat_F(0)) > kZeroTolerance) {
            e_phi = Eigen::Vector3f(-(tHat_F(1) + tHat_F(2)) / tHat_F(0), 1.0F, 1.0F);
        } else if (fabsf(tHat_F(1)) > kZeroTolerance) {
            e_phi = Eigen::Vector3f(1.0F, -(tHat_F(0) + tHat_F(2)) / tHat_F(1), 1.0F);
        } else {
            e_phi = Eigen::Vector3f(1.0F, 1.0F, -(tHat_F(0) + tHat_F(1)) / tHat_F(2));
        }
    } else if (fabsf(phi) < kZeroTolerance) {
        phi = 0.0F;
    }
    e_phi.normalize();
    const Eigen::Vector3f prv = phi * e_phi;
    return prvToDcm(prv);
}

/*! Compute the second rotation that zeroes the offset between the thrust direction and the CM-to-thruster vector. */
Eigen::Matrix3f tprComputeSecondRotation(const Eigen::Vector3f& r_CM_F,
                                         const Eigen::Vector3f& r_TM_F,
                                         const Eigen::Vector3f& r_CT_F,
                                         const Eigen::Vector3f& tHat_F) {
    const float a = r_TM_F.norm();
    const float b = r_CM_F.norm();
    const float c1 = r_CT_F.norm();

    float psi = 0.0F;
    if (fabsf(a) >= kZeroTolerance) {
        const float beta = safeAcosf(-r_TM_F.dot(tHat_F) / a);
        const float nu = safeAcosf(-r_TM_F.dot(r_CT_F) / (a * c1));
        const float c2 = (a * safeCosf(beta)) + safeSqrtf((b * b) - (a * a * safeSinf(beta) * safeSinf(beta)));
        const float cosGamma1 = ((a * a) + (b * b) - (c1 * c1)) / (2.0F * a * b);
        const float cosGamma2 = ((a * a) + (b * b) - (c2 * c2)) / (2.0F * a * b);
        psi = safeAsinf(((c1 * safeSinf(nu) * cosGamma2) - (c2 * safeSinf(beta) * cosGamma1)) / b);
    }

    Eigen::Vector3f e_psi = tHat_F.cross(r_CT_F);
    e_psi.normalize();
    const Eigen::Vector3f prv = psi * e_psi;
    return prvToDcm(prv);
}

/*! Compute the third rotation making the frame compliant with the tip-and-tilt platform constraint. */
Eigen::Matrix3f tprComputeThirdRotation(const Eigen::Vector3f& e_theta, const Eigen::Matrix3f& dcm_F2M) {
    const float e1 = e_theta(0);
    const float e2 = e_theta(1);
    const float e3 = e_theta(2);

    const float A =
        (2.0F * ((dcm_F2M(1, 0) * e2 * e2) + (dcm_F2M(0, 0) * e1 * e2) + (dcm_F2M(2, 0) * e2 * e3))) - dcm_F2M(1, 0);
    const float B = 2.0F * ((dcm_F2M(2, 0) * e1) - (dcm_F2M(0, 0) * e3));
    const float C = dcm_F2M(1, 0);
    const float Delta = (B * B) - (4.0F * A * C);

    float theta = 0.0F;
    if (fabsf(A) < kZeroTolerance) {
        if (fabsf(B) < kZeroTolerance) {
            // zero-th order equation has no solution; the minimum problem is solved by theta = pi
            theta = std::numbers::pi_v<float>;
        } else {
            // first order equation
            theta = 2.0F * safeAtanf(-C / B);
        }
    } else if (Delta < 0.0F) {
        // second order equation has no solution; find the best solution of the minimum problem
        float t = 0.0F;
        if (fabsf(B) >= kZeroTolerance) {
            const float q = (A - C) / B;
            const float t1 = q + safeSqrtf((q * q) + 1.0F);
            const float t2 = q - safeSqrtf((q * q) + 1.0F);
            const float y1 = ((A * t1 * t1) + (B * t1) + C) / (1.0F + (t1 * t1));
            const float y2 = ((A * t2 * t2) + (B * t2) + C) / (1.0F + (t2 * t2));
            // choose the root that yields the smaller function value
            t = (fabsf(y2) < fabsf(y1)) ? t2 : t1;
        }
        theta = 2.0F * safeAtanf(t);
        const float y = ((A * t * t) + (B * t) + C) / (1.0F + (t * t));
        // check whether the absolute function minimum is at theta = pi
        if (fabsf(A) < fabsf(y)) {
            theta = std::numbers::pi_v<float>;
        }
    } else {
        const float t1 = (-B + safeSqrtf(Delta)) / (2.0F * A);
        const float t2 = (-B - safeSqrtf(Delta)) / (2.0F * A);
        const float t = (fabsf(t2) < fabsf(t1)) ? t2 : t1;
        theta = 2.0F * safeAtanf(t);
    }

    const Eigen::Vector3f prv = theta * e_theta;
    return prvToDcm(prv);
}

/*! Compose the three rotations into the platform-frame DCM that aligns the thruster with the center of mass. */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- the vectors are distinct by frame and documented.
Eigen::Matrix3f tprComputeFinalRotation(const Eigen::Vector3f& r_CM_M,
                                        const Eigen::Vector3f& r_TM_F,
                                        const Eigen::Vector3f& thrust_F) {
    const Eigen::Vector3f tHat_F = thrust_F.normalized();
    const Eigen::Vector3f rHat_CM_F = r_CM_M.normalized();  // assume zero initial rotation between F and M

    const Eigen::Matrix3f dcm_F1M = tprComputeFirstRotation(tHat_F, rHat_CM_F);
    const Eigen::Vector3f r_CM_F = dcm_F1M * r_CM_M;
    const Eigen::Vector3f r_CT_F = r_CM_F - r_TM_F;
    const Eigen::Matrix3f dcm_F2F1 = tprComputeSecondRotation(r_CM_F, r_TM_F, r_CT_F, tHat_F);
    const Eigen::Matrix3f dcm_F2M = dcm_F2F1 * dcm_F1M;
    const Eigen::Vector3f e_theta = (dcm_F2M * r_CM_M).normalized();
    const Eigen::Matrix3f dcm_F3F2 = tprComputeThirdRotation(e_theta, dcm_F2M);
    return dcm_F3F2 * dcm_F2M;
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
    this->priorTime = 0;
}

/*! This method computes the reference platform tip and tilt angles that align the thruster with the system center of
 mass (optionally offset to dump reaction wheel momentum) and the associated body-heading, thruster-torque and
 thruster-configuration quantities.
 @return ThrusterPlatformReferenceOutput reference angles and derived body-frame thruster quantities
 @param in per-cycle inputs read from the input messages
 @param callTime The clock time at which the function was called (nanoseconds)
*/
ThrusterPlatformReferenceOutput ThrusterPlatformReferenceAlgorithm::update(const ThrusterPlatformReferenceInputs& in,
                                                                           const uint64_t callTime) {
    ThrusterPlatformReferenceOutput out{};

    const Eigen::Matrix3f dcm_MB = mrpToDcm(this->cfg.getSigma_MB());  // B to M DCM
    const Eigen::Vector3f r_CB_M = dcm_MB * in.r_CB_B;                 // position of C w.r.t. B in M-frame coordinates
    const Eigen::Vector3f r_CM_M = r_CB_M + this->cfg.getR_BM_M();     // position of C w.r.t. M in M-frame coordinates
    const Eigen::Vector3f r_TM_F = this->cfg.getR_FM_F() + in.r_TF_F;  // position of T w.r.t. M, F coordinates
    const Eigen::Vector3f thrust_F = in.thrust * in.tHat_F;            // thrust vector in F-frame coordinates

    Eigen::Matrix3f dcm_FM = tprComputeFinalRotation(r_CM_M, r_TM_F, thrust_F);

    if (this->cfg.getMomentumDumping()) {
        const ThrusterPlatformReferenceRwArrayConfiguration& rwConfig = this->cfg.getRwConfig();

        // compute net RW momentum in body frame
        Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
        for (uint32_t i = 0; i < rwConfig.numRW; ++i) {
            hs_B += rwConfig.JsList(i) * in.wheelSpeeds(i) * rwConfig.GsMatrix_B.col(i);
        }
        const Eigen::Vector3f hs_M = dcm_MB * hs_B;

        // update the trapezoidal integral of the RW momentum (dt is zero on the first call)
        const float dt = (this->priorTime == 0) ? 0.0F : static_cast<float>(callTime - this->priorTime) * kNano2SecF;
        this->hsInt_M += 0.5F * dt * (this->priorHs_M + hs_M);
        this->priorHs_M = hs_M;
        this->priorTime = callTime;

        // compute the offset vector that shifts the effective CM to produce the desired dumping torque
        const Eigen::Vector3f thrust_M = dcm_FM.transpose() * thrust_F;
        Eigen::Vector3f H_M = this->cfg.getK() * hs_M;
        if (this->cfg.getKi() > 0.0F) {
            H_M += this->cfg.getKi() * this->hsInt_M;
        }
        const Eigen::Vector3f d_M = -1.0F / thrust_M.dot(thrust_M) * thrust_M.cross(H_M);

        // recompute the platform rotation about the offset CM
        const Eigen::Vector3f r_CdM_M = r_CM_M + d_M;
        dcm_FM = tprComputeFinalRotation(r_CdM_M, r_TM_F, thrust_F);
    }

    float theta1 = safeAtan2f(dcm_FM(1, 2), dcm_FM(1, 1));
    float theta2 = safeAtan2f(dcm_FM(2, 0), dcm_FM(0, 0));

    // bound the reference angles between the allowed limits
    const float theta1Max = this->cfg.getTheta1Max();
    const float theta2Max = this->cfg.getTheta2Max();
    if ((theta1Max > kZeroTolerance) && (theta1 > theta1Max)) {
        theta1 = theta1Max;
    } else if ((theta1Max > kZeroTolerance) && (theta1 < -theta1Max)) {
        theta1 = -theta1Max;
    }
    if ((theta2Max > kZeroTolerance) && (theta2 > theta2Max)) {
        theta2 = theta2Max;
    } else if ((theta2Max > kZeroTolerance) && (theta2 < -theta2Max)) {
        theta2 = -theta2Max;
    }

    // rebuild the platform DCM with the bounded angles
    dcm_FM = eulerAngles123ToDcm(Eigen::Vector3f(theta1, theta2, 0.0F));

    out.theta1 = theta1;
    out.theta2 = theta2;

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
