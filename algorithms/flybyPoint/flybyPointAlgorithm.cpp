// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "flybyPointAlgorithm.h"
#include "utilities/safeMath.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

FlybyPointAlgorithm::FlybyPointAlgorithm(const FlybyPointConfig& config) : cfg(config) {}

void FlybyPointAlgorithm::setConfig(const FlybyPointConfig& config) { this->cfg = config; }

/*! This method is used to reset the module.
 @return void
 */
void FlybyPointAlgorithm::reset() {
    this->lastFilterReadTime = 0;
    this->firstRead = true;
}

/*! This function computes a reference attitude frame for a spacecraft in relative motion about a small body.
 @return AttRefMsgPayload
 @param currentSimNanos The current simulation time for system
 @param r_BN_N The relative position state
 @param v_BN_N The relative velocity state
 */
FlybyPointOutput FlybyPointAlgorithm::updateState(uint64_t currentSimNanos,
                                                  const Eigen::Vector3d& r_BN_N,
                                                  const Eigen::Vector3d& v_BN_N) {
    constexpr double eps = std::numeric_limits<double>::epsilon();
    if (r_BN_N.squaredNorm() < eps || v_BN_N.squaredNorm() < eps) {
        FSW_THROW_INVALID_ARGUMENT("inputs r and v must be non-zero");
    }

    /*! init diagnostic message */
    FlybyDiagnosticMsgPayload flybyDiagnosticMsgBuffer = {false, false, false, false};
    /*! compute dt from current time and last filter read time and get new states*/
    this->dt = static_cast<double>(currentSimNanos - this->lastFilterReadTime) * NANO2SEC;
    if ((this->dt >= this->cfg.getTimeBetweenFilterData()) || this->firstRead) {
        /*! If this is the first read, seed the algorithm with the solution  */
        if (this->firstRead) {
            if (this->checkValidityFirstRead(r_BN_N, v_BN_N, flybyDiagnosticMsgBuffer)) {
                this->timeOfFirstRead = static_cast<double>(currentSimNanos) * NANO2SEC;
                this->firstNavPosition = r_BN_N;
                this->firstNavVelocity = v_BN_N;
                this->computeFlybyParameters(r_BN_N, v_BN_N);
                this->computeRN(r_BN_N, v_BN_N);
                this->firstRead = false;
            }
        }
        /*! Protect against bad new solutions by checking validity */
        else if (this->checkValidity(currentSimNanos, r_BN_N, v_BN_N, flybyDiagnosticMsgBuffer)) {
            /*! update flyby parameters and guidance frame */
            this->computeFlybyParameters(r_BN_N, v_BN_N);
            this->computeRN(r_BN_N, v_BN_N);

            /*! update lastFilterReadTime to current time and dt to zero */
            this->lastFilterReadTime = currentSimNanos;
            this->dt = 0;
        }
    }
    auto [sigma_RN, omega_RN_N, omegaDot_RN_N] = this->computeGuidanceSolution();
    FlybyPointOutput output{};
    output.sigma_RN = sigma_RN;
    output.omega_RN_N = omega_RN_N;
    output.domega_RN_N = omegaDot_RN_N;
    output.collinearityTrigger = flybyDiagnosticMsgBuffer.collinearityTrigger;
    output.maxRateTrigger = flybyDiagnosticMsgBuffer.maxRateTrigger;
    output.maxAccelerationTrigger = flybyDiagnosticMsgBuffer.maxAccelerationTrigger;
    output.positionKnowledgeExceedTrigger = flybyDiagnosticMsgBuffer.positionKnowledgeExceedTrigger;
    return output;
}

void FlybyPointAlgorithm::computeFlybyParameters(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    // f0 derived from large-magnitude double inputs; ratio cancels scale, float precision suffices
    this->f0 = static_cast<float>(v_BN_N.norm() / r_BN_N.norm());

    /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
    const Eigen::Vector3d ur_N = r_BN_N.normalized();
    const Eigen::Vector3d uv_N = v_BN_N.normalized();

    const Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    const Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

    // gamma0 is a pure angle; computed in double for input precision, float storage is sufficient
    this->gamma0 = static_cast<float>(safeAtan(v_BN_N.dot(ur_N) / v_BN_N.dot(ut_N)));
}

bool FlybyPointAlgorithm::checkValidityFirstRead(const Eigen::Vector3d& r_BN_N,
                                                 const Eigen::Vector3d& v_BN_N,
                                                 FlybyDiagnosticMsgPayload& flybyDiagnosticMsgBuffer) const {
    bool valid = true;
    const Eigen::Vector3d ur_N = r_BN_N.normalized();
    const Eigen::Vector3d uv_N = v_BN_N.normalized();

    /*! assert r and v are not collinear (collision trajectory) */
    if (std::abs(1 - ur_N.dot(uv_N)) < this->cfg.getToleranceForCollinearity()) {
        valid = false;
        flybyDiagnosticMsgBuffer.collinearityTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.collinearityTrigger = false;
    }
    return valid;
}

bool FlybyPointAlgorithm::checkValidity(uint64_t currentSimNanos,
                                        const Eigen::Vector3d& r_BN_N,
                                        const Eigen::Vector3d& v_BN_N,
                                        FlybyDiagnosticMsgPayload& flybyDiagnosticMsgBuffer) const {
    bool valid = true;
    const Eigen::Vector3d ur_N = r_BN_N.normalized();
    const Eigen::Vector3d uv_N = v_BN_N.normalized();

    /*! assert r and v are not collinear (collision trajectory) */
    if (std::abs(1 - ur_N.dot(uv_N)) < this->cfg.getToleranceForCollinearity()) {
        valid = false;
        flybyDiagnosticMsgBuffer.collinearityTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.collinearityTrigger = false;
    }

    /*! check if the predicted rate exceeds the maximum rate of the spacecraft */
    const double distanceClosestApproach = -r_BN_N.norm() * safeSinf(this->gamma0);
    if (std::abs(distanceClosestApproach) < std::numeric_limits<double>::epsilon()) {
        // gamma0 ≈ 0: spacecraft is at or past closest approach — predicted rates and accelerations
        // are unbounded, so both checks trigger regardless of the configured thresholds
        valid = false;
        flybyDiagnosticMsgBuffer.maxRateTrigger = true;
        flybyDiagnosticMsgBuffer.maxAccelerationTrigger = true;
    } else {
        const double maxPredictedRate = v_BN_N.norm() / distanceClosestApproach * 180.0 / M_PI;
        if (maxPredictedRate > this->cfg.getMaxRateThreshold()) {
            valid = false;
            flybyDiagnosticMsgBuffer.maxRateTrigger = true;
        } else {
            flybyDiagnosticMsgBuffer.maxRateTrigger = false;
        }

        /*! check if the predicted acceleration exceeds the maximum acceleration of the spacecraft */
        const double angularRateAtCA = v_BN_N.norm() / distanceClosestApproach;
        const double maxPredictedAcceleration =
            3.0 * safeSqrt(3.0) / 8.0 * angularRateAtCA * angularRateAtCA * 180.0 / M_PI;
        if (maxPredictedAcceleration > this->cfg.getMaxAccelerationThreshold()) {
            valid = false;
            flybyDiagnosticMsgBuffer.maxAccelerationTrigger = true;
        } else {
            flybyDiagnosticMsgBuffer.maxAccelerationTrigger = false;
        }
    }

    /*! check if the position error exceeds a-priori sigma bound */
    const double deltaT = (static_cast<double>(currentSimNanos) * NANO2SEC) - this->timeOfFirstRead;
    const double deltaPositionNorm = (r_BN_N - (this->firstNavPosition + deltaT * this->firstNavVelocity)).norm();
    if (deltaPositionNorm > this->cfg.getPositionKnowledgeSigma()) {
        valid = false;
        flybyDiagnosticMsgBuffer.positionKnowledgeExceedTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.positionKnowledgeExceedTrigger = false;
    }

    return valid;
}

void FlybyPointAlgorithm::computeRN(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
    const Eigen::Vector3d ur_N = r_BN_N.normalized();
    const Eigen::Vector3d uv_N = v_BN_N.normalized();

    const Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    const Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

    /*! compute inertial-to-reference DCM at time of read */
    // Unit vectors computed in double for input precision; dimensionless rows, float storage is sufficient
    this->R0N.row(0) = ur_N.cast<float>();
    this->R0N.row(1) = ut_N.cast<float>();
    this->R0N.row(2) = uh_N.cast<float>();
}

std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> FlybyPointAlgorithm::computeGuidanceSolution() const {
    // dt is a sub-second to sub-minute time delta; float resolution (~120 ns) is adequate for guidance
    const auto dtF32 = static_cast<float>(this->dt);

    /*! compute DCM (RtR0) of reference frame from last read time */
    const float theta = safeAtanf(safeTanf(this->gamma0) + (this->f0 / safeCosf(this->gamma0) * dtF32)) - this->gamma0;
    const Eigen::Vector3f PRV_theta{0.0F, 0.0F, theta};
    const Eigen::Matrix3f RtR0 = prvToDcm(PRV_theta);

    /*! compute DCM of reference frame at time t_0 + dt with respect to inertial frame */
    Eigen::Matrix3f RtN = RtR0 * this->R0N;

    /*! compute scalar angular rate and acceleration of the reference frame in R-frame coordinates */
    const float den =
        ((this->f0 * this->f0 * dtF32 * dtF32) + (2.0F * this->f0 * safeSinf(this->gamma0) * dtF32) + 1.0F);
    const float thetaDot = this->f0 * safeCosf(this->gamma0) / den;
    const float thetaDDot = -2.0F * this->f0 * this->f0 * safeCosf(this->gamma0) *
                            (this->f0 * dtF32 + safeSinf(this->gamma0)) / (den * den);
    const Eigen::Vector3f omega_RN_R{0.0F, 0.0F, thetaDot};
    const Eigen::Vector3f omegaDot_RN_R{0.0F, 0.0F, thetaDDot};

    /*! populate attRefOut with reference frame information */
    Eigen::Vector3f sigma_RN = dcmToMrp(RtN);

    if (this->cfg.getSignOfOrbitNormalFrameVector() == -1) {
        const Eigen::Vector3f halfRotationX{1.0F, 0.0F, 0.0F};
        sigma_RN = addMrp(sigma_RN, halfRotationX);
    }
    const Eigen::Vector3f omega_RN_N = RtN.transpose() * omega_RN_R;
    const Eigen::Vector3f omegaDot_RN_N = RtN.transpose() * omegaDot_RN_R;

    return {sigma_RN, omega_RN_N, omegaDot_RN_N};
}
