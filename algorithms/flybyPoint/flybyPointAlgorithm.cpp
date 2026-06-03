// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "flybyPointAlgorithm.h"
#include "utilities/safeMath.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

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
    /*! init diagnostic message */
    FlybyDiagnosticMsgPayload flybyDiagnosticMsgBuffer = {false, false, false, false};
    /*! compute dt from current time and last filter read time and get new states*/
    this->dt = (currentSimNanos - this->lastFilterReadTime) * NANO2SEC;
    if ((this->dt >= this->timeBetweenFilterData) || this->firstRead) {
        /*! If this is the first read, seed the algorithm with the solution  */
        if (this->firstRead) {
            this->timeOfFirstRead = currentSimNanos * NANO2SEC;
            this->firstNavPosition = r_BN_N;
            this->firstNavVelocity = v_BN_N;
            this->computeFlybyParameters(r_BN_N, v_BN_N);
            this->computeRN(r_BN_N, v_BN_N);
            this->firstRead = false;
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
    Eigen::Vector3d ur_N = r_BN_N.normalized();
    Eigen::Vector3d uv_N = v_BN_N.normalized();

    Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

    // gamma0 is a pure angle; computed in double for input precision, float storage is sufficient
    this->gamma0 = static_cast<float>(safeAtan(v_BN_N.dot(ur_N) / v_BN_N.dot(ut_N)));
}

bool FlybyPointAlgorithm::checkValidity(uint64_t currentSimNanos,
                                        const Eigen::Vector3d& r_BN_N,
                                        const Eigen::Vector3d& v_BN_N,
                                        FlybyDiagnosticMsgPayload& flybyDiagnosticMsgBuffer) const {
    bool valid = true;
    Eigen::Vector3d ur_N = r_BN_N.normalized();
    Eigen::Vector3d uv_N = v_BN_N.normalized();

    /*! assert r and v are not collinear (collision trajectory) */
    if (std::abs(1 - ur_N.dot(uv_N)) < this->toleranceForCollinearity) {
        valid = false;
        flybyDiagnosticMsgBuffer.collinearityTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.collinearityTrigger = false;
    }

    /*! check if the predicted rate exceeds the maximum rate of the spacecraft */
    double distanceClosestApproach = -r_BN_N.norm() * safeSinf(this->gamma0);
    double maxPredictedRate = v_BN_N.norm() / distanceClosestApproach * 180.0 / M_PI;
    if (maxPredictedRate > this->maxRate && this->maxRate > 0.0F) {
        valid = false;
        flybyDiagnosticMsgBuffer.maxRateTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.maxRateTrigger = false;
    }

    /*! check if the predicted acceleration exceeds the maximum acceleration of the spacecraft */
    const double angularRateAtCA = v_BN_N.norm() / distanceClosestApproach;
    double maxPredictedAcceleration = 3.0 * safeSqrt(3.0) / 8.0 * angularRateAtCA * angularRateAtCA * 180.0 / M_PI;
    if (maxPredictedAcceleration > this->maxAcceleration && this->maxAcceleration > 0) {
        valid = false;
        flybyDiagnosticMsgBuffer.maxAccelerationTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.maxAccelerationTrigger = false;
    }

    /*! check if the position error exceeds a-priori sigma bound */
    double deltaT = currentSimNanos * NANO2SEC - this->timeOfFirstRead;
    double deltaPositionNorm = (r_BN_N - (this->firstNavPosition + deltaT * this->firstNavVelocity)).norm();
    if (deltaPositionNorm > this->positionKnowledgeSigma && this->positionKnowledgeSigma > 0) {
        valid = false;
        flybyDiagnosticMsgBuffer.positionKnowledgeExceedTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.positionKnowledgeExceedTrigger = false;
    }

    return valid;
}

void FlybyPointAlgorithm::computeRN(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
    Eigen::Vector3d ur_N = r_BN_N.normalized();
    Eigen::Vector3d uv_N = v_BN_N.normalized();

    Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

    /*! compute inertial-to-reference DCM at time of read */
    // Unit vectors computed in double for input precision; dimensionless rows, float storage is sufficient
    this->R0N.row(0) = ur_N.cast<float>();
    this->R0N.row(1) = ut_N.cast<float>();
    this->R0N.row(2) = uh_N.cast<float>();
}

std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> FlybyPointAlgorithm::computeGuidanceSolution() const {
    // dt is a sub-second to sub-minute time delta; float resolution (~120 ns) is adequate for guidance
    const auto dt = static_cast<float>(this->dt);

    /*! compute DCM (RtR0) of reference frame from last read time */
    float theta = safeAtanf(safeTanf(this->gamma0) + this->f0 / safeCosf(this->gamma0) * dt) - this->gamma0;
    Eigen::Vector3f PRV_theta{0.0F, 0.0F, theta};
    Eigen::Matrix3f RtR0 = prvToDcm(PRV_theta);

    /*! compute DCM of reference frame at time t_0 + dt with respect to inertial frame */
    Eigen::Matrix3f RtN = RtR0 * this->R0N;

    /*! compute scalar angular rate and acceleration of the reference frame in R-frame coordinates */
    float den = (this->f0 * this->f0 * dt * dt + 2.0F * this->f0 * safeSinf(this->gamma0) * dt + 1.0F);
    float thetaDot = this->f0 * safeCosf(this->gamma0) / den;
    float thetaDDot =
        -2.0F * this->f0 * this->f0 * safeCosf(this->gamma0) * (this->f0 * dt + safeSinf(this->gamma0)) / (den * den);
    Eigen::Vector3f omega_RN_R{0.0F, 0.0F, thetaDot};
    Eigen::Vector3f omegaDot_RN_R{0.0F, 0.0F, thetaDDot};

    /*! populate attRefOut with reference frame information */
    Eigen::Vector3f sigma_RN = dcmToMrp(RtN);

    if (this->signOfOrbitNormalFrameVector == -1) {
        Eigen::Vector3f halfRotationX{1.0F, 0.0F, 0.0F};
        sigma_RN = addMrp(sigma_RN, halfRotationX);
    }
    Eigen::Vector3f omega_RN_N = RtN.transpose() * omega_RN_R;
    Eigen::Vector3f omegaDot_RN_N = RtN.transpose() * omegaDot_RN_R;

    return {sigma_RN, omega_RN_N, omegaDot_RN_N};
}

double FlybyPointAlgorithm::getTimeBetweenFilterData() const { return this->timeBetweenFilterData; }

void FlybyPointAlgorithm::setTimeBetweenFilterData(double time) { this->timeBetweenFilterData = time; }

float FlybyPointAlgorithm::getToleranceForCollinearity() const { return this->toleranceForCollinearity; }

void FlybyPointAlgorithm::setToleranceForCollinearity(float tolerance) { this->toleranceForCollinearity = tolerance; }

/*! Get the sign (+1 or -1) of the axis of rotation of the Z axis during the flyby
 @param int sign (+1 or -1)
 */
int FlybyPointAlgorithm::getSignOfOrbitNormalFrameVector() const { return this->signOfOrbitNormalFrameVector; }

/*! Set the sign (+1 or -1) of the axis of rotation of the Z axis during the flyby
 @param int sign (+1 or -1)
 */
void FlybyPointAlgorithm::setSignOfOrbitNormalFrameVector(int sign) { this->signOfOrbitNormalFrameVector = sign; }

/*! Get the maximum acceleration threshold to consider a solution invalid
 @return double maximum accceleration
 */
float FlybyPointAlgorithm::getMaximumAccelerationThreshold() const { return this->maxAcceleration; }

/*! Set the maximum acceleration threshold to consider a solution invalid
 @param float maximum accceleration
 */
void FlybyPointAlgorithm::setMaximumAccelerationThreshold(float maxAccelerationThreshold) {
    this->maxAcceleration = maxAccelerationThreshold;
}

/*! Get the maximum rate threshold to consider a solution invalid
 @return maximum rate
 */
float FlybyPointAlgorithm::getMaximumRateThreshold() const { return this->maxRate; }

/*! Set the maximum rate threshold to consider a solution invalid
 @param maximum rate
 */
void FlybyPointAlgorithm::setMaximumRateThreshold(float maxRateThreshold) { this->maxRate = maxRateThreshold; }

/*! Get the ground based positional knowledge standard deviation
 @return sigma
 */
float FlybyPointAlgorithm::getPositionKnowledgeSigma() const { return this->positionKnowledgeSigma; }

/*! Set the ground based positional knowledge sigma
 @param sigma
 */
void FlybyPointAlgorithm::setPositionKnowledgeSigma(float positionKnowledgeStd) {
    this->positionKnowledgeSigma = positionKnowledgeStd;
}
