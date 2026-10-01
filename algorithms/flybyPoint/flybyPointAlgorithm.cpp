#include "flybyPointAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <Eigen/Geometry>
#include <numbers>

FlybyPointAlgorithm::FlybyPointAlgorithm(const FlybyPointConfig& config) : cfg(config) {}

void FlybyPointAlgorithm::setConfig(const FlybyPointConfig& config) { this->cfg = config; }

static constexpr double kRad2Deg = 180.0 / std::numbers::pi;
static constexpr double kMaxAccelCoeff = 3.0 * std::numbers::sqrt3 / 8.0;

/*! This method is used to reset the module.
 @return void
 */
void FlybyPointAlgorithm::reset() {
    this->firstRead = true;
    this->periodsSinceLastRead = 0;
    this->periodsSinceFirstRead = 0;
}

/*! This function computes a reference attitude frame for a spacecraft in relative motion about a small body.
 It must be called once per control period: time is counted in whole control periods since the last and the first
 filter read, and converted to seconds only for the guidance equations.
 @return AttGuideOutput containing reference attitude (sigma_RN, omega_RN_N, domega_RN_N) and validity flags
 @param r_BN_N The relative position state
 @param v_BN_N The relative velocity state
 */
AttGuideOutput FlybyPointAlgorithm::updateState(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    /*! advance the period counters first, so time keeps elapsing through calls that return early below */
    if (!this->firstRead) {
        ++this->periodsSinceLastRead;
        ++this->periodsSinceFirstRead;
    }

    /*! init diagnostic message */
    AttGuideOutput output{};
    if (r_BN_N.stableNorm() < 1e-3 || v_BN_N.stableNorm() < 1e-3) {
        return output;
    }
    /*! If this is the first read, seed the algorithm with the solution  */
    if (this->firstRead) {
        this->firstNavPosition = r_BN_N;
        this->firstNavVelocity = v_BN_N;
        this->computeFlybyParameters(r_BN_N, v_BN_N);
        this->computeRN(r_BN_N, v_BN_N);
        this->periodsSinceLastRead = 0;
        this->periodsSinceFirstRead = 0;
        this->firstRead = false;
    }
    /*! At the re-read cadence, protect against bad new solutions by checking validity */
    else if (this->periodsSinceLastRead >= this->cfg.getFilterReadPeriods() &&
             this->checkValidity(r_BN_N, v_BN_N, output)) {
        /*! update flyby parameters and guidance frame */
        this->computeFlybyParameters(r_BN_N, v_BN_N);
        this->computeRN(r_BN_N, v_BN_N);

        /*! restart the count from the new filter read */
        this->periodsSinceLastRead = 0;
    }
    /*! [s] time since the last accepted filter read */
    const double dt = static_cast<double>(this->periodsSinceLastRead) * this->cfg.getControlPeriod();
    auto [sigma_RN, omega_RN_N, omegaDot_RN_N] = this->computeGuidanceSolution(dt);
    output.sigma_RN = sigma_RN.cast<float>();
    output.omega_RN_N = omega_RN_N.cast<float>();
    output.domega_RN_N = omegaDot_RN_N.cast<float>();
    output.validOutput = output.sigma_RN.allFinite() && output.omega_RN_N.allFinite() && output.domega_RN_N.allFinite();
    return output;
}

void FlybyPointAlgorithm::computeFlybyParameters(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    this->f0 = v_BN_N.norm() / r_BN_N.norm();

    /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
    const Eigen::Vector3d ur_N = r_BN_N.normalized();
    const Eigen::Vector3d uv_N = v_BN_N.normalized();

    const Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    const Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

    // compute flight path angle at the time of read
    this->gamma0 = safeAtan2(v_BN_N.dot(ur_N), v_BN_N.dot(ut_N));
}

bool FlybyPointAlgorithm::checkValidity(const Eigen::Vector3d& r_BN_N,
                                        const Eigen::Vector3d& v_BN_N,
                                        AttGuideOutput& output) const {
    bool valid = true;
    const Eigen::Vector3d ur_N = r_BN_N.normalized();
    const Eigen::Vector3d uv_N = v_BN_N.normalized();

    /*! assert r and v are not collinear, parallel or anti-parallel (collision trajectory) */
    if (1.0 - fabs(ur_N.dot(uv_N)) < this->cfg.getToleranceForCollinearity()) {
        valid = false;
        output.collinearityTrigger = true;
    } else {
        output.collinearityTrigger = false;
    }

    /*! the predicted peak rate and acceleration occur near closest approach of the candidate's own rectilinear
     trajectory, at distance d_CA = |r x v| / |v| = |r| |cos(gamma)|: peak rate |v| / d_CA and peak acceleration
     3 sqrt(3) / 8 (|v| / d_CA)^2. An exactly collinear candidate has d_CA = 0, so its peaks are unbounded and exceed
     both limits */
    const double distanceClosestApproach = r_BN_N.cross(v_BN_N).norm() / v_BN_N.norm();
    bool rateExceeded = true;
    bool accelerationExceeded = true;
    if (distanceClosestApproach > 0.0) {
        const double speedOverDistance = v_BN_N.norm() / distanceClosestApproach;
        const double maxPredictedRate = speedOverDistance * kRad2Deg;
        rateExceeded =
            maxPredictedRate > this->cfg.getMaximumRateThreshold() && this->cfg.getMaximumRateThreshold() > 0;
        const double maxPredictedAcceleration = kMaxAccelCoeff * speedOverDistance * speedOverDistance * kRad2Deg;
        accelerationExceeded = maxPredictedAcceleration > this->cfg.getMaximumAccelerationThreshold() &&
                               this->cfg.getMaximumAccelerationThreshold() > 0;
    }

    /*! check if the predicted rate exceeds the maximum rate of the spacecraft */
    if (rateExceeded) {
        valid = false;
        output.maxRateTrigger = true;
    } else {
        output.maxRateTrigger = false;
    }

    /*! check if the predicted acceleration exceeds the maximum acceleration of the spacecraft */
    if (accelerationExceeded) {
        valid = false;
        output.maxAccelerationTrigger = true;
    } else {
        output.maxAccelerationTrigger = false;
    }

    /*! check if the position error exceeds a-priori sigma bound */
    const double deltaT = static_cast<double>(this->periodsSinceFirstRead) * this->cfg.getControlPeriod();
    const double deltaPositionNorm = (r_BN_N - (this->firstNavPosition + deltaT * this->firstNavVelocity)).norm();
    if (deltaPositionNorm > this->cfg.getPositionKnowledgeSigma() && this->cfg.getPositionKnowledgeSigma() > 0) {
        valid = false;
        output.positionKnowledgeExceedTrigger = true;
    } else {
        output.positionKnowledgeExceedTrigger = false;
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
    this->R0N.row(0) = ur_N.cast<float>();
    this->R0N.row(1) = ut_N.cast<float>();
    this->R0N.row(2) = uh_N.cast<float>();
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> FlybyPointAlgorithm::computeGuidanceSolution(
    const double dt) const {
    /*! compute DCM (RtR0) of reference frame from last read time */
    const double theta = safeAtan(safeTan(this->gamma0) + (this->f0 / safeCos(this->gamma0) * dt)) - this->gamma0;
    const Eigen::Vector3d PRV_theta{0, 0, theta};
    const Eigen::Matrix3d RtR0 = prvToDcm(PRV_theta);

    /*! compute DCM of reference frame at time t_0 + dt with respect to inertial frame */
    const Eigen::Matrix3d RtN = RtR0 * this->R0N.cast<double>();

    /*! compute scalar angular rate and acceleration of the reference frame in R-frame coordinates */
    const double den = ((this->f0 * this->f0 * dt * dt) + (2 * this->f0 * safeSin(this->gamma0) * dt) + 1);
    const double thetaDot = this->f0 * safeCos(this->gamma0) / den;
    const double thetaDDot =
        -2 * this->f0 * this->f0 * safeCos(this->gamma0) * (this->f0 * dt + safeSin(this->gamma0)) / (den * den);
    const Eigen::Vector3d omega_RN_R{0, 0, thetaDot};
    const Eigen::Vector3d omegaDot_RN_R{0, 0, thetaDDot};

    /*! populate attRefOut with reference frame information */
    Eigen::Vector3d sigma_RN = dcmToMrp(RtN);

    if (this->cfg.getSignOfOrbitNormalFrameVector() == -1) {
        Eigen::Vector3d const halfRotationX{1, 0, 0};
        sigma_RN = addMrp(sigma_RN, halfRotationX);
    }
    const Eigen::Vector3d omega_RN_N = RtN.transpose() * omega_RN_R;
    const Eigen::Vector3d omegaDot_RN_N = RtN.transpose() * omegaDot_RN_R;

    return {sigma_RN, omega_RN_N, omegaDot_RN_N};
}
