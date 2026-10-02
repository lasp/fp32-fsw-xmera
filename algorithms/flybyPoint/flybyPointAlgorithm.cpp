#include "flybyPointAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <Eigen/Geometry>
#include <numbers>
#include <optional>

FlybyPointAlgorithm::FlybyPointAlgorithm(const FlybyPointConfig& config) : cfg(config) {}

/*! Replace the stored configuration at runtime. The partial averaging window is discarded: its samples were
 propagated to the end of a window of the old length, which a new filterReadPeriods would move.
 @param config The validated configuration to install
 */
void FlybyPointAlgorithm::setConfig(const FlybyPointConfig& config) {
    this->cfg = config;
    this->window = {};
}

static constexpr double kRad2Deg = 180.0 / std::numbers::pi;
static constexpr double kMaxAccelCoeff = 3.0 * std::numbers::sqrt3 / 8.0;

namespace {
/*! Check whether a filter sample can seed the profile or enter the averaging window: it must be finite, neither r
 nor v may be (near) zero, and the products formed from it must be finite, |r||v| (closest approach) and
 f0^2 = (|v|/|r|)^2 (frame acceleration).
 @return true if the sample is usable
 @param r_BN_N [m] relative position
 @param v_BN_N [m/s] relative velocity
 */
bool isUsableSample(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    const double rNorm = r_BN_N.stableNorm();
    const double vNorm = v_BN_N.stableNorm();
    return r_BN_N.allFinite() && v_BN_N.allFinite() && rNorm >= 1e-3 && vNorm >= 1e-3 &&
           fsw::is_finite(rNorm * vNorm) && fsw::is_finite((vNorm / rNorm) * (vNorm / rNorm));
}

/*! Copy the validity checks of a re-read attempt into the diagnostic output.
 @return void
 @param output The output whose trigger flags are set
 @param triggers The checks that rejected the candidate; all false if none did
 */
void applyTriggers(AttGuideOutput& output, const FlybyValidityTriggers& triggers) {
    output.collinearityTrigger = triggers.collinearityTrigger;
    output.maxRateTrigger = triggers.maxRateTrigger;
    output.maxAccelerationTrigger = triggers.maxAccelerationTrigger;
    output.positionKnowledgeExceedTrigger = triggers.positionKnowledgeExceedTrigger;
}

/*! Check whether r and v are collinear (parallel or anti-parallel, a collision course): 1 - |cos| of their angle is
 below toleranceForCollinearity, or |r_hat x v_hat| is below FlybyPointAlgorithm::kMinOrbitNormalNorm, below which the
 orbit-normal direction would be dominated by rounding (only reachable with a toleranceForCollinearity below double
 resolution).
 @return true if r and v are collinear
 @param r_BN_N [m] relative position
 @param v_BN_N [m/s] relative velocity
 @param toleranceForCollinearity [-] tolerance on 1 - |cos| of the r-v angle
 */
bool isCollinear(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N, const float toleranceForCollinearity) {
    const Eigen::Vector3d ur_N = r_BN_N.normalized();
    const Eigen::Vector3d uv_N = v_BN_N.normalized();
    return 1.0 - fabs(ur_N.dot(uv_N)) < toleranceForCollinearity ||
           ur_N.cross(uv_N).norm() < FlybyPointAlgorithm::kMinOrbitNormalNorm;
}

/*! Check a re-read candidate (r, v) against the configured thresholds and against the rectilinear prediction from the
 first read.
 @return std::nullopt if the candidate passes every check, otherwise the checks that rejected it
 @param r_BN_N [m] candidate relative position
 @param v_BN_N [m/s] candidate relative velocity
 @param rPredicted_BN_N [m] rectilinear prediction of the position at the candidate's time, made from the first read
 @param cfg validated configuration (collinearity tolerance and the rate, acceleration and position thresholds)
 */
std::optional<FlybyValidityTriggers> checkValidity(const Eigen::Vector3d& r_BN_N,
                                                   const Eigen::Vector3d& v_BN_N,
                                                   const Eigen::Vector3d& rPredicted_BN_N,
                                                   const FlybyPointConfig& cfg) {
    FlybyValidityTriggers triggers{};
    triggers.collinearityTrigger = isCollinear(r_BN_N, v_BN_N, cfg.getToleranceForCollinearity());

    /*! the predicted peak rate and acceleration occur near closest approach of the candidate's own rectilinear
     trajectory, at distance d_CA = |r x v| / |v| = |r| |cos(gamma)|: peak rate |v| / d_CA and peak acceleration
     3 sqrt(3) / 8 (|v| / d_CA)^2, compared with the spacecraft limits. An exactly collinear candidate has d_CA = 0, so
     its peaks are unbounded and exceed both limits */
    const double distanceClosestApproach = r_BN_N.cross(v_BN_N).norm() / v_BN_N.norm();
    if (distanceClosestApproach > 0.0) {
        const double speedOverDistance = v_BN_N.norm() / distanceClosestApproach;
        triggers.maxRateTrigger = speedOverDistance * kRad2Deg > cfg.getMaximumRateThreshold();
        triggers.maxAccelerationTrigger =
            kMaxAccelCoeff * speedOverDistance * speedOverDistance * kRad2Deg > cfg.getMaximumAccelerationThreshold();
    } else {
        triggers.maxRateTrigger = true;
        triggers.maxAccelerationTrigger = true;
    }

    /*! position error with respect to the prediction from the first read against the a-priori sigma bound */
    const double deltaPositionNorm = (r_BN_N - rPredicted_BN_N).norm();
    triggers.positionKnowledgeExceedTrigger = deltaPositionNorm > cfg.getPositionKnowledgeSigma();

    if (triggers.collinearityTrigger || triggers.maxRateTrigger || triggers.maxAccelerationTrigger ||
        triggers.positionKnowledgeExceedTrigger) {
        return triggers;
    }
    return std::nullopt;
}
}  // namespace

/*! This method is used to reset the module.
 @return void
 */
void FlybyPointAlgorithm::reset() {
    this->firstRead = true;
    this->periodsSinceLastRead = 0;
    this->periodsSinceFirstRead = 0;
    this->window = {};
}

/*! This function computes a reference attitude frame for a spacecraft in relative motion about a small body.
 It must be called once per control period: time is counted in whole control periods since the last and the first
 filter read, and converted to seconds only for the guidance equations. Each usable sample is propagated to the end
 of the current averaging window and accumulated; at the window end the average is the re-read candidate. An unusable
 sample is left out and flagged in the diagnostics; after the first seed the guidance output stays valid, since it
 comes from the last accepted profile and not from the current sample. The reference is zero before the first seed,
 and zero if the guidance solution is not finite.
 @return AttGuideOutput containing reference attitude (sigma_RN, omega_RN_N, domega_RN_N) and diagnostic flags
 @param r_BN_N The relative position state
 @param v_BN_N The relative velocity state
 */
AttGuideOutput FlybyPointAlgorithm::updateState(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    /*! advance the period counters first, so time keeps elapsing through calls with an unusable sample */
    if (!this->firstRead) {
        ++this->periodsSinceLastRead;
        ++this->periodsSinceFirstRead;
        ++this->window.periods;
    }

    /*! init diagnostic message */
    AttGuideOutput output{};
    const bool usableSample = isUsableSample(r_BN_N, v_BN_N);
    output.inputSampleRejected = !usableSample;

    if (this->firstRead) {
        /*! seed the algorithm with the first usable, non-collinear solution; it is not otherwise checked, since the
         algorithm needs a seed. Until then there is no profile, so the reference stays zero */
        if (!usableSample) {
            return output;
        }
        /*! a collinear seed (collision course) defines no orbit normal, so the flyby frame would be undefined; it is
         refused like a collinear re-read candidate, and the next non-collinear sample seeds */
        if (isCollinear(r_BN_N, v_BN_N, this->cfg.getToleranceForCollinearity())) {
            output.collinearityTrigger = true;
            return output;
        }
        this->firstNavPosition = r_BN_N;
        this->firstNavVelocity = v_BN_N;
        this->computeFlybyParameters(r_BN_N, v_BN_N);
        this->computeRN(r_BN_N, v_BN_N);
        this->periodsSinceLastRead = 0;
        this->periodsSinceFirstRead = 0;
        this->window = {};
        this->firstRead = false;
    } else {
        /*! low-pass filter the filter states: accumulate every usable sample of the window */
        if (usableSample) {
            this->accumulateSample(r_BN_N, v_BN_N);
        }

        /*! at the window end, report the unusable samples of the window, re-read from the window average and start
         a new window, whatever the outcome. window.periods equals filterReadPeriods here, so the count cannot
         underflow */
        if (this->window.periods >= this->cfg.getFilterReadPeriods()) {
            output.rejectedSamplesInWindow = this->window.periods - this->window.samples;
            applyTriggers(output, this->reReadFromWindow());
            this->window = {};
        }
    }
    /*! [s] time since the last accepted filter read */
    const double dt = static_cast<double>(this->periodsSinceLastRead) * this->cfg.getControlPeriod();
    auto [sigma_RN, omega_RN_N, omegaDot_RN_N] = this->computeGuidanceSolution(dt);

    /*! a solution that is finite in double can still overflow float (e.g. f0^2 in the acceleration); like the other
     guidance algorithms, output a zero reference rather than a non-finite one */
    const Eigen::Vector3f sigma_RN_f = sigma_RN.cast<float>();
    const Eigen::Vector3f omega_RN_N_f = omega_RN_N.cast<float>();
    const Eigen::Vector3f domega_RN_N_f = omegaDot_RN_N.cast<float>();
    if (sigma_RN_f.allFinite() && omega_RN_N_f.allFinite() && domega_RN_N_f.allFinite()) {
        output.sigma_RN = sigma_RN_f;
        output.omega_RN_N = omega_RN_N_f;
        output.domega_RN_N = domega_RN_N_f;
    }
    return output;
}

/*! Propagate a usable sample to the end of the current averaging window with the rectilinear model (constant
 velocity) and add it to the window sums. The sample arrives a whole number of control periods before the window end,
 so the propagation time carries no time rounding error. With filterReadPeriods = 1 the propagation time is zero and
 the sample is accumulated unchanged.
 @return void
 @param r_BN_N The relative position state
 @param v_BN_N The relative velocity state
 */
void FlybyPointAlgorithm::accumulateSample(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    /*! - [s] time from this sample to the window end; window.periods never exceeds filterReadPeriods, because the
     window is cleared when it reaches it and whenever the configuration changes */
    const double timeToWindowEnd =
        static_cast<double>(this->cfg.getFilterReadPeriods() - this->window.periods) * this->cfg.getControlPeriod();
    this->window.rSumAtEnd_N += r_BN_N + timeToWindowEnd * v_BN_N;
    this->window.vSum_N += v_BN_N;
    ++this->window.samples;
}

/*! Re-read from the average of the window that just ended: re-seed the profile if the average passes the validity
 checks, otherwise keep extrapolating the last accepted profile.
 @return the checks that rejected the average; all false if it was accepted, or if no re-read was attempted (no usable
 sample in the window, or an unusable average such as cancelling velocities)
 */
FlybyValidityTriggers FlybyPointAlgorithm::reReadFromWindow() {
    if (this->window.samples == 0U) {
        return {};
    }
    const double sampleCount = static_cast<double>(this->window.samples);
    const Eigen::Vector3d rAverage_N = this->window.rSumAtEnd_N / sampleCount;
    const Eigen::Vector3d vAverage_N = this->window.vSum_N / sampleCount;
    if (!isUsableSample(rAverage_N, vAverage_N)) {
        return {};
    }
    const double deltaT = static_cast<double>(this->periodsSinceFirstRead) * this->cfg.getControlPeriod();
    const Eigen::Vector3d rPredicted_N = this->firstNavPosition + deltaT * this->firstNavVelocity;
    if (const std::optional<FlybyValidityTriggers> rejection =
            checkValidity(rAverage_N, vAverage_N, rPredicted_N, this->cfg)) {
        return *rejection;
    }
    this->computeFlybyParameters(rAverage_N, vAverage_N);
    this->computeRN(rAverage_N, vAverage_N);
    this->periodsSinceLastRead = 0;
    return {};
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
    const std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> zeroSolution{
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    /*! compute the rotation angle of the reference frame from the last read time, and its scalar angular rate and
     acceleration in R-frame coordinates */
    const double theta = safeAtan(safeTan(this->gamma0) + (this->f0 / safeCos(this->gamma0) * dt)) - this->gamma0;
    const double den = ((this->f0 * this->f0 * dt * dt) + (2 * this->f0 * safeSin(this->gamma0) * dt) + 1);
    const double thetaDot = this->f0 * safeCos(this->gamma0) / den;
    const double thetaDDot =
        -2 * this->f0 * this->f0 * safeCos(this->gamma0) * (this->f0 * dt + safeSin(this->gamma0)) / (den * den);

    /*! a profile that overflows (e.g. an f0 = |v| / |r| too large for double) has no finite solution; return the zero
     reference before any attitude conversion, which must not be given non-finite values */
    if (!fsw::is_finite(this->f0) || !fsw::is_finite(this->gamma0) || !fsw::is_finite(theta) ||
        !fsw::is_finite(thetaDot) || !fsw::is_finite(thetaDDot)) {
        return zeroSolution;
    }

    /*! compute DCM (RtR0) of reference frame from last read time */
    const Eigen::Vector3d PRV_theta{0, 0, theta};
    const Eigen::Matrix3d RtR0 = prvToDcm(PRV_theta);

    /*! compute DCM of reference frame at time t_0 + dt with respect to inertial frame */
    const Eigen::Matrix3d RtN = RtR0 * this->R0N.cast<double>();
    const Eigen::Vector3d omega_RN_R{0, 0, thetaDot};
    const Eigen::Vector3d omegaDot_RN_R{0, 0, thetaDDot};

    /*! populate attRefOut with reference frame information */
    Eigen::Vector3d sigma_RN = dcmToMrp(RtN);
    if (!sigma_RN.allFinite()) {
        return zeroSolution;
    }

    if (this->cfg.getSignOfOrbitNormalFrameVector() == -1) {
        Eigen::Vector3d const halfRotationX{1, 0, 0};
        sigma_RN = addMrp(sigma_RN, halfRotationX);
    }
    const Eigen::Vector3d omega_RN_N = RtN.transpose() * omega_RN_R;
    const Eigen::Vector3d omegaDot_RN_N = RtN.transpose() * omegaDot_RN_R;

    return {sigma_RN, omega_RN_N, omegaDot_RN_N};
}
