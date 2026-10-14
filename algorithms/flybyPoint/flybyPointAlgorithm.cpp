#include "flybyPointAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <numbers>
#include <optional>

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

/*! Check whether r and v are collinear: 1 - |cos| of their angle is below toleranceForCollinearity,
 * or |r_hat x v_hat| is below FlybyPointAlgorithm::kMinOrbitNormalNorm.
 @return true if r and v are collinear
 @param r_BN_N [m] relative position
 @param v_BN_N [m/s] relative velocity
 @param toleranceForCollinearity [-] tolerance on 1 - |cos| of the r-v angle
 */
bool isCollinear(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N, const float toleranceForCollinearity) {
    const Eigen::Vector3d ur_N = r_BN_N.stableNormalized();
    const Eigen::Vector3d uv_N = v_BN_N.stableNormalized();
    return 1.0 - fabs(ur_N.dot(uv_N)) < toleranceForCollinearity ||
           ur_N.cross(uv_N).stableNorm() < FlybyPointAlgorithm::kMinOrbitNormalNorm;
}

/*! Check a re-read candidate (r, v) against the configured thresholds and against the rectilinear prediction from the
 last accepted read.
 @return std::nullopt if the candidate passes every check, otherwise the checks that rejected it
 @param r_BN_N [m] candidate relative position
 @param v_BN_N [m/s] candidate relative velocity
 @param rPredicted_BN_N [m] rectilinear prediction of the position at the candidate's time, made from the last accepted
 read
 @param cfg validated configuration (collinearity tolerance and the rate, acceleration and position thresholds)
 */
std::optional<FlybyValidityTriggers> checkValidity(
    const Eigen::Vector3d& r_BN_N,
    const Eigen::Vector3d& v_BN_N,  // NOLINT(bugprone-easily-swappable-parameters)
    const Eigen::Vector3d& rPredicted_BN_N,
    const FlybyPointConfig& cfg) {
    FlybyValidityTriggers triggers{};
    triggers.collinearityTrigger = isCollinear(r_BN_N, v_BN_N, cfg.getToleranceForCollinearity());

    /*! the predicted peak rate and acceleration occur near closest approach of the candidate's own rectilinear
     trajectory, at distance d_CA = |r x v| / |v| = |r| |cos(gamma)|: peak rate |v| / d_CA and peak acceleration
     3 sqrt(3) / 8 (|v| / d_CA)^2, compared with the spacecraft limits. An exactly collinear candidate has d_CA = 0, so
     its peaks are unbounded and exceed both limits */
    const double distanceClosestApproach = r_BN_N.cross(v_BN_N).stableNorm() / v_BN_N.stableNorm();
    if (distanceClosestApproach > 0.0) {
        const double speedOverDistance = v_BN_N.stableNorm() / distanceClosestApproach;
        triggers.maxRateTrigger = speedOverDistance * kRad2Deg > cfg.getMaximumRateThreshold();
        triggers.maxAccelerationTrigger =
            kMaxAccelCoeff * speedOverDistance * speedOverDistance * kRad2Deg > cfg.getMaximumAccelerationThreshold();
    } else {
        triggers.maxRateTrigger = true;
        triggers.maxAccelerationTrigger = true;
    }

    /*! position error with respect to the prediction from the last accepted read against the a-priori sigma bound */
    const double deltaPositionNorm = (r_BN_N - rPredicted_BN_N).stableNorm();
    triggers.positionKnowledgeExceedTrigger = deltaPositionNorm > cfg.getPositionKnowledgeSigma();

    if (triggers.collinearityTrigger || triggers.maxRateTrigger || triggers.maxAccelerationTrigger ||
        triggers.positionKnowledgeExceedTrigger) {
        return triggers;
    }
    return std::nullopt;
}
}  // namespace

/*! Construct the algorithm with a validated configuration and no profile.
 @param config The validated configuration to install
 */
FlybyPointAlgorithm::FlybyPointAlgorithm(const FlybyPointConfig& config) : cfg(config) {}

/*! Replace the configuration. The partial averaging window is discarded, since its samples were propagated to the
 end of a window of the old length.
 @return void
 @param config The validated configuration to install
 */
void FlybyPointAlgorithm::setConfig(const FlybyPointConfig& config) {
    this->cfg = config;
    this->window = {};
}

/*! Forget the profile; the next usable, non-collinear sample seeds a new one.
 @return void
 */
void FlybyPointAlgorithm::reset() {
    this->profile.reset();
    this->window = {};
}

/*! Compute the reference attitude for this control period from the filter sample (r, v). Must be called once per
 control period. The reference is zero until the first seed, and zero if the guidance solution is not finite.
 @return AttGuideOutput containing reference attitude (sigma_RN, omega_RN_N, domega_RN_N) and diagnostic flags
 @param r_BN_N [m] relative position from the filter
 @param v_BN_N [m/s] relative velocity from the filter
 */
AttGuideOutput FlybyPointAlgorithm::updateState(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    AttGuideOutput output{};
    const bool usableSample = isUsableSample(r_BN_N, v_BN_N);
    output.inputSampleRejected = !usableSample;

    if (!this->profile) {
        /*! 1. Seed: no profile (zero reference) until the first usable, non-collinear sample, which is not otherwise
         checked since the algorithm needs a seed. The window is still at its reset value, since nothing accumulates
         before the seed */
        if (!usableSample) {
            return output;
        }
        if (isCollinear(r_BN_N, v_BN_N, this->cfg.getToleranceForCollinearity())) {
            output.collinearityTrigger = true;
            return output;
        }
        this->seedProfile(r_BN_N, v_BN_N);
    } else {
        /*! 2. Advance time, whether or not the sample is usable */
        ++this->profile->periodsSinceRead;
        ++this->window.periods;

        /*! 3. Low-pass filter: propagate a usable sample to the window end with the rectilinear model (constant
         velocity) and accumulate it. The propagation time is a whole number of control periods, zero when
         filterReadPeriods = 1 */
        if (usableSample) {
            const double timeToWindowEnd =
                static_cast<double>(this->cfg.getFilterReadPeriods() - this->window.periods) *
                this->cfg.getControlPeriod();
            this->window.rSumAtEnd_N += r_BN_N + timeToWindowEnd * v_BN_N;
            this->window.vSum_N += v_BN_N;
            ++this->window.samples;
        }

        /*! 4. Window end: report the unusable samples and the checks that rejected the window average (all false if it
         was accepted or no re-read was attempted), then start a new window */
        if (this->window.periods >= this->cfg.getFilterReadPeriods()) {
            output.rejectedSamplesInWindow = this->window.periods - this->window.samples;
            const FlybyValidityTriggers triggers = this->reReadFromWindow(*this->profile);
            output.collinearityTrigger = triggers.collinearityTrigger;
            output.maxRateTrigger = triggers.maxRateTrigger;
            output.maxAccelerationTrigger = triggers.maxAccelerationTrigger;
            output.positionKnowledgeExceedTrigger = triggers.positionKnowledgeExceedTrigger;
            this->window = {};
        }
    }

    /*! 5. Output the reference propagated from the last accepted read */
    if (this->profile) {
        const GuidanceReference reference = this->computeGuidanceReference(*this->profile);
        output.sigma_RN = reference.sigma_RN;
        output.omega_RN_N = reference.omega_RN_N;
        output.domega_RN_N = reference.domega_RN_N;
    }
    return output;
}

/*! Start a new pointing profile from (r, v): the flyby parameters f0 and gamma0, the inertial-to-reference DCM R0N,
 and the time since the last accepted read.
 @return void
 @param r_BN_N [m] relative position of the accepted read
 @param v_BN_N [m/s] relative velocity of the accepted read
 */
void FlybyPointAlgorithm::seedProfile(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    /*! radial (ur_N), velocity (uv_N), out-of-plane (uh_N) and along-track (ut_N) unit direction vectors */
    const Eigen::Vector3d ur_N = r_BN_N.stableNormalized();
    const Eigen::Vector3d uv_N = v_BN_N.stableNormalized();
    const Eigen::Vector3d uh_N = ur_N.cross(uv_N).stableNormalized();
    const Eigen::Vector3d ut_N = uh_N.cross(ur_N).stableNormalized();

    Eigen::Matrix3f R0N;
    R0N.row(0) = ur_N.cast<float>();
    R0N.row(1) = ut_N.cast<float>();
    R0N.row(2) = uh_N.cast<float>();
    const double f0 = v_BN_N.stableNorm() / r_BN_N.stableNorm();
    const double gamma0 = safeAtan2(v_BN_N.dot(ur_N), v_BN_N.dot(ut_N));  // flight path angle
    this->profile =
        Profile{.r_N = r_BN_N, .v_N = v_BN_N, .f0 = f0, .gamma0 = gamma0, .R0N = R0N, .periodsSinceRead = 0};
}

/*! Re-read from the average of the window that just ended: re-seed the profile if the average passes the validity
 checks, otherwise keep extrapolating the last accepted profile.
 @return the checks that rejected the average; all false if it was accepted, or if no re-read was attempted (no
 usable sample in the window, or an unusable average such as cancelling velocities)
 @param p the profile of the last accepted read
 */
FlybyValidityTriggers FlybyPointAlgorithm::reReadFromWindow(const Profile& p) {
    if (this->window.samples == 0U) {
        return {};
    }
    const auto sampleCount = static_cast<double>(this->window.samples);
    const Eigen::Vector3d rAverage_N = this->window.rSumAtEnd_N / sampleCount;
    const Eigen::Vector3d vAverage_N = this->window.vSum_N / sampleCount;
    if (!isUsableSample(rAverage_N, vAverage_N)) {
        return {};
    }
    /*! p is read here, before seedProfile() replaces the profile, and not used afterwards */
    const double deltaT = static_cast<double>(p.periodsSinceRead) * this->cfg.getControlPeriod();
    const Eigen::Vector3d rPredicted_N = p.r_N + deltaT * p.v_N;
    if (const std::optional<FlybyValidityTriggers> rejection =
            checkValidity(rAverage_N, vAverage_N, rPredicted_N, this->cfg)) {
        return *rejection;
    }
    this->seedProfile(rAverage_N, vAverage_N);
    return {};
}

/*! Compute the reference attitude, rate and acceleration of the profile propagated to the current control period.
 @return the reference; all zero if the solution is not finite, in double or after the cast to float
 @param p the profile of the last accepted read
 */
FlybyPointAlgorithm::GuidanceReference FlybyPointAlgorithm::computeGuidanceReference(const Profile& p) const {
    /*! rotation angle of the reference frame since the last read, and its scalar rate and acceleration in R-frame
     coordinates, dt [s] after that read */
    const double f0 = p.f0;
    const double gamma0 = p.gamma0;
    const double dt = static_cast<double>(p.periodsSinceRead) * this->cfg.getControlPeriod();
    const double theta = safeAtan(safeTan(gamma0) + (f0 / safeCos(gamma0) * dt)) - gamma0;
    const double den = ((f0 * f0 * dt * dt) + (2 * f0 * safeSin(gamma0) * dt) + 1);
    const double thetaDot = f0 * safeCos(gamma0) / den;
    const double thetaDDot = -2 * f0 * f0 * safeCos(gamma0) * ((f0 * dt) + safeSin(gamma0)) / (den * den);

    /*! a profile that overflows (e.g. an f0 too large for double) has no finite solution; stop before the attitude
     conversions, which must not be given non-finite values */
    if (!fsw::is_finite(f0) || !fsw::is_finite(gamma0) || !fsw::is_finite(theta) || !fsw::is_finite(thetaDot) ||
        !fsw::is_finite(thetaDDot)) {
        return {};
    }

    /*! DCM of the reference frame at the last read time + dt with respect to the inertial frame */
    const Eigen::Matrix3d RtN = prvToDcm(Eigen::Vector3d{0, 0, theta}) * p.R0N.cast<double>();
    Eigen::Vector3d sigma_RN = dcmToMrp(RtN);
    if (this->cfg.getSignOfOrbitNormalFrameVector() == -1) {
        sigma_RN = addMrp(sigma_RN, Eigen::Vector3d{1, 0, 0});
    }

    /*! a solution finite in double can still overflow float (e.g. f0^2 in the acceleration) */
    const Eigen::Vector3f sigma_RN_f = sigma_RN.cast<float>();
    const Eigen::Vector3f omega_RN_N_f = (RtN.transpose() * Eigen::Vector3d{0, 0, thetaDot}).cast<float>();
    const Eigen::Vector3f domega_RN_N_f = (RtN.transpose() * Eigen::Vector3d{0, 0, thetaDDot}).cast<float>();
    if (sigma_RN_f.allFinite() && omega_RN_N_f.allFinite() && domega_RN_N_f.allFinite()) {
        return {.sigma_RN = sigma_RN_f, .omega_RN_N = omega_RN_N_f, .domega_RN_N = domega_RN_N_f};
    }
    return {};
}
