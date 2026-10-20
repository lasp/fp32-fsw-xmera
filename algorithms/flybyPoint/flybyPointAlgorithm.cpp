#include "flybyPointAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <numbers>
#include <optional>

static constexpr float kRad2Deg = 180.0F / std::numbers::pi_v<float>;
static constexpr float kMaxAccelCoeff = 3.0F * std::numbers::sqrt3_v<float> / 8.0F;

namespace {
/*! Ground-based state and its one-sigma uncertainty on each inertial axis, propagated to a later time */
struct GroundPrediction {
    Eigen::Vector3f r_N;       //!< [m] ground position propagated to t
    Eigen::Vector3f v_N;       //!< [m/s] ground velocity, constant under the rectilinear model
    Eigen::Vector3f sigmaR_N;  //!< [m] one-sigma position uncertainty per axis
    Eigen::Vector3f sigmaV_N;  //!< [m/s] one-sigma velocity uncertainty per axis
};

/*! Propagate the ground-based state x0 = (r0, v0) and its diagonal covariance P0 = diag(sigmaR0^2, sigmaV0^2), both
 given at the ground state time t0, to t seconds later with the rectilinear state transition matrix
 Phi(t) = [[I, t I], [0, I]]: x(t) = Phi x0 = (r0 + t v0, v0), and the diagonal of P(t) = Phi P0 Phi^T is
 sigmaR0^2 + t^2 sigmaV0^2 for position and sigmaV0^2 for velocity, axis by axis.
 @return the propagated ground state and one-sigma uncertainty per axis
 @param cfg validated configuration (ground-based state and covariance diagonals)
 @param t [s] time since the ground state
 */
GroundPrediction propagateGroundState(const FlybyPointConfig& cfg, const float t) {
    const Eigen::Vector3f& positionVariance_N = cfg.getGroundPositionCovarianceDiag_N();
    const Eigen::Vector3f& velocityVariance_N = cfg.getGroundVelocityCovarianceDiag_N();
    return {.r_N = cfg.getGroundPosition_BN_N() + t * cfg.getGroundVelocity_BN_N(),
            .v_N = cfg.getGroundVelocity_BN_N(),
            .sigmaR_N = (positionVariance_N + t * t * velocityVariance_N).cwiseSqrt(),
            .sigmaV_N = velocityVariance_N.cwiseSqrt()};
}

/*! Check a re-read candidate (r, v) against the configured thresholds and against the ground-based trajectory and its
 uncertainty at the candidate's time.
 @return std::nullopt if the candidate passes every check, otherwise the checks that rejected it
 @param r_BN_N [m] candidate relative position
 @param v_BN_N [m/s] candidate relative velocity
 @param ground ground-based state and uncertainty propagated to the candidate's time
 @param cfg validated configuration (collinearity tolerance, the rate and acceleration thresholds, numberOfSigmas)
 */
std::optional<FlybyValidityTriggers> checkValidity(const Eigen::Vector3f& r_BN_N,
                                                   const Eigen::Vector3f& v_BN_N,
                                                   const GroundPrediction& ground,
                                                   const FlybyPointConfig& cfg) {
    FlybyValidityTriggers triggers{};
    triggers.collinearityTrigger = FlybyPointConfig::isCollinear(r_BN_N, v_BN_N, cfg.getToleranceForCollinearity());

    /*! the predicted peak rate and acceleration occur near closest approach of the candidate's own rectilinear
     trajectory, at distance d_CA = |r x v| / |v| = |r| |cos(gamma)|: peak rate |v| / d_CA and peak acceleration
     3 sqrt(3) / 8 (|v| / d_CA)^2, compared with the spacecraft limits. An exactly collinear candidate has d_CA = 0, so
     its peaks are unbounded and exceed both limits */
    const float distanceClosestApproach = r_BN_N.cross(v_BN_N).stableNorm() / v_BN_N.stableNorm();
    if (distanceClosestApproach > 0.0F) {
        const float speedOverDistance = v_BN_N.stableNorm() / distanceClosestApproach;
        triggers.maxRateTrigger = speedOverDistance * kRad2Deg > cfg.getMaximumRateThreshold();
        triggers.maxAccelerationTrigger =
            kMaxAccelCoeff * speedOverDistance * speedOverDistance * kRad2Deg > cfg.getMaximumAccelerationThreshold();
    } else {
        triggers.maxRateTrigger = true;
        triggers.maxAccelerationTrigger = true;
    }

    /*! position and velocity errors with respect to the propagated ground state, axis by axis, against numberOfSigmas
     times the propagated ground sigmas */
    const float numberOfSigmas = cfg.getNumberOfSigmas();
    const bool positionExceeds =
        ((r_BN_N - ground.r_N).cwiseAbs().array() > numberOfSigmas * ground.sigmaR_N.array()).any();
    const bool velocityExceeds =
        ((v_BN_N - ground.v_N).cwiseAbs().array() > numberOfSigmas * ground.sigmaV_N.array()).any();
    triggers.stateKnowledgeExceedTrigger = positionExceeds || velocityExceeds;

    if (triggers.collinearityTrigger || triggers.maxRateTrigger || triggers.maxAccelerationTrigger ||
        triggers.stateKnowledgeExceedTrigger) {
        return triggers;
    }
    return std::nullopt;
}
}  // namespace

/*! Construct the algorithm with a validated configuration, and seed the profile from its ground-based state.
 @param config The validated configuration to install
 */
FlybyPointAlgorithm::FlybyPointAlgorithm(const FlybyPointConfig& config)
    : cfg(config), profile(makeProfile(config.getGroundPosition_BN_N(), config.getGroundVelocity_BN_N())) {}

/*! Replace the configuration and re-seed the profile from its ground-based state. The partial averaging window is
 discarded, since its samples belong to the replaced profile and were propagated to the end of a window of the old
 length.
 @return void
 @param config The validated configuration to install
 */
void FlybyPointAlgorithm::setConfig(const FlybyPointConfig& config) {
    this->cfg = config;
    this->reset();
}

/*! Re-seed the profile from the configuration's ground-based state, discard the partial averaging window, and restart
 the time since the ground state. The next updateState() call outputs the profile at zero elapsed time.
 @return void
 */
void FlybyPointAlgorithm::reset() {
    this->profile = makeProfile(this->cfg.getGroundPosition_BN_N(), this->cfg.getGroundVelocity_BN_N());
    this->window = {};
    this->periodsSinceGroundState = 0;
}

/*! Compute the reference attitude for this control period from the filter sample (r, v). Must be called once per
 control period. The reference is zero only if the guidance solution is not finite.
 @return AttGuideOutput containing reference attitude (sigma_RN, omega_RN_N, domega_RN_N) and diagnostic flags
 @param r_BN_N [m] relative position from the filter
 @param v_BN_N [m/s] relative velocity from the filter
 */
AttGuideOutput FlybyPointAlgorithm::updateState(const Eigen::Vector3f& r_BN_N, const Eigen::Vector3f& v_BN_N) {
    AttGuideOutput output{};
    const bool usableSample = FlybyPointConfig::isUsableSample(r_BN_N, v_BN_N);
    output.inputSampleRejected = !usableSample;

    /*! 1. Low-pass filter: propagate a usable sample to the window end with the rectilinear model (constant velocity)
     and update the window's running means. The sample fills slot window.periods (counted from 0) of the window, so
     the propagation time is a whole number of control periods, zero for the last slot and when filterReadPeriods = 1 */
    if (usableSample) {
        const float timeToWindowEnd = static_cast<float>(this->cfg.getFilterReadPeriods() - 1U - this->window.periods) *
                                      this->cfg.getControlPeriod();
        const Eigen::Vector3f rAtEnd_N = r_BN_N + timeToWindowEnd * v_BN_N;
        ++this->window.samples;
        const auto sampleCount = static_cast<float>(this->window.samples);
        this->window.rMeanAtEnd_N += (rAtEnd_N - this->window.rMeanAtEnd_N) / sampleCount;
        this->window.vMean_N += (v_BN_N - this->window.vMean_N) / sampleCount;
    }
    ++this->window.periods;

    /*! 2. Window end: report the unusable samples and the checks that rejected the window average (all false if it
     was accepted or no re-read was attempted), then start a new window */
    if (this->window.periods >= this->cfg.getFilterReadPeriods()) {
        output.rejectedSamplesInWindow = this->window.periods - this->window.samples;
        const FlybyValidityTriggers triggers = this->reReadFromWindow();
        output.collinearityTrigger = triggers.collinearityTrigger;
        output.maxRateTrigger = triggers.maxRateTrigger;
        output.maxAccelerationTrigger = triggers.maxAccelerationTrigger;
        output.stateKnowledgeExceedTrigger = triggers.stateKnowledgeExceedTrigger;
        this->window = {};
    }

    /*! 3. Output the reference propagated from the last accepted read, or from the ground state */
    const GuidanceReference reference = this->computeGuidanceReference(this->profile);
    output.sigma_RN = reference.sigma_RN;
    output.omega_RN_N = reference.omega_RN_N;
    output.domega_RN_N = reference.domega_RN_N;

    /*! 4. Advance time to the next control period, whether or not the sample is usable */
    ++this->profile.periodsSinceRead;
    ++this->periodsSinceGroundState;
    return output;
}

/*! Make a new pointing profile from (r, v): the flyby parameters f0 and gamma0, the inertial-to-reference DCM R0N,
 and the time since the read, zero.
 @return the profile of the read
 @param r_BN_N [m] relative position of the accepted read, or of the ground state
 @param v_BN_N [m/s] relative velocity of the accepted read, or of the ground state
 */
FlybyPointAlgorithm::Profile FlybyPointAlgorithm::makeProfile(const Eigen::Vector3f& r_BN_N,
                                                              const Eigen::Vector3f& v_BN_N) {
    /*! radial (ur_N), velocity (uv_N), out-of-plane (uh_N) and along-track (ut_N) unit direction vectors */
    const Eigen::Vector3f ur_N = r_BN_N.stableNormalized();
    const Eigen::Vector3f uv_N = v_BN_N.stableNormalized();
    const Eigen::Vector3f uh_N = ur_N.cross(uv_N).stableNormalized();
    const Eigen::Vector3f ut_N = uh_N.cross(ur_N).stableNormalized();

    Eigen::Matrix3f R0N;
    R0N.row(0) = ur_N;
    R0N.row(1) = ut_N;
    R0N.row(2) = uh_N;
    const float f0 = v_BN_N.stableNorm() / r_BN_N.stableNorm();
    const float gamma0 = safeAtan2f(v_BN_N.dot(ur_N), v_BN_N.dot(ut_N));  // flight path angle
    return {.f0 = f0, .gamma0 = gamma0, .R0N = R0N, .periodsSinceRead = 0};
}

/*! Re-read from the average of the window that just ended: re-seed the profile if the average passes the validity
 checks, otherwise keep extrapolating the last accepted profile. The state knowledge check compares the average with
 the ground-based trajectory and its uncertainty, both propagated from the ground state to the window end.
 @return the checks that rejected the average; all false if it was accepted, or if no re-read was attempted (no
 usable sample in the window, or an unusable average such as cancelling velocities)
 */
FlybyValidityTriggers FlybyPointAlgorithm::reReadFromWindow() {
    if (this->window.samples == 0U) {
        return {};
    }
    const Eigen::Vector3f rAverage_N = this->window.rMeanAtEnd_N;
    const Eigen::Vector3f vAverage_N = this->window.vMean_N;
    if (!FlybyPointConfig::isUsableSample(rAverage_N, vAverage_N)) {
        return {};
    }
    const float timeSinceGroundState = static_cast<float>(this->periodsSinceGroundState) * this->cfg.getControlPeriod();
    const GroundPrediction ground = propagateGroundState(this->cfg, timeSinceGroundState);
    if (const std::optional<FlybyValidityTriggers> rejection =
            checkValidity(rAverage_N, vAverage_N, ground, this->cfg)) {
        return *rejection;
    }
    this->profile = makeProfile(rAverage_N, vAverage_N);
    return {};
}

/*! Compute the reference attitude, rate and acceleration of the profile propagated to the current control period.
 @return the reference; all zero if the solution is not finite
 @param p the profile of the last accepted read
 */
FlybyPointAlgorithm::GuidanceReference FlybyPointAlgorithm::computeGuidanceReference(const Profile& p) const {
    /*! rotation angle of the reference frame since the last read, and its scalar rate and acceleration in R-frame
     coordinates, dt [s] after that read */
    const float f0 = p.f0;
    const float gamma0 = p.gamma0;
    const float dt = static_cast<float>(p.periodsSinceRead) * this->cfg.getControlPeriod();
    const float cosGamma0 = safeCosf(gamma0);
    const float sinGamma0 = safeSinf(gamma0);
    const float theta = safeAtanf(safeTanf(gamma0) + (f0 / cosGamma0 * dt)) - gamma0;
    /*! den = f0^2 dt^2 + 2 f0 sin(gamma0) dt + 1, written as a sum of squares so it doesn't cancel in float near a
     head-on closest approach, where its minimum cos^2(gamma0) is far below the rounding of the expanded form */
    const float alongTrack = (f0 * dt) + sinGamma0;  // zero at closest approach
    const float den = (alongTrack * alongTrack) + (cosGamma0 * cosGamma0);
    const float thetaDot = f0 * cosGamma0 / den;
    const float thetaDDot = -2.0F * f0 * f0 * cosGamma0 * alongTrack / (den * den);

    /*! a profile that overflows (e.g. f0^2 in the acceleration) has no finite solution; stop before the attitude
     conversions, which must not be given non-finite values */
    if (!fsw::is_finite(f0) || !fsw::is_finite(gamma0) || !fsw::is_finite(theta) || !fsw::is_finite(thetaDot) ||
        !fsw::is_finite(thetaDDot)) {
        return {};
    }

    /*! DCM of the reference frame at the last read time + dt with respect to the inertial frame */
    const Eigen::Matrix3f RtN = prvToDcm(Eigen::Vector3f{0.0F, 0.0F, theta}) * p.R0N;

    /*! for sign -1 the frame is turned 180 deg about its first axis, diag(1, -1, -1) * RtN, applied exactly by negating
     the last two rows. It doesn't change the inertial rate and acceleration, which are computed from RtN */
    Eigen::Matrix3f RN = RtN;
    if (this->cfg.getSignOfOrbitNormalFrameVector() == -1) {
        RN.bottomRows<2>() *= -1.0F;
    }
    const Eigen::Vector3f sigma_RN = dcmToMrp(RN);

    /*! the rotated rate and acceleration can still overflow (e.g. a very large thetaDDot) */
    const Eigen::Vector3f omega_RN_N = RtN.transpose() * Eigen::Vector3f{0.0F, 0.0F, thetaDot};
    const Eigen::Vector3f domega_RN_N = RtN.transpose() * Eigen::Vector3f{0.0F, 0.0F, thetaDDot};
    if (sigma_RN.allFinite() && omega_RN_N.allFinite() && domega_RN_N.allFinite()) {
        return {.sigma_RN = sigma_RN, .omega_RN_N = omega_RN_N, .domega_RN_N = domega_RN_N};
    }
    return {};
}
