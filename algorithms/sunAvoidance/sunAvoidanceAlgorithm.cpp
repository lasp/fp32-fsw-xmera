#include "sunAvoidanceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"
#include <Eigen/Geometry>
#include <numbers>

/*! Construct from a validated configuration and seed the runtime maneuver state. */
SunAvoidanceAlgorithm::SunAvoidanceAlgorithm(const SunAvoidanceConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! Replace the configuration at runtime; the runtime maneuver state is preserved. */
void SunAvoidanceAlgorithm::setConfig(const SunAvoidanceConfig& config) { this->cfg = config; }

/*! Re-seed the runtime maneuver state so the next update re-initializes the maneuver. */
void SunAvoidanceAlgorithm::reInitialize() { this->maneuver.reset(); }

/*! Compute the Sun-avoidance maneuver-adjusted reference frame; the maneuver is initialized on the
 first call and fed forward at the configured rate thereafter.
 @param sigma_BN measured MRP attitude of B wrt N
 @param ref attitude reference inputs
 @param r_BN_N spacecraft inertial position
 @param r_SN_N sun inertial position
 @param callTime call time (nanoseconds)
 @return the maneuver-adjusted reference frame
 */
SunAvoidanceOutput SunAvoidanceAlgorithm::update(const Eigen::Vector3f& sigma_BN,
                                                 const SunAvoidanceAttRefInputs& ref,
                                                 const Eigen::Vector3d& r_BN_N,
                                                 const Eigen::Vector3d& r_SN_N,
                                                 const uint64_t callTime) {
    if (!this->maneuver.has_value()) {
        Maneuver m{};
        // Difference the large inertial positions in double precision, then reduce the unit direction to float.
        const Eigen::Vector3f sHat_N = (r_SN_N - r_BN_N).stableNormalized().cast<float>();
        // Skip the maneuver (pass-through) when no usable Sun information is available: a zero Sun
        // position (no ephemeris) or a Sun direction coincident with the spacecraft.
        if (r_SN_N.stableNorm() > 0.0 && sHat_N.stableNorm() > 0.0F) {
            m = initializeManeuver(sigma_BN, ref, sHat_N);
        }
        m.startTime = callTime;
        this->maneuver = m;
    }

    return computeAdjustedReference(sigma_BN, ref, callTime);
}

/*! Initialize the Sun-avoidance maneuver state from the current geometry, taking the final body attitude
 to coincide with the reference frame.
 @param sigma_BN measured MRP attitude of B wrt N
 @param ref attitude reference inputs
 @param sHat_N inertial unit vector from the spacecraft to the Sun
 @return the initialized maneuver (axis and angle; start time is set by the caller)
 */
SunAvoidanceAlgorithm::Maneuver SunAvoidanceAlgorithm::initializeManeuver(const Eigen::Vector3f& sigma_BN,
                                                                          const SunAvoidanceAttRefInputs& ref,
                                                                          const Eigen::Vector3f& sHat_N) const {
    // Phase 1: compute the maneuver -- the short-way principal rotation from the body to the reference.
    // stableNormalized/stableNorm keep a zero rotation (body already at the reference) finite, not NaN.
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(ref.sigma_RN);
    const Eigen::Matrix3f dcm_BR = dcm_BN * dcm_RN.transpose();
    const Eigen::Vector3f prv_BR = dcmToPrv(dcm_BR);
    Maneuver maneuver{.axis_B = prv_BR.stableNormalized(),
                      .angle = prv_BR.stableNorm()};  // magnitude = angle, direction = axis

    // Phase 2: determine short vs long way -- reverse the maneuver if the short slew would sweep the
    // sensitive axis across the Sun.
    const Eigen::Vector3f sensitiveHat_B = this->cfg.getSensitiveHat_B();
    // Sensitive axis (inertial) at the start (body) and end (reference) attitudes.
    const Eigen::Vector3f sensitiveInitial_N = dcm_BN.transpose() * sensitiveHat_B;
    const Eigen::Vector3f sensitiveFinal_N = dcm_RN.transpose() * sensitiveHat_B;

    // The long-way test needs three well-defined directions; stableNormalized() returns a zero vector when
    // any degenerates, and a zero in any of them keeps us on the short way:
    //  - sweep axis: zero when the initial and final sensitive axes are parallel or anti-parallel
    //  - Sun in the sweep plane: zero when the Sun is parallel to the sweep axis,
    //  - initial-to-Sun axis: zero when the Sun is parallel to the initial sensitive axis.
    const Eigen::Vector3f sensitiveSweepAxis_N = (sensitiveInitial_N.cross(sensitiveFinal_N)).stableNormalized();
    const Eigen::Vector3f sunInSweepPlane_N =
        (sHat_N - (sensitiveSweepAxis_N.dot(sHat_N)) * sensitiveSweepAxis_N).stableNormalized();
    const Eigen::Vector3f initialToSunAxis_N = (sensitiveInitial_N.cross(sHat_N)).stableNormalized();
    const bool avoidanceGeometryDefined = sensitiveSweepAxis_N.stableNorm() > 0.0F &&
                                          sunInSweepPlane_N.stableNorm() > 0.0F &&
                                          initialToSunAxis_N.stableNorm() > 0.0F;

    // Sweep extent, the Sun's position along it, and whether the maneuver (initial -> reference) turns the
    // sensitive axis toward the Sun. maneuver.axis_B is reference -> initial, so the maneuver is its reverse.
    const float sensitiveSweepAngle = safeAcosf(sensitiveInitial_N.dot(sensitiveFinal_N));
    const float initialToSunAngle = safeAcosf(sensitiveInitial_N.dot(sunInSweepPlane_N));
    const Eigen::Vector3f initialToReferenceAxis_N = -(dcm_BN.transpose() * maneuver.axis_B);
    const bool maneuverTowardSun = initialToSunAxis_N.dot(initialToReferenceAxis_N) > 0.0F;

    // Reverse to the long way when the maneuver turns the sensitive axis toward the Sun and the Sun lies
    // within the swept arc. Degenerate geometry leaves this decision ambiguous, so the short way is kept.
    if (avoidanceGeometryDefined && maneuverTowardSun && initialToSunAngle < sensitiveSweepAngle) {
        maneuver.angle = (2.0F * std::numbers::pi_v<float>)-maneuver.angle;
        maneuver.axis_B = -maneuver.axis_B;
    }

    return maneuver;
}

/*! Superimpose the current maneuver rotation on the input reference frame; attTrackingError forms the
 tracking error from this adjusted reference downstream.
 @param sigma_BN measured MRP attitude of B wrt N
 @param ref attitude reference inputs
 @param callTime call time (nanoseconds)
 @return the maneuver-adjusted reference frame
 */
SunAvoidanceOutput SunAvoidanceAlgorithm::computeAdjustedReference(const Eigen::Vector3f& sigma_BN,
                                                                   const SunAvoidanceAttRefInputs& ref,
                                                                   const uint64_t callTime) const {
    const Eigen::Matrix3f dcm_RN = mrpToDcm(ref.sigma_RN);

    // update() always initializes the maneuver before this runs; value_or keeps a default (zero-angle,
    // pass-through) maneuver as a defined fallback if it is somehow unset.
    const Maneuver maneuver = this->maneuver.value_or(Maneuver{});

    // Residual maneuver angle, fed forward at the configured rate and clamped at zero.
    const float dtSeconds = static_cast<float>(callTime - maneuver.startTime) * kNano2SecF;
    float remainingManeuverAngle = maneuver.angle - (this->cfg.getSlewRate() * dtSeconds);
    remainingManeuverAngle = remainingManeuverAngle < 0.0F ? 0.0F : remainingManeuverAngle;

    SunAvoidanceOutput out{};

    // Adjusted reference attitude: input reference rotated by the residual maneuver.
    const Eigen::Vector3f prv_RcR = remainingManeuverAngle * maneuver.axis_B;
    const Eigen::Matrix3f dcm_RcR = prvToDcm(prv_RcR);
    const Eigen::Matrix3f dcm_RcN = dcm_RcR * dcm_RN;
    out.sigma_RN = dcmToMrp(dcm_RcN);

    // Feed-forward maneuver rate (N frame), applied while the maneuver is active.
    Eigen::Vector3f omega_RcN_N = ref.omega_RN_N;
    if (remainingManeuverAngle > 0.0F) {
        const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
        omega_RcN_N -= this->cfg.getSlewRate() * (dcm_BN.transpose() * maneuver.axis_B);
    }
    out.omega_RN_N = omega_RcN_N;

    // Constant-rate maneuver adds no angular acceleration.
    out.domega_RN_N = ref.domega_RN_N;

    return out;
}
