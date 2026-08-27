#include "thrustVectoringAlgorithm.h"

#include <math.h>

#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <Eigen/Geometry>

namespace {
constexpr float kSmallAngle = 1e-3F;  // small angle tolerance [rad]

/*! Solve for the thrust direction that makes the thruster produce the requested torque Lreq about the system
 center of mass.
 @return the thrust unit direction, in the frame the arguments were given in
*/
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- the vectors are distinct quantities and documented.
Eigen::Vector3f solveThrustDirection(const Eigen::Vector3f& r_MC,
                                     const Eigen::Vector3f& tHatNeutral,
                                     float thrust,
                                     const Eigen::Vector3f& Lreq) {
    const float b = r_MC.norm();  // moment arm about the joint; the configuration guarantees b > kMinR_CM
    const Eigen::Vector3f rHat_MC = r_MC / b;

    // (1) Invert |L| = thrust * b * |tPerp| for the requested torque. The cross product also discards the
    //     component of the request along r_MC, which no orientation can produce.
    const Eigen::Vector3f tPerpRequested = (Lreq / (thrust * b)).cross(rHat_MC);

    // (2) Its length is the delivered torque as a fraction of the largest available, so a magnitude above one asks
    //     for more than thrust * b; limiting it there saturates at the maximum in the requested direction.
    const float tPerpMagnitude = fminf(tPerpRequested.stableNorm(), 1.0F);
    const Eigen::Vector3f tPerp = tPerpMagnitude * tPerpRequested.stableNormalized();

    // (3) Compute the component along r_MC, exactly zero once saturated. Both signs deliver the same torque, since
    //     this component produces no torque, so take the one leaving the thrust nearer its un-deflected direction.
    const float alongSign = (rHat_MC.dot(tHatNeutral) >= 0.0F) ? 1.0F : -1.0F;
    const float tAlongMagnitude = alongSign * safeSqrtf(1.0F - (tPerpMagnitude * tPerpMagnitude));
    const Eigen::Vector3f tAlong = tAlongMagnitude * rHat_MC;

    const Eigen::Vector3f tHat = (tPerp + tAlong).stableNormalized();

    return tHat;
}

/*! Clamp the thrust direction so its deflection from the un-deflected direction stays within the cone of
 half-angle thetaMax.
 @return the thrust unit direction, deflected from tHatNeutral by at most thetaMax
*/
Eigen::Vector3f clampThrustDeflection(const Eigen::Vector3f& tHat, const Eigen::Vector3f& tHatNeutral, float thetaMax) {
    Eigen::Vector3f clamped = tHat;  // left alone while the deflection is inside the cone

    if (safeAcosf(tHat.dot(tHatNeutral)) > thetaMax) {
        // Split the direction into its axial and perpendicular parts, then rebuild it at the cone half-angle. The
        // rebuilt vector is renormalized: it is assembled from tHatNeutral, whose own rounding it would inherit.
        const Eigen::Vector3f perp = tHat - (tHatNeutral * tHat.dot(tHatNeutral));
        // nearly antiparallel: the rotation plane is ill-defined, so any perpendicular direction will do
        const Eigen::Vector3f perpHat =
            (perp.norm() < kSmallAngle) ? tHatNeutral.unitOrthogonal() : perp.stableNormalized();
        clamped = ((cosf(thetaMax) * tHatNeutral) + (sinf(thetaMax) * perpHat)).stableNormalized();
    }

    return clamped;
}
}  // namespace

/*! @brief Construct the algorithm with a validated configuration.
 @param config Validated configuration (platform mounting geometry, thruster geometry and center of mass).
*/
ThrustVectoringAlgorithm::ThrustVectoringAlgorithm(const ThrustVectoringConfig& config) : cfg(config) {
    this->setConfig(config);
}

/*! @brief Replace the stored configuration at runtime.
 @param config New validated configuration to apply.
*/
void ThrustVectoringAlgorithm::setConfig(const ThrustVectoringConfig& config) {
    this->cfg = config;
    this->tHatNeutral_B = -mrpToDcm(this->cfg.getPlatformConfiguration().sigma_MB).row(2).transpose().normalized();
}

/*! This method computes the platform reference orientation that points the thruster so it produces the requested
 torque about the system center of mass (a zero request aligns the thruster line of action with the center of mass)
 and the associated body-heading and thruster-configuration quantities.
 @return ThrustVectoringOutput derived body-frame thruster quantities
 @param Lreq_B [Nm] requested thruster torque about the center of mass, body frame
*/
ThrustVectoringOutput ThrustVectoringAlgorithm::update(const Eigen::Vector3f& Lreq_B) const {
    const ThrustVectoringPlatformConfiguration& platform = this->cfg.getPlatformConfiguration();
    const ThrustVectoringThrusterConfiguration& thruster = this->cfg.getThrusterConfiguration();

    const Eigen::Vector3f r_MC_B = platform.r_MB_B - this->cfg.getR_CB_B();

    // Requested thrust direction to achieve the reachable part of the requested torque
    const Eigen::Vector3f tHatRequested_B = solveThrustDirection(r_MC_B, this->tHatNeutral_B, thruster.thrust, Lreq_B);
    // Clamp the thrust direction to respect the deflection limits of the gimbal
    const Eigen::Vector3f tHat_B = clampThrustDeflection(tHatRequested_B, this->tHatNeutral_B, platform.thetaMax);

    ThrustVectoringOutput out{};
    out.tHat_B = tHat_B;
    out.r_TB_B = platform.r_MB_B - thruster.armLength * tHat_B;
    out.thrust = thruster.thrust;

    return out;
}
