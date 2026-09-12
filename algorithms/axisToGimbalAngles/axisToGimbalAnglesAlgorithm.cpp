#include "axisToGimbalAnglesAlgorithm.h"

#include "utilities/fsw/safeMath.h"

#include <math.h>

namespace {
//! [-] shortest perpendicular component for which the plane holding the request and the neutral axis is defined
constexpr float kMinPerpendicular = 1e-3F;
}  // namespace

/*! @brief Construct the algorithm with a validated configuration.
 @param config Validated configuration (gimbal mount orientation and travel).
*/
AxisToGimbalAnglesAlgorithm::AxisToGimbalAnglesAlgorithm(const AxisToGimbalAnglesConfig& config) : cfg(config) {
    this->setConfig(config);
}

/*! @brief Replace the stored configuration at runtime.
 @param config New validated configuration to apply.
*/
void AxisToGimbalAnglesAlgorithm::setConfig(const AxisToGimbalAnglesConfig& config) {
    this->cfg = config;
    this->dcm_MB = mrpToDcm(this->cfg.getSigma_MB());
    this->cosThetaMax = cosf(this->cfg.getThetaMax());
    this->sinThetaMax = sinf(this->cfg.getThetaMax());
}

/*! Pull a request that is outside the travel of the mechanism back onto the cone of half-angle thetaMax, in the
 plane that the request and the neutral axis span.
 @return a unit direction whose deflection from the mount +z axis is at most thetaMax
 @param thrustHat_M [-] requested thrust direction, unit length, mount frame coordinates
*/
Eigen::Vector3f AxisToGimbalAnglesAlgorithm::clampDeflection(const Eigen::Vector3f& thrustHat_M) const {
    // The request has unit length, so its z component is the cosine of the deflection.
    Eigen::Vector3f clampedThrustHat_M = thrustHat_M;

    if (thrustHat_M.z() < this->cosThetaMax) {
        const Eigen::Vector3f perpendicular = thrustHat_M - (Eigen::Vector3f::UnitZ() * thrustHat_M.z());
        // A request exactly opposite the neutral axis leaves no plane, so any perpendicular direction will do.
        const Eigen::Vector3f perpendicularHat = (perpendicular.stableNorm() > kMinPerpendicular)
                                                     ? perpendicular.stableNormalized()
                                                     : Eigen::Vector3f::UnitX();

        clampedThrustHat_M = (this->cosThetaMax * Eigen::Vector3f::UnitZ()) + (this->sinThetaMax * perpendicularHat);
    }

    return clampedThrustHat_M;
}

/*! This method determines the two gimbal angles that align the gimbal thrust axis with the requested thrust
direction. Each angle is the inclination of the thrust axis projected into one of the two mount planes containing
the un-deflected axis, so the pair are independent of one another. A request beyond the travel of the mechanism is
first pulled back onto the cone of half-angle thetaMax.
 @return AxisToGimbalAnglesOutput gimbal angles
 @param thrustHat_B [-] commanded thrust direction, body frame coordinates
*/
AxisToGimbalAnglesOutput AxisToGimbalAnglesAlgorithm::update(const Eigen::Vector3f& thrustHat_B) const {
    const Eigen::Vector3f thrustHat_M = (this->dcm_MB * thrustHat_B).stableNormalized();

    AxisToGimbalAnglesOutput output{};

    // A request of zero length, or one that is not a number, carries no direction at all. The gimbal then stays
    // at its neutral position, which is the zeroed output above.
    if (thrustHat_M.allFinite() && !thrustHat_M.isZero()) {
        const Eigen::Vector3f clampedThrustHat_M = this->clampDeflection(thrustHat_M);

        // thetaMax is less than 90 degrees, so the z component stays above zero and both angles stay within it.
        output.gimbalAngle1 = safeAtan2f(-clampedThrustHat_M.y(), clampedThrustHat_M.z());
        output.gimbalAngle2 = safeAtan2f(clampedThrustHat_M.x(), clampedThrustHat_M.z());
    }

    return output;
}
