#include "axisToGimbalAnglesAlgorithm.h"

#include "utilities/fsw/safeMath.h"

/*! @brief Construct the algorithm with a validated configuration.
 @param config Validated configuration (gimbal mount orientation).
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
}

/*! This method determines the two gimbal angles that align the gimbal thrust axis with the requested thrust
direction. Each angle is the inclination of the thrust axis projected into one of the two mount planes containing
the un-deflected axis, so the pair are independent of one another.
 @return AxisToGimbalAnglesOutput gimbal angles
 @param thrustHat_B [-] commanded thrust direction, body frame coordinates
*/
AxisToGimbalAnglesOutput AxisToGimbalAnglesAlgorithm::update(const Eigen::Vector3f& thrustHat_B) const {
    const Eigen::Vector3f thrustDir_M = (this->dcm_MB * thrustHat_B).stableNormalized();

    // Resolvable only in the half-space the thrust fires into; anything else leaves the gimbal angles zeroed.
    const float towardsThrust = -thrustDir_M.z();
    if (!(towardsThrust > 0.0F)) {
        return AxisToGimbalAnglesOutput{};
    }

    AxisToGimbalAnglesOutput output{};
    output.gimbalAngle1 = safeAtan2f(thrustDir_M.y(), towardsThrust);
    output.gimbalAngle2 = safeAtan2f(-thrustDir_M.x(), towardsThrust);

    return output;
}
