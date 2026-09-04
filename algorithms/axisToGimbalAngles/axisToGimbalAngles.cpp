#include "axisToGimbalAngles.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

/*! @brief Build the validated configuration from the public properties.
 @return AxisToGimbalAnglesConfig validated configuration
*/
AxisToGimbalAnglesConfig AxisToGimbalAngles::toConfig() const {
    return AxisToGimbalAnglesConfig::create(this->sigma_MB);
}

/*! This method validates the required input message and builds the algorithm from the current configuration.
 @return void
 @param callTime [ns] time the method is called
*/
void AxisToGimbalAngles::reset(const uint64_t callTime) {
    if (!this->thrustDirectionInMsg.isLinked()) {
        throw std::invalid_argument("axisToGimbalAngles.thrustDirectionInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<AxisToGimbalAnglesAlgorithm>(this->toConfig());
}

/*! @brief Re-push the current properties into the running algorithm.
 @return void
*/
void AxisToGimbalAngles::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("AxisToGimbalAngles reset() has not been called.");
    }

    this->algorithm->setConfig(this->toConfig());
}

/*! This method reads the commanded thrust direction, delegates the gimbal angle solve to the algorithm, and
writes the resulting gimbal angles to the output message.
 @return void
 @param callTime [ns] the current time of simulation
*/
void AxisToGimbalAngles::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("AxisToGimbalAngles reset() has not been called.");
    }

    const Eigen::Vector3f thrustHat_B = cArrayToEigenVector3<float>(this->thrustDirectionInMsg().rHat_XB_B);

    const AxisToGimbalAnglesOutput out = this->algorithm->update(thrustHat_B);

    TwoAxisGimbalMsgF32Payload twoAxisGimbalOut{};
    twoAxisGimbalOut.theta1 = out.gimbalAngle1;
    twoAxisGimbalOut.theta2 = out.gimbalAngle2;
    this->twoAxisGimbalOutMsg.write(twoAxisGimbalOut, this->moduleID, callTime);
}
