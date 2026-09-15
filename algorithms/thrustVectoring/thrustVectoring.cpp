#include "thrustVectoring.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <cmath>
#include <stdexcept>

/*! @brief Build the validated configuration from the public properties and the fixed input messages.
 The vehicle and thruster configurations do not change while the module runs, so they are read once here rather
 than on every update; call reconfigure() to pick up a new value.
 @return ThrustVectoringConfig validated configuration
*/
ThrustVectoringConfig ThrustVectoring::toConfig() {
    const VehicleConfigMsgF32Payload vehConfigIn = this->vehConfigInMsg();
    // Only the thrust magnitude is used: the module places the thrust on a line through M and solves in the
    // body frame, so how the mechanism describes the nozzle in a frame of its own does not enter.
    const THRConfigMsgF32Payload thrusterConfigIn = this->thrusterConfigInMsg();

    if (!std::isfinite(this->armLength) || this->armLength < 0.0F) {
        throw std::invalid_argument("thrustVectoring.armLength must be finite and non-negative.");
    }

    return ThrustVectoringConfig::create(
        this->r_MB_B, thrusterConfigIn.maxThrust, cArrayToEigenVector3<float>(vehConfigIn.CoM_B));
}

/*! This method performs a complete reset of the module: it validates the required input messages and (re)creates
 the algorithm from the current configuration.
 @return void
 @param callTime [ns] time the method is called
*/
void ThrustVectoring::reset(const uint64_t callTime) {
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.vehConfigInMsg wasn't connected.");
    }
    if (!this->thrusterConfigInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.thrusterConfigInMsg wasn't connected.");
    }
    if (!this->cmdTorqueInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.cmdTorqueInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<ThrustVectoringAlgorithm>(this->toConfig());
}

/*! @brief Re-read the configuration input messages and re-push the current properties into the algorithm.
 @return void
*/
void ThrustVectoring::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! This method computes the thrust direction that produces the requested torque about the system center of mass
 (a zero request aligns the thrust line of action with the center of mass) and writes the body-heading and
 thruster-configuration output messages.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ThrustVectoring::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }

    const Eigen::Vector3f Lreq_B = cArrayToEigenVector3<float>(this->cmdTorqueInMsg().torqueRequestBody);

    const Eigen::Vector3f tHat_B = this->algorithm->update(Lreq_B);

    // the body-frame thrust heading equals the body-frame thrust unit direction
    BodyHeadingMsgF32Payload bodyHeadingOut{};
    eigenVectorToCArray(tHat_B, bodyHeadingOut.rHat_XB_B);
    this->bodyHeadingOutMsg.write(bodyHeadingOut, this->moduleID, callTime);

    // The thruster fires from a point armLength behind the joint, along the thrust, so its line of action runs
    // through the joint for every direction the module gives.
    const Eigen::Vector3f r_TB_B = this->r_MB_B - (this->armLength * tHat_B);

    THRConfigMsgF32Payload thrusterConfigOut{};
    eigenVectorToCArray(r_TB_B, thrusterConfigOut.rThrust_B);
    eigenVectorToCArray(tHat_B, thrusterConfigOut.tHatThrust_B);
    thrusterConfigOut.maxThrust = this->algorithm->getConfig().getThrust();
    this->thrusterConfigOutMsg.write(thrusterConfigOut, this->moduleID, callTime);
}
