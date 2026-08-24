#include "thrustVectoring.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

/*! @brief Build the validated configuration from the public properties and the fixed input messages.
 The vehicle and thruster configurations do not change while the module runs, so they are read once here rather
 than on every update; call reconfigure() to pick up a new value.
 @return ThrustVectoringConfig validated configuration
*/
ThrustVectoringConfig ThrustVectoring::toConfig() {
    const VehicleConfigMsgF32Payload vehConfigIn = this->vehConfigInMsg();
    const THRConfigMsgF32Payload thrusterConfigFIn = this->thrusterConfigFInMsg();

    const ThrustVectoringPlatformConfiguration platformConfig{
        .sigma_MB = this->sigma_MB, .r_MB_B = this->r_MB_B, .r_FM_F = this->r_FM_F, .thetaMax = this->thetaMax};
    const ThrustVectoringThrusterConfiguration thrusterConfig{
        .r_TF_F = cArrayToEigenVector3<float>(thrusterConfigFIn.rThrust_B),
        .tHat_F = cArrayToEigenVector3<float>(thrusterConfigFIn.tHatThrust_B),
        .thrust = thrusterConfigFIn.maxThrust};

    return ThrustVectoringConfig::create(
        platformConfig, thrusterConfig, cArrayToEigenVector3<float>(vehConfigIn.CoM_B));
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
    if (!this->thrusterConfigFInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.thrusterConfigFInMsg wasn't connected.");
    }
    if (!this->cmdTorqueInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.cmdTorqueInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<ThrustVectoringAlgorithm>(this->toConfig());
}

/*! @brief Re-push the current configuration properties into the running algorithm, keeping its runtime state.
 @return void
*/
void ThrustVectoring::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! @brief Re-seed the running algorithm's runtime state from its configured initial values.
 @return void
*/
void ThrustVectoring::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method computes the platform reference orientation that points the thruster so it produces the requested
 torque about the system center of mass (a zero request aligns the thruster line of action with the center of mass)
 and writes the body-heading and thruster-configuration output messages.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ThrustVectoring::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }

    const Eigen::Vector3f Lreq_B = cArrayToEigenVector3<float>(this->cmdTorqueInMsg().torqueRequestBody);

    const ThrustVectoringOutput out = this->algorithm->update(Lreq_B);

    // the body-frame thrust heading equals the body-frame thrust unit direction
    BodyHeadingMsgF32Payload bodyHeadingOut{};
    eigenVectorToCArray(out.tHat_B, bodyHeadingOut.rHat_XB_B);
    this->bodyHeadingOutMsg.write(bodyHeadingOut, this->moduleID, callTime);

    THRConfigMsgF32Payload thrusterConfigOut{};
    eigenVectorToCArray(out.r_TB_B, thrusterConfigOut.rThrust_B);
    eigenVectorToCArray(out.tHat_B, thrusterConfigOut.tHatThrust_B);
    thrusterConfigOut.maxThrust = out.thrust;
    this->thrusterConfigBOutMsg.write(thrusterConfigOut, this->moduleID, callTime);
}
