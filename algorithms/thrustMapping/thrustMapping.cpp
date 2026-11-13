#include "thrustMapping.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param callTime [ns] time the method is called
*/
void ThrustMapping::reset(const uint64_t callTime) {
    if (!this->thrConfigInMsg.isLinked()) {
        throw std::invalid_argument("thrustMapping.thrConfigInMsg was not connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("thrustMapping.vehConfigInMsg was not connected.");
    }

    VehicleConfigMsgF32Payload vehConfigIn = this->vehConfigInMsg();
    THRArrayConfigMsgF32Payload thrConfigIn = this->thrConfigInMsg();

    ThrusterArrayConfiguration thrusterConfiguration = this->toThrusterArrayConfiguration(thrConfigIn);

    this->algorithm = std::make_unique<ThrustMappingAlgorithm>(ThrustMappingConfig::create(
        thrusterConfiguration, cArrayToEigenVector(vehConfigIn.CoM_B), this->desiredControlAxes_B));
}

/*! Build the algorithm's thruster array from the configuration message. Availability defaults to every
 *  thruster available; when the optional message is connected, its flags replace that default.
 @return ThrusterArrayConfiguration
 @param thrConfigIn thruster cluster configuration payload
*/
ThrusterArrayConfiguration ThrustMapping::toThrusterArrayConfiguration(const THRArrayConfigMsgF32Payload& thrConfigIn) {
    ThrusterArrayConfiguration thrusterConfiguration{};
    thrusterConfiguration.numThrusters = thrConfigIn.numThrusters;
    thrusterConfiguration.thrusterAvailability.fill(fsw::DeviceAvailability::Available);
    for (uint32_t i = 0; i < thrConfigIn.numThrusters; ++i) {
        if (thrConfigIn.thrusters[i].maxThrust <= 0.0F) {
            throw std::invalid_argument(
                "thrustMapping: A configured thruster has a non-sensible "
                "saturation limit of <= 0 N!");
        }
        for (uint32_t j = 0; j < 3; ++j) {
            thrusterConfiguration.thrusters.at(i).r_TB_B.at(j) = thrConfigIn.thrusters[i].rThrust_B[j];
            thrusterConfiguration.thrusters.at(i).tHat_B.at(j) = thrConfigIn.thrusters[i].tHatThrust_B[j];
        }
    }

    if (this->thrAvailInMsg.isLinked()) {
        const THRArrayAvailabilityMsgF32Payload availabilityIn = this->thrAvailInMsg();
        for (uint32_t i = 0; i < thrConfigIn.numThrusters; ++i) {
            thrusterConfiguration.thrusterAvailability.at(i) =
                fsw::toDeviceAvailability(availabilityIn.thrusterAvailability[i]);
        }
    }

    return thrusterConfiguration;
}

ThrustMappingConfig ThrustMapping::toConfig() {
    VehicleConfigMsgF32Payload vehConfigIn = this->vehConfigInMsg();
    THRArrayConfigMsgF32Payload thrConfigIn = this->thrConfigInMsg();

    ThrusterArrayConfiguration thrusterConfiguration = this->toThrusterArrayConfiguration(thrConfigIn);

    return ThrustMappingConfig::create(
        thrusterConfiguration, cArrayToEigenVector(vehConfigIn.CoM_B), this->desiredControlAxes_B);
}

void ThrustMapping::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustMapping reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ThrustMapping::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustMapping reset() has not been called.");
    }

    Eigen::Vector3f cmdTorque{Eigen::Vector3f::Zero()};
    Eigen::Vector3f cmdForce{Eigen::Vector3f::Zero()};

    /* Check if torque message is linked and read, zero out if not*/
    if (this->cmdTorqueInMsg.isLinked()) {
        CmdTorqueBodyMsgF32Payload cmdTorqueIn = this->cmdTorqueInMsg();
        cmdTorque = cArrayToEigenVector(cmdTorqueIn.torqueRequestBody);
    }

    /* Check if force message is linked and read, zero out if not*/
    if (this->cmdForceInMsg.isLinked()) {
        CmdForceBodyMsgF32Payload cmdForceIn = this->cmdForceInMsg();
        cmdForce = cArrayToEigenVector(cmdForceIn.forceRequestBody);
    }

    const Eigen::Vector<float, kMaxThrusterCount> thrForce = this->algorithm->update(cmdTorque, cmdForce);

    THRArrayCmdForceMsgF32Payload thrForceCmdOut{};
    eigenVectorToCArray(thrForce, thrForceCmdOut.thrForce);
    this->thrForceCmdOutMsg.write(thrForceCmdOut, this->moduleID, callTime);
}

/*! Setter for the desiredControlAxes_B axis selection. See the algorithm class for the layout
 *  (torque xyz then force xyz, all in body frame B).
 @return void
 @param desiredControlAxes the axes the mapping controls
*/
void ThrustMapping::setDesiredControlAxes(const std::array<bool, 6>& desiredControlAxes) {
    this->desiredControlAxes_B = desiredControlAxes;
}

/*! Getter for the desiredControlAxes_B axis selection.
 @return std::array<bool, 6>
*/
std::array<bool, 6> ThrustMapping::getDesiredControlAxes() const { return this->desiredControlAxes_B; }
