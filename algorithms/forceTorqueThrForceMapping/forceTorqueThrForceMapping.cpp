#include "forceTorqueThrForceMapping.h"

#include <architecture/utilities/eigenSupport.h>
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param callTime [ns] time the method is called
*/
void ForceTorqueThrForceMapping::reset(const uint64_t callTime) {
    if (!this->thrConfigInMsg.isLinked()) {
        throw std::invalid_argument("forceTorqueThrForceMapping.thrConfigInMsg was not connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("forceTorqueThrForceMapping.vehConfigInMsg was not connected.");
    }

    VehicleConfigMsgF32Payload vehConfigIn = this->vehConfigInMsg();
    THRArrayConfigMsgF32Payload thrConfigIn = this->thrConfigInMsg();

    ThrusterArrayConfig thrusterConfig{};
    thrusterConfig.numThrusters = thrConfigIn.numThrusters;
    for (uint32_t i = 0; i < thrConfigIn.numThrusters; ++i) {
        if (thrConfigIn.thrusters[i].maxThrust <= 0.0F) {
            throw std::invalid_argument(
                "forceTorqueThrForceMapping: A configured thruster has a non-sensible "
                "saturation limit of <= 0 N!");
        }
        for (uint32_t j = 0; j < 3; ++j) {
            thrusterConfig.thrusters.at(i).rThrust_B.at(j) = thrConfigIn.thrusters[i].rThrust_B[j];
            thrusterConfig.thrusters.at(i).tHatThrust_B.at(j) = thrConfigIn.thrusters[i].tHatThrust_B[j];
        }
    }

    this->algorithm.setCoM_B(cArrayToEigenVector(vehConfigIn.CoM_B));
    this->algorithm.setThrusters(thrusterConfig);

    this->algorithm.computeThrusterMapping();
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ForceTorqueThrForceMapping::updateState(const uint64_t callTime) {
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

    const Eigen::Vector<float, MAX_EFF_CNT> thrForce = this->algorithm.update(cmdTorque, cmdForce);

    THRArrayCmdForceMsgF32Payload thrForceCmdOut{};
    eigenVectorToCArray(thrForce, thrForceCmdOut.thrForce);
    this->thrForceCmdOutMsg.write(&thrForceCmdOut, this->moduleID, callTime);
}
