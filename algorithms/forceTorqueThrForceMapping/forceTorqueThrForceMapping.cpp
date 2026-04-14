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

    VehicleConfigMsgPayload vehConfigIn = this->vehConfigInMsg();
    THRArrayConfigMsgPayload thrConfigIn = this->thrConfigInMsg();

    const Eigen::Vector3d CoM_B = cArrayToEigenVector(vehConfigIn.CoM_B);

    Eigen::Matrix<double, 3, MAX_EFF_CNT> rThruster_B{Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero()};
    Eigen::Matrix<double, 3, MAX_EFF_CNT> gtThruster_B{Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero()};
    for (uint32_t i = 0; i < thrConfigIn.numThrusters; ++i) {
        rThruster_B.col(i) = cArrayToEigenVector(thrConfigIn.thrusters[i].rThrust_B);
        gtThruster_B.col(i) = cArrayToEigenVector(thrConfigIn.thrusters[i].tHatThrust_B);
        if (thrConfigIn.thrusters[i].maxThrust <= 0.0) {
            throw std::invalid_argument(
                "forceTorqueThrForceMapping: A configured thruster has a non-sensible "
                "saturation limit of <= 0 N!");
        }
    }

    this->algorithm.reset(thrConfigIn.numThrusters, CoM_B, rThruster_B, gtThruster_B);
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ForceTorqueThrForceMapping::updateState(const uint64_t callTime) {
    Eigen::Vector3d cmdTorque{Eigen::Vector3d::Zero()};
    Eigen::Vector3d cmdForce{Eigen::Vector3d::Zero()};

    /* Check if torque message is linked and read, zero out if not*/
    if (this->cmdTorqueInMsg.isLinked()) {
        CmdTorqueBodyMsgPayload cmdTorqueIn = this->cmdTorqueInMsg();
        cmdTorque = cArrayToEigenVector(cmdTorqueIn.torqueRequestBody);
    }

    /* Check if force message is linked and read, zero out if not*/
    if (this->cmdForceInMsg.isLinked()) {
        CmdForceBodyMsgPayload cmdForceIn = this->cmdForceInMsg();
        cmdForce = cArrayToEigenVector(cmdForceIn.forceRequestBody);
    }

    const Eigen::Vector<double, MAX_EFF_CNT> thrForce = this->algorithm.update(cmdTorque, cmdForce);

    THRArrayCmdForceMsgPayload thrForceCmdOut{};
    eigenVectorToCArray(thrForce, thrForceCmdOut.thrForce);
    this->thrForceCmdOutMsg.write(&thrForceCmdOut, this->moduleID, callTime);
}
