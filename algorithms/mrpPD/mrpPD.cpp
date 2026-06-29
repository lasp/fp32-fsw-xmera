#include "mrpPD.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <utilities/fsw/eigenSupport.h>

/*! Reset method for the BSK module adapter interface. Reads the spacecraft inertia from the vehicle config
 message, builds the validated configuration, and constructs the algorithm.
 @return void
 @param callTime [ns] Time the method is called
*/
void MrpPD::reset(uint64_t callTime) {
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("mrpPD.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("mrpPD.vehConfigInMsg wasn't connected.");
    }

    Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
    if (this->vehConfigInMsg.isWritten()) {
        inertia = cArrayToEigenMatrix3(this->vehConfigInMsg().ISCPntB_B);
    }

    auto config = MrpPDConfig::create(this->K, this->P, this->knownTorquePntB_B, inertia);
    this->algorithm = std::make_unique<MrpPDAlgorithm>(config);
}

MrpPDConfig MrpPD::toConfig() {
    Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
    if (this->vehConfigInMsg.isWritten()) {
        inertia = cArrayToEigenMatrix3(this->vehConfigInMsg().ISCPntB_B);
    }

    return MrpPDConfig::create(this->K, this->P, this->knownTorquePntB_B, inertia);
}

void MrpPD::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpPD reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Update method for the BSK module adapter interface. This method also calls the algorithm update method.
 @return void
 @param callTime [ns] Time the method is called
*/
void MrpPD::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpPD reset() has not been called.");
    }

    auto torqueCmdMsgF32Payload = CmdTorqueBodyMsgF32Payload();
    if (this->guidInMsg.isWritten()) {
        auto localGuidInMsg = this->guidInMsg();
        Eigen::Vector3f const sigma_BR = cArrayToEigenVector(localGuidInMsg.sigma_BR);
        Eigen::Vector3f const omega_BR_B = cArrayToEigenVector(localGuidInMsg.omega_BR_B);
        Eigen::Vector3f const domega_RN_B = cArrayToEigenVector(localGuidInMsg.domega_RN_B);

        // Call the algorithm update method
        const auto torque = this->algorithm->update(sigma_BR, omega_BR_B, domega_RN_B);
        eigenVectorToCArray(torque, torqueCmdMsgF32Payload.torqueRequestBody);
    }

    this->cmdTorqueOutMsg.write(torqueCmdMsgF32Payload, moduleID, callTime);
}
