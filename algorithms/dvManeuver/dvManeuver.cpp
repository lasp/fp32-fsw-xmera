#include "dvManeuver.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

/*! Validates that the required input messages are connected and constructs the algorithm. */
void DvManeuver::reset(const uint64_t callTime) {
    if (!this->navDataInMsg.isLinked()) {
        throw std::invalid_argument("dvManeuver.navDataInMsg wasn't connected.");
    }
    if (!this->burnDataInMsg.isLinked()) {
        throw std::invalid_argument("dvManeuver.burnDataInMsg wasn't connected.");
    }
    this->algorithm = std::make_unique<DvManeuverAlgorithm>(this->toConfig());
}

DvManeuverConfig DvManeuver::toConfig() const {
    return DvManeuverConfig::create(this->minTime, this->maxTime, this->controlPeriod, this->cmdForce_B);
}

void DvManeuver::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvManeuver reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

void DvManeuver::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvManeuver reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! Compares the accumulated Delta-V against the commanded Delta-V and writes the body force command every
    update: the configured force while the burn executes, zero otherwise. Also flags whether the burn is
    executing and whether it has completed. */
void DvManeuver::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvManeuver reset() has not been called.");
    }

    const NavTransMsgF32Payload navData = this->navDataInMsg();
    const DvBurnCmdMsgF32Payload localBurnData = this->burnDataInMsg();

    const Eigen::Vector3f vehAccumDV = cArrayToEigenVector3<float>(navData.vehAccumDV);
    const Eigen::Vector3f dvInrtlCmd = cArrayToEigenVector3<float>(localBurnData.dvInrtlCmd);

    const DvManeuverOutput out = this->algorithm->update(callTime, vehAccumDV, dvInrtlCmd, localBurnData.burnStartTime);

    CmdForceBodyMsgF32Payload forceMsgOut{};
    eigenVectorToCArray(out.cmdForce_B, forceMsgOut.forceRequestBody);
    this->cmdForceOutMsg.write(forceMsgOut, this->moduleID, callTime);

    DvExecutionDataMsgF32Payload localExeData = {};
    localExeData.burnComplete = out.burnComplete;
    localExeData.burnExecuting = out.burnExecuting;
    this->burnExecOutMsg.write(localExeData, this->moduleID, callTime);
}
