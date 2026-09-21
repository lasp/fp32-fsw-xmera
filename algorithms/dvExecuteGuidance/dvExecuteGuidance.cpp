#include "dvExecuteGuidance.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

DvExecuteGuidanceConfig DvExecuteGuidance::toConfig() const {
    return DvExecuteGuidanceConfig::create(this->minTime, this->maxTime, this->controlPeriod);
}

/*! Validates that the required input messages are connected and constructs the algorithm. */
void DvExecuteGuidance::reset(const uint64_t callTime) {
    if (!this->navDataInMsg.isLinked()) {
        throw std::invalid_argument("dvExecuteGuidance.navDataInMsg wasn't connected.");
    }
    if (!this->burnDataInMsg.isLinked()) {
        throw std::invalid_argument("dvExecuteGuidance.burnDataInMsg wasn't connected.");
    }
    this->algorithm = std::make_unique<DvExecuteGuidanceAlgorithm>(this->toConfig());
}

void DvExecuteGuidance::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvExecuteGuidance reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

void DvExecuteGuidance::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvExecuteGuidance reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! Compares the accumulated Delta-V against the commanded Delta-V and, once the burn is complete, writes a
    zeroed thruster on-time command to turn the thrusters off. Also flags whether the burn is executing and
    whether it has completed. */
void DvExecuteGuidance::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvExecuteGuidance reset() has not been called.");
    }

    const NavTransMsgF32Payload navData = this->navDataInMsg();
    const DvBurnCmdMsgF32Payload localBurnData = this->burnDataInMsg();

    const Eigen::Vector3f vehAccumDV = cArrayToEigenVector3<float>(navData.vehAccumDV);
    const Eigen::Vector3f dvInrtlCmd = cArrayToEigenVector3<float>(localBurnData.dvInrtlCmd);

    const DvExecuteGuidanceOutput out =
        this->algorithm->update(callTime, vehAccumDV, dvInrtlCmd, localBurnData.burnStartTime);

    if (out.commandThrustersOff) {
        const THRArrayOnTimeCmdMsgF32Payload onTimeMsgOut = {};
        this->thrCmdOutMsg.write(onTimeMsgOut, this->moduleID, callTime);
    }

    DvExecutionDataMsgF32Payload localExeData = {};
    localExeData.burnComplete = out.burnComplete;
    localExeData.burnExecuting = out.burnExecuting;
    this->burnExecOutMsg.write(localExeData, this->moduleID, callTime);
}
