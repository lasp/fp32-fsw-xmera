#include "dvExecuteGuidance.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

DvExecuteGuidanceConfig DvExecuteGuidance::toConfig() const {
    return DvExecuteGuidanceConfig::create(this->minTime, this->maxTime, this->controlPeriod);
}

/*! @brief This resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvExecuteGuidance::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->navDataInMsg.isLinked()) {
        throw std::invalid_argument("dvExecuteGuidance.navDataInMsg wasn't connected.");
    }
    if (!this->burnDataInMsg.isLinked()) {
        throw std::invalid_argument("dvExecuteGuidance.burnDataInMsg wasn't connected.");
    }
    this->algorithm = std::make_unique<DvExecuteGuidanceAlgorithm>(this->toConfig());
}

void DvExecuteGuidance::reconfigure() {
    if (this->algorithm) {
        this->algorithm->setConfig(this->toConfig());
    }
}

void DvExecuteGuidance::reInitialize() {
    if (this->algorithm) {
        this->algorithm->reInitialize();
    }
}

/*! This method compares the accumulated Delta-V against the commanded Delta-V and, once the burn is complete,
    writes a zeroed thruster on-time command to turn the thrusters off. It also flags whether the burn is
    executing and whether it has completed.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvExecuteGuidance::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvExecuteGuidance reset() has not been called.");
    }

    // read in messages
    const NavTransMsgF32Payload navData = this->navDataInMsg();
    const DvBurnCmdMsgF32Payload localBurnData = this->burnDataInMsg();

    const Eigen::Vector3f vehAccumDV = cArrayToEigenVector3<float>(navData.vehAccumDV);
    const Eigen::Vector3f dvInrtlCmd = cArrayToEigenVector3<float>(localBurnData.dvInrtlCmd);

    const DvExecuteGuidanceOutput out =
        this->algorithm->update(callTime, vehAccumDV, dvInrtlCmd, localBurnData.burnStartTime);

    if (out.commandThrustersOff) {
        const THRArrayOnTimeCmdMsgF32Payload effCmd = {};
        this->thrCmdOutMsg.write(effCmd, this->moduleID, callTime);
    }

    DvExecutionDataMsgF32Payload localExeData = {};
    localExeData.burnComplete = out.burnComplete;
    localExeData.burnExecuting = out.burnExecuting;
    this->burnExecOutMsg.write(localExeData, this->moduleID, callTime);
}
