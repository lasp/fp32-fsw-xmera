#include "dvExecuteGuidance.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <stdexcept>

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
    this->rebuildAlgorithmConfig();
    this->algorithm.reInitialize();
}

void DvExecuteGuidance::setMinTime(const float minTime) {
    if (!DvExecuteGuidanceConfig::isValidMinTime(minTime)) {
        FSW_THROW_INVALID_ARGUMENT("dvExecuteGuidance: minTime must be non-negative and finite.");
    }
    this->minTime = minTime;
    this->rebuildAlgorithmConfig();
}

void DvExecuteGuidance::setMaxTime(const float maxTime) {
    if (!DvExecuteGuidanceConfig::isValidMaxTime(maxTime)) {
        FSW_THROW_INVALID_ARGUMENT("dvExecuteGuidance: maxTime must be non-negative and finite.");
    }
    this->maxTime = maxTime;
    this->rebuildAlgorithmConfig();
}

void DvExecuteGuidance::setDefaultControlPeriod(const float defaultControlPeriod) {
    if (!DvExecuteGuidanceConfig::isValidDefaultControlPeriod(defaultControlPeriod)) {
        FSW_THROW_INVALID_ARGUMENT("dvExecuteGuidance: defaultControlPeriod must be positive and finite.");
    }
    this->defaultControlPeriod = defaultControlPeriod;
    this->rebuildAlgorithmConfig();
}

float DvExecuteGuidance::getMinTime() const { return this->minTime; }

float DvExecuteGuidance::getMaxTime() const { return this->maxTime; }

float DvExecuteGuidance::getDefaultControlPeriod() const { return this->defaultControlPeriod; }

void DvExecuteGuidance::rebuildAlgorithmConfig() {
    const DvExecuteGuidanceConfig cfg =
        DvExecuteGuidanceConfig::create(this->minTime, this->maxTime, this->defaultControlPeriod);
    this->algorithm.setConfig(cfg);
}

/*! This method compares the accumulated Delta-V against the commanded Delta-V and, once the burn is complete,
    writes a zeroed thruster on-time command to turn the thrusters off. It also flags whether the burn is
    executing and whether it has completed.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvExecuteGuidance::updateState(const uint64_t callTime) {
    // read in messages
    const NavTransMsgF32Payload navData = this->navDataInMsg();
    const DvBurnCmdMsgF32Payload localBurnData = this->burnDataInMsg();

    const Eigen::Vector3f vehAccumDV = cArrayToEigenVector3<float>(navData.vehAccumDV);
    const Eigen::Vector3f dvInrtlCmd = cArrayToEigenVector3<float>(localBurnData.dvInrtlCmd);

    const DvExecuteGuidanceOutput out =
        this->algorithm.update(callTime, vehAccumDV, dvInrtlCmd, localBurnData.burnStartTime);

    if (out.commandThrustersOff) {
        const THRArrayOnTimeCmdMsgF32Payload effCmd = {};
        this->thrCmdOutMsg.write(effCmd, this->moduleID, callTime);
    }

    DvExecutionDataMsgF32Payload localExeData = {};
    localExeData.burnComplete = out.burnComplete;
    localExeData.burnExecuting = out.burnExecuting;
    this->burnExecOutMsg.write(localExeData, this->moduleID, callTime);
}
