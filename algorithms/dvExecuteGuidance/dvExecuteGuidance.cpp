#include "dvExecuteGuidance.h"
#include "utilities/fsw/eigenSupport.h"

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
    this->algorithm.minTime = this->minTime;
    this->algorithm.maxTime = this->maxTime;
    this->algorithm.defaultControlPeriod = this->defaultControlPeriod;
    this->algorithm.reset();
}

/*! This method compares the accumulated Delta-V against the commanded Delta-V and, once the burn is complete,
    writes a zeroed thruster on-time command to turn the thrusters off. It also flags whether the burn is
    executing and whether it has completed.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvExecuteGuidance::updateState(const uint64_t callTime) {
    // read in messages
    const NavTransMsgPayload navData = this->navDataInMsg();
    const DvBurnCmdMsgPayload localBurnData = this->burnDataInMsg();

    const Eigen::Vector3d vehAccumDV = cArrayToEigenVector3<double>(navData.vehAccumDV);
    const Eigen::Vector3d dvInrtlCmd = cArrayToEigenVector3<double>(localBurnData.dvInrtlCmd);

    const DvExecuteGuidanceOutput out =
        this->algorithm.update(callTime, vehAccumDV, dvInrtlCmd, localBurnData.burnStartTime);

    if (out.commandThrustersOff) {
        const THRArrayOnTimeCmdMsgPayload effCmd = {};
        this->thrCmdOutMsg.write(effCmd, this->moduleID, callTime);
    }

    DvExecutionDataMsgPayload localExeData = {};
    localExeData.burnComplete = out.burnComplete;
    localExeData.burnExecuting = out.burnExecuting;
    this->burnExecOutMsg.write(localExeData, this->moduleID, callTime);
}
