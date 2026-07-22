#include "dvExecuteGuidance.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <math.h>

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
    this->algorithm.reset();
}

void DvExecuteGuidance::setMinTime(const double minTime) {
    if (!(minTime >= 0.0) || isfinite(minTime) == 0) {
        FSW_THROW_INVALID_ARGUMENT("dvExecuteGuidance: minTime must be non-negative and finite.");
    }
    this->algorithm.minTime = minTime;
}

void DvExecuteGuidance::setMaxTime(const double maxTime) {
    if (!(maxTime >= 0.0) || isfinite(maxTime) == 0) {
        FSW_THROW_INVALID_ARGUMENT("dvExecuteGuidance: maxTime must be non-negative and finite.");
    }
    this->algorithm.maxTime = maxTime;
}

void DvExecuteGuidance::setDefaultControlPeriod(const double defaultControlPeriod) {
    if (!(defaultControlPeriod >= 0.0) || isfinite(defaultControlPeriod) == 0) {
        FSW_THROW_INVALID_ARGUMENT("dvExecuteGuidance: defaultControlPeriod must be non-negative and finite.");
    }
    this->algorithm.defaultControlPeriod = defaultControlPeriod;
}

double DvExecuteGuidance::getMinTime() const { return this->algorithm.minTime; }

double DvExecuteGuidance::getMaxTime() const { return this->algorithm.maxTime; }

double DvExecuteGuidance::getDefaultControlPeriod() const { return this->algorithm.defaultControlPeriod; }

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
