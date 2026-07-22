#include "dvExecuteGuidance.h"
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.h>
#include <string.h>

#include <stdexcept>

/*! @brief This resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvExecuteGuidance::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->navDataInMsg.isLinked()) {
        throw std::invalid_argument("dvExecuteGuidance.navDataInMsg wasn't connected.");
    }
    if (!this->burnDataInMsg.isLinked()) {
        throw std::invalid_argument("dvExecuteGuidance.burnDataInMsg wasn't connected.");
    }
    this->prevCallTime = 0;

    /*! - use default value of 2 seconds for control period of first call if not specified.
     * Control period (FSW rate) is computed dynamically for any subsequent calls.
     */
    this->defaultControlPeriod = (0.0 == this->defaultControlPeriod) ? 2.0 : this->defaultControlPeriod;
}

/*! This method compares the accumulated Delta-V against the commanded Delta-V and, once the burn is complete,
    writes a zeroed thruster on-time command to turn the thrusters off. It also flags whether the burn is
    executing and whether it has completed.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvExecuteGuidance::updateState(uint64_t callTime) {
    double burnAccum[3];
    double dvExecuteMag;
    double burnDt;
    double dvMag;

    NavTransMsgPayload navData;
    DvBurnCmdMsgPayload localBurnData;
    DvExecutionDataMsgPayload localExeData;
    THRArrayOnTimeCmdMsgPayload effCmd;

    // read in messages
    navData = this->navDataInMsg();
    localBurnData = this->burnDataInMsg();

    /*! - The first time update() is called there is no information on the time step.
     *    Use control period (FSW time step) as burn time delta-t */
    if (this->prevCallTime == 0) {
        burnDt = this->defaultControlPeriod;
    } else {
        /*! - compute burn time delta-t (control time period) */
        burnDt = (double)((int64_t)callTime - (int64_t)this->prevCallTime) * NANO2SEC;
    }
    this->prevCallTime = callTime;
    v3SetZero(burnAccum);
    if ((this->burnExecuting == 0 && callTime >= localBurnData.burnStartTime) && this->burnComplete != 1) {
        this->burnExecuting = 1;
        v3Copy(navData.vehAccumDV, this->dvInit);
        this->burnComplete = 0;
    }

    if (this->burnExecuting) {
        this->burnTime += burnDt;
    }

    v3Subtract(navData.vehAccumDV, this->dvInit, burnAccum);

    dvMag = v3Norm(localBurnData.dvInrtlCmd);
    dvExecuteMag = v3Norm(burnAccum);
    this->burnComplete = this->burnComplete == 1 || dvExecuteMag >= dvMag;
    this->burnComplete &= this->burnTime > this->minTime;
    this->burnComplete |= (this->maxTime != 0.0 && this->burnTime > this->maxTime);
    this->burnExecuting = this->burnComplete != 1 && this->burnExecuting == 1;

    if (this->burnComplete || this->burnExecuting != 1) {
        effCmd = {};
        this->thrCmdOutMsg.write(&effCmd, this->moduleID, callTime);
    }

    localExeData = {};
    localExeData.burnComplete = this->burnComplete;
    localExeData.burnExecuting = this->burnExecuting;
    this->burnExecOutMsg.write(&localExeData, this->moduleID, callTime);

    return;
}
