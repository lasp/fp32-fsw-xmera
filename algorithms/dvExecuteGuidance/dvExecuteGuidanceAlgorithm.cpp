#include "dvExecuteGuidanceAlgorithm.h"
#include "utilities/fsw/timeConstants.h"

//! Default control period [s] used for the first call when the user leaves defaultControlPeriod unset.
static constexpr double kDefaultControlPeriodSeconds = 2.0;

void DvExecuteGuidanceAlgorithm::reset() {
    this->prevCallTime = 0;
    this->burnExecuting = 0;
    this->burnComplete = 0;
    this->burnTime = 0.0;
    this->dvInit = Eigen::Vector3d::Zero();

    /*! - use default value of 2 seconds for control period of first call if not specified.
     * Control period (FSW rate) is computed dynamically for any subsequent calls.
     */
    this->defaultControlPeriod =
        (0.0 == this->defaultControlPeriod) ? kDefaultControlPeriodSeconds : this->defaultControlPeriod;
}

DvExecuteGuidanceOutput DvExecuteGuidanceAlgorithm::update(const uint64_t callTime,
                                                           const Eigen::Vector3d& vehAccumDV,
                                                           const Eigen::Vector3d& dvInrtlCmd,
                                                           const uint64_t burnStartTime) {
    double burnDt;

    /*! - The first time update() is called there is no information on the time step.
     *    Use control period (FSW time step) as burn time delta-t */
    if (this->prevCallTime == 0) {
        burnDt = this->defaultControlPeriod;
    } else {
        /*! - compute burn time delta-t (control time period) */
        burnDt =
            static_cast<double>(static_cast<int64_t>(callTime) - static_cast<int64_t>(this->prevCallTime)) * kNano2Sec;
    }
    this->prevCallTime = callTime;

    if ((this->burnExecuting == 0 && callTime >= burnStartTime) && this->burnComplete != 1) {
        this->burnExecuting = 1;
        this->dvInit = vehAccumDV;
        this->burnComplete = 0;
    }

    if (this->burnExecuting) {
        this->burnTime += burnDt;
    }

    const Eigen::Vector3d burnAccum = vehAccumDV - this->dvInit;

    const double dvMag = dvInrtlCmd.norm();
    const double dvExecuteMag = burnAccum.norm();
    this->burnComplete = this->burnComplete == 1 || dvExecuteMag >= dvMag;
    this->burnComplete &= this->burnTime > this->minTime;
    this->burnComplete |= (this->maxTime != 0.0 && this->burnTime > this->maxTime);
    this->burnExecuting = this->burnComplete != 1 && this->burnExecuting == 1;

    DvExecuteGuidanceOutput out;
    out.burnExecuting = this->burnExecuting;
    out.burnComplete = this->burnComplete;
    out.commandThrustersOff = (this->burnComplete || this->burnExecuting != 1);
    return out;
}
