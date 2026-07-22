#include "dvExecuteGuidanceAlgorithm.h"

DvExecuteGuidanceAlgorithm::DvExecuteGuidanceAlgorithm(const DvExecuteGuidanceConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void DvExecuteGuidanceAlgorithm::setConfig(const DvExecuteGuidanceConfig& config) { this->cfg = config; }

void DvExecuteGuidanceAlgorithm::reInitialize() {
    this->burnExecuting = 0;
    this->burnComplete = 0;
    this->burnTime = 0.0F;
    this->dvInit = Eigen::Vector3f::Zero();
}

DvExecuteGuidanceOutput DvExecuteGuidanceAlgorithm::update(const uint64_t callTime,
                                                           const Eigen::Vector3f& vehAccumDV,
                                                           const Eigen::Vector3f& dvInrtlCmd,
                                                           const uint64_t burnStartTime) {
    /*! - the control period (FSW time step) is used as the burn time delta-t */
    const float burnDt = this->cfg.getControlPeriod();

    if ((this->burnExecuting == 0 && callTime >= burnStartTime) && this->burnComplete != 1) {
        this->burnExecuting = 1;
        this->dvInit = vehAccumDV;
        this->burnComplete = 0;
    }

    if (this->burnExecuting) {
        this->burnTime += burnDt;
    }

    const Eigen::Vector3f burnAccum = vehAccumDV - this->dvInit;

    const float dvMag = dvInrtlCmd.norm();
    const float dvExecuteMag = burnAccum.norm();
    this->burnComplete = this->burnComplete == 1 || dvExecuteMag >= dvMag;
    this->burnComplete &= this->burnTime > this->cfg.getMinTime();
    this->burnComplete |= (this->cfg.getMaxTime() != 0.0F && this->burnTime > this->cfg.getMaxTime());
    this->burnExecuting = this->burnComplete != 1 && this->burnExecuting == 1;

    DvExecuteGuidanceOutput out;
    out.burnExecuting = this->burnExecuting;
    out.burnComplete = this->burnComplete;
    out.commandThrustersOff = (this->burnComplete || this->burnExecuting != 1);
    return out;
}
