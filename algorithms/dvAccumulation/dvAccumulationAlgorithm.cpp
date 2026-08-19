#include "dvAccumulationAlgorithm.h"

DvAccumulationAlgorithm::DvAccumulationAlgorithm(const DvAccumulationConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void DvAccumulationAlgorithm::setConfig(const DvAccumulationConfig& config) { this->cfg = config; }

void DvAccumulationAlgorithm::reInitializeExceptPersistentStates() {
    /*! - reset only the non-persistent accumulator */
    this->vehAccumDV_B.setZero();
}

void DvAccumulationAlgorithm::reInitialize() {
    /*! - zero the accumulator and re-arm firstCall together: leaving firstCall set while zeroing the
     *    accumulator would integrate a full step into a fresh window and put the accumulated
     *    Delta-V one interval ahead of the elapsed time */
    this->reInitializeExceptPersistentStates();
    this->firstCall = true;
}

Eigen::Vector3f DvAccumulationAlgorithm::update(const Eigen::Vector3f& rDDotNoGravity_BN_B) {
    /*! - the first call starts the accumulation window; N samples bound N-1 intervals, so there is
     *    no elapsed interval to integrate over yet */
    if (this->firstCall) {
        this->firstCall = false;
    } else {
        this->vehAccumDV_B += this->cfg.getControlPeriod() * rDDotNoGravity_BN_B;
    }

    return this->vehAccumDV_B;
}
