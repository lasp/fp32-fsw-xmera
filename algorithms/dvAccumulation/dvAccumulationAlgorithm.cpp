#include "dvAccumulationAlgorithm.h"

DvAccumulationAlgorithm::DvAccumulationAlgorithm(const DvAccumulationConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void DvAccumulationAlgorithm::setConfig(const DvAccumulationConfig& config) { this->cfg = config; }

void DvAccumulationAlgorithm::reInitialize() {
    this->vehAccumDV_B.setZero();
    this->firstCall = true;
}

Eigen::Vector3f DvAccumulationAlgorithm::update(const Eigen::Vector3f& rDDotNoGravity_BN_B) {
    /*! - the first call starts the accumulation window. The elapsed time is zero */
    if (this->firstCall) {
        this->firstCall = false;
    } else {
        /*! - update() subtracts the bias before it multiplies by the control period. Thus the
         *    accumulator stays correct at each step, also when setConfig() changes the bias */
        this->vehAccumDV_B += this->cfg.getControlPeriod() * (rDDotNoGravity_BN_B - this->cfg.getAccelBias_B());
    }

    return this->vehAccumDV_B;
}
