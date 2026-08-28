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

Eigen::Vector3f DvAccumulationAlgorithm::update(const Eigen::Vector3f& rDDotNoGravity_BN_B,
                                                const Eigen::Vector3f& accelBias_B) {
    /*! - the first call starts the accumulation window: there is no elapsed interval yet */
    if (this->firstCall) {
        this->firstCall = false;
    } else {
        /*! - the bias is subtracted before scaling: identical to correcting the accumulated Delta-V
         *    afterwards for a constant bias, but it keeps the accumulator correct at every step even
         *    when the caller varies the bias between calls */
        this->vehAccumDV_B += this->cfg.getControlPeriod() * (rDDotNoGravity_BN_B - accelBias_B);
    }

    return this->vehAccumDV_B;
}
