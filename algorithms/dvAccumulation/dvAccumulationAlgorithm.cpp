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
    /*! - the first call starts the accumulation window. The elapsed time is zero */
    if (this->firstCall) {
        this->firstCall = false;
    } else {
        /*! - update() subtracts the bias before it multiplies by the control period. For a constant
         *    bias, the result is the same as a correction of the accumulated Delta-V at the end. But
         *    this order keeps the accumulator correct at each step, also when the caller changes the
         *    bias between calls */
        this->vehAccumDV_B += this->cfg.getControlPeriod() * (rDDotNoGravity_BN_B - accelBias_B);
    }

    return this->vehAccumDV_B;
}
