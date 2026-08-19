#include "dvAccumulationAlgorithm.h"
#include "utilities/fsw/timeConstants.h"

DvAccumulationAlgorithm::DvAccumulationAlgorithm() { this->reInitialize(); }

void DvAccumulationAlgorithm::reInitializeExceptPersistentStates() {
    /*! - reset only the non-persistent accumulator; previousTime persists */
    this->vehAccumDV_B = Eigen::Vector3f::Zero();
}

void DvAccumulationAlgorithm::reInitialize() {
    /*! - reset all state, including the persistent time reference */
    this->reInitializeExceptPersistentStates();
    this->previousTime = 0U;
}

Eigen::Vector3f DvAccumulationAlgorithm::update(const uint64_t callTime, const Eigen::Vector3f& rDDotNoGravity_BN_B) {
    /*! - a non-advancing callTime is ignored; the first call (previousTime == 0) only latches the clock */
    if (callTime > this->previousTime) {
        if (this->previousTime != 0U) {
            const float dt = static_cast<float>(callTime - this->previousTime) * kNano2SecF;
            this->vehAccumDV_B += dt * rDDotNoGravity_BN_B;
        }
        this->previousTime = callTime;
    }

    return this->vehAccumDV_B;
}
