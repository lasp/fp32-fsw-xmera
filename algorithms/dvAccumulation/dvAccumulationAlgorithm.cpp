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

DvAccumulationOutput DvAccumulationAlgorithm::update(const uint64_t callTime,
                                                     const Eigen::Vector3f& rDDotNoGravity_BN_B) {
    /*! - On the first call after a reInitialize (previousTime == 0), latch the clock so dt doesn't
     *    blow up against a zero baseline; otherwise integrate over the elapsed step */
    if (this->previousTime == 0U) {
        this->previousTime = callTime;
    } else if (callTime > this->previousTime) {
        const float dt = static_cast<float>(callTime - this->previousTime) * kNano2SecF;
        this->vehAccumDV_B += dt * rDDotNoGravity_BN_B;
        this->previousTime = callTime;
    }

    DvAccumulationOutput out{};
    out.timeTag = static_cast<double>(this->previousTime) * kNano2Sec;
    out.vehAccumDV_B = this->vehAccumDV_B;
    return out;
}
