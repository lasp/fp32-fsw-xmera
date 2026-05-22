#include "dvAccumulationAlgorithm.h"
#include "utilities/timeConstants.h"

#include <algorithm>
#include <ranges>

namespace {
/*! @brief Sort an AccDataMsgF32Payload::accPkts buffer in place by ascending measTime. */
void sortByMeasTime(AccDataMsgF32Payload& accData) {
    std::ranges::sort(accData.accPkts, std::ranges::less{}, &AccPktDataMsgF32Payload::measTime);
}
}  // namespace

void DvAccumulationAlgorithm::resetState(const AccDataMsgF32Payload& accData) {
    /*! - reset accumulator and bookkeeping */
    this->vehAccumDV_B[0] = 0.0;
    this->vehAccumDV_B[1] = 0.0;
    this->vehAccumDV_B[2] = 0.0;
    this->previousTime = 0U;
    this->dvInitialized = 0U;

    /*! - sort a local copy so the caller's buffer is untouched */
    AccDataMsgF32Payload sorted = accData;
    sortByMeasTime(sorted);

    /*! - seed previousTime to the latest non-zero measTime in the buffer so the first update() only
     *    integrates packets that arrived after reset */
    for (int i = (MAX_ACC_BUF_PKT - 1); i >= 0; i--) {
        if (sorted.accPkts[i].measTime > 0) {
            this->previousTime = sorted.accPkts[i].measTime;
            break;
        }
    }
}

DvAccumulationOutput DvAccumulationAlgorithm::update(const AccDataMsgF32Payload& accData) {
    /*! - work on a local sorted copy */
    AccDataMsgF32Payload sorted = accData;
    sortByMeasTime(sorted);

    /*! - On the first call ever, if reset's seed found nothing, latch onto the first new measTime
     *    here so dt doesn't blow up against a zero baseline */
    if (this->dvInitialized == 0U) {
        for (uint32_t i = 0U; i < MAX_ACC_BUF_PKT; i++) {
            if (sorted.accPkts[i].measTime > this->previousTime) {
                this->previousTime = sorted.accPkts[i].measTime;
                this->dvInitialized = 1U;
                break;
            }
        }
    }

    /*! - integrate every packet newer than previousTime */
    for (uint32_t i = 0U; i < MAX_ACC_BUF_PKT; i++) {
        if (sorted.accPkts[i].measTime > this->previousTime) {
            const double dt = static_cast<double>(sorted.accPkts[i].measTime - this->previousTime) * kNano2Sec;
            const double frameDV_B[3] = {dt * static_cast<double>(sorted.accPkts[i].accel_B[0]),
                                         dt * static_cast<double>(sorted.accPkts[i].accel_B[1]),
                                         dt * static_cast<double>(sorted.accPkts[i].accel_B[2])};
            this->vehAccumDV_B[0] += frameDV_B[0];
            this->vehAccumDV_B[1] += frameDV_B[1];
            this->vehAccumDV_B[2] += frameDV_B[2];
            this->previousTime = sorted.accPkts[i].measTime;
        }
    }

    DvAccumulationOutput out{};
    out.timeTag = static_cast<double>(this->previousTime) * kNano2Sec;
    out.vehAccumDV_B[0] = this->vehAccumDV_B[0];
    out.vehAccumDV_B[1] = this->vehAccumDV_B[1];
    out.vehAccumDV_B[2] = this->vehAccumDV_B[2];
    return out;
}
