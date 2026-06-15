#include "dvAccumulationAlgorithm.h"
#include "architecture/utilities/eigenSupport.h"
#include "utilities/timeConstants.h"

#include <algorithm>
#include <ranges>

namespace {
/*! @brief Sort an AccDataMsgF32Payload::accPkts buffer in place by ascending measTime. */
void sortByMeasTime(AccDataMsgF32Payload& accData) {
    std::ranges::sort(accData.accPkts, std::ranges::less{}, &AccPktDataMsgF32Payload::measTime);
}
}  // namespace

DvAccumulationAlgorithm::DvAccumulationAlgorithm(const DvAccumulationConfig& config) : cfg(config) {}

void DvAccumulationAlgorithm::setConfig(const DvAccumulationConfig& config) { this->cfg = config; }

void DvAccumulationAlgorithm::resetState(const AccDataMsgF32Payload& accData) {
    /*! - reset accumulator and bookkeeping */
    this->vehAccumDV_B = Eigen::Vector3f::Zero();
    this->previousTime = 0U;
    this->dvInitialized = 0U;

    /*! - sort a local copy so the caller's buffer is untouched */
    AccDataMsgF32Payload sorted = accData;
    sortByMeasTime(sorted);

    /*! - seed previousTime to the latest non-zero measTime in the buffer so the first update() only
     *    integrates packets that arrived after reset */
    for (auto const& accPkt : sorted.accPkts | std::views::reverse) {
        if (accPkt.measTime > 0) {
            this->previousTime = accPkt.measTime;
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
        for (auto const& accPkt : sorted.accPkts) {
            if (accPkt.measTime > this->previousTime) {
                this->previousTime = accPkt.measTime;
                this->dvInitialized = 1U;
                break;
            }
        }
    }

    /*! - integrate every packet newer than previousTime */
    for (auto const& accPkt : sorted.accPkts) {
        if (accPkt.measTime > this->previousTime) {
            const float dt = static_cast<float>(accPkt.measTime - this->previousTime) * kNano2SecF;
            const Eigen::Vector3f accel_B = cArrayToEigenVector3(accPkt.accel_B);
            this->vehAccumDV_B += dt * accel_B;
            this->previousTime = accPkt.measTime;
        }
    }

    DvAccumulationOutput out{};
    out.timeTag = static_cast<double>(this->previousTime) * kNano2Sec;
    out.vehAccumDV_B = this->vehAccumDV_B;
    return out;
}
