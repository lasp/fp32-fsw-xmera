#include "dvAccumulationAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/timeConstants.h"

#include <array>
#include <utility>

namespace {
/*! @brief Sort an AccDataMsgF32Payload::accPkts buffer in place by ascending measTime.
 *
 * Iterative (stack-based) quicksort, mirroring the original Xmera dvAccumulation sort. A hand-written
 * sort is used rather than std::ranges::sort because libstdc++'s sort implementation trips
 * clang-analyzer-security.ArrayBound (its unguarded insertion-sort reads one element before the
 * array) when applied to the raw C array. */
void sortByMeasTime(AccDataMsgF32Payload& accData) {
    std::array<int, MAX_ACC_BUF_PKT> indexStack{};
    int top = -1;
    ++top;
    indexStack[static_cast<size_t>(top)] = 0;
    ++top;
    indexStack[static_cast<size_t>(top)] = MAX_ACC_BUF_PKT - 1;

    while (top >= 1) {
        auto const end = indexStack[static_cast<size_t>(top)];
        auto const start = indexStack[static_cast<size_t>(top - 1)];
        top -= 2;

        /*! - Lomuto partition on measTime with the last element as pivot */
        auto const pivot = accData.accPkts[static_cast<size_t>(end)].measTime;
        int partitionIndex = start;
        for (int i = start; i < end; ++i) {
            if (accData.accPkts[static_cast<size_t>(i)].measTime <= pivot) {
                std::swap(accData.accPkts[static_cast<size_t>(i)],
                          accData.accPkts[static_cast<size_t>(partitionIndex)]);
                ++partitionIndex;
            }
        }
        std::swap(accData.accPkts[static_cast<size_t>(partitionIndex)], accData.accPkts[static_cast<size_t>(end)]);

        /*! - push sub-ranges that still contain more than one element */
        if (partitionIndex - 1 > start) {
            ++top;
            indexStack[static_cast<size_t>(top)] = start;
            ++top;
            indexStack[static_cast<size_t>(top)] = partitionIndex - 1;
        }
        if (partitionIndex + 1 < end) {
            ++top;
            indexStack[static_cast<size_t>(top)] = partitionIndex + 1;
            ++top;
            indexStack[static_cast<size_t>(top)] = end;
        }
    }
}
}  // namespace

DvAccumulationAlgorithm::DvAccumulationAlgorithm() { this->reInitialize(); }

void DvAccumulationAlgorithm::reInitializeExceptPersistentStates() {
    /*! - reset only the non-persistent accumulator; previousTime and dvInitialized persist */
    this->vehAccumDV_B = Eigen::Vector3f::Zero();
}

void DvAccumulationAlgorithm::reInitialize() {
    /*! - reset all state, including the persistent integration bookkeeping */
    this->reInitializeExceptPersistentStates();
    this->previousTime = 0U;
    this->dvInitialized = 0U;
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
