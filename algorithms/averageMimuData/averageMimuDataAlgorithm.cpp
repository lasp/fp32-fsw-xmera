#include "averageMimuDataAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/timeConstants.h"
#include "utilities/fsw/validDcmCheck.h"

#include <algorithm>

/*! @brief Ingest new packets from the input snapshot into the internal ring,
 *  then return the rolling average of fresh samples currently in the ring.
 *
 *  Phase 1 (ingest): Each input packet carries a `measTime` that is the
 *  first sample's timestamp (`firstSampleTime`). A packet is ingested iff
 *  its `firstSampleTime` is strictly greater than the largest first-sample
 *  time ever ingested before this update() call. The whole packet is
 *  copied into the next ring slot, overwriting the oldest slot when
 *  capacity is reached. Multiple packets within one snapshot can be
 *  ingested; already-seen or older-than-prior-max packets are dropped.
 *
 *  Phase 2 (average): Per-sample times are derived from each ring slot's
 *  `measTime` plus `s * kMimuSamplePeriodNs`. The maxTimeTag is the
 *  newest slot's tail sample: `max(slot.measTime) + (N - 1) * period_ns`.
 *  Every sample whose derived time is within `averagingWindowNs` of
 *  maxTimeTag contributes equally to the body-frame average via `dcm_BP`.
 *  Returns zeros if the ring is empty.
 *
 *  @param localPkts InputPktsData: 4-packet snapshot from the caller.
 *  @return OutputAverageAccelAngleVel: body-frame rolling average.
 */
OutputAverageAccelAngleVel AverageMimuDataAlgorithm::update(InputPktsData const& localPkts) {
    // Phase 1: ingest. Freeze prior max so within one snapshot all packets
    // are evaluated against the previous-call boundary, not against each
    // other - this lets multiple new packets in the same snapshot all land.
    const uint64_t priorMax = this->lastIngestedMaxMeasTime;
    for (auto const& packet : localPkts.packets) {
        if (!packet.isValid) {
            continue;
        }
        const uint64_t firstSampleTime = packet.measTime;
        if ((firstSampleTime == 0U) || (firstSampleTime <= priorMax)) {
            continue;
        }

        this->ring.at(this->insertIdx).isValid = true;
        this->ring.at(this->insertIdx).measTime = firstSampleTime;
        this->ring.at(this->insertIdx).samples = packet.samples;
        this->insertIdx = (this->insertIdx + 1U) % kRingCapacity;
        this->lastIngestedMaxMeasTime = std::max(this->lastIngestedMaxMeasTime, firstSampleTime);
    }

    // Phase 2: derive maxTimeTag from the newest stored packet's tail sample.
    // Per-sample measTimes are reconstructed from slot.measTime + s * period_ns.
    uint64_t maxSlotMeasTime = 0U;
    for (auto const& slot : this->ring) {
        if (slot.isValid && (slot.measTime > maxSlotMeasTime)) {
            maxSlotMeasTime = slot.measTime;
        }
    }

    OutputAverageAccelAngleVel out{};
    if (maxSlotMeasTime == 0U) {
        return out;
    }

    const uint64_t maxTimeTag =
        maxSlotMeasTime + ((MAX_MIMU_SAMPLES_PER_PKT_C - 1U) * kMimuSamplePeriodNs);

    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t measAvgCount = 0U;

    for (auto const& slot : this->ring) {
        if (!slot.isValid) {
            continue;
        }
        for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT_C; ++s) {
            const uint64_t sampleMeasTime = slot.measTime + (s * kMimuSamplePeriodNs);
            if ((maxTimeTag - sampleMeasTime) <= this->averagingWindowNs) {
                gyroSum_P += slot.samples.at(s).gyro_P;
                accelSum_P += slot.samples.at(s).accel_P;
                measAvgCount++;
            }
        }
    }

    if (measAvgCount > 0U) {
        gyroSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const gyroSum_B = this->dcm_BP * gyroSum_P;
        accelSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const accelSum_B = this->dcm_BP * accelSum_P;
        out.gyroOmega_B = gyroSum_B;
        out.accel_B = accelSum_B;
    }

    return out;
}

void AverageMimuDataAlgorithm::setAveragingWindow(double const window) {
    if (window < 0.0 || window > kMaxAveragingWindowSec) {
        FSW_THROW_INVALID_ARGUMENT("AveragingWindow cannot be smaller than 0.0 or greater than 2.0 seconds");
    }
    this->averagingWindowNs = static_cast<std::uint64_t>(window * kSec2Nano);
}

double AverageMimuDataAlgorithm::getAveragingWindow() const {
    return this->averagingWindowNs * kNano2Sec;
}

void AverageMimuDataAlgorithm::setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn) {
    if (!isValidDcm(dcm_BPIn)) {
        FSW_THROW_INVALID_ARGUMENT("dcm_BP must be orthonormal with det=+1.");
    }
    this->dcm_BP = dcm_BPIn;
}

Eigen::Matrix3f AverageMimuDataAlgorithm::getDcmPltfToBdy() const { return this->dcm_BP; }
