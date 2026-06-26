#include "averageMimuDataAlgorithm.h"
#include <utilities/fsw/eigenSupport.h>
#include <utilities/fsw/freestandingInvalidArgument.h>
#include <utilities/fsw/timeConstants.h>
#include <utilities/fsw/validDcmCheck.h>

#include <algorithm>

/*! @brief Ingest new packets from the input snapshot into the internal ring,
 *  then return the rolling average of fresh samples currently in the ring.
 *
 *  Phase 1 (ingest): Each input packet carries a `measTime` that is the
 *  first sample's timestamp (`firstSampleTime`). A packet is ingested only if
 *  its `firstSampleTime` is strictly greater than the largest first-sample
 *  time ever ingested before this update() call. The whole packet is
 *  copied into the next ring slot, overwriting the oldest slot when
 *  capacity is reached. Multiple packets within one snapshot can be
 *  ingested; already-seen or older-than-prior-max packets are dropped.
 *
 *  Phase 2 (average): Per-sample times are derived from each ring slot's
 *  `measTime` plus `s * kMimuSamplePeriodNs`. The maxTimeTag is the
 *  newest slot's tail sample: `max(slot.measTime) + (N - 1) * period_ns`.
 *  Gyro and acceleration data are averaged independently: a sample contributes to
 *  the gyro mean when its age relative to maxTimeTag is within
 *  `gyroAveragingWindowNs`, and to the accel mean when within
 *  `accelAveragingWindowNs`. Each mean is rotated to the body frame via
 *  `dcm_BP`. Components with no in-window samples (or an empty ring) stay zero.
 *
 *  @param localPkts InputPktsData: 4-packet snapshot from the caller.
 *  @return OutputAverageAccelAngleVel: body-frame rolling average.
 */
OutputAverageAccelAngleVel AverageMimuDataAlgorithm::update(InputPktsData const& localPkts) {
    // Phase 1: Ingest packets. Freeze prior maximum time so within one snapshot
    // all packets are evaluated against the previous-call boundary.
    const uint64_t priorMax = this->lastIngestedMaxMeasTime;
    for (const auto& [isValid, measTime, samples] : localPkts.packets) {
        if (!isValid) {
            continue;
        }
        const uint64_t firstSampleTime = measTime;
        if (firstSampleTime == 0U || firstSampleTime <= priorMax) {
            continue;
        }

        this->ring.at(this->insertIdx).isValid = true;
        this->ring.at(this->insertIdx).measTime = firstSampleTime;
        this->ring.at(this->insertIdx).samples = samples;
        this->insertIdx = (this->insertIdx + 1U) % kRingCapacity;
        this->lastIngestedMaxMeasTime = std::max(this->lastIngestedMaxMeasTime, firstSampleTime);
    }

    // Phase 2: compute the maxTimeTag from the newest stored packet's tail sample.
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

    const uint64_t maxTimeTag = maxSlotMeasTime + ((MAX_MIMU_SAMPLES_PER_PKT_C - 1U) * kMimuSamplePeriodNs);

    // Gyro and accel each accumulate over their own window, so a sample may
    // contribute to one running mean and not the other.
    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t gyroAvgCount = 0U;
    uint64_t accelAvgCount = 0U;

    for (const auto& [isValid, measTime, samples] : this->ring) {
        if (!isValid) {
            continue;
        }
        for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT_C; ++s) {
            const uint64_t sampleMeasTime = measTime + (s * kMimuSamplePeriodNs);
            const uint64_t age = maxTimeTag - sampleMeasTime;
            if (age <= this->gyroAveragingWindowNs) {
                gyroSum_P += samples.at(s).gyro_P;
                gyroAvgCount++;
            }
            if (age <= this->accelAveragingWindowNs) {
                accelSum_P += samples.at(s).accel_P;
                accelAvgCount++;
            }
        }
    }

    if (gyroAvgCount > 0U) {
        gyroSum_P /= static_cast<float>(gyroAvgCount);
        out.gyroOmega_B = this->dcm_BP * gyroSum_P;
    }
    if (accelAvgCount > 0U) {
        accelSum_P /= static_cast<float>(accelAvgCount);
        out.accel_B = this->dcm_BP * accelSum_P;
    }

    return out;
}

void AverageMimuDataAlgorithm::setGyroAveragingWindow(double const window) {
    if (window < 0.0 || window > kMaxAveragingWindowSec) {
        FSW_THROW_INVALID_ARGUMENT("gyroAveragingWindow cannot be smaller than 0.0 or greater than 2.0 seconds");
    }
    this->gyroAveragingWindowNs = static_cast<std::uint64_t>(window * kSec2Nano);
}

double AverageMimuDataAlgorithm::getGyroAveragingWindow() const {
    return static_cast<double>(this->gyroAveragingWindowNs) * kNano2Sec;
}

void AverageMimuDataAlgorithm::setAccelAveragingWindow(double const window) {
    if (window < 0.0 || window > kMaxAveragingWindowSec) {
        FSW_THROW_INVALID_ARGUMENT("accelAveragingWindow cannot be smaller than 0.0 or greater than 2.0 seconds");
    }
    this->accelAveragingWindowNs = static_cast<std::uint64_t>(window * kSec2Nano);
}

double AverageMimuDataAlgorithm::getAccelAveragingWindow() const {
    return static_cast<double>(this->accelAveragingWindowNs) * kNano2Sec;
}

void AverageMimuDataAlgorithm::setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn) {
    if (!isValidDcm(dcm_BPIn)) {
        FSW_THROW_INVALID_ARGUMENT("dcm_BP must be orthonormal with det=+1.");
    }
    this->dcm_BP = dcm_BPIn;
}

Eigen::Matrix3f AverageMimuDataAlgorithm::getDcmPltfToBdy() const { return this->dcm_BP; }
