#include "averageMimuDataAlgorithm.h"
#include <utilities/fsw/timeConstants.h>

AverageMimuDataAlgorithm::AverageMimuDataAlgorithm(const AverageMimuDataConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void AverageMimuDataAlgorithm::setConfig(const AverageMimuDataConfig& config) {
    this->cfg = config;
    this->gyroAveragingWindowNs = static_cast<std::uint64_t>(config.getGyroAveragingWindow() * kSec2Nano);
    this->accelAveragingWindowNs = static_cast<std::uint64_t>(config.getAccelAveragingWindow() * kSec2Nano);
}

void AverageMimuDataAlgorithm::reInitialize() {
    this->ring = {};
    this->insertIdx = 0U;
}

/*! @brief Ingest new packets from the input snapshot into the internal ring,
 *  then return the rolling average of fresh samples currently in the ring.
 *
 *  Phase 1 (ingest): Each input packet carries a `measTime` that is the
 *  first sample's timestamp. A packet is ingested if it is valid and carries a
 *  nonzero `measTime`, including out-of-order packets. The whole packet is
 *  copied into the next ring slot, overwriting the oldest slot when capacity is
 *  reached. Multiple packets within one snapshot can be ingested.
 *
 *  Phase 2 (average): Per-sample times are derived from each ring slot's
 *  `measTime` plus `s * kMimuSamplePeriodNs`. The maxTimeTag is the
 *  newest slot's tail sample: `max(slot.measTime) + (N - 1) * period_ns`.
 *  Gyro and acceleration data are averaged independently: a sample contributes to
 *  the gyro mean when its age relative to maxTimeTag is within
 *  `gyroAveragingWindowNs`, and to the accel mean when within
 *  `accelAveragingWindowNs`. Each mean is rotated to the body frame via
 *  `dcm_BC`. Components with no in-window samples (or an empty ring) stay zero.
 *
 *  @param localPkts InputPktsData: 4-packet snapshot from the caller.
 *  @return OutputAverageAccelAngleVel: body-frame rolling average.
 */
OutputAverageAccelAngleVel AverageMimuDataAlgorithm::update(InputPktsData const& localPkts) {
    // Phase 1: Ingest packets. A packet enters the ring only if it is valid and
    // carries a nonzero measTime; out-of-order packets are accepted.
    for (const auto& [isValid, measTime, samples] : localPkts.packets) {
        if (!isValid || measTime == 0U) {
            continue;
        }

        this->ring.at(this->insertIdx).isValid = true;
        this->ring.at(this->insertIdx).measTime = measTime;
        this->ring.at(this->insertIdx).samples = samples;
        this->insertIdx = (this->insertIdx + 1U) % kRingCapacity;
    }

    // Phase 2: compute the maxTimeTag from the newest stored packet's tail sample.
    // Per-sample measTimes are reconstructed from slot.measTime + s * period_ns.
    uint64_t maxSlotMeasTime = 0U;
    for (auto const& slot : this->ring) {
        if (slot.isValid && (slot.measTime > maxSlotMeasTime)) {
            maxSlotMeasTime = slot.measTime;
        }
    }

    const uint64_t maxTimeTag = maxSlotMeasTime + ((MAX_MIMU_SAMPLES_PER_PKT_C - 1U) * kMimuSamplePeriodNs);

    // Gyro and accel each accumulate over their own window, so a sample may
    // contribute to one running mean and not the other. An empty ring leaves
    // both counts at zero, so the zero-initialized output is returned unchanged.
    Eigen::Vector3f gyroSum_C = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_C = Eigen::Vector3f::Zero();
    uint64_t gyroAvgCount = 0U;
    uint64_t accelAvgCount = 0U;

    for (const auto& [isValid, measTime, samples] : this->ring) {
        // Only valid measurements are ever stored, so isValid here just skips
        // ring slots that have not been written yet.
        if (!isValid) {
            continue;
        }
        for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT_C; ++s) {
            const uint64_t sampleMeasTime = measTime + (s * kMimuSamplePeriodNs);
            const uint64_t age = maxTimeTag - sampleMeasTime;
            if (age <= this->gyroAveragingWindowNs) {
                gyroSum_C += samples.at(s).gyro_C;
                gyroAvgCount++;
            }
            if (age <= this->accelAveragingWindowNs) {
                accelSum_C += samples.at(s).accel_C;
                accelAvgCount++;
            }
        }
    }

    OutputAverageAccelAngleVel out{};
    if (gyroAvgCount > 0U) {
        gyroSum_C /= static_cast<float>(gyroAvgCount);
        out.gyroOmega_B = this->cfg.getDcmChuToBody() * gyroSum_C;
    }
    if (accelAvgCount > 0U) {
        accelSum_C /= static_cast<float>(accelAvgCount);
        out.accel_B = this->cfg.getDcmChuToBody() * accelSum_C;
    }

    return out;
}
