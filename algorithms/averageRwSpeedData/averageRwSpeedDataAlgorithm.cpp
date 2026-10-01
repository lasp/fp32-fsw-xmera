#include "averageRwSpeedDataAlgorithm.h"
#include <utilities/fsw/timeConstants.h>

AverageRwSpeedDataAlgorithm::AverageRwSpeedDataAlgorithm(const AverageRwSpeedDataConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void AverageRwSpeedDataAlgorithm::setConfig(const AverageRwSpeedDataConfig& config) {
    this->cfg = config;
    this->rwSpeedAveragingWindowNs = static_cast<std::uint64_t>(config.getRwSpeedAveragingWindow() * kSec2Nano);
}

void AverageRwSpeedDataAlgorithm::reInitialize() {
    this->ring = {};
    this->insertIdx = 0U;
}

/*! @brief Ingest new RW speed data into the internal ring, then return the rolling average
 * of fresh samples currently in the ring.
 *
 * Phase 1 (ingest): Each input sample carries a `measTime` that corresponds to the sample's timestamp. All
 * reaction wheels are assumed to have been measured at that time. A sample is ingested if it carries a
 *  nonzero `measTime`, including out-of-order samples. The whole sample and its timestamp are copied into
 *  the next ring slot, overwriting the oldest slot when capacity is reached.
 *
 *  Phase 2 (average): Per-sample times are derived from each ring slot's `measTime`. The maxTimeTag is the
 *  newest slot's tail sample. A sample contributes to the mean when its age relative to maxTimeTag is within
 *  `rwSpeedAveragingWindowNs`. Components with no in-window samples (or an empty ring) stay zero.
 *
 *  @param wheelData RwSpeedSample: RW speed sample from the caller.
 *  @return td::array<float, kMaxNumRw>: rolling average.
 */
std::array<float, kMaxNumRw> AverageRwSpeedDataAlgorithm::update(RwSpeedSample const& wheelData) {
    // Phase 1: Ingest samples. A packet enters the ring only if it carries a nonzero measTime.
    if (wheelData.measTime != 0U) {
        this->ring.at(this->insertIdx) = wheelData;
        this->insertIdx = (this->insertIdx + 1U) % kRingCapacity;
    }

    // Phase 2: compute the maxTimeTag from the newest stored tail sample.
    uint64_t maxTimeTag = 0U;
    for (auto const& slot : this->ring) {
        if (slot.measTime > maxTimeTag) {
            maxTimeTag = slot.measTime;
        }
    }

    // An empty ring leaves the count at zero, so the zero-initialized output is returned unchanged.
    std::array<float, kMaxNumRw> rwSpeedSum{};
    uint64_t rwSpeedSampleCount = 0U;
    for (const auto& [measTime, wheelSpeeds] : this->ring) {
        const uint64_t age = maxTimeTag - measTime;
        if (age <= this->rwSpeedAveragingWindowNs) {
            for (size_t wheelIdx = 0; const auto& wheelSpeed : wheelSpeeds) {
                rwSpeedSum.at(wheelIdx) += wheelSpeed;
                ++wheelIdx;
            }
            rwSpeedSampleCount++;
        }
    }

    std::array<float, kMaxNumRw> out{};
    if (rwSpeedSampleCount > 0U) {
        for (auto& wheelSpeedSum : rwSpeedSum) {
            wheelSpeedSum /= static_cast<float>(rwSpeedSampleCount);
        }
        out = rwSpeedSum;
    }
    return out;
}
