#include "averageRwSpeedDataAlgorithm.h"
#include <utilities/fsw/timeConstants.h>

#include <algorithm>

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
 *  nonzero `measTime` and is not older than the newest stored sample by more than kMaxAveragingWindowSec.
 *  Out-of-order samples inside that limit are ingested. The whole sample and its timestamp are copied into
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
    uint64_t maxTimeTag = 0U;
    for (auto const& slot : this->ring) {
        if (slot.measTime > maxTimeTag) {
            maxTimeTag = slot.measTime;
        }
    }

    // Phase 1: Ingest the sample. A sample that no window can reach would only evict a useful slot.
    // The maximum window size is used in case the current window size changes with a reconfig call
    if (wheelData.measTime != 0U && maxTimeTag <= wheelData.measTime + average_rw_speed_detail::kMaxAveragingWindowNs) {
        this->ring.at(this->insertIdx) = wheelData;
        this->insertIdx = (this->insertIdx + 1U) % kRingCapacity;
        maxTimeTag = std::max(maxTimeTag, wheelData.measTime);
    }

    // Phase 2: average the samples within the window of the newest stored sample.

    // An empty ring leaves the count at zero, so the zero-initialized output is returned unchanged.
    std::array<float, kMaxNumRw> rwSpeedSum{};
    uint64_t rwSpeedSampleCount = 0U;
    for (const auto& [measTime, wheelSpeeds] : this->ring) {
        // A sample with measTime 0 is never ingested, so measTime 0 marks a slot not written yet.
        if (measTime != 0U && maxTimeTag <= measTime + this->rwSpeedAveragingWindowNs) {
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
