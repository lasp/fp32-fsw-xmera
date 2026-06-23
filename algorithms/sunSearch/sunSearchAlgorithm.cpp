#include "sunSearchAlgorithm.h"

#include "utilities/fsw/timeConstants.h"

SunSearchAlgorithm::SunSearchAlgorithm(const SunSearchConfig& config) : cfg(config) { setConfig(config); }

void SunSearchAlgorithm::setConfig(const SunSearchConfig& config) {
    this->cfg = config;
    this->precomputeEndTimes();

    // Re-arm: the update() call will capture the sequence start time
    this->firstPass = true;
}

SunSearchOutput SunSearchAlgorithm::update(const uint64_t callTime, const Eigen::Vector3f& omega_BN_B) {
    // On the first call after (re)configuration, latch the current time as the sequence start
    if (this->firstPass) {
        this->sunSearchStartTime = callTime;
        this->firstPass = false;
    }
    const uint64_t elapsedTimeNs = callTime - this->sunSearchStartTime;

    // Determine which rotation is active based on elapsed time and calculate desired angular
    // velocity. Once the sequence has finished, hold the last slot's commanded velocity.
    uint32_t activeIndex = kNumRotations - 1U;
    for (uint32_t rotationIndex = 0U; rotationIndex < kNumRotations; ++rotationIndex) {
        if (elapsedTimeNs < this->rotationEndTimes.at(rotationIndex)) {
            activeIndex = rotationIndex;
            break;
        }
    }
    const RotationProperties& rot = this->cfg.getRotations().at(activeIndex);
    const Eigen::Vector3f omega_RN_B =
        Eigen::Vector3f::Unit(static_cast<Eigen::Index>(rot.rotationAxis)) * rot.rotationRate;

    return SunSearchOutput{.omega_RN_B = omega_RN_B, .omega_BR_B = omega_BN_B - omega_RN_B};
}

void SunSearchAlgorithm::precomputeEndTimes() {
    uint64_t cumulativeEndTimeNs = 0U;
    for (uint32_t i = 0U; i < kNumRotations; ++i) {
        cumulativeEndTimeNs +=
            static_cast<uint64_t>(static_cast<double>(this->cfg.getRotations().at(i).rotationDuration) * kSec2Nano);
        this->rotationEndTimes.at(i) = cumulativeEndTimeNs;
    }
}
