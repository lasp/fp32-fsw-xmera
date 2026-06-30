#include "mimuMajorityVoteAlgorithm.h"

MimuMajorityVoteAlgorithm::MimuMajorityVoteAlgorithm(const MimuMajorityVoteConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void MimuMajorityVoteAlgorithm::setConfig(const MimuMajorityVoteConfig& config) { this->cfg = config; }

void MimuMajorityVoteAlgorithm::reInitialize() { this->faultPersistenceCount.fill(0U); }

MimuMajorityVoteOutput MimuMajorityVoteAlgorithm::update(
    const std::array<Eigen::Vector3f, kMimuCount>& imuOmegas_BN_B) {
    // Stage 1: Compute average of all IMUs and find differences from average
    Eigen::Vector3f omegaAverage_BN_B = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < kMimuCount; ++i) {
        omegaAverage_BN_B += imuOmegas_BN_B.at(i);
    }
    omegaAverage_BN_B /= static_cast<float>(kMimuCount);

    MimuMajorityVoteOutput output{};
    output.validImus.fill(true);
    size_t maxDiffIndex = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        output.omegaDifferencesMag.at(i) = (imuOmegas_BN_B.at(i) - omegaAverage_BN_B).norm();
        if (output.omegaDifferencesMag.at(i) > output.omegaDifferencesMag.at(maxDiffIndex)) {
            maxDiffIndex = i;
        }
    }

    // Update persistence counter for the worst outlier
    bool faultDetected = false;
    if (output.omegaDifferencesMag.at(maxDiffIndex) >= this->cfg.getOmegaThreshold()) {
        ++this->faultPersistenceCount.at(maxDiffIndex);

        // Determine if the outlier has persisted long enough to be faulted
        if (this->faultPersistenceCount.at(maxDiffIndex) >= this->cfg.getFaultPersistenceLimit()) {
            faultDetected = true;
            output.validImus.at(maxDiffIndex) = false;
        }
    } else {
        this->faultPersistenceCount.at(maxDiffIndex) = 0U;
    }

    // Reset counters for non-outlier IMUs
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (i != maxDiffIndex) {
            this->faultPersistenceCount.at(i) = 0U;
        }
    }

    if (!faultDetected) {
        // No persisted fault - return full average
        output.avgOmega_BN_B = omegaAverage_BN_B;
    } else {
        // Outlier persisted - exclude it and average the remaining IMUs
        output.faultDetected = true;
        output.avgOmega_BN_B = (omegaAverage_BN_B * static_cast<float>(kMimuCount) - imuOmegas_BN_B.at(maxDiffIndex)) /
                               static_cast<float>(kMimuCount - 1U);
    }

    return output;
}
