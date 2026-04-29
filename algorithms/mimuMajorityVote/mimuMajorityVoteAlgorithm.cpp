#include "mimuMajorityVoteAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "utilities/freestandingInvalidArgument.h"

void MimuMajorityVoteAlgorithm::reset() { this->faultPersistenceCount.fill(0U); }

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
    if (output.omegaDifferencesMag.at(maxDiffIndex) >= this->omegaThreshold) {
        ++this->faultPersistenceCount.at(maxDiffIndex);

        // Determine if the outlier has persisted long enough to be faulted
        if (this->faultPersistenceCount.at(maxDiffIndex) >= this->faultPersistenceLimit) {
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

void MimuMajorityVoteAlgorithm::setOmegaThreshold(const float omegaThresholdIn) {
    if (omegaThresholdIn <= 0.0F) {
        FSW_THROW_INVALID_ARGUMENT("Zero or negative omegaThreshold is not valid");
    }
    this->omegaThreshold = omegaThresholdIn;
}

float MimuMajorityVoteAlgorithm::getOmegaThreshold() const { return this->omegaThreshold; }

void MimuMajorityVoteAlgorithm::setFaultPersistenceLimit(const uint32_t faultPersistenceLimitIn) {
    if (faultPersistenceLimitIn <= 0U) {
        FSW_THROW_INVALID_ARGUMENT("faultPersistenceLimit must be at least 1");
    }
    this->faultPersistenceLimit = faultPersistenceLimitIn;
}

uint32_t MimuMajorityVoteAlgorithm::getFaultPersistenceLimit() const { return this->faultPersistenceLimit; }
