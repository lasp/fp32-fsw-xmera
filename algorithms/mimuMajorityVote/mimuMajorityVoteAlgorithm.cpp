#include "mimuMajorityVoteAlgorithm.h"

MimuMajorityVoteAlgorithm::MimuMajorityVoteAlgorithm(const MimuMajorityVoteConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void MimuMajorityVoteAlgorithm::setConfig(const MimuMajorityVoteConfig& config) { this->cfg = config; }

void MimuMajorityVoteAlgorithm::reInitialize() {
    this->gyroFaultPersistenceCount.fill(0U);
    this->accelFaultPersistenceCount.fill(0U);
}

namespace {

/*! Runs the IMU majority vote on one measured quantity: averages the measurements, finds the worst
 outlier, advances its persistence counter when it exceeds the threshold, and once the counter
 reaches the persistence limit rejects that IMU and re-averages the remaining ones. The
 persistenceCount array carries over between calls and is updated in place. */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
MimuVoteResult majorityVote(const std::array<Eigen::Vector3f, kMimuCount>& measurements,
                            float threshold,
                            uint32_t persistenceLimit,
                            std::array<uint32_t, kMimuCount>& persistenceCount) {
    // Stage 1: Compute average of all IMUs and find differences from average
    Eigen::Vector3f average = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < kMimuCount; ++i) {
        average += measurements.at(i);
    }
    average /= static_cast<float>(kMimuCount);

    MimuVoteResult result{};
    result.imuValid.fill(true);
    size_t maxDiffIndex = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        result.imuDifferenceMag.at(i) = (measurements.at(i) - average).norm();
        if (result.imuDifferenceMag.at(i) > result.imuDifferenceMag.at(maxDiffIndex)) {
            maxDiffIndex = i;
        }
    }

    // Update persistence counter for the worst outlier
    bool faultDetected = false;
    if (result.imuDifferenceMag.at(maxDiffIndex) >= threshold) {
        ++persistenceCount.at(maxDiffIndex);

        // Determine if the outlier has persisted long enough to be faulted
        if (persistenceCount.at(maxDiffIndex) >= persistenceLimit) {
            faultDetected = true;
            result.imuValid.at(maxDiffIndex) = false;
        }
    } else {
        persistenceCount.at(maxDiffIndex) = 0U;
    }

    // Reset counters for non-outlier IMUs
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (i != maxDiffIndex) {
            persistenceCount.at(i) = 0U;
        }
    }

    if (!faultDetected) {
        // No persisted fault - return full average
        result.average = average;
    } else {
        // Outlier persisted - exclude it and average the remaining IMUs
        result.faultDetected = true;
        result.average = (average * static_cast<float>(kMimuCount) - measurements.at(maxDiffIndex)) /
                         static_cast<float>(kMimuCount - 1U);
    }

    return result;
}

}  // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
MimuMajorityVoteOutput MimuMajorityVoteAlgorithm::update(const std::array<Eigen::Vector3f, kMimuCount>& imuOmegas_BN_B,
                                                         const std::array<Eigen::Vector3f, kMimuCount>& imuAccels_B) {
    // Independent majority votes on the gyro (angular velocity) and accelerometer measurements.
    MimuMajorityVoteOutput output{};
    output.gyro = majorityVote(imuOmegas_BN_B,
                               this->cfg.getOmegaThreshold(),
                               this->cfg.getGyroFaultPersistenceLimit(),
                               this->gyroFaultPersistenceCount);
    output.accel = majorityVote(imuAccels_B,
                                this->cfg.getAccelThreshold(),
                                this->cfg.getAccelFaultPersistenceLimit(),
                                this->accelFaultPersistenceCount);
    return output;
}
