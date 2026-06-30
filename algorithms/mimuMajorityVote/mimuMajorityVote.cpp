#include "mimuMajorityVote.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

void MimuMajorityVote::reset(uint64_t const callTime) {
    if (this->actualNumberOfImus != kMimuCount) {
        throw std::invalid_argument(
            "Number of connected IMU messages must equal kMimuCount (3); call addImuInput() exactly 3 times.");
    }
    this->algorithm = std::make_unique<MimuMajorityVoteAlgorithm>(
        MimuMajorityVoteConfig::create(this->omegaThreshold, this->faultPersistenceLimit));
}

void MimuMajorityVote::updateState(uint64_t const callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MimuMajorityVote reset() has not been called.");
    }

    // Convert message payloads to algorithm input type
    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B = {};
    for (size_t index = 0U; index < kMimuCount; ++index) {
        auto payload = this->imuMessages.at(index).imuSensorBodyInMsg();
        imuOmegas_BN_B.at(index) = cArrayToEigenVector(payload.AngVelBody);
    }

    MimuMajorityVoteOutput output = this->algorithm->update(imuOmegas_BN_B);

    // Convert algorithm output to message payloads
    IMUSensorBodyMsgF32Payload imuOutPayload{};
    eigenVectorToCArray(output.avgOmega_BN_B, imuOutPayload.AngVelBody);

    MimuFaultMsgPayload faultPayload{};
    faultPayload.faultDetected = output.faultDetected;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        faultPayload.validImus[i] = output.validImus.at(i);
        faultPayload.omegaDifferencesMag[i] = output.omegaDifferencesMag.at(i);
    }

    this->imuSensorBodyOutMsg.write(imuOutPayload, this->moduleID, callTime);
    this->mimuFaultMsg.write(faultPayload, this->moduleID, callTime);
}

/*! Re-validate the current module parameters and push them onto the live algorithm without resetting
 its persistence counters. Rebuilds the validated config from the public members via setConfig(). */
void MimuMajorityVote::reConfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MimuMajorityVote reConfigure() before reset().");
    }
    this->algorithm->setConfig(MimuMajorityVoteConfig::create(this->omegaThreshold, this->faultPersistenceLimit));
}

/*! Reset the algorithm's fault persistence counters without rebuilding the config; a simple
 pass-through to the algorithm's reInitialize(). */
void MimuMajorityVote::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MimuMajorityVote reInitialize() before reset().");
    }
    this->algorithm->reInitialize();
}

// Add imu to the majority vote module
void MimuMajorityVote::addImuInput(const ImuMessage& imu) {
    if (this->actualNumberOfImus >= kMimuCount) {
        throw std::invalid_argument("Cannot add more than kMimuCount (3) IMU inputs");
    }
    this->imuMessages.at(this->actualNumberOfImus) = imu;
    this->actualNumberOfImus++;
}
