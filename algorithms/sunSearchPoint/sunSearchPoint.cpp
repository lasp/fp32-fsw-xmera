#include "sunSearchPoint.h"

#include "utilities/xmera/xmeraLifecycleException.h"
#include <utilities/fsw/eigenSupport.h>
#include <stdexcept>

/*! Reset method for the BSK module adapter interface. Validates that required input messages are
 linked and (re)builds the algorithm from the configured parameters; an unconfigured rotation
 sequence is rejected here by SunSearchPointConfig::create.
 @return void
 @param callTime [ns] Time the method is called
*/
void SunSearchPoint::reset(uint64_t callTime) {
    if (!this->sunDirectionInMsg.isLinked()) {
        throw std::invalid_argument("sunSearchPoint.sunDirectionInMsg wasn't connected.");
    }
    if (!this->rateInMsg.isLinked()) {
        throw std::invalid_argument("sunSearchPoint.rateInMsg wasn't connected.");
    }
    if (!this->filterResidualsInMsg.isLinked()) {
        throw std::invalid_argument("sunSearchPoint.filterResidualsInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<SunSearchPointAlgorithm>(SunSearchPointConfig::create(
        this->rotations, this->sHatBdyCmd, this->sunAxisSpinRate, this->omega_RN_B, this->observationThreshold));
}

/*! Re-validate the current module parameters and push them onto the live algorithm without
 re-initializing its runtime state (the search phase is preserved). Rebuilds the validated config
 from the public members and installs it via setConfig().
 @return void
*/
void SunSearchPoint::reConfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunSearchPoint reConfigure() before reset().");
    }
    this->algorithm->setConfig(SunSearchPointConfig::create(
        this->rotations, this->sHatBdyCmd, this->sunAxisSpinRate, this->omega_RN_B, this->observationThreshold));
}

/*! Re-arm the algorithm's runtime state machine (the search phase) without rebuilding the config; a
 simple pass-through to the algorithm's reInitialize().
 @return void
*/
void SunSearchPoint::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunSearchPoint reInitialize() before reset().");
    }
    this->algorithm->reInitialize();
}

/*! Update method for the BSK module adapter interface. This method also calls the algorithm update method.
 @return void
 @param callTime [ns] Time the method is called
*/
void SunSearchPoint::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunSearchPoint reset() has not been called.");
    }

    auto rateMsgPayload = NavAttMsgF32Payload();
    if (this->rateInMsg.isWritten()) {
        rateMsgPayload = this->rateInMsg();
    }
    auto sunDirectionMsgPayload = NavAttMsgF32Payload();
    if (this->sunDirectionInMsg.isWritten()) {
        sunDirectionMsgPayload = this->sunDirectionInMsg();
    }
    auto filterResidualsMsgPayload = FilterResidualsMsgF32Payload();
    if (this->filterResidualsInMsg.isWritten()) {
        filterResidualsMsgPayload = this->filterResidualsInMsg();
    }

    Eigen::Vector3f const vehSunPntBdy = cArrayToEigenVector(sunDirectionMsgPayload.vehSunPntBdy);
    Eigen::Vector3f const omega_BN_B = cArrayToEigenVector(rateMsgPayload.omega_BN_B);

    // Call the algorithm update method
    SunSearchPointOutput output =
        this->algorithm->update(callTime, vehSunPntBdy, omega_BN_B, filterResidualsMsgPayload.sizeOfObservations);

    // Convert algorithm output to MsgPayload
    AttGuidMsgF32Payload attGuidanceOutBuffer{};
    eigenVectorToCArray(output.sigma_BR, attGuidanceOutBuffer.sigma_BR);
    eigenVectorToCArray(output.omega_BR_B, attGuidanceOutBuffer.omega_BR_B);
    eigenVectorToCArray(output.omega_RN_B, attGuidanceOutBuffer.omega_RN_B);

    this->attGuidanceOutMsg.write(attGuidanceOutBuffer, moduleID, callTime);

    const SunSearchPointFaultMsgPayload faultBuffer{output.faultDetected};
    this->sunSearchPointFaultOutMsg.write(faultBuffer, moduleID, callTime);
}

/*! Setter for a single rotation in the sun-search sequence. Applied at the next reset().
 @return void
 @param index Rotation slot index
 @param rotation Rotation properties to store
*/
void SunSearchPoint::setRotation(const uint32_t index, const RotationProperties& rotation) {
    this->rotations.at(index) = rotation;
}

/*! Getter for a single rotation in the sun-search sequence.
 @return RotationProperties
 @param index Rotation slot index
*/
RotationProperties SunSearchPoint::getRotation(const uint32_t index) const { return this->rotations.at(index); }
