#include "thrFiringRemainder.h"

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <algorithm>
#include <memory>
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->thrConfInMsg.isLinked()) {
        throw std::invalid_argument("thrFiringRemainder.thrConfInMsg wasn't connected.");
    }
    if (!this->thrForceInMsg.isLinked()) {
        throw std::invalid_argument("thrFiringRemainder.thrForceInMsg wasn't connected.");
    }

    /*! - read in the thruster configuration message and map to the validated thruster array */
    const auto [numThrusters, thrusters] = this->thrConfInMsg();
    ThrFiringRemainderThrusterArray thrusterArray{};
    thrusterArray.numThrusters = numThrusters;
    for (std::uint32_t i = 0U; i < numThrusters && i < kMaxThrusterCount; ++i) {
        thrusterArray.maxThrust.at(i) = thrusters[i].maxThrust;
    }

    const ThrFiringControlParameters controlParameters{
        this->thrMinFireTime, this->controlPeriod, this->onTimeSaturationFactor, this->thrustPulsingRegime};

    const auto config = ThrFiringRemainderConfig::create(thrusterArray, controlParameters);
    this->algorithm = std::make_unique<ThrFiringRemainderAlgorithm>(config);
}

ThrFiringRemainderConfig ThrFiringRemainder::toConfig() {
    const auto [numThrusters, thrusters] = this->thrConfInMsg();
    ThrFiringRemainderThrusterArray thrusterArray{};
    thrusterArray.numThrusters = numThrusters;
    for (std::uint32_t i = 0U; i < numThrusters && i < kMaxThrusterCount; ++i) {
        thrusterArray.maxThrust.at(i) = thrusters[i].maxThrust;
    }

    const ThrFiringControlParameters controlParameters{
        this->thrMinFireTime, this->controlPeriod, this->onTimeSaturationFactor, this->thrustPulsingRegime};

    return ThrFiringRemainderConfig::create(thrusterArray, controlParameters);
}

void ThrFiringRemainder::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrFiringRemainder reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

void ThrFiringRemainder::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrFiringRemainder reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrFiringRemainder reset() has not been called.");
    }

    /*! - read in the force command message and map to freestanding type */
    const auto [thrForce] = this->thrForceInMsg();
    ThrusterForceCmd thrusterForceCmd{};
    std::ranges::copy(thrForce, thrusterForceCmd.thrForce.begin());

    /*! - call algorithm update */
    const auto [onTimeRequest] = this->algorithm->update(thrusterForceCmd);

    /*! - map freestanding type back to message payload and write */
    THRArrayOnTimeCmdMsgF32Payload onTimeMsgOut{};
    std::ranges::copy(onTimeRequest, onTimeMsgOut.onTimeRequest);
    this->onTimeOutMsg.write(onTimeMsgOut, this->moduleID, callTime);
}
