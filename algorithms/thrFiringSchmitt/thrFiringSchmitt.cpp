#include "thrFiringSchmitt.h"

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <algorithm>
#include <memory>
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmitt::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->thrConfInMsg.isLinked()) {
        throw std::invalid_argument("thrFiringSchmitt.thrConfInMsg wasn't connected.");
    }
    if (!this->thrForceInMsg.isLinked()) {
        throw std::invalid_argument("thrFiringSchmitt.thrForceInMsg wasn't connected.");
    }

    /*! - read in the thruster configuration message and map to the validated thruster array */
    const auto [numThrusters, thrusters] = this->thrConfInMsg();
    ThrFiringSchmittThrusterArray thrusterArray{};
    thrusterArray.numThrusters = numThrusters;
    for (std::uint32_t i = 0U; i < numThrusters && i < kMaxThrusterCount; ++i) {
        thrusterArray.maxThrust.at(i) = thrusters[i].maxThrust;
    }

    const ThrFiringSchmittControlParameters controlParameters{this->levelOn,
                                                              this->levelOff,
                                                              this->thrMinFireTime,
                                                              this->controlPeriod,
                                                              this->onTimeSaturationFactor,
                                                              this->thrustPulsingRegime};

    const auto config = ThrFiringSchmittConfig::create(thrusterArray, controlParameters);
    this->algorithm = std::make_unique<ThrFiringSchmittAlgorithm>(config);
}

ThrFiringSchmittConfig ThrFiringSchmitt::toConfig() {
    const auto [numThrusters, thrusters] = this->thrConfInMsg();
    ThrFiringSchmittThrusterArray thrusterArray{};
    thrusterArray.numThrusters = numThrusters;
    for (std::uint32_t i = 0U; i < numThrusters && i < kMaxThrusterCount; ++i) {
        thrusterArray.maxThrust.at(i) = thrusters[i].maxThrust;
    }

    const ThrFiringSchmittControlParameters controlParameters{this->levelOn,
                                                              this->levelOff,
                                                              this->thrMinFireTime,
                                                              this->controlPeriod,
                                                              this->onTimeSaturationFactor,
                                                              this->thrustPulsingRegime};

    return ThrFiringSchmittConfig::create(thrusterArray, controlParameters);
}

void ThrFiringSchmitt::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrFiringSchmitt reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

void ThrFiringSchmitt::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrFiringSchmitt reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method maps the input thruster command forces into thruster on times using a Schmitt-trigger logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmitt::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrFiringSchmitt reset() has not been called.");
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
