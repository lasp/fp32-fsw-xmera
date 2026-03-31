#include "thrFiringSchmitt.h"

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"

#include <algorithm>
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

    /*! - read in the support messages and map to freestanding type */
    const auto [numThrusters, thrusters] = this->thrConfInMsg();
    ThrusterArrayConfig thrusterConfig{};
    thrusterConfig.numThrusters = numThrusters;
    for (std::uint32_t i = 0; i < numThrusters; ++i) {
        thrusterConfig.thrusters.at(i).rThrust_B = {
            thrusters[i].rThrust_B[0], thrusters[i].rThrust_B[1], thrusters[i].rThrust_B[2]};
        thrusterConfig.thrusters.at(i).tHatThrust_B = {
            thrusters[i].tHatThrust_B[0], thrusters[i].tHatThrust_B[1], thrusters[i].tHatThrust_B[2]};
        thrusterConfig.thrusters.at(i).maxThrust = thrusters[i].maxThrust;
    }

    this->algorithm.setupThrusters(thrusterConfig);
    this->algorithm.reset();
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmitt::updateState(uint64_t callTime) {
    /*! - read in the force command message and map to freestanding type */
    const auto [thrForce] = this->thrForceInMsg();
    ThrusterForceCmd thrusterForceCmd{};
    std::ranges::copy(thrForce, thrusterForceCmd.thrForce.begin());

    /*! - call algorithm update */
    const auto [onTimeRequest] = this->algorithm.update(thrusterForceCmd);

    /*! - map freestanding type back to message payload and write */
    THRArrayOnTimeCmdMsgF32Payload onTimeMsgOut{};
    std::ranges::copy(onTimeRequest, onTimeMsgOut.onTimeRequest);
    this->onTimeOutMsg.write(&onTimeMsgOut, this->moduleID, callTime);
}

/*! Setter method for ON and OFF duty cycle fractions.
 @return void
 @param levelOn [-] ON duty cycle fraction
 @param levelOff [-] OFF duty cycle fraction
 */
void ThrFiringSchmitt::setLevelsOnOff(const float levelOn, const float levelOff) {
    this->algorithm.setLevelsOnOff(levelOn, levelOff);
}

/*! Getter method for ON and OFF duty cycle fractions.
 @return std::array<float, 2U> containing levelOn (index 0) and levelOff (index 1)
 */
std::array<float, 2U> ThrFiringSchmitt::getLevelsOnOff() const { return this->algorithm.getLevelsOnOff(); }

/**
 * @brief Get the minimum ON time for thrusters.
 * @return float The current minimum ON time in seconds.
 */
float ThrFiringSchmitt::getThrMinFireTime() const { return this->algorithm.getThrMinFireTime(); }

/**
 * @brief Set the minimum ON time for thrusters.
 * @param time The new minimum ON time in seconds to set.
 */
void ThrFiringSchmitt::setThrMinFireTime(float time) { this->algorithm.setThrMinFireTime(time); }

/*! Getter method for thrustPulsingRegime.
 @return ThrustPulsingRegime
 */
ThrustPulsingRegime ThrFiringSchmitt::getThrustPulsingRegime() const {
    return this->algorithm.getThrustPulsingRegime();
}

/*! Setter method for thrustPulsingRegime.
 @return void
 @param pulsingRegime the pulsing regime (ON_PULSING or OFF_PULSING)
 */
void ThrFiringSchmitt::setThrustPulsingRegime(const ThrustPulsingRegime pulsingRegime) {
    this->algorithm.setThrustPulsingRegime(pulsingRegime);
}

/*! Getter method for controlPeriod.
 @return const float
*/
float ThrFiringSchmitt::getControlPeriod() const { return this->algorithm.getControlPeriod(); }

/*! Setter method for controlPeriod.
 @return void
 @param period [s] control period (time between two algorithm update calls)
 */
void ThrFiringSchmitt::setControlPeriod(const float period) { this->algorithm.setControlPeriod(period); }

/*! Setter method for onTimeSaturationFactor.
 @return void
 @param factor [-] must be >= 1.0
 */
void ThrFiringSchmitt::setOnTimeSaturationFactor(const float factor) {
    this->algorithm.setOnTimeSaturationFactor(factor);
}

/*! Getter method for onTimeSaturationFactor.
 @return float
 */
float ThrFiringSchmitt::getOnTimeSaturationFactor() const { return this->algorithm.getOnTimeSaturationFactor(); }
