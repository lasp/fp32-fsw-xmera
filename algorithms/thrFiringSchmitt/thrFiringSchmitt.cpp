#include "thrFiringSchmitt.h"
#include <architecture/utilities/macroDefinitions.h>
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

    this->algorithm.setLevelsOnOff(this->levelOn, this->levelOff);

    this->algorithm.configure(this->thrConfInMsg());
    this->algorithm.reset();

    this->prevCallTime = 0U;
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmitt::updateState(uint64_t callTime) {
    /*! - compute control time period Delta_t */
    float controlPeriod{};
    if (this->prevCallTime != 0U) {
        controlPeriod = static_cast<float>(static_cast<double>(callTime - this->prevCallTime) * NANO2SEC);
    }
    this->prevCallTime = callTime;

    THRArrayCmdForceMsgF32Payload thrForceIn = this->thrForceInMsg();
    THRArrayOnTimeCmdMsgF32Payload thrOnTimeOut = this->algorithm.update(controlPeriod, thrForceIn);
    this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);
}

/**
 * @brief Get the ON duty cycle fraction.
 * @return float The current ON duty cycle fraction.
 */
float ThrFiringSchmitt::getLevelOn() const { return this->levelOn; }

/**
 * @brief Set the ON duty cycle fraction.
 * @param level The new ON duty cycle fraction to set.
 */
void ThrFiringSchmitt::setLevelOn(float level) { this->levelOn = level; }

/**
 * @brief Get the OFF duty cycle fraction.
 * @return float The current OFF duty cycle fraction.
 */
float ThrFiringSchmitt::getLevelOff() const { return this->levelOff; }

/**
 * @brief Set the OFF duty cycle fraction.
 * @param level The new OFF duty cycle fraction to set.
 */
void ThrFiringSchmitt::setLevelOff(float level) { this->levelOff = level; }

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

/**
 * @brief Get the base thrust state.
 * @return int The current base thrust state (0 for off-pulsing, 1 for on-pulsing).
 */
uint32_t ThrFiringSchmitt::getBaseThrustState() const {
    return static_cast<uint32_t>(this->algorithm.getBaseThrustState());
}

/**
 * @brief Set the base thrust state.
 * @param state The new base thrust state to set (0 for off-pulsing, 1 for on-pulsing).
 */
void ThrFiringSchmitt::setBaseThrustState(uint32_t state) {
    this->algorithm.setBaseThrustState(static_cast<PulsingRegime>(state));
}

/**
 * @brief Get the first call pulse duration
 * @return float The duration of the first call pulse. This should be at least the duration of the control period (1/fsw_rate)
 */
float ThrFiringSchmitt::getFirstCallPulse() const { return this->algorithm.getFirstCallPulse(); }

/**
 * @brief Set the first call pulse duration
 * @param time The duration of the first call pulse. This should be at least the duration of the control period (1/fsw_rate)
 */
void ThrFiringSchmitt::setFirstCallPulse(float time) { this->algorithm.setFirstCallPulse(time); }
