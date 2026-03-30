#include "thrFiringSchmitt.h"
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

    this->algorithm.configure(this->thrConfInMsg());
    this->algorithm.reset();
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmitt::updateState(uint64_t callTime) {
    THRArrayCmdForceMsgF32Payload thrForceIn = this->thrForceInMsg();
    THRArrayOnTimeCmdMsgF32Payload thrOnTimeOut = this->algorithm.update(thrForceIn);
    this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);
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

/*! Getter method for controlPeriod.
 @return const float
*/
float ThrFiringSchmitt::getControlPeriod() const { return this->algorithm.getControlPeriod(); }

/*! Setter method for controlPeriod.
 @return void
 @param period [s] control period (time between two algorithm update calls)
 */
void ThrFiringSchmitt::setControlPeriod(const float period) { this->algorithm.setControlPeriod(period); }
