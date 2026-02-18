/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "thrFiringRemainder.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->thrConfInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrConfInMsg wasn't connected.");
    }
    if (!this->thrForceInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrForceInMsg wasn't connected.");
    }

    /*! - read in the support messages */
    const THRArrayConfigMsgF32Payload localThrusterData = this->thrConfInMsg();
    this->algorithm.reset(localThrusterData);
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::updateState(uint64_t callTime) {
    THRArrayCmdForceMsgF32Payload const thrForceIn = this->thrForceInMsg();

    THRArrayOnTimeCmdMsgF32Payload thrOnTimeOut = this->algorithm.update(callTime, thrForceIn);
    this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);
}

/*! Setter method for thrMinFireTime.
 @return void
 @param thrMinFireTime
*/
void ThrFiringRemainder::setThrMinFireTime(const double thrMinFireTime) {
    this->algorithm.setThrMinFireTime(static_cast<float>(thrMinFireTime));
}

/*! Getter method for thrMinFireTime.
 @return const double
*/
double ThrFiringRemainder::getThrMinFireTime() const { return this->algorithm.getThrMinFireTime(); }

/*! Setter method for thrustPulsingRegime.
 @return void
 @param thrustPulsingRegime
*/
void ThrFiringRemainder::setThrustPulsingRegime(const ThrustPulsingRegime thrustPulsingRegime) {
    this->algorithm.setThrustPulsingRegime(thrustPulsingRegime);
}

/*! Getter method for thrustPulsingRegime.
 @return ThrustPulsingRegime
*/
ThrustPulsingRegime ThrFiringRemainder::getThrustPulsingRegime() const {
    return this->algorithm.getThrustPulsingRegime();
}

/*! Setter method for controlPeriod.
 @return void
 @param controlPeriod
*/
void ThrFiringRemainder::setControlPeriod(const double controlPeriod) {
    this->algorithm.setControlPeriod(static_cast<float>(controlPeriod));
}

/*! Getter method for controlPeriod.
 @return const double
*/
double ThrFiringRemainder::getControlPeriod() const { return this->algorithm.getControlPeriod(); }
