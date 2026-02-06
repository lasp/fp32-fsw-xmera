/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "thrFiringRemainder.h"

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"

#include <algorithm>

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
    this->algorithm.reset(thrusterConfig);
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::updateState(const uint64_t callTime) {
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
