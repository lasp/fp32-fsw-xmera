/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "thrFiringRemainderAlgorithm.h"
#include "thrFiringRemainderTypes.h"

#include "architecture/utilities/macroDefinitions.h"
#include "freestandingInvalidArgument.h"

#include <array>

/*! This method performs a complete reset of the algorithm.  All algorithm variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param thrConfigInMsgPayload The thruster configuration data
 */
void ThrFiringRemainderAlgorithm::reset(const THRArrayConfigMsgF32Payload& thrConfigInMsgPayload) {
    /*! - store the number of installed thrusters */
    this->numThrusters = thrConfigInMsgPayload.numThrusters;
    this->pulseRemainder = {0.0F};

    /*! - loop over all thrusters and for each copy over maximum thrust, zero the impulse remainder */
    for (int i = 0; i < this->numThrusters; i++) {
        this->maxThrust.at(i) = thrConfigInMsgPayload.thrusters[i].maxThrust;
    }
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param thrForceInMsgPayload The commanded thruster forces
 */
THRArrayOnTimeCmdMsgF32Payload ThrFiringRemainderAlgorithm::update(THRArrayCmdForceMsgF32Payload thrForceInMsgPayload) {
    std::array<float, MAX_EFF_CNT> onTime{}; /* [s] array of commanded on time for thrusters */
    THRArrayOnTimeCmdMsgF32Payload thrOnTimeOut = {}; /* [-] copy of the thruster on-time output message */

    /*! - Loop through thrusters */
    for (int i = 0; i < this->numThrusters; i++) {
        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
        if (this->thrustPulsingRegime == ThrustPulsingRegime::OFF_PULSING) {
            thrForceInMsgPayload.thrForce[i] += this->maxThrust.at(i);
        }

        /*! - Do not allow thrust requests less than zero */
        if (thrForceInMsgPayload.thrForce[i] < 0.0F) {
            thrForceInMsgPayload.thrForce[i] = 0.0F;
        }

        /*! - Compute T_on from thrust request, max thrust, and control period */
        onTime.at(i) = thrForceInMsgPayload.thrForce[i] / this->maxThrust.at(i) * this->controlPeriod;
        /*! - Add in remainder from the last control step */
        onTime.at(i) += this->pulseRemainder.at(i) * this->thrMinFireTime;
        /*! - Set pulse remainder to zero. Remainder now stored in onTime */
        this->pulseRemainder.at(i) = 0.0F;

        /* Pulse remainder logic */
        if (onTime.at(i) < this->thrMinFireTime) {
            /*! - If request is less than minimum pulse time zero onTime an store remainder */
            this->pulseRemainder.at(i) = onTime.at(i) / this->thrMinFireTime;
            onTime.at(i) = 0.0F;
        } else if (onTime.at(i) >= this->controlPeriod) {
            /*! - If request is greater than control period then oversaturate onTime */
            onTime.at(i) = 1.1F * this->controlPeriod;
        } else {
            /* no action required. else clause included for MISRA */
        }

        /*! - Set the output data for each thruster */
        thrOnTimeOut.OnTimeRequest[i] = onTime.at(i);
    }

    return thrOnTimeOut;
}

/*! Setter method for thrMinFireTime.
 @return void
 @param thrMinFireTime
*/
void ThrFiringRemainderAlgorithm::setThrMinFireTime(const float thrMinFireTime) {
    if (thrMinFireTime < 0.0) {
        FS_THROW_INVALID_ARGUMENT("ThrFiringRemainderAlgorithm::thrMinFireTime cannot be < 0.0");
    }
    this->thrMinFireTime = thrMinFireTime;
}

/*! Getter method for thrMinFireTime.
 @return const float
*/
float ThrFiringRemainderAlgorithm::getThrMinFireTime() const { return this->thrMinFireTime; }

/*! Setter method for thrustPulsingRegime.
 @return void
 @param thrustPulsingRegime
*/
void ThrFiringRemainderAlgorithm::setThrustPulsingRegime(const ThrustPulsingRegime thrustPulsingRegime) {
    this->thrustPulsingRegime = thrustPulsingRegime;
}

/*! Getter method for thrustPulsingRegime.
 @return ThrustPulsingRegime
*/
ThrustPulsingRegime ThrFiringRemainderAlgorithm::getThrustPulsingRegime() const { return this->thrustPulsingRegime; }

/*! Setter method for controlPeriod.
 @return void
 @param period
*/
void ThrFiringRemainderAlgorithm::setControlPeriod(const float period) {
    if (period <= 0.0) {
        FS_THROW_INVALID_ARGUMENT("ThrFiringRemainderAlgorithm::controlPeriod must be > 0.0");
    }
    this->controlPeriod = period;
}

/*! Getter method for controlPeriod.
 @return const float
*/
float ThrFiringRemainderAlgorithm::getControlPeriod() const { return this->controlPeriod; }
