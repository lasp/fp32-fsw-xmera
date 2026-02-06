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
 @param thrusterConfig The thruster configuration data
 */
void ThrFiringRemainderAlgorithm::reset(const ThrusterArrayConfig& thrusterConfig) {
    /*! - store the number of installed thrusters */
    this->numThrusters = thrusterConfig.numThrusters;
    this->pulseRemainder = {0.0F};

    /*! - loop over all thrusters and for each copy over maximum thrust, zero the impulse remainder */
    for (std::uint32_t i = 0; i < this->numThrusters; ++i) {
        this->maxThrust.at(i) = thrusterConfig.thrusters.at(i).maxThrust;
    }
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return ThrusterOnTimeCmd
 @param thrusterForceCmd The commanded thruster forces
 */
ThrusterOnTimeCmd ThrFiringRemainderAlgorithm::update(ThrusterForceCmd thrusterForceCmd) {
    ThrusterOnTimeCmd thrOnTimeOut{};

    /*! - Loop through thrusters */
    for (std::uint32_t i = 0; i < this->numThrusters; ++i) {
        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
        if (this->thrustPulsingRegime == ThrustPulsingRegime::OFF_PULSING) {
            thrusterForceCmd.thrForce.at(i) += this->maxThrust.at(i);
        }

        /*! - Do not allow thrust requests less than zero */
        if (thrusterForceCmd.thrForce.at(i) < 0.0F) {
            thrusterForceCmd.thrForce.at(i) = 0.0F;
        }

        /*! - Compute T_on from thrust request, max thrust, and control period */
        float onTime = thrusterForceCmd.thrForce.at(i) / this->maxThrust.at(i) * this->controlPeriod;
        /*! - Add in remainder from the last control step */
        onTime += this->pulseRemainder.at(i) * this->thrMinFireTime;
        /*! - Set pulse remainder to zero. Remainder now stored in onTime */
        this->pulseRemainder.at(i) = 0.0F;

        /* Pulse remainder logic */
        if (onTime < this->thrMinFireTime) {
            /*! - If request is less than minimum pulse time zero onTime and store remainder */
            this->pulseRemainder.at(i) = onTime / this->thrMinFireTime;
            onTime = 0.0F;
        } else if (onTime >= this->controlPeriod) {
            /*! - If request is greater than control period then oversaturate onTime */
            onTime = this->onTimeSaturationFactor * this->controlPeriod;
        } else {
            /* no action required. else clause included for MISRA */
        }

        /*! - Set the output data for each thruster */
        thrOnTimeOut.onTimeRequest.at(i) = onTime;
    }

    return thrOnTimeOut;
}

/*! Setter method for thrMinFireTime.
 @return void
 @param minFireTime
*/
void ThrFiringRemainderAlgorithm::setThrMinFireTime(const float minFireTime) {
    if (minFireTime < 0.0) {
        FS_THROW_INVALID_ARGUMENT("ThrFiringRemainderAlgorithm::thrMinFireTime cannot be < 0.0");
    }
    this->thrMinFireTime = minFireTime;
}

/*! Getter method for thrMinFireTime.
 @return const float
*/
float ThrFiringRemainderAlgorithm::getThrMinFireTime() const { return this->thrMinFireTime; }

/*! Setter method for thrustPulsingRegime.
 @return void
 @param pulsingRegime
*/
void ThrFiringRemainderAlgorithm::setThrustPulsingRegime(const ThrustPulsingRegime pulsingRegime) {
    this->thrustPulsingRegime = pulsingRegime;
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

/*! Setter method for onTimeSaturationFactor.
 @return void
 @param factor
*/
void ThrFiringRemainderAlgorithm::setOnTimeSaturationFactor(const float factor) {
    this->onTimeSaturationFactor = factor;
}

/*! Getter method for onTimeSaturationFactor.
 @return factor
*/
float ThrFiringRemainderAlgorithm::getOnTimeSaturationFactor() const { return this->onTimeSaturationFactor; }
