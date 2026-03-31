#include "thrFiringSchmittAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <algorithm>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 */
void ThrFiringSchmittAlgorithm::reset() { this->prevThrustState.fill(ThrusterState::OFF); }

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return ThrusterOnTimeCmd
 @param thrusterForceCmd The commanded thruster forces
 */
ThrusterOnTimeCmd ThrFiringSchmittAlgorithm::update(ThrusterForceCmd thrusterForceCmd) {
    ThrusterOnTimeCmd thrOnTimeOut{};

    /*! - Loop through thrusters */
    for (uint32_t i = 0U; i < this->numThrusters; ++i) {
        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
        if (this->thrustPulsingRegime == ThrustPulsingRegime::OFF_PULSING) {
            thrusterForceCmd.thrForce.at(i) += this->maxThrust.at(i);
        }

        /*! - Do not allow thrust requests less than zero */
        thrusterForceCmd.thrForce.at(i) = std::max(thrusterForceCmd.thrForce.at(i), 0.0F);
        /*! - Compute T_on from thrust request, max thrust, and control period */
        float onTime = thrusterForceCmd.thrForce.at(i) / this->maxThrust.at(i) * this->controlPeriod;

        /*! - Apply Schmitt trigger logic */
        if (onTime < this->thrMinFireTime) {
            /*! - Request is less than minimum fire time */
            const float level = onTime / this->thrMinFireTime; /* [-] duty cycle fraction */
            if (level >= this->levelOn) {
                this->prevThrustState.at(i) = ThrusterState::ON;
                onTime = this->thrMinFireTime;
            } else if (level <= this->levelOff) {
                this->prevThrustState.at(i) = ThrusterState::OFF;
                onTime = 0.0F;
            } else if (this->prevThrustState.at(i) == ThrusterState::ON) {
                onTime = this->thrMinFireTime;
            } else {
                onTime = 0.0F;
            }
        } else if (onTime >= this->controlPeriod) {
            /*! - Request is greater than control period then oversaturate onTime */
            this->prevThrustState.at(i) = ThrusterState::ON;
            onTime = this->onTimeSaturationFactor * this->controlPeriod;
        } else {
            /*! - Request is greater than minimum fire time and less than control period */
            this->prevThrustState.at(i) = ThrusterState::ON;
        }

        /*! Set the output data */
        thrOnTimeOut.onTimeRequest.at(i) = onTime;
    }
    return thrOnTimeOut;
}

/*! Setter method for thruster configurations.
 @return void
 @param thrusterConfig thruster array configuration
 */
void ThrFiringSchmittAlgorithm::setupThrusters(ThrusterArrayConfig const& thrusterConfig) {
    /*! - store the number of installed thrusters */
    this->numThrusters = thrusterConfig.numThrusters;
    /*! - loop over all thrusters and for each copy over maximum thrust */
    for (uint32_t i = 0U; i < this->numThrusters; ++i) {
        this->maxThrust.at(i) = thrusterConfig.thrusters.at(i).maxThrust;
    }
}

/**
 * @brief Get the ON and OFF duty cycle fractions.
 * @return std::array<float, 2U> The current ON (1st element) and OFF (2nd element) duty cycle fraction.
 */
std::array<float, 2U> ThrFiringSchmittAlgorithm::getLevelsOnOff() const { return {this->levelOn, this->levelOff}; }

/**
 * @brief Set the ON and OFF duty cycle fractions.
 * @param levelOn The new ON duty cycle fraction to set.
 * @param levelOff The new OFF duty cycle fraction to set.
 */
void ThrFiringSchmittAlgorithm::setLevelsOnOff(const float levelOn, const float levelOff) {
    if (levelOn <= 0.0 || levelOn > 1.0) {
        FSW_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.levelOn must be within the bounds 0.0 < levelOn <= 1.0.");
    }
    if (levelOff < 0.0 || levelOff >= 1.0) {
        FSW_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.levelOff must be within the bounds 0.0 <= levelOff < 1.0.");
    }
    if (levelOn < levelOff) {
        FSW_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.levelOn must not be less than ThrFiringSchmitt.levelOff.");
    }
    this->levelOn = levelOn;
    this->levelOff = levelOff;
}

/**
 * @brief Get the minimum ON time for thrusters.
 * @return float The current minimum ON time in seconds.
 */
float ThrFiringSchmittAlgorithm::getThrMinFireTime() const { return this->thrMinFireTime; }

/**
 * @brief Set the minimum ON time for thrusters.
 * @param time The new minimum ON time in seconds to set.
 */
void ThrFiringSchmittAlgorithm::setThrMinFireTime(float time) {
    if (time <= 0.0) {
        FSW_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.thrMinFireTime must be positive.");
    }
    this->thrMinFireTime = time;
}

/*! Getter method for thrustPulsingRegime.
 @return ThrustPulsingRegime
 */
ThrustPulsingRegime ThrFiringSchmittAlgorithm::getThrustPulsingRegime() const { return this->thrustPulsingRegime; }

/*! Setter method for thrustPulsingRegime.
 @return void
 @param pulsingRegime the pulsing regime (ON_PULSING or OFF_PULSING)
 */
void ThrFiringSchmittAlgorithm::setThrustPulsingRegime(const ThrustPulsingRegime pulsingRegime) {
    this->thrustPulsingRegime = pulsingRegime;
}

/*! Setter method for controlPeriod.
 @return void
 @param period [s] control period (time between two algorithm update calls)
 */
void ThrFiringSchmittAlgorithm::setControlPeriod(const float period) {
    if (period <= 0.0) {
        FSW_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.controlPeriod must be > 0.0");
    }
    this->controlPeriod = period;
}

/*! Getter method for controlPeriod.
 @return const float
*/
float ThrFiringSchmittAlgorithm::getControlPeriod() const { return this->controlPeriod; }

/*! Setter method for onTimeSaturationFactor.
 @return void
 @param factor [-] must be >= 1.0
 */
void ThrFiringSchmittAlgorithm::setOnTimeSaturationFactor(const float factor) {
    if (factor < 1.0) {
        FSW_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.onTimeSaturationFactor must be >= 1.0");
    }
    this->onTimeSaturationFactor = factor;
}

/*! Getter method for onTimeSaturationFactor.
 @return float
 */
float ThrFiringSchmittAlgorithm::getOnTimeSaturationFactor() const { return this->onTimeSaturationFactor; }
