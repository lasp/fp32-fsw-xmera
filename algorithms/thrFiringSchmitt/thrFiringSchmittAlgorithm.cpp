#include "thrFiringSchmittAlgorithm.h"
#include "../freestandingInvalidArgument.h"
#include <architecture/utilities/macroDefinitions.h>
#include <algorithm>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 */
void ThrFiringSchmittAlgorithm::reset() {
    this->prevCallTime = 0U;
    this->prevThrustState.fill(ThrusterState::OFF);
}

/*! This method configures the module by populating any necessary class members.
 @return void
 @param thrusterConfigPayload thruster config message payload
 */
void ThrFiringSchmittAlgorithm::configure(THRArrayConfigMsgF32Payload const& thrusterConfigPayload) {
    /*! - store the number of installed thrusters */
    this->numThrusters = thrusterConfigPayload.numThrusters;
    /*! - loop over all thrusters and for each copy over maximum thrust, set last state to off */
    for (uint32_t i = 0U; i < this->numThrusters; ++i) {
        this->maxThrust[i] = thrusterConfigPayload.thrusters[i].maxThrust;
    }
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param thrForceIn Thruster array commanded force message payload
 */
THRArrayOnTimeCmdMsgF32Payload ThrFiringSchmittAlgorithm::update(uint64_t callTime,
                                                                 THRArrayCmdForceMsgF32Payload& thrForceIn) {
    THRArrayOnTimeCmdMsgF32Payload thrOnTimeOut{}; /* -- thruster on-time output payload */

    std::array<float, MAX_EFF_CNT> thrForce{};
    std::ranges::copy(std::begin(thrForceIn.thrForce), std::end(thrForceIn.thrForce), std::begin(thrForce));

    /*! - the first time update() is called there is no information on the time step.  Here
     return either all thrusters off or on depending on the baseThrustState state */
    if (this->prevCallTime == 0U) {
        this->prevCallTime = callTime;

        for (uint32_t i = 0U; i < this->numThrusters; ++i) {
            constexpr float firstCallPulse = 2.0F;  // 2 seconds, needs to be greater than FSW update time step
            thrOnTimeOut.onTimeRequest[i] = this->baseThrustState == PulsingRegime::ONPULSING ? 0.0F : firstCallPulse;
        }
    } else {
        /*! - compute control time period Delta_t */
        const auto controlPeriod = static_cast<float>(static_cast<double>(callTime - this->prevCallTime) * NANO2SEC);
        this->prevCallTime = callTime;

        std::array<float, MAX_EFF_CNT> onTime{}; /* [s] array of commanded on time for thrusters */
        /*! - Loop through thrusters */
        for (uint32_t i = 0U; i < this->numThrusters; ++i) {
            /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
             needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
             be set to the maximum thrust value */
            if (this->baseThrustState == PulsingRegime::OFFPULSING) {
                thrForce[i] += this->maxThrust[i];
            }

            /*! - Do not allow thrust requests less than zero */
            thrForce[i] = std::max(thrForce[i], 0.0F);
            /*! - Compute T_on from thrust request, max thrust, and control period */
            onTime[i] = thrForce[i] / this->maxThrust[i] * controlPeriod;

            /*! - Apply Schmitt trigger logic */
            if (onTime[i] < this->thrMinFireTime) {
                /*! - Request is less than minimum fire time */
                const float level = onTime[i] / this->thrMinFireTime; /* [-] duty cycle fraction */
                if (level >= this->levelOn) {
                    this->prevThrustState[i] = ThrusterState::ON;
                    onTime[i] = this->thrMinFireTime;
                } else if (level <= this->levelOff) {
                    this->prevThrustState[i] = ThrusterState::OFF;
                    onTime[i] = 0.0F;
                } else if (this->prevThrustState[i] == ThrusterState::ON) {
                    onTime[i] = this->thrMinFireTime;
                } else {
                    onTime[i] = 0.0F;
                }
            } else if (onTime[i] >= controlPeriod) {
                /*! - Request is greater than control period then oversaturate onTime */
                this->prevThrustState[i] = ThrusterState::ON;
                constexpr float overSaturationFactor = 1.1F;  // oversaturate to avoid numerical error
                onTime[i] = overSaturationFactor * controlPeriod;
            } else {
                /*! - Request is greater than minimum fire time and less than control period */
                this->prevThrustState[i] = ThrusterState::ON;
            }

            /*! Set the output data */
            thrOnTimeOut.onTimeRequest[i] = onTime[i];
        }
    }
    return thrOnTimeOut;
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
        FS_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.levelOn must be within the bounds 0.0 < levelOn <= 1.0.");
    }
    if (levelOff < 0.0 || levelOff >= 1.0) {
        FS_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.levelOff must be within the bounds 0.0 <= levelOff < 1.0.");
    }
    if (levelOn < levelOff) {
        FS_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.levelOn must not be less than ThrFiringSchmitt.levelOff.");
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
        FS_THROW_INVALID_ARGUMENT("ThrFiringSchmitt.thrMinFireTime must be positive.");
    }
    this->thrMinFireTime = time;
}

/**
 * @brief Get the base thrust state.
 * @return int The current base thrust state (0 for off-pulsing, 1 for on-pulsing).
 */
PulsingRegime ThrFiringSchmittAlgorithm::getBaseThrustState() const { return this->baseThrustState; }

/**
 * @brief Set the base thrust state.
 * @param state The new base thrust state to set (0 for off-pulsing, 1 for on-pulsing).
 */
void ThrFiringSchmittAlgorithm::setBaseThrustState(PulsingRegime state) { this->baseThrustState = state; }
