#include "thrFiringSchmittAlgorithm.h"
#include <algorithm>

/*! @brief Construct the algorithm with a validated configuration. The Schmitt-trigger state starts cleared. */
ThrFiringSchmittAlgorithm::ThrFiringSchmittAlgorithm(const ThrFiringSchmittConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replace the stored configuration at runtime. The Schmitt-trigger state is preserved. */
void ThrFiringSchmittAlgorithm::setConfig(const ThrFiringSchmittConfig& config) { this->cfg = config; }

/*! Reset the algorithm's time-varying state. The per-thruster ON/OFF history is cleared to OFF.
 @return void
 */
void ThrFiringSchmittAlgorithm::reInitialize() { this->prevThrustState.fill(ThrusterState::OFF); }

/*! This method maps the input thruster command forces into thruster on times using a Schmitt-trigger logic.
 @return ThrusterOnTimeCmd
 @param thrusterForceCmd The commanded thruster forces
 */
ThrusterOnTimeCmd ThrFiringSchmittAlgorithm::update(ThrusterForceCmd thrusterForceCmd) {
    ThrusterOnTimeCmd thrOnTimeOut{};

    const ThrFiringSchmittThrusterArray& thrusterArray = this->cfg.getThrusterArray();
    const ThrFiringSchmittControlParameters& params = this->cfg.getControlParameters();

    /*! - Loop through thrusters */
    for (uint32_t i = 0U; i < thrusterArray.numThrusters; ++i) {
        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
        if (params.pulsingRegime == ThrustPulsingRegime::OFF_PULSING) {
            thrusterForceCmd.thrForce.at(i) += thrusterArray.maxThrust.at(i);
        }

        /*! - Do not allow thrust requests less than zero */
        thrusterForceCmd.thrForce.at(i) = std::max(thrusterForceCmd.thrForce.at(i), 0.0F);
        /*! - Compute T_on from thrust request, max thrust, and control period */
        float onTime = thrusterForceCmd.thrForce.at(i) / thrusterArray.maxThrust.at(i) * params.controlPeriod;

        /*! - Apply Schmitt trigger logic */
        if (onTime < params.thrMinFireTime) {
            /*! - Request is less than minimum fire time */
            const float level = onTime / params.thrMinFireTime; /* [-] duty cycle fraction */
            if (level >= params.levelOn) {
                this->prevThrustState.at(i) = ThrusterState::ON;
                onTime = params.thrMinFireTime;
            } else if (level <= params.levelOff) {
                this->prevThrustState.at(i) = ThrusterState::OFF;
                onTime = 0.0F;
            } else if (this->prevThrustState.at(i) == ThrusterState::ON) {
                onTime = params.thrMinFireTime;
            } else {
                onTime = 0.0F;
            }
        } else if (onTime >= params.controlPeriod) {
            /*! - Request is greater than control period then oversaturate onTime */
            this->prevThrustState.at(i) = ThrusterState::ON;
            onTime = params.onTimeSaturationFactor * params.controlPeriod;
        } else {
            /*! - Request is greater than minimum fire time and less than control period */
            this->prevThrustState.at(i) = ThrusterState::ON;
        }

        /*! Set the output data */
        thrOnTimeOut.onTimeRequest.at(i) = onTime;
    }
    return thrOnTimeOut;
}
