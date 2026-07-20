#include "thrFiringRemainderAlgorithm.h"

#include <algorithm>

/*! @brief Construct the algorithm with a validated configuration. The pulse remainder state starts cleared. */
ThrFiringRemainderAlgorithm::ThrFiringRemainderAlgorithm(const ThrFiringRemainderConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replace the stored configuration at runtime. The pulse remainder state is preserved. */
void ThrFiringRemainderAlgorithm::setConfig(const ThrFiringRemainderConfig& config) { this->cfg = config; }

/*! Reset the algorithm's time-varying state. The accumulated pulse remainder is cleared.
 @return void
 */
void ThrFiringRemainderAlgorithm::reInitialize() { this->pulseRemainder = {}; }

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return ThrusterOnTimeCmd
 @param thrusterForceCmd The commanded thruster forces
 */
ThrusterOnTimeCmd ThrFiringRemainderAlgorithm::update(ThrusterForceCmd thrusterForceCmd) {
    ThrusterOnTimeCmd thrOnTimeOut{};

    const ThrFiringRemainderThrusterArray& thrusterArray = this->cfg.getThrusterArray();
    const ThrFiringControlParameters& params = this->cfg.getControlParameters();

    /*! - Loop through thrusters */
    for (std::uint32_t i = 0U; i < thrusterArray.numThrusters; ++i) {
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
        /*! - Add in remainder from the last control step */
        onTime += this->pulseRemainder.at(i) * params.thrMinFireTime;
        /*! - Set pulse remainder to zero. Remainder now stored in onTime */
        this->pulseRemainder.at(i) = 0.0F;

        /* Pulse remainder logic */
        if (onTime < params.thrMinFireTime) {
            /*! - If request is less than minimum pulse time zero onTime and store remainder */
            this->pulseRemainder.at(i) = onTime / params.thrMinFireTime;
            onTime = 0.0F;
        } else if (onTime >= params.controlPeriod) {
            /*! - If request is greater than control period then oversaturate onTime */
            onTime = params.onTimeSaturationFactor * params.controlPeriod;
        } else {
            /* no action required. else clause included for MISRA */
        }

        /*! - Set the output data for each thruster */
        thrOnTimeOut.onTimeRequest.at(i) = onTime;
    }

    return thrOnTimeOut;
}
