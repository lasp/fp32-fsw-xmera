#include "thrDesatDutyCycleAlgorithm.h"

/*! @brief Construct the gate with a validated configuration. The cycle starts at its firing window. */
ThrDesatDutyCycleAlgorithm::ThrDesatDutyCycleAlgorithm(const ThrDesatDutyCycleConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replace the stored configuration at runtime. The cadence counter is preserved.
 @param config The validated configuration to install
 */
void ThrDesatDutyCycleAlgorithm::setConfig(const ThrDesatDutyCycleConfig& config) {
    this->cfg = config;
    /*! - the sum is at least one and cannot wrap, since the configuration requires at least one firing period
     and a full cycle length that fits in a uint32_t */
    this->cycleLength = config.getFiringPeriods() + config.getSettlingPeriods();
}

/*! Restart the duty cycle, so the next update falls at the start of a firing window.
 @return void
 */
void ThrDesatDutyCycleAlgorithm::reInitialize() {
    /*! - sit on the final position of the cycle, so the next update advances onto position zero, where the
     firing window starts */
    this->previousPositionInCycle = this->cycleLength - 1U;
}

/*! This method gates the commanded thruster force on and off in a fixed duty cycle. The force is passed through
 unchanged during the firing window and replaced by zero during the settling window, which leaves the reaction
 wheels quiet periods in which to re-stabilize the attitude. The cadence is free-running, so the counter advances
 whether or not any force is commanded.
 @return [N] the commanded per-thruster force while firing, zero while settling
 @param thrusterForceCmd [N] The commanded thruster forces
 */
std::array<float, kMaxThrusterCount> ThrDesatDutyCycleAlgorithm::update(
    const std::array<float, kMaxThrusterCount>& thrusterForceCmd) {
    /*! - advance one position, wrapping at the end of the cycle; the wrap also puts the position back in range
     when setConfig() has shortened the cycle below the position already reached */
    const uint32_t positionInCycle = (this->previousPositionInCycle + 1U) % this->cycleLength;

    /*! - the firing window occupies the leading positions of the cycle; the rest commands zero force */
    std::array<float, kMaxThrusterCount> thrForceOut{};
    if (positionInCycle < this->cfg.getFiringPeriods()) {
        thrForceOut = thrusterForceCmd;
    }

    this->previousPositionInCycle = positionInCycle;

    return thrForceOut;
}
