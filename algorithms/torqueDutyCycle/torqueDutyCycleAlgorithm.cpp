#include "torqueDutyCycleAlgorithm.h"

/*! @brief Makes the gate from a validated configuration. The cycle starts at its on window. */
TorqueDutyCycleAlgorithm::TorqueDutyCycleAlgorithm(const TorqueDutyCycleConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replaces the stored configuration at runtime. The position in the cycle does not change.
 @param config The validated configuration to store
 */
void TorqueDutyCycleAlgorithm::setConfig(const TorqueDutyCycleConfig& config) {
    this->cfg = config;
    /*! - the configuration has at least one on period, and its full cycle length is not more than UINT32_MAX.
     Thus, the sum is at least one and cannot overflow. */
    this->cycleLength = config.getOnPeriods() + config.getOffPeriods();
}

/*! Starts the duty cycle again, so that the next update is at the start of an on window.
 @return void
 */
void TorqueDutyCycleAlgorithm::reInitialize() {
    /*! - set the previous position to the last position of the cycle. The next update then moves forward to
     position zero, which is the start of the on window. */
    this->previousPositionInCycle = this->cycleLength - 1U;
}

/*! This method applies a fixed duty cycle to the commanded torque. In the on window, the output torque is equal to
 the commanded torque. In the off window, the output torque is zero. The cadence is continuous. Thus, the position
 in the cycle moves forward at each update, also when the torque command is zero.
 @return [Nm] the commanded body torque in an on period, zero in an off period
 @param cmdTorque_B [Nm] The commanded body torque
 */
Eigen::Vector3f TorqueDutyCycleAlgorithm::update(const Eigen::Vector3f& cmdTorque_B) {
    /*! - move forward by one position. At the end of the cycle, the position goes back to zero. If setConfig() made
     the cycle shorter than the previous position, the modulo also puts the position back in the cycle. */
    const uint32_t positionInCycle = (this->previousPositionInCycle + 1U) % this->cycleLength;

    /*! - the on window is the first positions of the cycle. In the remaining positions, the output torque is zero. */
    Eigen::Vector3f torqueOut_B = Eigen::Vector3f::Zero();
    if (positionInCycle < this->cfg.getOnPeriods()) {
        torqueOut_B = cmdTorque_B;
    }

    this->previousPositionInCycle = positionInCycle;

    return torqueOut_B;
}
