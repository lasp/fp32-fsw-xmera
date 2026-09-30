#include "thrDesatDutyCycle.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

/*! This method performs a complete reset of the module. It validates that the required input message is linked
 and builds the algorithm, whose constructor installs the configuration and restarts the duty cycle.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrDesatDutyCycle::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->cmdTorqueInMsg.isLinked()) {
        throw std::invalid_argument("thrDesatDutyCycle.cmdTorqueInMsg wasn't connected.");
    }

    /*! - create the algorithm, whose constructor installs the configuration and restarts the duty cycle
     (throws on an invalid config) */
    this->algorithm = std::make_unique<ThrDesatDutyCycleAlgorithm>(this->toConfig());
}

/*! Build a validated algorithm configuration from the current module properties. The whole configuration is
 held in module properties, so no input message is read here.
 @return ThrDesatDutyCycleConfig validated configuration
 */
ThrDesatDutyCycleConfig ThrDesatDutyCycle::toConfig() const {
    return ThrDesatDutyCycleConfig::create(this->firingPeriods, this->settlingPeriods);
}

/*! Re-validate the current module properties and push them onto the live algorithm without restarting the
 cadence. Rebuilds the validated config from the public members and installs it via setConfig().
 @return void
 */
void ThrDesatDutyCycle::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrDesatDutyCycle reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Restart the duty cycle at the beginning of its firing window; a simple pass-through to the algorithm's
 reInitialize().
 @return void
 */
void ThrDesatDutyCycle::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrDesatDutyCycle reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! The requested desaturation torque is gated on and off in a fixed duty cycle, so the reaction wheels get quiet
 windows in which to re-stabilize the attitude between desaturation pulses.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrDesatDutyCycle::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrDesatDutyCycle reset() has not been called.");
    }

    /*! - read in the torque command message and map to the freestanding type */
    const CmdTorqueBodyMsgF32Payload cmdTorqueIn = this->cmdTorqueInMsg();
    const Eigen::Vector3f cmdTorque_B = cArrayToEigenVector3<float>(cmdTorqueIn.torqueRequestBody);

    /*! - call algorithm update */
    const Eigen::Vector3f gatedTorque_B = this->algorithm->update(cmdTorque_B);

    /*! - map the freestanding type back to the message payload and write */
    CmdTorqueBodyMsgF32Payload cmdTorqueOut{};
    eigenVectorToCArray(gatedTorque_B, cmdTorqueOut.torqueRequestBody);

    this->cmdTorqueOutMsg.write(cmdTorqueOut, this->moduleID, callTime);
}
