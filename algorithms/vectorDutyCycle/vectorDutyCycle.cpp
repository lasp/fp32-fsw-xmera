#include "vectorDutyCycle.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

/*! This method performs a complete reset of the module. It fixes the vectorType selection, validates that the
 selected input message is linked, and builds the algorithm, whose constructor installs the configuration and
 restarts the duty cycle.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void VectorDutyCycle::reset(const uint64_t callTime) {
    // check if the input message of the selected vector type is included
    switch (this->vectorType) {
        case VectorType::Force:
            if (!this->cmdForceInMsg.isLinked()) {
                throw std::invalid_argument("vectorDutyCycle.cmdForceInMsg wasn't connected.");
            }
            break;
        case VectorType::Torque:
            if (!this->cmdTorqueInMsg.isLinked()) {
                throw std::invalid_argument("vectorDutyCycle.cmdTorqueInMsg wasn't connected.");
            }
            break;
        default:
            throw std::invalid_argument("vectorDutyCycle.vectorType is not a valid VectorType.");
    }

    /*! - create the algorithm, whose constructor installs the configuration and restarts the duty cycle
     (throws on an invalid config) */
    this->algorithm = std::make_unique<VectorDutyCycleAlgorithm>(this->toConfig());
    this->activeVectorType = this->vectorType;
}

/*! Build a validated algorithm configuration from the current module properties. The whole configuration is
 held in module properties, so no input message is read here.
 @return VectorDutyCycleConfig validated configuration
 */
VectorDutyCycleConfig VectorDutyCycle::toConfig() const {
    return VectorDutyCycleConfig::create(this->onPeriods, this->offPeriods);
}

/*! Re-validate the current module properties and push them onto the live algorithm without restarting the
 cadence. Rebuilds the validated config from the public members and installs it via setConfig().
 @return void
 */
void VectorDutyCycle::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("VectorDutyCycle reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Restart the duty cycle at the beginning of its on window; a simple pass-through to the algorithm's
 reInitialize().
 @return void
 */
void VectorDutyCycle::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("VectorDutyCycle reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! The command of the vector type selected at reset() is gated on and off in a fixed duty cycle.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void VectorDutyCycle::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("VectorDutyCycle reset() has not been called.");
    }

    if (this->activeVectorType == VectorType::Force) {
        this->updateForce(callTime);
    } else {
        this->updateTorque(callTime);
    }
}

/*! Gates the commanded force and writes the force output message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void VectorDutyCycle::updateForce(const uint64_t callTime) {
    const CmdForceBodyMsgF32Payload cmdForceIn = this->cmdForceInMsg();
    const Eigen::Vector3f cmdForce_B = cArrayToEigenVector3<float>(cmdForceIn.forceRequestBody);

    const Eigen::Vector3f gatedForce_B = this->algorithm->update(cmdForce_B);

    CmdForceBodyMsgF32Payload cmdForceOut{};
    eigenVectorToCArray(gatedForce_B, cmdForceOut.forceRequestBody);

    this->cmdForceOutMsg.write(cmdForceOut, this->moduleID, callTime);
}

/*! Gates the commanded torque and writes the torque output message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void VectorDutyCycle::updateTorque(const uint64_t callTime) {
    const CmdTorqueBodyMsgF32Payload cmdTorqueIn = this->cmdTorqueInMsg();
    const Eigen::Vector3f cmdTorque_B = cArrayToEigenVector3<float>(cmdTorqueIn.torqueRequestBody);

    const Eigen::Vector3f gatedTorque_B = this->algorithm->update(cmdTorque_B);

    CmdTorqueBodyMsgF32Payload cmdTorqueOut{};
    eigenVectorToCArray(gatedTorque_B, cmdTorqueOut.torqueRequestBody);

    this->cmdTorqueOutMsg.write(cmdTorqueOut, this->moduleID, callTime);
}
