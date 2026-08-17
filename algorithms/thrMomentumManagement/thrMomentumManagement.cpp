/*
    Thruster RW Momentum Management

 */

#include "thrMomentumManagement.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

// The algorithm's C-boundary RW count must match the system-wide RW_EFF_CNT, otherwise the
// payload GsMatrix_B / JsList / wheelSpeeds arrays would not map onto the algorithm's fixed-size types.
static_assert(kMaxNumRw == RW_EFF_CNT, "THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW must match RW_EFF_CNT");

/*! This method performs a complete reset of the module.  It validates that the required input messages
 are linked, caches the RW configuration, and re-arms the one-shot momentum dumping request.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrMomentumManagement::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->rwConfigDataInMsg.isLinked()) {
        throw std::invalid_argument("thrMomentumManagement.rwConfigDataInMsg wasn't connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()) {
        throw std::invalid_argument("thrMomentumManagement.rwSpeedsInMsg wasn't connected.");
    }

    /*! - create the algorithm, whose constructor installs the configuration and arms the momentum dumping
     request (throws on an invalid config) */
    this->algorithm = std::make_unique<ThrMomentumManagementAlgorithm>(this->toConfig());
}

/*! Build a validated algorithm configuration from the current module properties and the RW configuration
 message. Not const: it reads the RW configuration input message.
 @return ThrMomentumManagementConfig validated configuration
 */
ThrMomentumManagementConfig ThrMomentumManagement::toConfig() {
    /*! - read in the RW configuration message and convert it to the algorithm's own types */
    const RWArrayConfigMsgF32Payload rwConfigParams = this->rwConfigDataInMsg();
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;
    rwArrayConfig.numRW = static_cast<uint32_t>(rwConfigParams.numRW);
    rwArrayConfig.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(rwConfigParams.GsMatrix_B);
    rwArrayConfig.JsList = cArrayToEigenVector(rwConfigParams.JsList);

    return ThrMomentumManagementConfig::create(this->hsMin, rwArrayConfig);
}

/*! Re-validate the current module properties and push them onto the live algorithm without re-arming the
 momentum dumping request. Rebuilds the validated config from the public members and installs it via setConfig().
 @return void
 */
void ThrMomentumManagement::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrMomentumManagement reconfigure() before reset().");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Re-arm the one-shot momentum dumping request without rebuilding the config; a simple pass-through to the
 algorithm's reInitialize().
 @return void
 */
void ThrMomentumManagement::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrMomentumManagement reInitialize() before reset().");
    }
    this->algorithm->reInitialize();
}

/*! The RW momentum level is assessed to determine if a momentum dumping maneuver is required.
 This checking only happens once after the reset function is called.  To run this again afterwards,
 the reset function must be called again.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrMomentumManagement::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrMomentumManagement reset() has not been called.");
    }

    /*! - Read the input messages */
    const RWSpeedMsgF32Payload rwSpeedMsg = this->rwSpeedsInMsg(); /* Reaction wheel speed estimate message */

    const std::optional<Eigen::Vector3f> Delta_H_B =
        this->algorithm->update(cArrayToEigenVector(rwSpeedMsg.wheelSpeeds));

    /*! - write out the output message only while the one-shot dumping check is armed */
    if (Delta_H_B.has_value()) {
        CmdTorqueBodyMsgF32Payload controlOutMsg = {}; /* Control torque output message */
        eigenVectorToCArray(*Delta_H_B, controlOutMsg.torqueRequestBody);

        this->deltaHOutMsg.write(controlOutMsg, moduleID, callTime);
    }
}
