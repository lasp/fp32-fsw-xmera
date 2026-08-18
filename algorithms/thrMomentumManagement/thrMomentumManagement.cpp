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
 are linked and caches the RW configuration.
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

    /*! - create the algorithm, whose constructor installs the configuration (throws on an invalid config) */
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

    const ThrMomentumManagementControlParameters controlParameters{.hsMin = this->hsMin, .K = this->K};

    return ThrMomentumManagementConfig::create(controlParameters, rwArrayConfig);
}

/*! Re-validate the current module properties and push them onto the live algorithm. Rebuilds the validated
 config from the public members and installs it via setConfig().
 @return void
 */
void ThrMomentumManagement::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrMomentumManagement reconfigure() before reset().");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! The RW momentum level is assessed on every update to determine the torque required to dump the momentum
 held above the threshold.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrMomentumManagement::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrMomentumManagement reset() has not been called.");
    }

    /*! - Read the input messages */
    const RWSpeedMsgF32Payload rwSpeedMsg = this->rwSpeedsInMsg(); /* Reaction wheel speed estimate message */

    const Eigen::Vector3f Lr_B = this->algorithm->update(cArrayToEigenVector(rwSpeedMsg.wheelSpeeds));

    /*! - write out the output message */
    CmdTorqueBodyMsgF32Payload controlOutMsg = {}; /* Control torque output message */
    eigenVectorToCArray(Lr_B, controlOutMsg.torqueRequestBody);

    this->cmdTorqueOutMsg.write(controlOutMsg, moduleID, callTime);
}
