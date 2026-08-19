/*
    Thruster RW Momentum Management

 */

#include "momentumManagement.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

// The algorithm's C-boundary RW count must match the system-wide RW_EFF_CNT, otherwise the
// payload GsMatrix_B / JsList / wheelSpeeds arrays would not map onto the algorithm's fixed-size types.
static_assert(kMaxNumRw == RW_EFF_CNT, "MOMENTUM_MANAGEMENT_MAX_NUM_RW must match RW_EFF_CNT");

/*! This method performs a complete reset of the module.  It validates that the required input messages
 are linked, caches the RW configuration, and seeds the integrator state.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MomentumManagement::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->rwConfigDataInMsg.isLinked()) {
        throw std::invalid_argument("momentumManagement.rwConfigDataInMsg wasn't connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()) {
        throw std::invalid_argument("momentumManagement.rwSpeedsInMsg wasn't connected.");
    }

    /*! - create the algorithm, whose constructor installs the configuration and seeds the integrator state
     (throws on an invalid config) */
    this->algorithm = std::make_unique<MomentumManagementAlgorithm>(this->toConfig());
}

/*! Build a validated algorithm configuration from the current module properties and the RW configuration
 message. Not const: it reads the RW configuration input message.
 @return MomentumManagementConfig validated configuration
 */
MomentumManagementConfig MomentumManagement::toConfig() {
    /*! - read in the RW configuration message and convert it to the algorithm's own types */
    const RWArrayConfigMsgF32Payload rwConfigParams = this->rwConfigDataInMsg();
    MomentumManagementRwArrayConfiguration rwArrayConfig;
    rwArrayConfig.numRW = static_cast<uint32_t>(rwConfigParams.numRW);
    rwArrayConfig.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(rwConfigParams.GsMatrix_B);
    rwArrayConfig.JsList = cArrayToEigenVector(rwConfigParams.JsList);

    const MomentumManagementControlParameters controlParameters{.hsMin = this->hsMin,
                                                                .K = this->K,
                                                                .Ki = this->Ki,
                                                                .integralLimit = this->integralLimit,
                                                                .controlPeriod = this->controlPeriod};

    return MomentumManagementConfig::create(controlParameters, rwArrayConfig);
}

/*! Re-validate the current module properties and push them onto the live algorithm without disturbing its
 integrator state. Rebuilds the validated config from the public members and installs it via setConfig().
 @return void
 */
void MomentumManagement::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MomentumManagement reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Re-seed the algorithm's runtime integrator state without rebuilding the config; a simple pass-through to the
 algorithm's reInitialize().
 @return void
 */
void MomentumManagement::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MomentumManagement reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! The RW momentum level is assessed on every update to determine the torque required to dump the momentum
 held above the threshold.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MomentumManagement::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MomentumManagement reset() has not been called.");
    }

    /*! - Read the input messages */
    const RWSpeedMsgF32Payload rwSpeedMsg = this->rwSpeedsInMsg(); /* Reaction wheel speed estimate message */

    const Eigen::Vector3f Lr_B = this->algorithm->update(cArrayToEigenVector(rwSpeedMsg.wheelSpeeds));

    /*! - write out the output message */
    CmdTorqueBodyMsgF32Payload controlOutMsg = {}; /* Control torque output message */
    eigenVectorToCArray(Lr_B, controlOutMsg.torqueRequestBody);

    this->cmdTorqueOutMsg.write(controlOutMsg, moduleID, callTime);
}
