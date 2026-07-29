#include "thrusterPlatformReference.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

// The algorithm's C-boundary RW count must match the system-wide RW_EFF_CNT, otherwise the payload
// GsMatrix_B / JsList / wheelSpeeds arrays would not map onto the algorithm's fixed-size types.
static_assert(kMaxNumRw == RW_EFF_CNT, "THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW must match RW_EFF_CNT");

/*! @brief Build the validated configuration from the public properties and the reaction-wheel input messages.
 Momentum dumping is enabled only when both the RW configuration and RW speed messages are linked.
 @return ThrusterPlatformReferenceConfig validated configuration
*/
ThrusterPlatformReferenceConfig ThrusterPlatformReference::toConfig() {
    const bool momentumDumping = this->rwConfigDataInMsg.isLinked() && this->rwSpeedsInMsg.isLinked();

    ThrusterPlatformReferenceRwArrayConfiguration rwConfig{};
    if (momentumDumping) {
        const RWArrayConfigMsgF32Payload rwConfigParams = this->rwConfigDataInMsg();
        rwConfig.numRW = static_cast<uint32_t>(rwConfigParams.numRW);
        rwConfig.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(rwConfigParams.GsMatrix_B);
        rwConfig.JsList = cArrayToEigenVector(rwConfigParams.JsList);
    }

    return ThrusterPlatformReferenceConfig::create(this->sigma_MB,
                                                   this->r_BM_M,
                                                   this->r_FM_F,
                                                   this->K,
                                                   this->Ki,
                                                   this->controlPeriod,
                                                   this->thetaMax,
                                                   momentumDumping,
                                                   rwConfig);
}

/*! This method performs a complete reset of the module: it validates the required input messages and (re)creates
 the algorithm from the current configuration.
 @return void
 @param callTime [ns] time the method is called
*/
void ThrusterPlatformReference::reset(const uint64_t callTime) {
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("thrusterPlatformReference.vehConfigInMsg wasn't connected.");
    }
    if (!this->thrusterConfigFInMsg.isLinked()) {
        throw std::invalid_argument("thrusterPlatformReference.thrusterConfigFInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<ThrusterPlatformReferenceAlgorithm>(this->toConfig());
}

/*! @brief Re-push the current configuration properties into the running algorithm, keeping its runtime state.
 @return void
*/
void ThrusterPlatformReference::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrusterPlatformReference reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! @brief Re-seed the running algorithm's runtime integrator state from its configured initial values.
 @return void
*/
void ThrusterPlatformReference::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrusterPlatformReference reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method computes the platform reference orientation that aligns the thruster line of action with the system
 center of mass (optionally offset to dump reaction wheel momentum) and writes the body-heading, thruster-torque and
 thruster-configuration output messages.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ThrusterPlatformReference::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrusterPlatformReference reset() has not been called.");
    }

    const VehicleConfigMsgF32Payload vehConfigMsgIn = this->vehConfigInMsg();
    const THRConfigMsgF32Payload thrusterConfigFIn = this->thrusterConfigFInMsg();

    ThrusterPlatformReferenceInputs inputs{};
    inputs.r_CB_B = cArrayToEigenVector3<float>(vehConfigMsgIn.CoM_B);
    inputs.r_TF_F = cArrayToEigenVector3<float>(thrusterConfigFIn.rThrust_B);
    inputs.tHat_F = cArrayToEigenVector3<float>(thrusterConfigFIn.tHatThrust_B);
    inputs.thrust = thrusterConfigFIn.maxThrust;
    if (this->rwSpeedsInMsg.isLinked()) {
        const RWSpeedMsgF32Payload rwSpeedMsgIn = this->rwSpeedsInMsg();
        inputs.wheelSpeeds = cArrayToEigenVector(rwSpeedMsgIn.wheelSpeeds);
    }

    const ThrusterPlatformReferenceOutput out = this->algorithm->update(inputs);

    // the body-frame thrust heading equals the body-frame thrust unit direction
    BodyHeadingMsgF32Payload bodyHeadingOut{};
    eigenVectorToCArray(out.tHat_B, bodyHeadingOut.rHat_XB_B);
    this->bodyHeadingOutMsg.write(&bodyHeadingOut, this->moduleID, callTime);

    CmdTorqueBodyMsgF32Payload thrusterTorqueOut{};
    eigenVectorToCArray(out.Lreq_B, thrusterTorqueOut.torqueRequestBody);
    this->thrusterTorqueOutMsg.write(&thrusterTorqueOut, this->moduleID, callTime);

    THRConfigMsgF32Payload thrusterConfigOut{};
    eigenVectorToCArray(out.r_TB_B, thrusterConfigOut.rThrust_B);
    eigenVectorToCArray(out.tHat_B, thrusterConfigOut.tHatThrust_B);
    thrusterConfigOut.maxThrust = out.thrust;
    this->thrusterConfigBOutMsg.write(&thrusterConfigOut, this->moduleID, callTime);
}
