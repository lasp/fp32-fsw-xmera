#include "thrustVectoring.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

// The algorithm's C-boundary RW count must match the system-wide RW_EFF_CNT, otherwise the payload
// GsMatrix_B / JsList / wheelSpeeds arrays would not map onto the algorithm's fixed-size types.
static_assert(kMaxNumRw == RW_EFF_CNT, "THRUST_VECTORING_MAX_NUM_RW must match RW_EFF_CNT");

/*! @brief Build the validated configuration from the public properties and the reaction-wheel configuration message.
 @return ThrustVectoringConfig validated configuration
*/
ThrustVectoringConfig ThrustVectoring::toConfig() {
    const RWArrayConfigMsgF32Payload rwConfigParams = this->rwConfigDataInMsg();

    ThrustVectoringRwArrayConfiguration rwConfig{};
    rwConfig.numRW = static_cast<uint32_t>(rwConfigParams.numRW);
    rwConfig.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(rwConfigParams.GsMatrix_B);
    rwConfig.JsList = cArrayToEigenVector(rwConfigParams.JsList);

    return ThrustVectoringConfig::create(this->sigma_MB,
                                         this->r_MB_B,
                                         this->r_FM_F,
                                         this->K,
                                         this->Ki,
                                         this->integralLimit,
                                         this->controlPeriod,
                                         this->thetaMax,
                                         rwConfig);
}

/*! This method performs a complete reset of the module: it validates the required input messages and (re)creates
 the algorithm from the current configuration.
 @return void
 @param callTime [ns] time the method is called
*/
void ThrustVectoring::reset(const uint64_t callTime) {
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.vehConfigInMsg wasn't connected.");
    }
    if (!this->thrusterConfigFInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.thrusterConfigFInMsg wasn't connected.");
    }
    if (!this->rwConfigDataInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.rwConfigDataInMsg wasn't connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.rwSpeedsInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<ThrustVectoringAlgorithm>(this->toConfig());
}

/*! @brief Re-push the current configuration properties into the running algorithm, keeping its runtime state.
 @return void
*/
void ThrustVectoring::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! @brief Re-seed the running algorithm's runtime integrator state from its configured initial values.
 @return void
*/
void ThrustVectoring::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method computes the platform reference orientation that points the thruster line of action through the
 system center of mass (or produces a torque to dump reaction-wheel momentum) and writes the body-heading and
 thruster-configuration output messages.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ThrustVectoring::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }

    const VehicleConfigMsgF32Payload vehConfigMsgIn = this->vehConfigInMsg();
    const THRConfigMsgF32Payload thrusterConfigFIn = this->thrusterConfigFInMsg();

    ThrustVectoringInputs inputs{};
    inputs.r_CB_B = cArrayToEigenVector3<float>(vehConfigMsgIn.CoM_B);
    inputs.r_TF_F = cArrayToEigenVector3<float>(thrusterConfigFIn.rThrust_B);
    inputs.tHat_F = cArrayToEigenVector3<float>(thrusterConfigFIn.tHatThrust_B);
    inputs.thrust = thrusterConfigFIn.maxThrust;
    inputs.wheelSpeeds = cArrayToEigenVector(this->rwSpeedsInMsg().wheelSpeeds);

    const ThrustVectoringOutput out = this->algorithm->update(inputs);

    // the body-frame thrust heading equals the body-frame thrust unit direction
    BodyHeadingMsgF32Payload bodyHeadingOut{};
    eigenVectorToCArray(out.tHat_B, bodyHeadingOut.rHat_XB_B);
    this->bodyHeadingOutMsg.write(bodyHeadingOut, this->moduleID, callTime);

    THRConfigMsgF32Payload thrusterConfigOut{};
    eigenVectorToCArray(out.r_TB_B, thrusterConfigOut.rThrust_B);
    eigenVectorToCArray(out.tHat_B, thrusterConfigOut.tHatThrust_B);
    thrusterConfigOut.maxThrust = out.thrust;
    this->thrusterConfigBOutMsg.write(thrusterConfigOut, this->moduleID, callTime);
}
