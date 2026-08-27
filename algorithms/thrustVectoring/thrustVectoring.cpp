#include "thrustVectoring.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

namespace {
//! The platform frame is defined with its -z axis along the thrust, so this is what the input message must report.
constexpr float kThrusterMountingTolerance = 1e-3F;
}  // namespace

/*! @brief Build the validated configuration from the public properties and the fixed input messages.
 The vehicle and thruster configurations do not change while the module runs, so they are read once here rather
 than on every update; call reconfigure() to pick up a new value.
 @return ThrustVectoringConfig validated configuration
*/
ThrustVectoringConfig ThrustVectoring::toConfig() {
    const VehicleConfigMsgF32Payload vehConfigIn = this->vehConfigInMsg();
    const THRConfigMsgF32Payload thrusterConfigFIn = this->thrusterConfigFInMsg();

    // This module models a thruster whose line of action runs through the joint M: it fires along the platform
    // frame's -z axis from a point on that axis. Check the incoming thruster description really says so, rather
    // than silently pointing a thruster the spacecraft does not have.
    const Eigen::Vector3f r_TF_F = cArrayToEigenVector3<float>(thrusterConfigFIn.rThrust_B);
    const Eigen::Vector3f tHat_F = cArrayToEigenVector3<float>(thrusterConfigFIn.tHatThrust_B);
    if (!r_TF_F.allFinite() || r_TF_F.norm() > kThrusterMountingTolerance) {
        throw std::invalid_argument(
            "thrustVectoring.thrusterConfigFInMsg reports a thrust application point away from the platform "
            "frame origin; this module requires rThrust_B == 0.");
    }
    if (!tHat_F.allFinite() || (tHat_F + Eigen::Vector3f::UnitZ()).norm() > kThrusterMountingTolerance) {
        throw std::invalid_argument(
            "thrustVectoring.thrusterConfigFInMsg reports a thrust direction off the platform -z axis; this "
            "module requires tHatThrust_B == [0, 0, -1] and carries the mounting orientation in sigma_MB.");
    }

    const ThrustVectoringPlatformConfiguration platformConfig{
        .sigma_MB = this->sigma_MB, .r_MB_B = this->r_MB_B, .thetaMax = this->thetaMax};
    const ThrustVectoringThrusterConfiguration thrusterConfig{.armLength = this->armLength,
                                                              .thrust = thrusterConfigFIn.maxThrust};

    return ThrustVectoringConfig::create(
        platformConfig, thrusterConfig, cArrayToEigenVector3<float>(vehConfigIn.CoM_B));
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
    if (!this->cmdTorqueInMsg.isLinked()) {
        throw std::invalid_argument("thrustVectoring.cmdTorqueInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<ThrustVectoringAlgorithm>(this->toConfig());
}

/*! @brief Re-read the configuration input messages and re-push the current properties into the algorithm.
 @return void
*/
void ThrustVectoring::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! This method computes the platform reference orientation that points the thruster so it produces the requested
 torque about the system center of mass (a zero request aligns the thruster line of action with the center of mass)
 and writes the body-heading and thruster-configuration output messages.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ThrustVectoring::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustVectoring reset() has not been called.");
    }

    const Eigen::Vector3f Lreq_B = cArrayToEigenVector3<float>(this->cmdTorqueInMsg().torqueRequestBody);

    const ThrustVectoringOutput out = this->algorithm->update(Lreq_B);

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
