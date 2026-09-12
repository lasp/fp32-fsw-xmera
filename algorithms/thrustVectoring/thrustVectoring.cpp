#include "thrustVectoring.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <cmath>
#include <stdexcept>

namespace {
//! How far the incoming thruster description may stray from the one mounting this module can represent.
constexpr float kThrusterMountingTolerance = 1e-3F;
}  // namespace

/*! @brief Build the validated configuration from the public properties and the fixed input messages.
 The vehicle and thruster configurations do not change while the module runs, so they are read once here rather
 than on every update; call reconfigure() to pick up a new value. The module works entirely in the body frame:
 the platform frame appears only in the contract the thruster description must satisfy.
 @return ThrustVectoringConfig validated configuration
*/
ThrustVectoringConfig ThrustVectoring::toConfig() {
    const VehicleConfigMsgF32Payload vehConfigIn = this->vehConfigInMsg();
    const THRConfigMsgF32Payload thrusterConfigFIn = this->thrusterConfigFInMsg();

    // Only maxThrust is taken from this message. The other two fields are a contract: the whole solve assumes
    // the line of action runs through the joint M, which holds only for a thruster sitting at the platform frame
    // origin and firing along that frame's +z axis. Reject any other description rather than reporting a
    // confidently wrong thruster for one the spacecraft does not have.
    const Eigen::Vector3f r_TF_F = cArrayToEigenVector3<float>(thrusterConfigFIn.rThrust_B);
    const Eigen::Vector3f tHat_F = cArrayToEigenVector3<float>(thrusterConfigFIn.tHatThrust_B);
    if (!r_TF_F.allFinite() || r_TF_F.stableNorm() > kThrusterMountingTolerance) {
        throw std::invalid_argument(
            "thrustVectoring.thrusterConfigFInMsg reports a thrust application point away from the platform "
            "frame origin; this module represents only a thruster with rThrust_B == 0.");
    }
    if (!tHat_F.allFinite() || (tHat_F - Eigen::Vector3f::UnitZ()).stableNorm() > kThrusterMountingTolerance) {
        throw std::invalid_argument(
            "thrustVectoring.thrusterConfigFInMsg reports a thrust direction off the platform +z axis; this "
            "module represents only a thruster with tHatThrust_B == [0, 0, 1].");
    }

    if (!std::isfinite(this->armLength) || this->armLength < 0.0F) {
        throw std::invalid_argument("thrustVectoring.armLength must be finite and non-negative.");
    }

    return ThrustVectoringConfig::create(
        this->r_MB_B, thrusterConfigFIn.maxThrust, cArrayToEigenVector3<float>(vehConfigIn.CoM_B));
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

    const Eigen::Vector3f tHat_B = this->algorithm->update(Lreq_B);

    // the body-frame thrust heading equals the body-frame thrust unit direction
    BodyHeadingMsgF32Payload bodyHeadingOut{};
    eigenVectorToCArray(tHat_B, bodyHeadingOut.rHat_XB_B);
    this->bodyHeadingOutMsg.write(bodyHeadingOut, this->moduleID, callTime);

    // The thruster fires along the thrust from a point armLength behind the joint, so its line of action runs
    // through the joint whatever the platform orientation.
    const Eigen::Vector3f r_TB_B = this->r_MB_B - (this->armLength * tHat_B);

    THRConfigMsgF32Payload thrusterConfigOut{};
    eigenVectorToCArray(r_TB_B, thrusterConfigOut.rThrust_B);
    eigenVectorToCArray(tHat_B, thrusterConfigOut.tHatThrust_B);
    thrusterConfigOut.maxThrust = this->algorithm->getConfig().getThrust();
    this->thrusterConfigBOutMsg.write(thrusterConfigOut, this->moduleID, callTime);
}
