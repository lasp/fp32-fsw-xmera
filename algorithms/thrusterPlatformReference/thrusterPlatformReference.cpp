#include "thrusterPlatformReference.h"

#include <stdexcept>

#include <architecture/utilities/eigenSupport.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
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

    this->algorithm.sigma_MB = this->sigma_MB;
    this->algorithm.r_BM_M = this->r_BM_M;
    this->algorithm.r_FM_F = this->r_FM_F;
    this->algorithm.K = this->K;
    this->algorithm.Ki = this->Ki;
    this->algorithm.theta1Max = this->theta1Max;
    this->algorithm.theta2Max = this->theta2Max;

    if (this->rwConfigDataInMsg.isLinked() && this->rwSpeedsInMsg.isLinked()) {
        this->algorithm.momentumDumping = true;
        const RWArrayConfigMsgF32Payload rwConfigParams = this->rwConfigDataInMsg();
        this->algorithm.rwConfig.numRW = static_cast<uint32_t>(rwConfigParams.numRW);
        this->algorithm.rwConfig.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(rwConfigParams.GsMatrix_B);
        this->algorithm.rwConfig.JsList = cArrayToEigenVector(rwConfigParams.JsList);
    } else {
        this->algorithm.momentumDumping = false;
    }

    this->algorithm.reset(callTime);
}

/*! This method computes the reference platform tip and tilt angles that align the thruster with the system center of
 mass (optionally offset to dump reaction wheel momentum) and writes the platform, body-heading, thruster-torque and
 thruster-configuration output messages.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ThrusterPlatformReference::updateState(const uint64_t callTime) {
    const VehicleConfigMsgF32Payload vehConfigMsgIn = this->vehConfigInMsg();
    const THRConfigMsgF32Payload thrusterConfigFIn = this->thrusterConfigFInMsg();

    ThrusterPlatformReferenceInputs inputs{};
    inputs.r_CB_B = cArrayToEigenVector3<float>(vehConfigMsgIn.CoM_B);
    inputs.rThrust_F = cArrayToEigenVector3<float>(thrusterConfigFIn.rThrust_B);
    inputs.tHatThrust_F = cArrayToEigenVector3<float>(thrusterConfigFIn.tHatThrust_B);
    inputs.maxThrust = thrusterConfigFIn.maxThrust;
    if (this->rwSpeedsInMsg.isLinked()) {
        const RWSpeedMsgF32Payload rwSpeedMsgIn = this->rwSpeedsInMsg();
        inputs.wheelSpeeds = cArrayToEigenVector(rwSpeedMsgIn.wheelSpeeds);
    }

    const ThrusterPlatformReferenceOutput out = this->algorithm.update(inputs, callTime);

    HingedRigidBodyMsgF32Payload hingedRigidBodyRef1Out{};
    hingedRigidBodyRef1Out.theta = out.theta1;
    hingedRigidBodyRef1Out.thetaDot = 0.0F;
    this->hingedRigidBodyRef1OutMsg.write(&hingedRigidBodyRef1Out, this->moduleID, callTime);

    HingedRigidBodyMsgF32Payload hingedRigidBodyRef2Out{};
    hingedRigidBodyRef2Out.theta = out.theta2;
    hingedRigidBodyRef2Out.thetaDot = 0.0F;
    this->hingedRigidBodyRef2OutMsg.write(&hingedRigidBodyRef2Out, this->moduleID, callTime);

    BodyHeadingMsgF32Payload bodyHeadingOut{};
    eigenVectorToCArray(out.rHat_XB_B, bodyHeadingOut.rHat_XB_B);
    this->bodyHeadingOutMsg.write(&bodyHeadingOut, this->moduleID, callTime);

    CmdTorqueBodyMsgF32Payload thrusterTorqueOut{};
    eigenVectorToCArray(out.torqueRequestBody, thrusterTorqueOut.torqueRequestBody);
    this->thrusterTorqueOutMsg.write(&thrusterTorqueOut, this->moduleID, callTime);

    THRConfigMsgF32Payload thrusterConfigOut{};
    eigenVectorToCArray(out.rThrust_B, thrusterConfigOut.rThrust_B);
    eigenVectorToCArray(out.tHatThrust_B, thrusterConfigOut.tHatThrust_B);
    thrusterConfigOut.maxThrust = out.maxThrust;
    this->thrusterConfigBOutMsg.write(&thrusterConfigOut, this->moduleID, callTime);
}
