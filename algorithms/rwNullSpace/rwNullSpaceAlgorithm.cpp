#include "rwNullSpaceAlgorithm.h"
#include "architecture/utilities/eigenSupport.h"

#include "utilities/freestandingInvalidArgument.h"

/*! @brief This resets the module to original states by reading in the RW configuration messages and recreating any
   module specific variables.  The output message is reset to zero.
    @return void
    @param rwConfigInMsg Reaction Wheel constellation input message
 */
void RwNullSpaceAlgorithm::reset(RWConstellationMsgF32Payload& rwConfigInMsg) {
    this->numWheels = static_cast<uint32_t>(rwConfigInMsg.numRW);

    Eigen::Matrix<float, 3, RW_EFF_CNT> G_s_B{};
    G_s_B.setZero();
    for (uint32_t i = 0; i < this->numWheels; i = i + 1) {
        G_s_B.col(i) = cArrayToEigenVector(rwConfigInMsg.reactionWheels[i].gsHat_B);
    }

    /* find the [tau] null space projection matrix [tau] = ([I] - [Gs]^T.([Gs].[Gs]^T)^-1.[Gs]) */
    this->tau = Eigen::Matrix<float, RW_EFF_CNT, RW_EFF_CNT>::Identity() -
                G_s_B.transpose() * (G_s_B * G_s_B.transpose()).inverse() * G_s_B;
}

/*! This method takes the input reaction wheel commands as well as the observed
    reaction wheel speeds and balances the commands so that the overall vehicle
    momentum is minimized.
 @return RwMotorTorqueMsgF32Payload
 @param controlRequest array of RW torques requested by control law
 @param rwSpeeds array of wheel speeds
 @param rwDesiredSpeeds array of desired wheel speeds
 */
RwMotorTorqueMsgF32Payload RwNullSpaceAlgorithm::update(RwMotorTorqueMsgF32Payload& controlRequest,
                                                        RWSpeedMsgF32Payload& rwSpeeds,
                                                        RWSpeedMsgF32Payload& rwDesiredSpeeds) const {
    RwMotorTorqueMsgF32Payload finalControl{}; /* [Nm]  array of final RW motor torques containing both
                                            the control and null motion torques */

    /* compute the wheel speed control vector d = -K.DeltaOmega */
    const Eigen::Vector<float, MAX_EFF_CNT> d = -this->omegaGain * (cArrayToEigenVector(rwSpeeds.wheelSpeeds) -
                                                                    cArrayToEigenVector(rwDesiredSpeeds.wheelSpeeds));

    /* compute the RW null space motor torque solution to reduce the wheel speeds */
    const Eigen::Vector<float, MAX_EFF_CNT> motorTorqueNullSpace = this->tau * d;

    /* add the null motion RW torque solution to the RW feedback control torque solution */
    const Eigen::Vector<float, MAX_EFF_CNT> motorTorque =
        motorTorqueNullSpace + cArrayToEigenVector(controlRequest.motorTorque);

    eigenVectorToCArray(motorTorque, finalControl.motorTorque);

    return finalControl;
}

/**
 * @brief Set the gain used for the wheel speed difference.
 * @param gain The gain used for the wheel speed difference.
 */
void RwNullSpaceAlgorithm::setOmegaGain(const float gain) {
    if (gain < 0.0) {
        FSW_THROW_INVALID_ARGUMENT("Feedback gain must not be negative");
    }
    this->omegaGain = gain;
}

/**
 * @brief Get the gain used for the wheel speed difference.
 * @return float The gain used for the wheel speed difference.
 */
float RwNullSpaceAlgorithm::getOmegaGain() const { return this->omegaGain; }
