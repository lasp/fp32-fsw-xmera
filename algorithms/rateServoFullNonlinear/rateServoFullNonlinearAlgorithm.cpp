/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "rateServoFullNonlinearAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>

#include "../freestandingInvalidArgument.h"
#include <math.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param vehConfigMsg vehicle config message
 @param rwConfigMsg reaction wheel config message
 @param rwIsLinked boolean indicating whether reaction wheel config message is linked
 */
void RateServoFullNonlinearAlgorithm::reset(VehicleConfigMsgF32Payload vehConfigMsg,
                                            const RWArrayConfigMsgF32Payload& rwConfigMsg,
                                            const bool rwIsLinked) {
    this->ISCPntB_B = cArrayToEigenMatrix3(vehConfigMsg.ISCPntB_B);

    this->rwConfigParams.numRW = 0;
    if (rwIsLinked) {
        this->rwConfigParams = rwConfigMsg;
    }

    /* Reset the integral measure of the rate tracking error */
    this->z = Eigen::Vector3f::Zero();

    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0U;
}

/*! This method takes and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param guidCmd Attitude tracking error message
 @param rateCmd Rate steering law message
 @param wheelSpeeds Reaction wheel speed message
 @param wheelsAvailability Reaction wheel availability message
 */
CmdTorqueBodyMsgF32Payload RateServoFullNonlinearAlgorithm::update(const uint64_t callTime,
                                                                   AttGuidMsgF32Payload guidCmd,
                                                                   RateCmdMsgF32Payload rateCmd,
                                                                   const RWSpeedMsgF32Payload& wheelSpeeds,
                                                                   const RWAvailabilityMsgPayload& wheelsAvailability) {
    CmdTorqueBodyMsgF32Payload controlOut{}; /*!< commanded torque output message */

    /*! - compute control update time */
    float dt{}; /* [s] control update period */
    if (this->priorTime == 0U) {
        dt = 0.0F;
    } else {
        dt = static_cast<float>(callTime - this->priorTime) * static_cast<float>(NANO2SEC);
    }
    this->priorTime = callTime;

    const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(guidCmd.omega_BR_B);
    const Eigen::Vector3f omega_RN_B = cArrayToEigenVector(guidCmd.omega_RN_B);
    const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(guidCmd.domega_RN_B);

    const Eigen::Vector3f omega_BastR_B = cArrayToEigenVector(rateCmd.omega_BastR_B);
    const Eigen::Vector3f omegap_BastR_B = cArrayToEigenVector(rateCmd.omegap_BastR_B);

    /*! - compute body rate */
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    /*! - compute the rate tracking error */
    const Eigen::Vector3f omega_BastN_B = omega_BastR_B + omega_RN_B;
    const Eigen::Vector3f omega_BBast_B = omega_BN_B - omega_BastN_B;

    /*! - integrate rate tracking error  */
    if (this->Ki > 0.0F) { /* check if integral feedback is turned on  */
        this->z += omega_BBast_B * dt;
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float intLimCheck = fabsf(this->z[i]);
            if (intLimCheck > this->integralLimit) {
                this->z[i] *= this->integralLimit / intLimCheck;
            }
        }
    } else {
        /* integral feedback is turned off through a negative gain setting */
        this->z = Eigen::Vector3f::Zero();
    }

    /*! - evaluate required attitude control torque Lr */
    Eigen::Vector3f Lr = this->P * omega_BBast_B + this->Ki * this->z;

    const Eigen::Matrix<float, 3, RW_EFF_CNT> G_s_B =
        cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(this->rwConfigParams.GsMatrix_B);

    Eigen::Vector3f H_B = this->ISCPntB_B * omega_BN_B;
    for (int i = 0; i < this->rwConfigParams.numRW; i++) {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) { /* check if wheel is available */
            const Eigen::Vector3f G_s_B_i = G_s_B.col(i);
            const Eigen::Vector3f h_s_i =
                this->rwConfigParams.JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.wheelSpeeds[i]) * G_s_B_i;
            H_B += h_s_i;
        }
    }
    Lr -= omega_BastN_B.cross(H_B);

    Lr += -this->ISCPntB_B * (omegap_BastR_B + domega_RN_B - omega_BN_B.cross(omega_RN_B)) + this->knownTorquePntB_B;

    /* Change sign to compute the net positive control torque onto the spacecraft */
    const Eigen::Vector3f u_s = -Lr;

    /*! - Set output message and pass it to the message bus */
    eigenVectorToCArray(u_s, controlOut.torqueRequestBody);

    return controlOut;
}

/*! Setter method for the gain P.
 @return void
 @param gain [N*m*s] Rate error feedback gain
*/
void RateServoFullNonlinearAlgorithm::setP(const float gain) {
    if (gain < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain P must not be negative");
    }
    this->P = gain;
}

/*! Getter method for the gain P.
 @return const float
*/
float RateServoFullNonlinearAlgorithm::getP() const { return this->P; }

/*! Setter method for the gain Ki.
 @return void
 @param gain [N*m] Integral feedback gain
*/
void RateServoFullNonlinearAlgorithm::setKi(const float gain) {
    if (gain < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Integral feedback gain Ki must not be negative");
    }
    this->Ki = gain;
}

/*! Getter method for the gain Ki.
 @return const float
*/
float RateServoFullNonlinearAlgorithm::getKi() const { return this->Ki; }

/*! Setter method for the integral limit.
 @return void
 @param limit [N*m*s] Integral limit
*/
void RateServoFullNonlinearAlgorithm::setIntegralLimit(const float limit) {
    if (limit < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Integral limit must not be negative");
    }
    this->integralLimit = limit;
}

/*! Getter method for the integral limit.
 @return const float
*/
float RateServoFullNonlinearAlgorithm::getIntegralLimit() const { return this->integralLimit; }

/*! Setter method for the known external torque about point B.
 @return void
 @param torque [N*m] Known external torque expressed in body frame components
*/
void RateServoFullNonlinearAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3f& torque) {
    this->knownTorquePntB_B = torque;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f
*/
Eigen::Vector3f RateServoFullNonlinearAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
