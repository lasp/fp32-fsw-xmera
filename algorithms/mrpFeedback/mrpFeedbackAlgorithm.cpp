#include "mrpFeedbackAlgorithm.h"
#include "architecture/utilities/eigenSupport.h"
#include <architecture/utilities/macroDefinitions.h>

#include <math.h>
#include "../freestandingInvalidArgument.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param vehConfigMsg vehicle config message
 @param rwConfigMsg reaction wheel config message
 @param rwIsLinked boolean indicating whether reaction wheel config message is linked
*/
void MrpFeedbackAlgorithm::reset(VehicleConfigMsgF32Payload vehConfigMsg,
                                 const RWArrayConfigMsgF32Payload rwConfigMsg,
                                 const bool rwIsLinked) {
    /*! - copy over spacecraft inertia tensor */
    this->ISCPntB_B = cArrayToEigenMatrix3(vehConfigMsg.ISCPntB_B);

    /*! - zero the number of RW by default */
    this->rwConfigParams.numRW = 0;

    /*! - check if RW configuration message exists */
    if (rwIsLinked) {
        /*! - Read static RW config data message and store it in module variables*/
        this->rwConfigParams = rwConfigMsg;
    }

    /*! - Reset the integral measure of the rate tracking error */
    this->int_sigma = Eigen::Vector3f::Zero();

    /*! - Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0;
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return MrpFeedbackOutput
 @param callTime The clock time at which the function was called (nanoseconds)
 @param guidCmd Attitude tracking error message
 @param wheelSpeeds Reaction wheel speed message
 @param wheelsAvailability Reaction wheel availability message
*/
MrpFeedbackOutput MrpFeedbackAlgorithm::update(uint64_t callTime,
                                               AttGuidMsgF32Payload guidCmd,
                                               RWSpeedMsgF32Payload wheelSpeeds,
                                               RWAvailabilityMsgPayload wheelsAvailability) {
    CmdTorqueBodyMsgF32Payload controlOut{};     /* output message */
    CmdTorqueBodyMsgF32Payload intFeedbackOut{}; /* output int feedback msg */

    /*! - compute control update time */
    float dt; /* [s] control update period */
    if (this->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = (callTime - this->priorTime) * NANO2SEC;
    }
    this->priorTime = callTime;

    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(guidCmd.sigma_BR);
    const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(guidCmd.omega_BR_B);
    const Eigen::Vector3f omega_RN_B = cArrayToEigenVector(guidCmd.omega_RN_B);
    const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(guidCmd.domega_RN_B);

    /*! - compute body rate */
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    /*! - evaluate integral term */
    Eigen::Vector3f z{Eigen::Vector3f::Zero()};
    if (this->Ki > 0) { /* check if integral feedback is turned on  */
        this->int_sigma += this->K * dt * sigma_BR;

        /* keep int_sigma less than integralLimit */
        for (uint32_t i = 0; i < 3; i++) {
            const float intCheck = fabs(this->int_sigma[i]);
            if (intCheck > this->integralLimit) {
                this->int_sigma[i] *= this->integralLimit / intCheck;
            }
        }
        z = this->int_sigma + this->ISCPntB_B * omega_BR_B;
    }

    /*! - evaluate required attitude control torque Lr */
    Eigen::Vector3f Lr = this->K * sigma_BR + this->P * omega_BR_B + this->P * this->Ki * z;

    const Eigen::Matrix<float, 3, RW_EFF_CNT> G_s_B =
        cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(this->rwConfigParams.GsMatrix_B);

    Eigen::Vector3f H_B = this->ISCPntB_B * omega_BN_B;
    for (uint32_t i = 0; i < this->rwConfigParams.numRW; i++) {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) { /* check if wheel is available */
            const Eigen::Vector3f G_s_B_i = G_s_B.col(i);
            const Eigen::Vector3f h_s_i =
                this->rwConfigParams.JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.wheelSpeeds[i]) * G_s_B_i;
            H_B += h_s_i;
        }
    }

    if (this->controlLawType == 0) {
        Lr -= (omega_RN_B + this->Ki * z).cross(H_B);
    } else {
        Lr -= omega_BN_B.cross(H_B);
    }

    Lr += this->ISCPntB_B * (omega_BN_B.cross(omega_RN_B) - domega_RN_B) + this->knownTorquePntB_B;

    const Eigen::Vector3f u_s = -Lr;
    const Eigen::Vector3f u_integral = -(this->P * this->Ki * z);

    eigenVectorToCArray(u_s, controlOut.torqueRequestBody);
    eigenVectorToCArray(u_integral, intFeedbackOut.torqueRequestBody);

    MrpFeedbackOutput mrpFeedbackOutput{};
    mrpFeedbackOutput.controlOut = controlOut;
    mrpFeedbackOutput.intFeedbackOut = intFeedbackOut;

    return mrpFeedbackOutput;
}

/*! Setter method for the gain K.
 @return void
 @param gain [N*m] Attitude error feedback gain
*/
void MrpFeedbackAlgorithm::setK(const float gain) {
    if (gain < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain K must not be negative");
    }
    this->K = gain;
}

/*! Getter method for the gain K.
 @return const float
*/
float MrpFeedbackAlgorithm::getK() const { return this->K; }

/*! Setter method for the gain P.
 @return void
 @param gain [N*m*s] Rate error feedback gain
*/
void MrpFeedbackAlgorithm::setP(const float gain) {
    if (gain < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain P must not be negative");
    }
    this->P = gain;
}

/*! Getter method for the gain P.
 @return const float
*/
float MrpFeedbackAlgorithm::getP() const { return this->P; }

/*! Setter method for the gain Ki.
 @return void
 @param gain [N*m] Integral feedback gain
*/
void MrpFeedbackAlgorithm::setKi(const float gain) { this->Ki = gain; }

/*! Getter method for the gain Ki.
 @return const float
*/
float MrpFeedbackAlgorithm::getKi() const { return this->Ki; }

/*! Setter method for the integral limit.
 @return void
 @param limit [N*m*s] Integral limit
*/
void MrpFeedbackAlgorithm::setIntegralLimit(const float limit) { this->integralLimit = limit; }

/*! Getter method for the integral limit.
 @return const float
*/
float MrpFeedbackAlgorithm::getIntegralLimit() const { return this->integralLimit; }

/*! Setter method for the control law type.
 @return void
 @param type control law type
*/
void MrpFeedbackAlgorithm::setControlLawType(const int type) { this->controlLawType = type; }

/*! Getter method for the control law type.
 @return const int
*/
int MrpFeedbackAlgorithm::getControlLawType() const { return this->controlLawType; }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void MrpFeedbackAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f
*/
Eigen::Vector3f MrpFeedbackAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
