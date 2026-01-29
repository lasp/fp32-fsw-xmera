#include "mrpFeedback.h"

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpFeedback::reset(const uint64_t callTime) {
    /* check that optional messages are correct connected */
    if (this->rwParamsInMsg.isLinked() && !this->rwSpeedsInMsg.isLinked()) {
        throw std::invalid_argument("MrpFeedback.rwSpeedsInMsg wasn't connected while rwParamsInMsg was connected.");
    }

    // check if the required message has not been connected
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("MrpFeedback.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("MrpFeedback.vehConfigInMsg wasn't connected.");
    }

    const VehicleConfigMsgF32Payload sc = this->vehConfigInMsg();
    RWArrayConfigMsgF32Payload rwConfigParams{};
    bool rwParamsIsLinked{};

    /*! - check if RW configuration message exists */
    if (this->rwParamsInMsg.isLinked()) {
        rwConfigParams = this->rwParamsInMsg();
        rwParamsIsLinked = true;
    }
    this->numRW = static_cast<uint32_t>(rwConfigParams.numRW);

    this->algorithm.reset(sc, rwConfigParams, rwParamsIsLinked);
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpFeedback::updateState(const uint64_t callTime) {
    AttGuidMsgF32Payload guidCmd = this->guidInMsg(); /* attitude tracking error message */
    RWSpeedMsgF32Payload wheelSpeeds{};               /* Reaction wheel speed message */
    RWAvailabilityMsgPayload wheelsAvailability{};    /* Reaction wheel availability message */

    /*! - read in optional RW speed and availability message */
    if (this->numRW > 0U) {
        wheelSpeeds = this->rwSpeedsInMsg();
        if (this->rwAvailInMsg.isLinked()) {
            wheelsAvailability = this->rwAvailInMsg();
        }
    }

    MrpFeedbackOutput mrpFeedbackOutput = algorithm.update(callTime, guidCmd, wheelSpeeds, wheelsAvailability);

    this->cmdTorqueOutMsg.write(&mrpFeedbackOutput.controlOut, moduleID, callTime);
    this->intFeedbackTorqueOutMsg.write(&mrpFeedbackOutput.intFeedbackOut, this->moduleID, callTime);
}

/*! Setter method for the gain K.
 @return void
 @param gain [N*m] Attitude error feedback gain
*/
void MrpFeedback::setK(const float gain) { this->algorithm.setK(gain); }

/*! Getter method for the gain K.
 @return const float
*/
float MrpFeedback::getK() const { return this->algorithm.getK(); }

/*! Setter method for the gain P.
 @return void
 @param gain [N*m*s] Rate error feedback gain
*/
void MrpFeedback::setP(const float gain) { this->algorithm.setP(gain); }

/*! Getter method for the gain P.
 @return const float
*/
float MrpFeedback::getP() const { return this->algorithm.getP(); }

/*! Setter method for the gain Ki.
 @return void
 @param gain [N*m] Integral feedback gain
*/
void MrpFeedback::setKi(const float gain) { this->algorithm.setKi(gain); }

/*! Getter method for the gain Ki.
 @return const float
*/
float MrpFeedback::getKi() const { return this->algorithm.getKi(); }

/*! Setter method for the integral limit.
 @return void
 @param limit [N*m*s] Integral limit
*/
void MrpFeedback::setIntegralLimit(const float limit) { this->algorithm.setIntegralLimit(limit); }

/*! Getter method for the integral limit.
 @return const float
*/
float MrpFeedback::getIntegralLimit() const { return this->algorithm.getIntegralLimit(); }

/*! Setter method for the control law type.
 @return void
 @param type control law type
*/
void MrpFeedback::setControlLawType(const int type) {
    this->algorithm.setControlLawType(static_cast<ControlLawType>(type));
}

/*! Getter method for the control law type.
 @return const int
*/
int MrpFeedback::getControlLawType() const { return static_cast<int>(this->algorithm.getControlLawType()); }

/*! Setter method for the known external torque about point B.
 @return void
 @param torque [N*m] Known external torque expressed in body frame components
*/
void MrpFeedback::setKnownTorquePntB_B(const Eigen::Vector3f& torque) { this->algorithm.setKnownTorquePntB_B(torque); }

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f
*/
Eigen::Vector3f MrpFeedback::getKnownTorquePntB_B() const { return this->algorithm.getKnownTorquePntB_B(); }
