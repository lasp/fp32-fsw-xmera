/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "mrpSteering.h"
#include "architecture/utilities/eigenSupport.h"
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpSteering::reset(const uint64_t callTime) {
    // make sure optional msg connections are correctly done */
    if (this->rwParamsInMsg.isLinked() && !this->rwSpeedsInMsg.isLinked()) {
        throw std::invalid_argument("mrpSteering.rwSpeedsInMsg wasn't connected while rwParamsInMsg was connected.");
    }
    // check for required input message
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("mrpSteering.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("mrpSteering.vehConfigInMsg wasn't connected.");
    }

    auto vehicleConfigInMsg = this->vehConfigInMsg();
    const Eigen::Matrix3f inertia = cArrayToEigenMatrix3(vehicleConfigInMsg.ISCPntB_B);
    this->algorithm.setSpacecraftInertia(inertia);

    RWArrayConfigMsgF32Payload rwConfigParams{};
    bool rwParamsIsLinked{};

    /*! - check if RW configuration message exists */
    if (this->rwParamsInMsg.isLinked()) {
        rwConfigParams = this->rwParamsInMsg();
        rwParamsIsLinked = true;
    }

    this->algorithm.reset(rwConfigParams, rwParamsIsLinked);
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpSteering::updateState(const uint64_t callTime) {
    AttGuidMsgF32Payload guidCmd = this->guidInMsg();
    RWSpeedMsgF32Payload wheelSpeeds{};            /*!< Reaction wheel speed estimates input message */
    RWAvailabilityMsgPayload wheelsAvailability{}; /*!< Reaction wheel availability input message */

    if (this->rwParamsInMsg.isLinked()) {
        wheelSpeeds = this->rwSpeedsInMsg();
        if (this->rwAvailInMsg.isLinked()) {
            wheelsAvailability = this->rwAvailInMsg();
        }
    }

    CmdTorqueBodyMsgF32Payload controlOut = algorithm.update(guidCmd, wheelSpeeds, wheelsAvailability);

    this->cmdTorqueOutMsg.write(&controlOut, moduleID, callTime);
}

/*! Set the linear feedback gain K1
 @return void
 @param gain [rad/s] linear feedback gain K1
*/
void MrpSteering::setK1(const float gain) { this->algorithm.setK1(gain); }

/*! Get the linear feedback gain K1
 @return float
*/
float MrpSteering::getK1() const { return this->algorithm.getK1(); }

/*! Set the cubic feedback gain K3
 @return void
 @param gain [rad/s] cubic feedback gain K3
*/
void MrpSteering::setK3(const float gain) { this->algorithm.setK3(gain); }

/*! Get the cubic feedback gain K3
 @return float
*/
float MrpSteering::getK3() const { return this->algorithm.getK3(); }

/*! Set the maximum rate command of steering control
 @return void
 @param omega [rad/s] maximum rate command of steering control
*/
void MrpSteering::setOmegaMax(const float omega) { this->algorithm.setOmegaMax(omega); }

/*! Get the maximum rate command of steering control
 @return float
*/
float MrpSteering::getOmegaMax() const { return this->algorithm.getOmegaMax(); }

/*! Set whether the outer loop feed-forward is ignored
 @return void
 @param ignore boolean whether the outer loop feed-forward should be ignored
*/
void MrpSteering::setIgnoreFeedforward(const bool ignore) { this->algorithm.setIgnoreFeedforward(ignore); }

/*! Get whether the outer loop feed-forward is ignored
 @return bool
*/
bool MrpSteering::getIgnoreFeedforward() const { return this->algorithm.getIgnoreFeedforward(); }

/*! Setter method for the gain P.
 @return void
 @param gain [N*m*s] Rate error feedback gain
*/
void MrpSteering::setP(const float gain) { this->algorithm.setP(gain); }

/*! Getter method for the gain P.
 @return const float
*/
float MrpSteering::getP() const { return this->algorithm.getP(); }

/*! Setter method for the gain Ki.
 @return void
 @param gain [N*m] Integral feedback gain
*/
void MrpSteering::setKi(const float gain) { this->algorithm.setKi(gain); }

/*! Getter method for the gain Ki.
 @return const float
*/
float MrpSteering::getKi() const { return this->algorithm.getKi(); }

/*! Setter method for the integral limit.
 @return void
 @param limit [rad] Integral limit
*/
void MrpSteering::setIntegralLimit(const float limit) { this->algorithm.setIntegralLimit(limit); }

/*! Getter method for the integral limit.
 @return const float
*/
float MrpSteering::getIntegralLimit() const { return this->algorithm.getIntegralLimit(); }

/*! Setter method for the known external torque about point B.
 @return void
 @param torque [N*m] Known external torque expressed in body frame components
*/
void MrpSteering::setKnownTorquePntB_B(const Eigen::Vector3f& torque) { this->algorithm.setKnownTorquePntB_B(torque); }

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f
*/
Eigen::Vector3f MrpSteering::getKnownTorquePntB_B() const { return this->algorithm.getKnownTorquePntB_B(); }

/*! Setter method for controlPeriod.
 @return void
 @param period [s] control period (time between two algorithm update calls)
 */
void MrpSteering::setControlPeriod(const float period) {
    this->algorithm.setControlPeriod(period);
}

/*! Getter method for controlPeriod.
 @return const float
*/
float MrpSteering::getControlPeriod() const { return this->algorithm.getControlPeriod(); }
