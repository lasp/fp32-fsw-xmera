/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XMERA_RATE_SERVO_FULL_NONLINEAR_ALGORITHM_H
#define F32XMERA_RATE_SERVO_FULL_NONLINEAR_ALGORITHM_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/RateCmdMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>

#include <stdint.h>
#include <Eigen/Core>

/*! @brief The configuration structure for the rateServoFullNonlinear module.  */
class RateServoFullNonlinearAlgorithm final {
   public:
    void reset(VehicleConfigMsgF32Payload vehConfigMsg,
               const RWArrayConfigMsgF32Payload& rwConfigMsg,
               bool rwIsConfigured);
    CmdTorqueBodyMsgF32Payload update(uint64_t callTime,
                                      AttGuidMsgF32Payload guidCmd,
                                      RateCmdMsgF32Payload rateCmd,
                                      const RWSpeedMsgF32Payload& wheelSpeeds,
                                      const RWAvailabilityMsgPayload& wheelsAvailability);

    void setP(float gain);
    float getP() const;
    void setKi(float gain);
    float getKi() const;
    void setIntegralLimit(float limit);
    float getIntegralLimit() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& torque);
    Eigen::Vector3f getKnownTorquePntB_B() const;

   private:
    float P{};              //!< [N*m*s]   Rate error feedback gain applied
    float Ki{};             //!< [N*m]     Integration feedback error on rate error
    float integralLimit{};  //!< [N*m]     Integration limit to avoid wind-up issue
    Eigen::Vector3f knownTorquePntB_B{
        Eigen::Vector3f::Zero()};  //!< [N*m]     known external torque in body frame vector components
    uint64_t priorTime{};          //!< [ns]      Last time the attitude control is called
    Eigen::Vector3f z{};           //!< [rad]     integral state of delta_omega
    Eigen::Matrix3f ISCPntB_B{};   //!< [kg m^2] Spacecraft Inertia
    RWArrayConfigMsgF32Payload
        rwConfigParams{};   //!< [-] struct to store message containing RW config parameters in body B frame
    bool rwIsConfigured{};  //!< [-] indicates whether reaction wheels are configured through the rwConfigMsg
};

#endif
