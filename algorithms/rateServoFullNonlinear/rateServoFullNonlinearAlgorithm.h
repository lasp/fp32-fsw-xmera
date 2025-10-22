/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef F32XIMERA_RATE_SERVO_FULL_NONLINEAR_ALGORITHM_H
#define F32XIMERA_RATE_SERVO_FULL_NONLINEAR_ALGORITHM_H

#include <stdint.h>

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/RateCmdMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"

#include <Eigen/Core>

/*! @brief The configuration structure for the rateServoFullNonlinear module.  */
class RateServoFullNonlinearAlgorithm {
   public:
    void reset(VehicleConfigMsgF32Payload vehConfigMsg,
               RWArrayConfigMsgF32Payload rwConfigMsg,
               bool rwIsLinked);
    CmdTorqueBodyMsgF32Payload update(uint64_t callTime,
                                      AttGuidMsgF32Payload guidCmd,
                                      RateCmdMsgF32Payload rateCmd,
                                      RWSpeedMsgF32Payload wheelSpeeds,
                                      RWAvailabilityMsgPayload wheelsAvailability);

    void setP(float gain);
    float getP() const;
    void setKi(float gain);
    float getKi() const;
    void setIntegralLimit(float limit);
    float getIntegralLimit() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f &knownTorquePntB_B);
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
        rwConfigParams{};  //!< [-] struct to store message containing RW config parameters in body B frame
};

#endif
