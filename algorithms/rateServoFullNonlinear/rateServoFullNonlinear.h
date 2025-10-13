/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_RATE_SERVO_FULL_NONLINEAR_H
#define F32XIMERA_RATE_SERVO_FULL_NONLINEAR_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/RateCmdMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "rateServoFullNonlinearAlgorithm.h"

#include <Eigen/Core>

/*! @brief The configuration structure for the rateServoFullNonlinear module.  */
class RateServoFullNonlinear : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setP(float gain);
    float getP() const;
    void setKi(float gain);
    float getKi() const;
    void setIntegralLimit(float limit);
    float getIntegralLimit() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f &knownTorquePntB_B);
    Eigen::Vector3f getKnownTorquePntB_B() const;

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< commanded torque output message
    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;             //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< vehicle configuration input message
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;         //!< (optional) RW speed input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;   //!< (optional) RW availability input message
    ReadFunctor<RWArrayConfigMsgF32Payload> rwParamsInMsg;   //!< (optional) RW configuration parameter input message
    ReadFunctor<RateCmdMsgF32Payload> rateSteeringInMsg;     //!< commanded rate input message

   private:
    RateServoFullNonlinearAlgorithm algorithm{};
    uint32_t numRW{};  //!< number of reaction wheels
};

#endif
