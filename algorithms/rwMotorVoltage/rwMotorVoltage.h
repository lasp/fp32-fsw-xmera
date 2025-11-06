/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_RW_MOTOR_VOLTAGE_H
#define F32XIMERA_RW_MOTOR_VOLTAGE_H

#include <stdint.h>

#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
#include "msgPayloadDef/RwMotorVoltageMsgF32Payload.h"
#include "rwMotorVoltageAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>

#include <Eigen/Core>

class RwMotorVoltage : public SysModel {
   public:
    RwMotorVoltage(float minVoltageMagnitude, float maxVoltageMagnitude);
    ~RwMotorVoltage() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setVoltageRange(float minVoltageMagnitude, float maxVoltageMagnitude);
    Eigen::Vector2f getVoltageRange() const;
    void setGainK(float gain);
    float getGainK() const;

    /* declare module IO interfaces */
    Message<RwMotorVoltageMsgF32Payload> voltageOutMsg;    /*!< voltage output message*/
    ReadFunctor<RwMotorTorqueMsgF32Payload> torqueInMsg;   /*!< Input torque message*/
    ReadFunctor<RWArrayConfigMsgF32Payload> rwParamsInMsg; /*!< RW array input message*/
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedInMsg;     /*!< [] The name for the reaction wheel speeds message. Must be
                                                        provided to enable speed tracking loop */
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg; /*!< [-] The name of the RWs availability message*/

   private:
    RwMotorVoltageAlgorithm algorithm;
};

#endif
