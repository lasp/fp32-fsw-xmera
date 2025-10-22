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

#ifndef F32XIMERA_RW_MOTOR_VOLTAGE_H
#define F32XIMERA_RW_MOTOR_VOLTAGE_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
#include "msgPayloadDef/RwMotorVoltageMsgF32Payload.h"
#include "rwMotorVoltageAlgorithm.h"

#include <Eigen/Core>

class RwMotorVoltage : public SysModel {
   public:
    RwMotorVoltage(const float minVoltageMagnitude, const float maxVoltageMagnitude);
    ~RwMotorVoltage() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setVoltageRange(const float minVoltageMagnitude, const float maxVoltageMagnitude);
    Eigen::Vector2f getVoltageRange() const;
    void setGainK(const float gain);
    float getGainK() const;

    /* declare module IO interfaces */
    Message<RwMotorVoltageMsgF32Payload> voltageOutMsg;    /*!< voltage output message*/
    ReadFunctor<RwMotorTorqueMsgF32Payload> torqueInMsg;   /*!< Input torque message*/
    ReadFunctor<RWArrayConfigMsgF32Payload> rwParamsInMsg; /*!< RW array input message*/
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedInMsg;        /*!< [] The name for the reaction wheel speeds message. Must be
                                                           provided to enable speed tracking loop */
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg; /*!< [-] The name of the RWs availability message*/

   private:
    RwMotorVoltageAlgorithm algorithm;
};

#endif
