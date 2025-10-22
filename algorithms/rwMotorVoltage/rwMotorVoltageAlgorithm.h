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

#ifndef F32XIMERA_RW_MOTOR_VOLTAGE_ALGORITHM_H
#define F32XIMERA_RW_MOTOR_VOLTAGE_ALGORITHM_H

#include <stdint.h>

#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
#include "msgPayloadDef/RwMotorVoltageMsgF32Payload.h"

#include <Eigen/Core>

/*! @brief module configuration message */
class RwMotorVoltageAlgorithm {
   public:
    RwMotorVoltageAlgorithm(float minVoltageMagnitude, float maxVoltageMagnitude);
    ~RwMotorVoltageAlgorithm() = default;

    void reset(const RWArrayConfigMsgF32Payload& rwParamsInMsg);
    RwMotorVoltageMsgF32Payload update(uint64_t callTime,
                                       RwMotorTorqueMsgF32Payload& torqueCmd,
                                       const RWAvailabilityMsgPayload& rwAvailability,
                                       const RWSpeedMsgF32Payload& rwSpeed,
                                       bool rwSpeedMsgIsLinked);

    void setVoltageRange(float minVoltageMagnitude, float maxVoltageMagnitude);
    Eigen::Vector2f getVoltageRange() const;
    void setGainK(float gain);
    float getGainK() const;

   private:
    float voltageMin{};                            /*!< [V]    minimum voltage below which the torque is zero */
    float voltageMax{};                            /*!< [V]    maximum output voltage */
    float K{};                                     /*!< [V/Nm] torque tracking gain for closed loop control.*/
    Eigen::Vector<float, RW_EFF_CNT> rwSpeedOld{}; /*!< [r/s]  the RW spin rates from the prior control step */
    uint64_t priorTime{};                           /*!< [ns]   Last time the module control was called */
    bool resetFlag{};                               /*!< []     Flag indicating that a module reset occurred */
    RWArrayConfigMsgF32Payload
        rwConfigParams{}; /*!< [-] struct to store message containing RW config parameters in body B frame */
};

#endif
