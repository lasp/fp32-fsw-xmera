// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TORQUE_SCHEDULER_H
#define F32XMERA_TORQUE_SCHEDULER_H

#include "msgPayloadDef/ArrayEffectorLockMsgF32Payload.h"
#include "msgPayloadDef/ArrayMotorTorqueMsgF32Payload.h"
#include "torqueSchedulerAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <memory>

/*! @brief Adapter routing two motor-torque inputs into a paired output and emitting a
    time-scheduled effector-lock output. */
class TorqueScheduler final : public SysModel {
   public:
    TorqueScheduler() = default;
    ~TorqueScheduler() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    // Phase 1: public config properties -- set before reset().
    LockFlag lockFlag = LockFlag::BothFree;  //!< schedule selector
    float tSwitch = 0.0F;                    //!< [s] time span after reset at which the schedule transitions

    ReadFunctor<ArrayMotorTorqueMsgF32Payload> motorTorque1InMsg;  //!< first motor-torque input
    ReadFunctor<ArrayMotorTorqueMsgF32Payload> motorTorque2InMsg;  //!< second motor-torque input
    Message<ArrayMotorTorqueMsgF32Payload> motorTorqueOutMsg;      //!< paired motor-torque output
    Message<ArrayEffectorLockMsgF32Payload> effectorLockOutMsg;    //!< per-motor lock-flag output

   private:
    std::unique_ptr<TorqueSchedulerAlgorithm> algorithm = nullptr;
    uint64_t t0{};  //!< [ns] epoch captured at reset()
};

#endif
