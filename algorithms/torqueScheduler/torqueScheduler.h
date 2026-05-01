// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TORQUE_SCHEDULER_H
#define F32XMERA_TORQUE_SCHEDULER_H

#include "torqueSchedulerAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/ArrayEffectorLockMsgPayload.h>
#include <architecture/msgPayloadDef/ArrayMotorTorqueMsgPayload.h>
#include <stdint.h>

/*! @brief Adapter routing two motor-torque inputs into a paired output and emitting a
    time-scheduled effector-lock output. */
class TorqueScheduler : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    int lockFlag{};    //!< schedule selector: 0 = both free, 1 = lock #2 then #1, 2 = lock #1 then #2, 3 = both locked
    double tSwitch{};  //!< [s] time span after reset at which the schedule transitions

    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorque1InMsg;  //!< first motor-torque input
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorque2InMsg;  //!< second motor-torque input
    Message<ArrayMotorTorqueMsgPayload> motorTorqueOutMsg;      //!< paired motor-torque output
    Message<ArrayEffectorLockMsgPayload> effectorLockOutMsg;    //!< per-motor lock-flag output

   private:
    TorqueSchedulerAlgorithm algorithm{};
    uint64_t t0{};  //!< [ns] epoch captured at reset()
};

#endif
