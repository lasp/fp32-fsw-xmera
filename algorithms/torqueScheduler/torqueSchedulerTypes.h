// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TORQUE_SCHEDULER_TYPES_H
#define F32XMERA_TORQUE_SCHEDULER_TYPES_H

#include "msgPayloadDef/ArrayEffectorLockMsgF32Payload.h"
#include "msgPayloadDef/ArrayMotorTorqueMsgF32Payload.h"
#include "utilities/freestandingInvalidArgument.h"

// Schedule selector for the two motors. Numeric values must stay in lockstep with the legacy
// integer encoding (0..3) used by callers and any C-shim mirror.
enum class LockFlag {
    BothFree = 0,             //!< Both motors free.
    LockSecondThenFirst = 1,  //!< Lock motor #2 until t > tSwitch, then lock motor #1 instead.
    LockFirstThenSecond = 2,  //!< Lock motor #1 until t > tSwitch, then lock motor #2 instead.
    BothLocked = 3            //!< Both motors locked.
};

struct TorqueSchedulerOutput {
    ArrayMotorTorqueMsgF32Payload motorTorqueOut{};    //!< paired motor-torque output
    ArrayEffectorLockMsgF32Payload effectorLockOut{};  //!< per-motor lock-flag output
};

class TorqueSchedulerConfig final {
   public:
    static TorqueSchedulerConfig create(LockFlag lockFlag, float tSwitch) {
        if (!isValidLockFlag(lockFlag)) {
            FSW_THROW_INVALID_ARGUMENT("torqueScheduler: lockFlag is not a defined LockFlag value");
        }
        if (!isValidTSwitch(tSwitch)) {
            FSW_THROW_INVALID_ARGUMENT("torqueScheduler: tSwitch must be >= 0");
        }
        return {lockFlag, tSwitch};
    }

    static bool isValidLockFlag(LockFlag lockFlag) {
        return lockFlag == LockFlag::BothFree || lockFlag == LockFlag::LockSecondThenFirst ||
               lockFlag == LockFlag::LockFirstThenSecond || lockFlag == LockFlag::BothLocked;
    }
    static bool isValidTSwitch(float tSwitch) { return tSwitch >= 0.0F; }

    LockFlag getLockFlag() const { return lockFlag; }
    float getTSwitch() const { return tSwitch; }

   private:
    TorqueSchedulerConfig(LockFlag lockFlag, float tSwitch) : lockFlag(lockFlag), tSwitch(tSwitch) {}

    LockFlag lockFlag;
    float tSwitch;
};

#endif
