// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TORQUE_SCHEDULER_TYPES_H
#define F32XMERA_TORQUE_SCHEDULER_TYPES_H

#include <architecture/msgPayloadDef/ArrayEffectorLockMsgPayload.h>
#include <architecture/msgPayloadDef/ArrayMotorTorqueMsgPayload.h>

// Schedule selector for the two motors. Numeric values must stay in lockstep with the legacy
// integer encoding (0..3) used by callers and any C-shim mirror.
enum class LockFlag {
    BothFree = 0,             //!< Both motors free.
    LockSecondThenFirst = 1,  //!< Lock motor #2 until t > tSwitch, then lock motor #1 instead.
    LockFirstThenSecond = 2,  //!< Lock motor #1 until t > tSwitch, then lock motor #2 instead.
    BothLocked = 3            //!< Both motors locked.
};

struct TorqueSchedulerOutput {
    ArrayMotorTorqueMsgPayload motorTorqueOut{};    //!< paired motor-torque output
    ArrayEffectorLockMsgPayload effectorLockOut{};  //!< per-motor lock-flag output
};

#endif
