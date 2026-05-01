// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "torqueScheduler.h"
#include <architecture/utilities/macroDefinitions.h>
#include <stdexcept>

void TorqueScheduler::reset(uint64_t callTime) {
    if (!this->motorTorque1InMsg.isLinked()) {
        throw std::invalid_argument("torqueScheduler.motorTorque1InMsg wasn't connected.");
    }
    if (!this->motorTorque2InMsg.isLinked()) {
        throw std::invalid_argument("torqueScheduler.motorTorque2InMsg wasn't connected.");
    }

    this->t0 = callTime;
}

void TorqueScheduler::updateState(uint64_t callTime) {
    ArrayMotorTorqueMsgPayload motorTorque1In = this->motorTorque1InMsg();
    ArrayMotorTorqueMsgPayload motorTorque2In = this->motorTorque2InMsg();
    ArrayMotorTorqueMsgPayload motorTorqueOut = {};
    ArrayEffectorLockMsgPayload effectorLockOut = {};

    // Seconds elapsed since the most recent reset().
    double t = ((callTime - this->t0) * NANO2SEC);

    motorTorqueOut.motorTorque[0] = motorTorque1In.motorTorque[0];
    motorTorqueOut.motorTorque[1] = motorTorque2In.motorTorque[0];

    switch (this->lockFlag) {
        case 0:
            effectorLockOut.effectorLockFlag[0] = 0;
            effectorLockOut.effectorLockFlag[1] = 0;
            break;

        case 1:
            if (t > this->tSwitch) {
                effectorLockOut.effectorLockFlag[0] = 1;
                effectorLockOut.effectorLockFlag[1] = 0;
            } else {
                effectorLockOut.effectorLockFlag[0] = 0;
                effectorLockOut.effectorLockFlag[1] = 1;
            }
            break;

        case 2:
            if (t > this->tSwitch) {
                effectorLockOut.effectorLockFlag[0] = 0;
                effectorLockOut.effectorLockFlag[1] = 1;
            } else {
                effectorLockOut.effectorLockFlag[0] = 1;
                effectorLockOut.effectorLockFlag[1] = 0;
            }
            break;

        case 3:
            effectorLockOut.effectorLockFlag[0] = 1;
            effectorLockOut.effectorLockFlag[1] = 1;
            break;

        default:
            // lockFlag outside [0, 3] leaves effectorLockOut at the zero-init default (both
            // motors unlocked). Step 14 Config validation will make this case unreachable.
            break;
    }

    this->motorTorqueOutMsg.write(&motorTorqueOut, this->moduleID, callTime);
    this->effectorLockOutMsg.write(&effectorLockOut, this->moduleID, callTime);
}
