// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "torqueScheduler.h"
#include <architecture/utilities/macroDefinitions.h>
#include <stdexcept>

void TorqueScheduler::reset(const uint64_t callTime) {
    if (!this->motorTorque1InMsg.isLinked()) {
        throw std::invalid_argument("torqueScheduler.motorTorque1InMsg wasn't connected.");
    }
    if (!this->motorTorque2InMsg.isLinked()) {
        throw std::invalid_argument("torqueScheduler.motorTorque2InMsg wasn't connected.");
    }

    this->t0 = callTime;
}

void TorqueScheduler::updateState(const uint64_t callTime) {
    const ArrayMotorTorqueMsgPayload motorTorque1In = this->motorTorque1InMsg();
    const ArrayMotorTorqueMsgPayload motorTorque2In = this->motorTorque2InMsg();

    // Seconds elapsed since the most recent reset().
    const double t = ((callTime - this->t0) * NANO2SEC);

    const TorqueSchedulerOutput out =
        this->algorithm.update(this->lockFlag, this->tSwitch, t, motorTorque1In, motorTorque2In);

    this->motorTorqueOutMsg.write(&out.motorTorqueOut, this->moduleID, callTime);
    this->effectorLockOutMsg.write(&out.effectorLockOut, this->moduleID, callTime);
}
