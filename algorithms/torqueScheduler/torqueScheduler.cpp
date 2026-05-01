// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "torqueScheduler.h"
#include "utilities/timeConstants.h"
#include "utilities/xmeraLifecycleException.h"
#include <stdexcept>

void TorqueScheduler::reset(const uint64_t callTime) {
    if (!this->motorTorque1InMsg.isLinked()) {
        throw std::invalid_argument("torqueScheduler.motorTorque1InMsg wasn't connected.");
    }
    if (!this->motorTorque2InMsg.isLinked()) {
        throw std::invalid_argument("torqueScheduler.motorTorque2InMsg wasn't connected.");
    }

    auto config = TorqueSchedulerConfig::create(this->lockFlag, this->tSwitch);
    this->algorithm = std::make_unique<TorqueSchedulerAlgorithm>(config);
    this->t0 = callTime;
}

void TorqueScheduler::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TorqueScheduler reset() has not been called.");
    }

    const ArrayMotorTorqueMsgF32Payload motorTorque1In = this->motorTorque1InMsg();
    const ArrayMotorTorqueMsgF32Payload motorTorque2In = this->motorTorque2InMsg();

    // Seconds elapsed since the most recent reset(). The subtraction is done in uint64 ns and the
    // multiplication in double to avoid precision loss for long sim runs; the result is cast down
    // to float for the algorithm input.
    const float t = static_cast<float>(static_cast<double>(callTime - this->t0) * kNano2Sec);

    const TorqueSchedulerOutput out = this->algorithm->update(t, motorTorque1In, motorTorque2In);

    this->motorTorqueOutMsg.write(&out.motorTorqueOut, this->moduleID, callTime);
    this->effectorLockOutMsg.write(&out.effectorLockOut, this->moduleID, callTime);
}
