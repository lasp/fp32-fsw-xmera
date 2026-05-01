// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "torqueSchedulerAlgorithm.h"

#include <utility>

TorqueSchedulerAlgorithm::TorqueSchedulerAlgorithm(TorqueSchedulerConfig config) : cfg(std::move(config)) {}

void TorqueSchedulerAlgorithm::setConfig(const TorqueSchedulerConfig& config) { this->cfg = config; }

TorqueSchedulerOutput TorqueSchedulerAlgorithm::update(const float t,
                                                       const ArrayMotorTorqueMsgF32Payload& motorTorque1,
                                                       const ArrayMotorTorqueMsgF32Payload& motorTorque2) const {
    TorqueSchedulerOutput out;

    out.motorTorqueOut.motorTorque[0] = motorTorque1.motorTorque[0];
    out.motorTorqueOut.motorTorque[1] = motorTorque2.motorTorque[0];

    const float tSwitch = this->cfg.getTSwitch();

    switch (this->cfg.getLockFlag()) {
        case LockFlag::BothFree:
            out.effectorLockOut.effectorLockFlag[0] = 0;
            out.effectorLockOut.effectorLockFlag[1] = 0;
            break;

        case LockFlag::LockSecondThenFirst:
            if (t > tSwitch) {
                out.effectorLockOut.effectorLockFlag[0] = 1;
                out.effectorLockOut.effectorLockFlag[1] = 0;
            } else {
                out.effectorLockOut.effectorLockFlag[0] = 0;
                out.effectorLockOut.effectorLockFlag[1] = 1;
            }
            break;

        case LockFlag::LockFirstThenSecond:
            if (t > tSwitch) {
                out.effectorLockOut.effectorLockFlag[0] = 0;
                out.effectorLockOut.effectorLockFlag[1] = 1;
            } else {
                out.effectorLockOut.effectorLockFlag[0] = 1;
                out.effectorLockOut.effectorLockFlag[1] = 0;
            }
            break;

        case LockFlag::BothLocked:
            out.effectorLockOut.effectorLockFlag[0] = 1;
            out.effectorLockOut.effectorLockFlag[1] = 1;
            break;

            // No default: TorqueSchedulerConfig::create rejects any LockFlag value outside the
            // four enumerators, so all reachable cases are handled above.
    }

    return out;
}
