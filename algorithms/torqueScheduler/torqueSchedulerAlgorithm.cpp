// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "torqueSchedulerAlgorithm.h"

TorqueSchedulerOutput TorqueSchedulerAlgorithm::update(const int lockFlag,
                                                       const float tSwitch,
                                                       const float t,
                                                       const ArrayMotorTorqueMsgF32Payload& motorTorque1,
                                                       const ArrayMotorTorqueMsgF32Payload& motorTorque2) const {
    TorqueSchedulerOutput out;

    out.motorTorqueOut.motorTorque[0] = motorTorque1.motorTorque[0];
    out.motorTorqueOut.motorTorque[1] = motorTorque2.motorTorque[0];

    switch (lockFlag) {
        case 0:
            out.effectorLockOut.effectorLockFlag[0] = 0;
            out.effectorLockOut.effectorLockFlag[1] = 0;
            break;

        case 1:
            if (t > tSwitch) {
                out.effectorLockOut.effectorLockFlag[0] = 1;
                out.effectorLockOut.effectorLockFlag[1] = 0;
            } else {
                out.effectorLockOut.effectorLockFlag[0] = 0;
                out.effectorLockOut.effectorLockFlag[1] = 1;
            }
            break;

        case 2:
            if (t > tSwitch) {
                out.effectorLockOut.effectorLockFlag[0] = 0;
                out.effectorLockOut.effectorLockFlag[1] = 1;
            } else {
                out.effectorLockOut.effectorLockFlag[0] = 1;
                out.effectorLockOut.effectorLockFlag[1] = 0;
            }
            break;

        case 3:
            out.effectorLockOut.effectorLockFlag[0] = 1;
            out.effectorLockOut.effectorLockFlag[1] = 1;
            break;

        default:
            // lockFlag outside [0, 3] leaves out.effectorLockOut at the zero-init default. Step 14
            // Config validation will make this case unreachable.
            break;
    }

    return out;
}
