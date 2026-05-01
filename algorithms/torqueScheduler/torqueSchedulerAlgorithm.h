// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TORQUE_SCHEDULER_ALGORITHM_H
#define F32XMERA_TORQUE_SCHEDULER_ALGORITHM_H

#include "torqueSchedulerTypes.h"

class TorqueSchedulerAlgorithm final {
   public:
    TorqueSchedulerOutput update(int lockFlag,
                                 double tSwitch,
                                 double t,
                                 const ArrayMotorTorqueMsgPayload& motorTorque1,
                                 const ArrayMotorTorqueMsgPayload& motorTorque2) const;
};

#endif
