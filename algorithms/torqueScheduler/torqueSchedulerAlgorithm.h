// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TORQUE_SCHEDULER_ALGORITHM_H
#define F32XMERA_TORQUE_SCHEDULER_ALGORITHM_H

#include "torqueSchedulerTypes.h"

class TorqueSchedulerAlgorithm final {
   public:
    explicit TorqueSchedulerAlgorithm(const TorqueSchedulerConfig& config);

    void setConfig(const TorqueSchedulerConfig& config);

    TorqueSchedulerOutput update(float t,
                                 const ArrayMotorTorqueMsgF32Payload& motorTorque1,
                                 const ArrayMotorTorqueMsgF32Payload& motorTorque2) const;

   private:
    TorqueSchedulerConfig cfg;
};

#endif
