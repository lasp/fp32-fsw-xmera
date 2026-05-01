# SPDX-License-Identifier: ISC
# Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

"""Module Name: torqueScheduler"""

import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import torqueSchedulerF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


_LOCK_FLAG_FROM_INT = {
    0: torqueSchedulerF32.LockFlag_BothFree,
    1: torqueSchedulerF32.LockFlag_LockSecondThenFirst,
    2: torqueSchedulerF32.LockFlag_LockFirstThenSecond,
    3: torqueSchedulerF32.LockFlag_BothLocked,
}


@pytest.mark.parametrize("lock_flag", [0, 1, 2, 3])
@pytest.mark.parametrize("t_switch", [3.0, 6.0])
def test_torque_scheduler(lock_flag, t_switch):
    """Verify motorTorqueOutMsg passes inputs through and effectorLockOutMsg matches the schedule."""
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(1)
    test_proc = sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(sim.CreateNewTask(unit_task_name, test_process_rate))

    scheduler = torqueSchedulerF32.TorqueScheduler()
    scheduler.modelTag = "torqueScheduler"
    scheduler.lockFlag = _LOCK_FLAG_FROM_INT[lock_flag]
    scheduler.tSwitch = t_switch
    sim.AddModelToTask(unit_task_name, scheduler)

    motor_torque_1_in_data = messaging.ArrayMotorTorqueMsgF32Payload()
    motor_torque_1_in_data.motorTorque = [1.0]
    motor_torque_1_in_msg = messaging.ArrayMotorTorqueMsgF32().write(motor_torque_1_in_data)
    scheduler.motorTorque1InMsg.subscribeTo(motor_torque_1_in_msg)

    motor_torque_2_in_data = messaging.ArrayMotorTorqueMsgF32Payload()
    motor_torque_2_in_data.motorTorque = [3.0]
    motor_torque_2_in_msg = messaging.ArrayMotorTorqueMsgF32().write(motor_torque_2_in_data)
    scheduler.motorTorque2InMsg.subscribeTo(motor_torque_2_in_msg)

    torque_log = scheduler.motorTorqueOutMsg.recorder()
    sim.AddModelToTask(unit_task_name, torque_log)
    lock_log = scheduler.effectorLockOutMsg.recorder()
    sim.AddModelToTask(unit_task_name, lock_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(10))
    sim.ExecuteSimulation()

    times_s = torque_log.times() * macros.NANO2SEC

    # FP32 tolerance.
    accuracy = 1e-6

    for i, t in enumerate(times_s):
        np.testing.assert_allclose(torque_log.motorTorque[i][0], 1.0, atol=accuracy)
        np.testing.assert_allclose(torque_log.motorTorque[i][1], 3.0, atol=accuracy)

        if lock_flag == 0:
            expected = (0, 0)
        elif lock_flag == 1:
            expected = (1, 0) if t > t_switch else (0, 1)
        elif lock_flag == 2:
            expected = (0, 1) if t > t_switch else (1, 0)
        else:
            expected = (1, 1)

        assert lock_log.effectorLockFlag[i][0] == expected[0]
        assert lock_log.effectorLockFlag[i][1] == expected[1]


if __name__ == "__main__":
    test_torque_scheduler(1, 5.0)
