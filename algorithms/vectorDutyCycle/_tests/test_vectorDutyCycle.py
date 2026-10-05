import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import vectorDutyCycleF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

# A representative torque command, distinct per axis with mixed signs.
NOMINAL_TORQUE = [1.2e-2, -3.5e-3, 7.0e-4]


@pytest.mark.parametrize(
    "on_periods, off_periods",
    [
        (1, 0),  # gate fully open, i.e. duty cycling disabled
        (1, 2),  # on for one period in three
        (2, 1),  # on for two periods in three
        (1, 20),  # off window longer than the run, so the gate passes through once and stays shut
    ],
)
def test_vector_duty_cycle(on_periods, off_periods):
    """Module Unit Test"""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = vectorDutyCycleF32.VectorDutyCycle()
    module.modelTag = "vectorDutyCycle"
    sim.AddModelToTask(task_name, module)

    # The declared defaults are on every period and never hold off, i.e. the gate starts fully open.
    np.testing.assert_equal(module.onPeriods, 1)
    np.testing.assert_equal(module.offPeriods, 0)

    module.onPeriods = on_periods
    module.offPeriods = off_periods

    # The module is configured through public properties rather than setter/getter pairs, so this checks the
    # SWIG-exposed properties round-trip what was written to them.
    np.testing.assert_equal(module.onPeriods, on_periods)
    np.testing.assert_equal(module.offPeriods, off_periods)

    # The commanded torque is held constant, so every variation in the output is the gate's doing.
    cmd_torque_message = messaging.CmdTorqueBodyMsgF32Payload()
    cmd_torque_message.torqueRequestBody = NOMINAL_TORQUE
    cmd_torque_in_msg = messaging.CmdTorqueBodyMsgF32().write(cmd_torque_message)

    data_log = module.cmdTorqueOutMsg.recorder()
    sim.AddModelToTask(task_name, data_log)

    module.cmdTorqueInMsg.subscribeTo(cmd_torque_in_msg)

    sim.InitializeSimulation()

    # Nine updates, enough for three whole cycles of the (1, 2) and (2, 1) cadences.
    num_updates = 9
    sim.ConfigureStopTime((num_updates - 1) * test_process_rate)
    sim.ExecuteSimulation()

    module_output = np.array(data_log.torqueRequestBody)
    np.testing.assert_equal(len(module_output), num_updates)

    # The gate passes the command through for the leading on_periods slots of each cycle and commands zero
    # for the remaining off_periods. It performs no arithmetic on the torque, so a passed-through component
    # is identical to what the module stores and the comparison is exact. The reference is therefore built
    # from the single-precision round-trip of the command rather than from the python doubles: 1.2e-2 is not
    # representable in float32, and that representation error is the module's input, not its error.
    on_torque = np.array(NOMINAL_TORQUE, dtype=np.float32)
    off_torque = np.zeros_like(on_torque)

    cycle_length = on_periods + off_periods
    true_vector = [
        on_torque if (update % cycle_length) < on_periods else off_torque
        for update in range(num_updates)
    ]

    np.testing.assert_allclose(module_output, true_vector, atol=0, rtol=0, verbose=True)

    # reset() built the algorithm config from these properties without modifying them, and pushing an edited
    # cadence onto the live algorithm through reconfigure() leaves them readable and unchanged.
    np.testing.assert_equal(module.onPeriods, on_periods)
    np.testing.assert_equal(module.offPeriods, off_periods)

    module.onPeriods = 2
    module.offPeriods = 5
    module.reconfigure()
    np.testing.assert_equal(module.onPeriods, 2)
    np.testing.assert_equal(module.offPeriods, 5)
