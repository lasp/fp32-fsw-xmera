import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import thrDesatDutyCycleF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

# A representative per-thruster desaturation force command for an eight-thruster RCS cluster, where the
# upstream mapping stage has left two thrusters idle.
NOMINAL_FORCES = [1.2, 0.2, 0.0, 1.6, 1.2, 0.2, 1.6, 0.0]


@pytest.mark.parametrize(
    "firing_periods, settling_periods",
    [
        (1, 0),  # gate fully open, i.e. duty cycling disabled
        (1, 2),  # fire one period in three
        (2, 1),  # fire two periods in three
        (1, 20),  # settling window longer than the run, so the gate fires once and stays shut
    ],
)
def test_thr_desat_duty_cycle(firing_periods, settling_periods):
    """Module Unit Test"""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = thrDesatDutyCycleF32.ThrDesatDutyCycle()
    module.modelTag = "thrDesatDutyCycle"
    sim.AddModelToTask(task_name, module)

    # The declared defaults fire every period and never hold off, i.e. the gate starts fully open.
    np.testing.assert_equal(module.firingPeriods, 1)
    np.testing.assert_equal(module.settlingPeriods, 0)

    module.firingPeriods = firing_periods
    module.settlingPeriods = settling_periods

    # The module is configured through public properties rather than setter/getter pairs, so this checks the
    # SWIG-exposed properties round-trip what was written to them.
    np.testing.assert_equal(module.firingPeriods, firing_periods)
    np.testing.assert_equal(module.settlingPeriods, settling_periods)

    # The commanded force is held constant, so every variation in the output is the gate's doing.
    thr_force_message = messaging.THRArrayCmdForceMsgF32Payload()
    thr_force_message.thrForce = NOMINAL_FORCES
    thr_force_in_msg = messaging.THRArrayCmdForceMsgF32().write(thr_force_message)

    data_log = module.thrForceOutMsg.recorder()
    sim.AddModelToTask(task_name, data_log)

    module.thrForceInMsg.subscribeTo(thr_force_in_msg)

    sim.InitializeSimulation()

    # Nine updates, enough for three whole cycles of the (1, 2) and (2, 1) cadences.
    num_updates = 9
    sim.ConfigureStopTime((num_updates - 1) * test_process_rate)
    sim.ExecuteSimulation()

    module_output = np.array(data_log.thrForce)[:, : len(NOMINAL_FORCES)]
    np.testing.assert_equal(len(module_output), num_updates)

    # The gate passes the command through for the leading firing_periods slots of each cycle and commands zero
    # for the remaining settling_periods. It performs no arithmetic on the force, so a passed-through entry is
    # bit-identical to what the module stores and the comparison is exact. The reference is therefore built from
    # the single-precision round-trip of the command rather than from the python doubles: 1.2 is not
    # representable in float32, and that ~5e-8 representation error is the module's input, not its error.
    fired_forces = np.array(NOMINAL_FORCES, dtype=np.float32)
    settled_forces = np.zeros_like(fired_forces)

    cycle_length = firing_periods + settling_periods
    true_vector = [
        fired_forces if (update % cycle_length) < firing_periods else settled_forces
        for update in range(num_updates)
    ]

    np.testing.assert_allclose(module_output, true_vector, atol=0, rtol=0, verbose=True)

    # reset() built the algorithm config from these properties without modifying them, and pushing an edited
    # cadence onto the live algorithm through reconfigure() leaves them readable and unchanged.
    np.testing.assert_equal(module.firingPeriods, firing_periods)
    np.testing.assert_equal(module.settlingPeriods, settling_periods)

    module.firingPeriods = 2
    module.settlingPeriods = 5
    module.reconfigure()
    np.testing.assert_equal(module.firingPeriods, 2)
    np.testing.assert_equal(module.settlingPeriods, 5)
