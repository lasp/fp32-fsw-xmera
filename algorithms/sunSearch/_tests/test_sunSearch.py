import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


from xmera.utilities import SimulationBaseClass
from xmera.fp32 import sunSearchF32
from xmera.utilities import macros
from xmera.architecture import messaging
from xmera.architecture import sim_model


@pytest.mark.parametrize("axes", [
    [sunSearchF32.RotationAxis_b1Hat_B, sunSearchF32.RotationAxis_b2Hat_B,
     sunSearchF32.RotationAxis_b3Hat_B, sunSearchF32.RotationAxis_b1Hat_B],
    [sunSearchF32.RotationAxis_b3Hat_B, sunSearchF32.RotationAxis_b3Hat_B,
     sunSearchF32.RotationAxis_b1Hat_B, sunSearchF32.RotationAxis_b2Hat_B],
])
@pytest.mark.parametrize("omega_BN_B", [[0, 0, 0], [0.01, -0.02, 0.03]])
def test_sun_search(show_plots, axes, omega_BN_B):

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    sim_model.setDefaultLogLevel(sim_model.BSK_WARNING)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.1)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    rotation_times = [5.0, 10.0, 7.0, 3.0]
    omega_norms = [0.1, 0.2, 0.15, 0.05]

    # Construct algorithm and associated C++ container
    module = sunSearchF32.SunSearch()
    module.modelTag = "sunSearch"

    for i in range(4):
        prop = sunSearchF32.RotationProperties()
        prop.rotationDuration = rotation_times[i]
        prop.rotationRate = omega_norms[i]
        prop.rotationAxis = axes[i]
        module.setRotation(i, prop)

    # Modify the second rotation via getRotation/setRotation
    modified_omega = 0.3
    rotation_1 = module.getRotation(1)
    rotation_1.rotationRate = modified_omega
    module.setRotation(1, rotation_1)
    omega_norms[1] = modified_omega

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Create input navigation message
    nav_att_data = messaging.NavAttMsgF32Payload()
    nav_att_data.omega_BN_B = omega_BN_B
    nav_att_msg = messaging.NavAttMsgF32().write(nav_att_data)
    module.attNavInMsg.subscribeTo(nav_att_msg)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.attGuidOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Run past the end of the scripted sequence to exercise the hold-last-omega behavior.
    total_time = sum(rotation_times)
    hold_time = 5.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(total_time + hold_time))

    # Begin the simulation time run set above
    unit_test_sim.ExecuteSimulation()

    time = data_log.times() * macros.NANO2SEC
    omega_BR_B = data_log.omega_BR_B
    omega_RN_B = data_log.omega_RN_B

    # Build truth arrays by mirroring the algorithm's active-slot selection:
    # use the first slot whose cumulative end time has not yet been reached, or hold the
    # last slot once the sequence has finished.
    omega_BR_B_truth = np.zeros((len(time), 3))
    omega_RN_B_truth = np.zeros((len(time), 3))

    rotation_end_times = np.cumsum(rotation_times)
    for i, t in enumerate(time):
        active_index = len(rotation_times) - 1
        for j in range(len(rotation_times)):
            if t < rotation_end_times[j]:
                active_index = j
                break
        axis_index = int(axes[active_index])
        omega_RN_B_truth[i, axis_index] = omega_norms[active_index]
        omega_BR_B_truth[i] = np.array(omega_BN_B) - omega_RN_B_truth[i]

    accuracy = 1e-6

    np.testing.assert_allclose(omega_BR_B, omega_BR_B_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omega_RN_B, omega_RN_B_truth, rtol=0, atol=accuracy, verbose=True)


if __name__ == "__main__":
    test_sun_search(False,
                    [sunSearchF32.RotationAxis_b1Hat_B, sunSearchF32.RotationAxis_b2Hat_B,
                     sunSearchF32.RotationAxis_b3Hat_B, sunSearchF32.RotationAxis_b1Hat_B],
                    [0, 0, 0])
