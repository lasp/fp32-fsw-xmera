import itertools

import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import dvManeuverF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

# parameters
dv_magnitude = [4.3, 5.0, 10.0]
min_time = [0.0, 4.0]
max_time = [3.0, 5.0]
start_time = [0.0, 1.0]

param_array = [dv_magnitude, min_time, max_time, start_time]
# exclude invalid min/max time configurations (maxTime must always be greater than minTime)
param_list = [p for p in itertools.product(*param_array) if p[2] > p[1]]



@pytest.mark.parametrize("p1_dv, p2_tmin, p3_tmax, p4_tstart", param_list)
def test_dv_maneuver(show_plots, p1_dv, p2_tmin, p3_tmax, p4_tstart):
    r"""
    **Validation Test Description**

    This test checks if the dv burn module works correctly for different Delta-V magnitudes, minimum times and
    maximum times.

    **Test Parameters**

    Args:
        :param show_plots: flag if plots should be shown
        :param p1_dv: Delta-V magnitude
        :param p2_tmin: minimum time
        :param p3_tmax: maximum time
        :param p4_tstart: burn start time

    **Description of Variables Being Tested**

    The content of the CmdForceBodyMsg and DvExecutionDataMsg output messages is compared with the true values.
    """

    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    update_rate = 0.5
    test_process_rate = macros.sec2nano(update_rate)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module = dvManeuverF32.DvManeuver()
    module.modelTag = "dvManeuver"

    # Add test module to runtime call list
    sim.AddModelToTask(task_name, module)

    # Initialize the test module configuration data
    module.controlPeriod = update_rate
    module.minTime = macros.sec2nano(p2_tmin)
    module.maxTime = macros.sec2nano(p3_tmax)
    cmd_force_B = np.array([1.0, -2.0, 5.0])  # [N] body force commanded while the burn executes
    module.cmdForce_B = cmd_force_B
    cmd_dv_N = np.array([0.0, 0.0, p1_dv])  # [m/s] commanded Delta-V
    module.cmdDv_N = cmd_dv_N
    module.burnStartTime = macros.sec2nano(p4_tstart)

    acceleration_N = np.array([0.0, 0.0, 2.0])  # acceleration of spacecraft due to thrusters

    # Configure input messages
    nav_trans_msg_data = messaging.NavTransMsgF32Payload()
    nav_trans_msg_data.vehAccumDV = np.array([0.0, 0.0, 0.0])
    nav_trans_msg = messaging.NavTransMsgF32().write(nav_trans_msg_data)

    # connect messages
    module.navDataInMsg.subscribeTo(nav_trans_msg)

    # Setup logging on the test module output messages so that we get all the writes to it
    cmd_force_data_log = module.cmdForceOutMsg.recorder()
    sim.AddModelToTask(task_name, cmd_force_data_log)
    burn_exec_data_log = module.burnExecOutMsg.recorder()
    sim.AddModelToTask(task_name, burn_exec_data_log)

    sim.InitializeSimulation()

    # compute true values
    num_time_steps = 16
    cmd_force_true = np.zeros([num_time_steps, 3])
    burn_executing_true = np.zeros([num_time_steps], dtype=bool)
    burn_complete_true = np.zeros([num_time_steps], dtype=bool)
    for i in range(0, num_time_steps):
        if update_rate * i > p4_tstart:
            nav_trans_msg_data.vehAccumDV = acceleration_N * (update_rate * i - p4_tstart)
        nav_trans_msg.write(nav_trans_msg_data, sim.TotalSim.getCurrentNanos())

        sim.ConfigureStopTime(i * test_process_rate)
        sim.ExecuteSimulation()

        if (update_rate * (i + 1) <= p4_tstart):
            burn_executing_true[i] = False
            burn_complete_true[i] = False
        elif (np.linalg.norm(nav_trans_msg_data.vehAccumDV) >= np.linalg.norm(cmd_dv_N)) and \
                (update_rate * (i + 1) - p4_tstart > p2_tmin) or \
                (update_rate * (i + 1) - p4_tstart > p3_tmax):
            burn_executing_true[i] = False
            burn_complete_true[i] = True
        else:
            cmd_force_true[i] = cmd_force_B
            burn_executing_true[i] = True
            burn_complete_true[i] = False

    # pull module output
    cmd_force = cmd_force_data_log.forceRequestBody
    burn_executing = burn_exec_data_log.burnExecuting
    burn_complete = burn_exec_data_log.burnComplete

    # compare the module results to the truth values
    params_string = ' for DV={}, min time={}, max time={}, start time={}'.format(
        str(p1_dv),
        str(p2_tmin),
        str(p3_tmax),
        str(p4_tstart))

    np.testing.assert_allclose(cmd_force,
                               cmd_force_true,
                               atol=1e-6,
                               rtol=1e-6,
                               err_msg=('Variable: cmd_force' + params_string),
                               verbose=True)

    np.testing.assert_equal(burn_executing,
                            burn_executing_true,
                            err_msg=('Variable: burn_executing' + params_string),
                            verbose=True)

    np.testing.assert_equal(burn_complete,
                            burn_complete_true,
                            err_msg=('Variable: burn_complete' + params_string),
                            verbose=True)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_dv_maneuver(False, dv_magnitude[0], min_time[0], max_time[0], start_time[1])
