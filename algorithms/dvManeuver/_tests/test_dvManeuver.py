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

    The content of the THRArrayOnTimeCmdMsg and DvExecutionDataMsg output messages is compared with the true values.
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
    module.minTime = p2_tmin
    module.maxTime = p3_tmax

    # thruster information
    num_thrusters = 6
    acceleration_N = np.array([0.0, 0.0, 2.0])  # acceleration of spacecraft due to thrusters

    # Configure input messages
    nav_trans_msg_data = messaging.NavTransMsgF32Payload()
    nav_trans_msg_data.vehAccumDV = np.array([0.0, 0.0, 0.0])
    nav_trans_msg = messaging.NavTransMsgF32().write(nav_trans_msg_data)

    dv_burn_cmd_msg_data = messaging.DvBurnCmdMsgF32Payload()
    dv_burn_cmd_msg_data.dvInrtlCmd = np.array([0.0, 0.0, p1_dv])
    dv_burn_cmd_msg_data.burnStartTime = macros.sec2nano(p4_tstart)
    dv_burn_cmd_msg = messaging.DvBurnCmdMsgF32().write(dv_burn_cmd_msg_data)

    # Create the thruster on-time message and assign it as the module output.
    # The test also writes an initial value so we can verify that the module overwrites it.
    on_time_cmd_msg = messaging.THRArrayOnTimeCmdMsgF32()
    on_time_cmd_msg_data = messaging.THRArrayOnTimeCmdMsgF32Payload()
    # Seed the message with a nonzero value; the module overwrites it with
    # the commanded on-time while executing or zero when the thrusters are off.
    default_on_time = np.ones(num_thrusters)
    on_time_cmd_msg_data.onTimeRequest = default_on_time
    on_time_cmd_msg.write(on_time_cmd_msg_data)
    module.thrCmdOutMsg = on_time_cmd_msg

    # connect messages
    module.navDataInMsg.subscribeTo(nav_trans_msg)
    module.burnDataInMsg.subscribeTo(dv_burn_cmd_msg)

    # Setup logging on the test module output messages so that we get all the writes to it
    on_time_data_log = on_time_cmd_msg.recorder()
    sim.AddModelToTask(task_name, on_time_data_log)
    burn_exec_data_log = module.burnExecOutMsg.recorder()
    sim.AddModelToTask(task_name, burn_exec_data_log)

    sim.InitializeSimulation()

    # compute true values
    num_time_steps = 16
    on_time_true = np.zeros([num_time_steps, num_thrusters])
    burn_executing_true = np.zeros([num_time_steps])
    burn_complete_true = np.zeros([num_time_steps])
    for i in range(0, num_time_steps):
        if update_rate * i > p4_tstart:
            nav_trans_msg_data.vehAccumDV = acceleration_N * (update_rate * i - p4_tstart)
        nav_trans_msg.write(nav_trans_msg_data, sim.TotalSim.getCurrentNanos())

        # Write the seeded value before each step so the test verifies that the module
        # writes a fresh on-time command every update.
        on_time_cmd_msg.write(on_time_cmd_msg_data, sim.TotalSim.getCurrentNanos())

        sim.ConfigureStopTime(i * test_process_rate)
        sim.ExecuteSimulation()

        if (update_rate * (i + 1) <= p4_tstart):
            on_time_true[i] = np.zeros(num_thrusters)
            burn_executing_true[i] = 0
            burn_complete_true[i] = 0
        elif (np.linalg.norm(nav_trans_msg_data.vehAccumDV) >= np.linalg.norm(dv_burn_cmd_msg_data.dvInrtlCmd)) and \
                (update_rate * (i + 1) - p4_tstart > module.minTime) or \
                (update_rate * (i + 1) - p4_tstart > module.maxTime):
            on_time_true[i] = np.zeros(num_thrusters)
            burn_executing_true[i] = 0
            burn_complete_true[i] = 1
        else:
            on_time_true[i] = np.full(num_thrusters, 1.1 * update_rate)
            burn_executing_true[i] = 1
            burn_complete_true[i] = 0

    # pull module output
    on_time = on_time_data_log.onTimeRequest[:, :num_thrusters]
    burn_executing = burn_exec_data_log.burnExecuting
    burn_complete = burn_exec_data_log.burnComplete

    # compare the module results to the truth values
    params_string = ' for DV={}, min time={}, max time={}, start time={}'.format(
        str(p1_dv),
        str(p2_tmin),
        str(p3_tmax),
        str(p4_tstart))

    np.testing.assert_allclose(on_time,
                               on_time_true,
                               atol=1e-6,
                               rtol=1e-6,
                               err_msg=('Variable: on_time' + params_string),
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
