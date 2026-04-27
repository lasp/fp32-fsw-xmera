import numpy
import pytest
from xmera.architecture import messaging
from xmera.fp32 import thrFiringSchmittF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

import sys
import os
file_path = os.path.dirname(os.path.abspath(__file__))
abs_path = os.path.abspath(os.path.join(file_path, "../../utilities"))
sys.path.insert(0, abs_path)
import fswSetupThrusters


@pytest.mark.parametrize("reset_check, dv_on", [
    (False, False),
    (True, False),
    (False, True),
    (True, True),
])
def test_thr_firing_schmitt(show_plots, reset_check, dv_on):
    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = thrFiringSchmittF32.ThrFiringSchmitt()
    module.modelTag = "thrFiringSchmitt"

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Initialize the test module configuration data
    module.thrMinFireTime = 0.2
    if dv_on == 1:
        module.baseThrustState = 1
    else:
        module.baseThrustState = 0

    module.levelOn = .75
    module.levelOff = .25
    module.controlPeriod = 0.5

    # setup thruster cluster message
    fswSetupThrusters.clearSetup()
    rcs_location_data = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070]
        ]
    rcs_direction_data = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0]
        ]

    for i in range(len(rcs_location_data)):
        fswSetupThrusters.create(rcs_location_data[i], rcs_direction_data[i], 0.5)
    thr_conf_msg = fswSetupThrusters.writeConfigMessage()
    num_thrusters = fswSetupThrusters.getNumOfDevices()
    module.thrConfInMsg.subscribeTo(thr_conf_msg)

    # setup thruster impulse request message
    input_message_data = messaging.THRArrayCmdForceMsgF32Payload()
    thr_cmd_msg = messaging.THRArrayCmdForceMsgF32()
    module.thrForceInMsg.subscribeTo(thr_cmd_msg)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.onTimeOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.

    if dv_on:
        eff_req1 = [0.0, -0.1, -0.2, -0.3, -0.349, -0.351, -0.451, -0.5]
        eff_req2 = [0.0, -0.1, -0.2, -0.3, -0.351, -0.351, -0.451, -0.5]
        eff_req3 = [0.0, -0.1, -0.2, -0.3, -0.5, -0.351, -0.451, -0.5]
        eff_req4 = [0.0, -0.1, -0.2, -0.3, -0.351, -0.351, -0.451, -0.5]

    else:
        eff_req1 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.49]
        eff_req2 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.11]
        eff_req3 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.01]
        eff_req4 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.11]

    input_message_data.thrForce = eff_req1
    thr_cmd_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    input_message_data.thrForce = eff_req2
    thr_cmd_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    input_message_data.thrForce = eff_req3
    thr_cmd_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.5))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    input_message_data.thrForce = eff_req4
    thr_cmd_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(3.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    if reset_check:
        # reset the module to test this functionality
        module.reset(macros.sec2nano(3.0))     # this module reset function needs a time input (in NanoSeconds)

        # run the module again for an additional 1.0 seconds
        unit_test_sim.ConfigureStopTime(macros.sec2nano(5.5))        # seconds to stop simulation
        unit_test_sim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    module_output = data_log.onTimeRequest[:, :num_thrusters]

    # set the filtered output truth states
    if reset_check==1:
        if dv_on == 1:
            true_vector = [
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   ]
        else:
            true_vector = [
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   ]

    else:
        if dv_on == 1:
            true_vector = [
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   ]
        else:
            true_vector = [
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   ]

    numpy.testing.assert_allclose(module_output, true_vector, atol=1e-12, err_msg="onTimeRequest")


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thr_firing_schmitt(False, True, False)
