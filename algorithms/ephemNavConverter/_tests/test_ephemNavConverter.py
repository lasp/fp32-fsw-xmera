import inspect
import os

import numpy as np

from xmera.architecture import messaging
from xmera.fp32 import ephemNavConverterF32
from xmera.utilities import SimulationBaseClass, macros
from xmera.utilities import astroFunctions

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def test_ephem_nav_converter():
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))  # Add a new task to the process

    # Construct the ephemNavConverter module
    # Set the names for the input messages
    ephem_nav = ephemNavConverterF32.EphemNavConverter()

    # This calls the algContain to setup the selfInit, update, and reset
    ephem_nav.modelTag = "ephemNavConverter"

    # Add the module to the task
    unit_test_sim.AddModelToTask(unit_task_name, ephem_nav)

    # Create the input message.
    input_ephem = messaging.EphemerisMsgF32Payload()

    # Get the Earth's position and velocity
    position, velocity = astroFunctions.Earth_RV(astroFunctions.JulianDate([2018, 10, 16]))
    input_ephem.r_BdyZero_N = position
    input_ephem.v_BdyZero_N = velocity
    input_ephem.timeTag = 1.0  # sec
    in_msg = messaging.EphemerisMsgF32().write(input_ephem)
    ephem_nav.ephInMsg.subscribeTo(in_msg)

    data_log = ephem_nav.stateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Initialize the simulation
    unit_test_sim.InitializeSimulation()

    # The result isn't going to change with more time. The module will continue to produce the same result
    unit_test_sim.ConfigureStopTime(test_process_rate)  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    output_r = data_log.r_BN_N
    output_v = data_log.v_BN_N
    output_time = data_log.timeTag

    true_r = [position, position]
    true_v = [velocity, velocity]
    true_time = [input_ephem.timeTag, input_ephem.timeTag]

    np.testing.assert_array_equal(output_r, true_r, err_msg="ephemNavConverter output Position")
    np.testing.assert_array_equal(output_v, true_v, err_msg="ephemNavConverter output Velocity")
    np.testing.assert_array_equal(output_time, true_time, err_msg="ephemNavConverter output Time")


if __name__ == '__main__':
    test_ephem_nav_converter()
