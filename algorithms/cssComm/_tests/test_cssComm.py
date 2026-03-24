"""
Module Name:        cssComm
Updated On:         February 10, 2019
"""

import inspect
import os

import pytest
import numpy as np
from xmera.architecture import messaging
from xmera.fp32 import cssCommF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

@pytest.mark.parametrize("num_sensors, sensor_data", [
    (4, [-100e-6, 200e-6, 600e-6, 300e-6]),  # Five data inputs used despite four sensors to ensure all reset conditions are tested.
    (messaging.MAX_NUM_CSS_SENSORS, [200e-6]*messaging.MAX_NUM_CSS_SENSORS)  # Indicate more sensor devices than is allowed.  The output should be clipped to the allowed length
])


def test_css_comm(num_sensors, sensor_data):
    """Exercise the Python/SWIG interface for CssComm.

    Verifies that the module can be configured, connected, and run
    within the simulation framework, and that it produces sensible output.
    Correctness is validated by the C++ unit and fuzz tests.
    """
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate)) # Add a new task to the process

    # Construct the cssComm module
    module = cssCommF32.CssComm()
    module.modelTag = "cssComm"
    module.numSensors = num_sensors
    module.maxSensorValue = 500e-6

    cheby_list =  [-1.734963346951471e+06, 3.294117146099591e+06,
                     -2.816333294617512e+06, 2.163709942144332e+06,
                     -1.488025993860025e+06, 9.107359382775769e+05,
                     -4.919712500291216e+05, 2.318436583511218e+05,
                     -9.376105045529010e+04, 3.177536873430168e+04]
    module.chebyCount = len(cheby_list)
    module.chebyPolynomials = cheby_list

    # Add the module to the task
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # The cssComm module reads in from the sensor list, so create that message here
    css_array_msg = messaging.CSSArraySensorMsgF32Payload()

    # NOTE: This is nonsense. These are more or less random numbers
    css_array_msg.CosValue = sensor_data
    css_in_msg = messaging.CSSArraySensorMsgF32().write(css_array_msg)
    module.sensorListInMsg.subscribeTo(css_in_msg)

    # Log the output message
    data_log = module.cssArrayOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Initialize the simulation
    unit_test_sim.InitializeSimulation()

    unit_test_sim.ConfigureStopTime(test_process_rate)
    unit_test_sim.ExecuteSimulation()

    # Get the output from this simulation
    MAX_NUM_CSS_SENSORS = messaging.MAX_NUM_CSS_SENSORS
    output_data = data_log.CosValue

    # Verify output shape
    assert output_data.shape == (2, MAX_NUM_CSS_SENSORS)

    # All output values must be in [0, 1] (clamped cosine range)
    assert np.all(output_data >= 0.0)
    assert np.all(output_data <= 1.0)

    # All output values must be finite
    assert np.all(np.isfinite(output_data))

    # Sensors beyond numSensors must be zero
    assert np.all(output_data[:, num_sensors:] == 0.0)

    accuracy = 1e-6

    # Getter/setter round-trips
    np.testing.assert_allclose(module.numSensors, num_sensors, atol=accuracy, rtol=accuracy)
    np.testing.assert_allclose(module.maxSensorValue, 500e-6, atol=accuracy, rtol=accuracy)
    np.testing.assert_allclose(module.chebyCount, len(cheby_list), atol=accuracy, rtol=accuracy)
    np.testing.assert_allclose(module.chebyPolynomials, cheby_list, atol=accuracy, rtol=accuracy)


if __name__ == '__main__':
    test_css_comm(4, [-100e-6, 200e-6, 600e-6, 300e-6, 200e-6])
