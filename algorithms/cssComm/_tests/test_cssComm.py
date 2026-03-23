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

@pytest.mark.parametrize("numSensors, sensorData", [
    (4, [-100e-6, 200e-6, 600e-6, 300e-6]),  # Five data inputs used despite four sensors to ensure all reset conditions are tested.
    (messaging.MAX_NUM_CSS_SENSORS, [200e-6]*messaging.MAX_NUM_CSS_SENSORS)  # Indicate more sensor devices than is allowed.  The output should be clipped to the allowed length
])


def test_cssComm(numSensors, sensorData):
    """Exercise the Python/SWIG interface for CssComm.

    Verifies that the module can be configured, connected, and run
    within the simulation framework, and that it produces sensible output.
    Correctness is validated by the C++ unit and fuzz tests.
    """
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate)) # Add a new task to the process

    # Construct the cssComm module
    module = cssCommF32.CssComm()
    module.modelTag = "cssComm"
    module.numSensors = numSensors
    module.maxSensorValue = 500e-6

    ChebyList =  [-1.734963346951471e+06, 3.294117146099591e+06,
                     -2.816333294617512e+06, 2.163709942144332e+06,
                     -1.488025993860025e+06, 9.107359382775769e+05,
                     -4.919712500291216e+05, 2.318436583511218e+05,
                     -9.376105045529010e+04, 3.177536873430168e+04,
                     -8.704033370738143e+03, 1.816188108176300e+03,
                     -2.581556805090373e+02, 1.888418924282780e+01]
    module.chebyCount = len(ChebyList)
    module.chebyPolynomials = ChebyList + [0] * (32 - len(ChebyList))

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, module)

    # The cssComm module reads in from the sensor list, so create that message here
    cssArrayMsg = messaging.CSSArraySensorMsgPayload()

    # NOTE: This is nonsense. These are more or less random numbers
    cssArrayMsg.CosValue = sensorData
    cssInMsg = messaging.CSSArraySensorMsg().write(cssArrayMsg)
    module.sensorListInMsg.subscribeTo(cssInMsg)

    # Log the output message
    dataLog = module.cssArrayOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    unitTestSim.ConfigureStopTime(testProcessRate)
    unitTestSim.ExecuteSimulation()

    # Get the output from this simulation
    MAX_NUM_CSS_SENSORS = messaging.MAX_NUM_CSS_SENSORS
    outputData = dataLog.CosValue

    # Verify output shape
    assert outputData.shape == (2, MAX_NUM_CSS_SENSORS)

    # All output values must be in [0, 1] (clamped cosine range)
    assert np.all(outputData >= 0.0)
    assert np.all(outputData <= 1.0)

    # All output values must be finite
    assert np.all(np.isfinite(outputData))

    # Sensors beyond numSensors must be zero
    assert np.all(outputData[:, numSensors:] == 0.0)

    # Getter/setter round-trips
    np.testing.assert_allclose(module.numSensors, numSensors)
    np.testing.assert_allclose(module.maxSensorValue, 500e-6, atol=1e-6)
    np.testing.assert_allclose(module.chebyCount, len(ChebyList))


if __name__ == '__main__':
    test_cssComm(4, [-100e-6, 200e-6, 600e-6, 300e-6, 200e-6])