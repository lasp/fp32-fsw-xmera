import inspect
import os

import numpy as np

from xmera.architecture import messaging
from xmera.fp32 import ephemNavConverterF32
from xmera.utilities import SimulationBaseClass, unitTestSupport, macros
from xmera.utilities import astroFunctions

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def test_ephemNavConverter():
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # Add a new task to the process

    # Construct the ephemNavConverter module
    # Set the names for the input messages
    ephemNav = ephemNavConverterF32.EphemNavConverter()

    # This calls the algContain to setup the selfInit, update, and reset
    ephemNav.modelTag = "ephemNavConverter"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, ephemNav)

    # Create the input message.
    inputEphem = messaging.EphemerisMsgF32Payload()

    # Get the Earth's position and velocity
    position, velocity = astroFunctions.Earth_RV(astroFunctions.JulianDate([2018, 10, 16]))
    inputEphem.r_BdyZero_N = position
    inputEphem.v_BdyZero_N = velocity
    inputEphem.timeTag = 1.0  # sec
    inMsg = messaging.EphemerisMsgF32().write(inputEphem)
    ephemNav.ephInMsg.subscribeTo(inMsg)

    dataLog = ephemNav.stateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(testProcessRate)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    outputR = dataLog.r_BN_N
    outputV = dataLog.v_BN_N
    outputTime = dataLog.timeTag

    trueR = [position, position]
    trueV = [velocity, velocity]
    trueTime = [inputEphem.timeTag, inputEphem.timeTag]

    posAccuracy = 1e1
    velAccuracy = 1e-4

    np.testing.assert_allclose(outputR, trueR, atol=posAccuracy, rtol=0, err_msg="ephemNavConverter output Position")
    np.testing.assert_allclose(outputV, trueV, atol=velAccuracy, rtol=0, err_msg="ephemNavConverter output Velocity")
    np.testing.assert_allclose(outputTime, trueTime, atol=velAccuracy, rtol=0, err_msg="ephemNavConverter output Time")


if __name__ == '__main__':
    test_ephemNavConverter()
