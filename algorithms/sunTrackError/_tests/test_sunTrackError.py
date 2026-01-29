#
#   Unit Test Script
#   Module Name: sunTrackError
#

import os
import pytest
import numpy as np

from xmera import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

# Import the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.fp32 import sunTrackErrorF32
from xmera.utilities import macros
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging


@pytest.mark.parametrize("sunAvoidance", [True, False])
@pytest.mark.parametrize("rateCatchUp", [True, False])
def test_sunTrackError(sunAvoidance, rateCatchUp):
    """Module Unit Test"""
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = sunTrackErrorF32.SunTrackError()
    module.modelTag = "sunTrackError"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    sigma_R0R = [[0.01], [0.05], [-0.55]]
    sigmaTest_R0R = [sigma_R0R[0][0], sigma_R0R[1][0], sigma_R0R[2][0]]
    module.sigma_R0R = sigma_R0R

    #
    # Navigation Message
    #
    NavStateOutData = messaging.NavAttMsgF32Payload()  # Create a structure for the input message
    sigma_BN = [0.25, -0.45, 0.75]
    NavStateOutData.sigma_BN = sigma_BN
    omega_BN_B = [-0.015, -0.012, 0.005]
    NavStateOutData.omega_BN_B = omega_BN_B
    navStateInMsg = messaging.NavAttMsgF32().write(NavStateOutData)

    #
    # Reference Frame Message
    #
    RefStateOutData = messaging.AttRefMsgF32Payload()  # Create a structure for the input message
    sigma_RN = [0.35, -0.25, 0.15]
    RefStateOutData.sigma_RN = sigma_RN
    omega_RN_N = [0.018, -0.032, 0.015]
    RefStateOutData.omega_RN_N = omega_RN_N
    domega_RN_N = [0.048, -0.022, 0.025]
    RefStateOutData.domega_RN_N = domega_RN_N
    refInMsg = messaging.AttRefMsgF32().write(RefStateOutData)

    if sunAvoidance:
        # Set maneuver rate and sensitive surface
        module.angleRate = 1 * np.pi / 180.0
        module.sensitiveHat_B = [[0.0], [-1.0], [0.0]]

        # Initialize ephemeris and celestial body information
        transNavData = messaging.NavTransMsgF32Payload()
        transNavData.r_BN_N = [-30, 20, -50]
        transNavMsg = messaging.NavTransMsgF32().write(transNavData)
        module.transNavInMsg.subscribeTo(transNavMsg)
        ephemerisData = messaging.EphemerisMsgF32Payload()
        ephemerisData.r_BdyZero_N = np.array([1, 2, 3])
        ephemerisMsg = messaging.EphemerisMsgF32().write(ephemerisData)
        module.ephemerisInMsg.subscribeTo(ephemerisMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.attGuidOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.attNavInMsg.subscribeTo(navStateInMsg)
    module.attRefInMsg.subscribeTo(refInMsg)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    if rateCatchUp:
        simTime = 100.0
    else:
        simTime = 5.0

    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))  # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    #
    # check sigma_BR
    #
    moduleOutput = dataLog.sigma_BR[-1]

    tolerance = 1e-6

    sigma_RN2 = rbk.addMRP(np.array(sigma_RN), -np.array(sigmaTest_R0R))
    RN = rbk.MRP2C(sigma_RN2)
    BN = rbk.MRP2C(np.array(sigma_BN))
    BR = np.dot(BN, RN.T)

    if sunAvoidance:
        if rateCatchUp:
            trueVector = [-0.37195403, 0.19732229, 0.20039198]
        else:
            trueVector = [-0.01740496, 0.00923336, 0.009377]
    else:
        trueVector = rbk.C2MRP(BR)

    # compare the module results to the truth values
    np.testing.assert_allclose(moduleOutput, trueVector, rtol=tolerance, atol=tolerance, verbose=True)

    #
    # check omega_BR_B
    #
    moduleOutput = dataLog.omega_BR_B[-1]

    # set the filtered output truth states
    if sunAvoidance:
        if rateCatchUp:
            trueVector = [-0.02573383, -0.00153051, -0.02691185]
        else:
            trueVector = [-0.02573383, -0.00153051, -0.02691185]
    else:
        trueVector = np.array(omega_BN_B) - np.dot(BN, np.array(omega_RN_N))

    # compare the module results to the truth values
    np.testing.assert_allclose(moduleOutput, trueVector, rtol=tolerance, atol=tolerance, verbose=True)

    #
    # check omega_RN_B
    #
    moduleOutput = dataLog.omega_RN_B[-1]

    # set the filtered output truth states
    # set the filtered output truth states
    if sunAvoidance:
        if rateCatchUp:
            trueVector = [0.01073383, -0.01046949, 0.03191185]
        else:
            trueVector = [0.01073383, -0.01046949, 0.03191185]
    else:
        trueVector = np.dot(BN, np.array(omega_RN_N))

    # compare the module results to the truth values
    np.testing.assert_allclose(moduleOutput, trueVector, rtol=tolerance, atol=tolerance, verbose=True)

    #
    # check domega_RN_B
    #
    moduleOutput = dataLog.domega_RN_B[-1]

    # set the filtered output truth states
    trueVector = np.dot(BN, np.array(domega_RN_N))

    # compare the module results to the truth values
    np.testing.assert_allclose(moduleOutput, trueVector, rtol=tolerance, atol=tolerance, verbose=True)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_sunTrackError(sunAvoidance=True, rateCatchUp=False)
