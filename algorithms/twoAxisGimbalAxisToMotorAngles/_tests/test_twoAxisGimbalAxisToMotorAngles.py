#  ISC License
#
#  Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
import inspect
import os
import sys

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.ema import twoAxisGimbalAxisToMotorAngles
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + "/../../../..")

from supportData.sep_gimbal_interpolation.sep_gimbal_interpolation import load_interpolation_tables


@pytest.mark.parametrize("gimbalTipAngleRef", [0.0 * macros.D2R, 10.1 * macros.D2R, -10.9 * macros.D2R])
@pytest.mark.parametrize("gimbalTiltAngleRef", [0.0 * macros.D2R, 10.1 * macros.D2R, -10.9 * macros.D2R])
@pytest.mark.parametrize("show_plots", [True])
def test_twoAxisGimbalAxisToMotorAngles(show_plots, gimbalTipAngleRef, gimbalTiltAngleRef):
    r"""
    **Validation Test Description**

    This unit test ensures that the two-axis gimbal flight software module twoAxisGimbalAxisToMotorAngles correctly
    determines the gimbal sequential tip and tilt angles corresponding to the reference body-frame thrust direction
    vector.

    **Test Parameters**

    Args:
        show_plots (bool):                          Variable for choosing whether plots should be displayed
        gimbalTipAngleRef (float):                  Gimbal tip reference angle
        gimbalTiltAngleRef (float):                 Gimbal tilt reference angle

    **Description of Variables Being Tested**

    This test checks that the gimbal angles determined and output by the module correctly correspond to the reference
    gimbal tip and tilt angles.
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testTimeStepSec = 0.1  # [s]
    testProcessRate = macros.sec2nano(testTimeStepSec)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Specify the commanded acceleration direction (negative thrust direction) in hub frame components
    # fmt:off
    thrustDirHatRef_B = np.array([np.sin(gimbalTiltAngleRef),
                                  -np.cos(gimbalTiltAngleRef) * np.sin(gimbalTipAngleRef),
                                  np.cos(gimbalTiltAngleRef) * np.cos(gimbalTipAngleRef)])
    # fmt:on

    # Specify the gimbal mount frame hub-relative attitude
    dcm_MB = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

    # Create thrust direction reference message
    thrustDirectionMessageData = messaging.BodyHeadingMsgPayload()
    thrustDirectionMessageData.rHat_XB_B = thrustDirHatRef_B
    thrustDirectionMessage = messaging.BodyHeadingMsg().write(thrustDirectionMessageData)

    # Load the gimbal interpolation tables
    (
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    ) = load_interpolation_tables()

    # Create the gimbal lookup table class
    gimbalLookupTables = twoAxisGimbalAxisToMotorAngles.TwoAxisGimbalLookupTables(
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    )

    # Create the twoAxisGimbalAxisToMotorAngles module (Module tested in this script)
    gimbalController = twoAxisGimbalAxisToMotorAngles.TwoAxisGimbalAxisToMotorAngles(gimbalLookupTables)
    gimbalController.modelTag = "twoAxisGimbalController"
    gimbalController.setDcmMB(dcm_MB)
    gimbalController.thrustDirectionInMsg.subscribeTo(thrustDirectionMessage)
    unitTestSim.AddModelToTask(unitTaskName, gimbalController)

    # Set up data logging for the module unit test
    gimbalAngleData = gimbalController.twoAxisGimbalOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, gimbalAngleData)

    # Run the simulation
    simTime = 5.0  # [s]
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for data comparison
    gimbalTipAnglesSim = macros.R2D * gimbalAngleData.theta1  # [deg]
    gimbalTiltAnglesSim = macros.R2D * gimbalAngleData.theta2  # [deg]

    #
    # Unit Test Verification
    #

    # # Print unit test checks
    # print("REFERENCE GIMBAL TIP ANGLE: ")
    # print(macros.R2D * gimbalTipAngleRef)
    # print("MODULE-DETERMINED TIP ANGLE: ")
    # print(gimbalTipAnglesSim[-1])
    #
    # print("REFERENCE GIMBAL TILT ANGLE: ")
    # print(macros.R2D * gimbalTiltAngleRef)
    # print("MODULE-DETERMINED TILT ANGLE: ")
    # print(gimbalTiltAnglesSim[-1])

    # Check that the module-determined angles match the reference values
    np.testing.assert_allclose(macros.R2D * gimbalTipAngleRef, gimbalTipAnglesSim[-1], atol=1e-8, verbose=True)
    np.testing.assert_allclose(macros.R2D * gimbalTiltAngleRef, gimbalTiltAnglesSim[-1], atol=1e-8, verbose=True)


if __name__ == "__main__":
    test_twoAxisGimbalAxisToMotorAngles(
        True,  # show_plots
        14.6 * macros.D2R,  # [rad] gimbalTipAngleRef
        -4.8 * macros.D2R,  # [rad] gimbalTiltAngleRef
    )
