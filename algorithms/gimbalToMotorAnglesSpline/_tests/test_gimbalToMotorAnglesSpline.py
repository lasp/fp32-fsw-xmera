"""
Module Name:        gimbalToMotorAnglesSpline

Interface test for the fp32 gimbalToMotorAnglesSpline module. The GTest suite tests the algorithm. This test sets the
adapter properties from the fitted configuration, sends one gimbal command through twoAxisGimbalInMsg, and checks
motor1AngleOutMsg and motor2AngleOutMsg.
"""

import inspect
import os

import numpy as np
from xmera.architecture import messaging
from xmera.fp32 import gimbalToMotorAnglesSplineF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


def test_gimbal_to_motor_angles_spline():
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # [rad/step] Step angle of the test configuration, the same as in the GTest suite
    step_angle = 0.01

    # Fitted rhombus (center tip, center tilt, tip half-diagonal, tilt half-diagonal) [rad] and coefficients [steps]
    domain = np.loadtxt(os.path.join(path, "splineDomain.csv"), delimiter=",")
    motor_1_coefficients = np.loadtxt(os.path.join(path, "splineCoefficientsMotor1.csv"), delimiter=",")
    motor_2_coefficients = np.loadtxt(os.path.join(path, "splineCoefficientsMotor2.csv"), delimiter=",")

    module = gimbalToMotorAnglesSplineF32.GimbalToMotorAnglesSpline()
    module.modelTag = "gimbalToMotorAnglesSpline"
    module.stepAngle = step_angle
    module.defaultMotor1Angle = -1.0
    module.defaultMotor2Angle = 1.0
    module.centerTipAngle = domain[0]
    module.centerTiltAngle = domain[1]
    module.halfDiagonalTip = domain[2]
    module.halfDiagonalTilt = domain[3]
    module.motor1SplineCoefficients = motor_1_coefficients.tolist()
    module.motor2SplineCoefficients = motor_2_coefficients.tolist()
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # One (tip, tilt) [rad], (steps1, steps2) [steps] row of the scipy (float64) evaluation of the fitted surfaces
    tip, tilt, steps1_truth, steps2_truth = np.loadtxt(
        os.path.join(path, "splineReference.csv"), delimiter=",", comments="#"
    )[0]

    gimbal_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_message_data.theta1 = tip
    gimbal_message_data.theta2 = tilt
    gimbal_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_message_data)
    module.twoAxisGimbalInMsg.subscribeTo(gimbal_message)

    motor_1_log = module.motor1AngleOutMsg.recorder()
    motor_2_log = module.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_log)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))
    unit_test_sim.ExecuteSimulation()

    # The motor angles are the motor steps times the step angle
    motor_1_steps = np.asarray(motor_1_log.theta) / step_angle
    motor_2_steps = np.asarray(motor_2_log.theta) / step_angle

    # [steps] The fp32 evaluation tolerance of the fitted surfaces, which the GTest suite computes and is less than
    # half a step
    tolerance = 0.5
    np.testing.assert_allclose(motor_1_steps, steps1_truth, atol=tolerance, rtol=0.0, verbose=True)
    np.testing.assert_allclose(motor_2_steps, steps2_truth, atol=tolerance, rtol=0.0, verbose=True)


if __name__ == "__main__":
    test_gimbal_to_motor_angles_spline()
