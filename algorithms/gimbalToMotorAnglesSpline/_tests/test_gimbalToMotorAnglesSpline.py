import inspect
import os

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fp32 import gimbalToMotorAnglesSplineF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

motor_angle_home = 103.2242 * macros.D2R

# [rad] fp32 evaluation bound against the float64 scipy reference. The C++ test derives it from the committed
# coefficients (evaluationTolerance in gimbalToMotorAnglesSplineTestHelpers.hpp), which gives 4.2e-5 rad.
reference_tolerance = 4.3e-5

# [rad] Max fit error against the measured table inside the rhombus (0.0428 deg from the fit tool) plus fp32 error
table_tolerance = 0.045 * macros.D2R


def load_domain():
    """Return the fitted rhombus as (center tip, center tilt, tip half-diagonal, tilt half-diagonal) [rad]."""
    return np.loadtxt(os.path.join(path, "splineDomain.csv"), delimiter=",")


def load_coefficients():
    """Return the fitted spline control coefficients [rad]."""
    return np.loadtxt(os.path.join(path, "splineCoefficients.csv"), delimiter=",")


def load_samples(file_name):
    """Return the (tip, tilt, theta1, theta2) [rad] rows of a fixture file."""
    return np.loadtxt(os.path.join(path, file_name), delimiter=",", comments="#")


def run_module(gimbal_angles, steps_per_command=2):
    """Run the module for a sequence of gimbal commands and return the motor angles after each command.

    Args:
        gimbal_angles: Sequence of (tip, tilt) [rad] commands. Each command is written once.
        steps_per_command: Simulation steps per command.

    Returns:
        Array of (theta1, theta2) [rad], one row per command.
    """
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    domain = load_domain()
    module = gimbalToMotorAnglesSplineF32.GimbalToMotorAnglesSpline()
    module.modelTag = "gimbalToMotorAnglesSpline"
    module.centerTipAngle = domain[0]
    module.centerTiltAngle = domain[1]
    module.halfDiagonalTip = domain[2]
    module.halfDiagonalTilt = domain[3]
    module.splineCoefficients = load_coefficients().tolist()
    unit_test_sim.AddModelToTask(unit_task_name, module)

    gimbal_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_message = messaging.TwoAxisGimbalMsgF32()
    module.twoAxisGimbalInMsg.subscribeTo(gimbal_message)

    motor_1_recorder = module.motor1AngleOutMsg.recorder()
    motor_2_recorder = module.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_recorder)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_recorder)

    unit_test_sim.InitializeSimulation()
    results = []
    for k, (tip, tilt) in enumerate(gimbal_angles):
        gimbal_message_data.theta1 = tip
        gimbal_message_data.theta2 = tilt
        gimbal_message.write(gimbal_message_data, unit_test_sim.TotalSim.getCurrentNanos())
        unit_test_sim.ConfigureStopTime(macros.sec2nano((k + 1) * steps_per_command * test_time_step_sec))
        unit_test_sim.ExecuteSimulation()
        results.append((motor_1_recorder.theta[-1], motor_2_recorder.theta[-1]))
    return np.array(results)


def test_matches_scipy_reference_evaluation():
    samples = load_samples("splineReference.csv")
    # The last four rows are the rhombus vertices
    selected = np.vstack([samples[::20], samples[-4:]])
    results = run_module(selected[:, 0:2])

    np.testing.assert_allclose(results[:, 0], selected[:, 2], atol=reference_tolerance, rtol=0.0)
    np.testing.assert_allclose(results[:, 1], selected[:, 3], atol=reference_tolerance, rtol=0.0)


def test_matches_measured_table_inside_domain():
    samples = load_samples("tablePointsInDomain.csv")
    selected = samples[::97]
    results = run_module(selected[:, 0:2])

    np.testing.assert_allclose(results[:, 0], selected[:, 2], atol=table_tolerance, rtol=0.0)
    np.testing.assert_allclose(results[:, 1], selected[:, 3], atol=table_tolerance, rtol=0.0)


@pytest.mark.parametrize("gimbal_angles", [(25.0 * macros.D2R, 0.0), (0.0, -40.0 * macros.D2R)])
def test_first_command_outside_domain_returns_home(gimbal_angles):
    results = run_module([gimbal_angles])

    np.testing.assert_allclose(results[0], [motor_angle_home, motor_angle_home], atol=1e-6, rtol=0.0)


def test_outside_domain_holds_previous_valid_output():
    results = run_module([(2.0 * macros.D2R, -3.0 * macros.D2R), (0.0, 40.0 * macros.D2R)])

    np.testing.assert_array_equal(results[1], results[0])


def test_reset_without_linked_input_raises():
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess("TestProcess")
    test_proc.addTask(unit_test_sim.CreateNewTask("unitTask", macros.sec2nano(0.1)))
    domain = load_domain()
    module = gimbalToMotorAnglesSplineF32.GimbalToMotorAnglesSpline()
    module.centerTipAngle = domain[0]
    module.centerTiltAngle = domain[1]
    module.halfDiagonalTip = domain[2]
    module.halfDiagonalTilt = domain[3]
    module.splineCoefficients = load_coefficients().tolist()
    unit_test_sim.AddModelToTask("unitTask", module)

    with pytest.raises(Exception):
        unit_test_sim.InitializeSimulation()


def test_invalid_configuration_raises_at_reset():
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess("TestProcess")
    test_proc.addTask(unit_test_sim.CreateNewTask("unitTask", macros.sec2nano(0.1)))
    module = gimbalToMotorAnglesSplineF32.GimbalToMotorAnglesSpline()
    module.halfDiagonalTip = 0.0  # a degenerate rhombus is rejected
    module.halfDiagonalTilt = 0.4
    module.splineCoefficients = load_coefficients().tolist()
    gimbal_message = messaging.TwoAxisGimbalMsgF32().write(messaging.TwoAxisGimbalMsgF32Payload())
    module.twoAxisGimbalInMsg.subscribeTo(gimbal_message)
    unit_test_sim.AddModelToTask("unitTask", module)

    with pytest.raises(Exception):
        unit_test_sim.InitializeSimulation()


if __name__ == "__main__":
    test_matches_scipy_reference_evaluation()
