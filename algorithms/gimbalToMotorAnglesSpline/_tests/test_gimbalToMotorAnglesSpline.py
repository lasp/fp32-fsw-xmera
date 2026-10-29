"""
Module Name:        gimbalToMotorAnglesSpline

Fit tests for the fp32 gimbalToMotorAnglesSpline module. The GTest suite tests the algorithm. These tests check the
committed configuration (splineDomain.csv, splineCoefficientsMotor1.csv, and splineCoefficientsMotor2.csv) against
the lookup tables in _tests, like the oeStateEphem test checks its Chebyshev fit against SPICE. The reference values
come from the tables and from scipy at run time, so no reference data is committed.

- The adapter output matches the scipy (float64) evaluation of the committed coefficients at the table entries in the
  circle of radius thetaMax, within the fp32 evaluation tolerance.
- The adapter output at these entries, mapped back to gimbal angles with the motor-steps-to-gimbal table, is within
  MAX_ROUND_TRIP_ERROR of the requested gimbal angles.
- The committed configuration is what the fit tool gives for the gimbal-to-motor-steps table, and its coefficients are
  the least-squares solution.
"""

import importlib.util
import os

import numpy as np
import pytest
from scipy.interpolate import NdBSpline
from xmera.architecture import messaging
from xmera.fp32 import gimbalToMotorAnglesSplineF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

path = os.path.dirname(os.path.abspath(__file__))
TOOL_PATH = os.path.join(path, "..", "_tools", "fitGimbalToMotorSpline.py")

# [steps] Motor steps per revolution of the adapter in the tests. Any value works: the tests change the motor angles
# back to steps with the same step angle.
STEPS_PER_REVOLUTION = 628
SPLINE_DEGREE = 3
EPS32 = float(np.finfo(np.float32).eps)
# [deg] Largest distance between the requested gimbal angles and the gimbal angles that the motor-steps-to-gimbal table
# gives for the motor steps of the adapter. The committed configuration gives about 0.043 deg at most, and the two
# tables alone give about 0.006 deg (the motor steps of the table, mapped back).
MAX_ROUND_TRIP_ERROR = 0.05


def load_tool():
    """Import the offline fit tool, which is a script in _tools and not a package."""
    spec = importlib.util.spec_from_file_location("fitGimbalToMotorSpline", TOOL_PATH)
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    return tool


tool = load_tool()


class Configuration:
    """The committed configuration: rhombus [rad], thetaMax [rad], and the coefficients [steps] of the two motors."""

    def __init__(self):
        domain = np.loadtxt(os.path.join(path, "splineDomain.csv"), delimiter=",")
        self.half_diagonal_tip, self.half_diagonal_tilt, self.theta_max = domain  # [rad]
        self.coefficients = [
            np.loadtxt(os.path.join(path, f"splineCoefficientsMotor{motor}.csv"), delimiter=",") for motor in (1, 2)
        ]  # [steps]
        self.num_coeffs_per_axis = int(round(np.sqrt(self.coefficients[0].size)))
        self.intervals = self.num_coeffs_per_axis - SPLINE_DEGREE
        self.knots = tool.knot_vector(self.intervals)

    def uv(self, tip, tilt):
        """Return the normalized coordinates (u, v) [-] of gimbal angles [rad], in float64."""
        x = np.asarray(tip, dtype=float) / self.half_diagonal_tip
        y = np.asarray(tilt, dtype=float) / self.half_diagonal_tilt
        return x + y, x - y

    def surface(self, motor, tip, tilt):
        """Return the scipy (float64) value [steps] of the surface of a motor at gimbal angles [rad] in the rhombus."""
        u, v = self.uv(tip, tilt)
        coefficients = self.coefficients[motor - 1].reshape(self.num_coeffs_per_axis, self.num_coeffs_per_axis)
        spline = NdBSpline((self.knots, self.knots), coefficients, SPLINE_DEGREE)
        return spline(np.column_stack([np.clip(u, -1.0, 1.0), np.clip(v, -1.0, 1.0)]))

    def step_tolerance(self, motor):
        """Return the tolerance [steps] of the adapter output against the float64 surface of a motor, as
        stepTolerance in the GTest suite: the fp32 evaluation error of the surface value, the error of u and v times
        the slope bound of the surface, and the error of the conversion between steps and motor angles."""
        coefficients = self.coefficients[motor - 1]
        grid = coefficients.reshape(self.num_coeffs_per_axis, self.num_coeffs_per_axis)
        max_difference = max(np.max(np.abs(np.diff(grid, axis=0))), np.max(np.abs(np.diff(grid, axis=1))))
        slope_bound = max_difference / (2.0 / self.intervals)
        max_coefficient = np.max(np.abs(coefficients))
        surface_value_tolerance = 3.0 * 20.5 * EPS32 * max_coefficient
        coordinate_tolerance = 3.0 * 2.0 * 9.0 * EPS32 * slope_bound
        return surface_value_tolerance + coordinate_tolerance + EPS32 * max_coefficient


@pytest.fixture(scope="module")
def config():
    return Configuration()


@pytest.fixture(scope="module")
def gimbal_to_steps_table():
    """The gimbal-to-motor-steps lookup table that the fit uses."""
    return tool.StepsTable(path)


@pytest.fixture(scope="module")
def steps_to_gimbal_table():
    """The motor-steps-to-gimbal lookup table, which the fit never uses."""
    return tool.GimbalTable(path)


def run_adapter(config, tip, tilt):
    """Send each pair of gimbal angles [rad] to the adapter in its own task step and return the motor steps of the
    outputs. The steps are the motor angles divided by the fp32 step angle that the algorithm uses.

    Args:
        config: Configuration to set on the adapter.
        tip: Gimbal tip angles [rad].
        tilt: Gimbal tilt angles [rad].

    Returns:
        (n1, n2) [steps] arrays.
    """
    task_name = "unitTask"
    process_name = "TestProcess"
    time_step = macros.sec2nano(1.0)

    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess(process_name)
    process.addTask(sim.CreateNewTask(task_name, time_step))

    module = gimbalToMotorAnglesSplineF32.GimbalToMotorAnglesSpline()
    module.modelTag = "gimbalToMotorAnglesSpline"
    module.stepsPerRevolution = STEPS_PER_REVOLUTION
    module.halfDiagonalTip = config.half_diagonal_tip
    module.halfDiagonalTilt = config.half_diagonal_tilt
    module.thetaMax = config.theta_max
    module.motor1SplineCoefficients = config.coefficients[0].tolist()
    module.motor2SplineCoefficients = config.coefficients[1].tolist()
    sim.AddModelToTask(task_name, module)

    gimbal_message = messaging.TwoAxisGimbalMsgF32()
    module.twoAxisGimbalInMsg.subscribeTo(gimbal_message)
    gimbal_message_data = messaging.TwoAxisGimbalMsgF32Payload()

    sim.InitializeSimulation()
    motor_1_angles = np.zeros(len(tip))  # [rad]
    motor_2_angles = np.zeros(len(tip))  # [rad]
    for k, (tip_k, tilt_k) in enumerate(zip(tip, tilt)):
        # A new write time each step, so the adapter reads every request
        gimbal_message_data.theta1 = float(tip_k)
        gimbal_message_data.theta2 = float(tilt_k)
        gimbal_message.write(gimbal_message_data, k * time_step)
        sim.ConfigureStopTime(k * time_step)
        sim.ExecuteSimulation()
        motor_1_angles[k] = module.motor1AngleOutMsg.read().theta
        motor_2_angles[k] = module.motor2AngleOutMsg.read().theta

    step_angle = float(np.float32(2.0) * np.float32(np.pi) / np.float32(STEPS_PER_REVOLUTION))  # [rad/step]
    return motor_1_angles / step_angle, motor_2_angles / step_angle


@pytest.fixture(scope="module")
def table_cell_requests(config, gimbal_to_steps_table):
    """Send every table entry in the circle of radius thetaMax to the adapter, once for all the tests that use it. The
    rhombus is rotated from the table grid, so the entries cover every spline interval at many positions in it.

    Returns:
        (tip, tilt) [rad] fp32 gimbal angles of the requests, and (n1, n2) [steps] motor steps of the outputs.
    """
    d2r = np.pi / 180.0
    in_circle = np.hypot(gimbal_to_steps_table.tip, gimbal_to_steps_table.tilt) * d2r <= config.theta_max
    # The adapter receives fp32 angles, so the references use the same fp32 values
    tip = (gimbal_to_steps_table.tip[in_circle] * d2r).astype(np.float32)  # [rad]
    tilt = (gimbal_to_steps_table.tilt[in_circle] * d2r).astype(np.float32)  # [rad]
    return (tip, tilt), run_adapter(config, tip, tilt)


def test_matches_scipy_evaluation(config, table_cell_requests):
    """The adapter output matches the scipy (float64) evaluation of the committed coefficients at every table entry in
    the circle of radius thetaMax, within the fp32 evaluation tolerance. The table entries differ from the surfaces by
    the fit error, so the adapter is compared with scipy, not with the table."""
    (tip, tilt), steps = table_cell_requests
    for motor in (1, 2):
        np.testing.assert_allclose(steps[motor - 1], config.surface(motor, tip, tilt),
                                   atol=config.step_tolerance(motor), rtol=0.0, err_msg=f"motor {motor}")


def test_configuration_matches_fit_tool(config, gimbal_to_steps_table):
    """The committed configuration is what the fit tool gives for the gimbal-to-motor-steps table in _tests: the rhombus
    for the committed thetaMax, and the coefficients for that rhombus and the committed number of intervals. This fails
    if the table or the tool change and the configuration is not written again with --emit. The tool makes the rhombus
    with its margin past the circle, and before it writes the configuration it checks that fp32 rounding keeps the
    circle in the rhombus and that the rhombus is inside the populated table cells, so the committed rhombus has these
    properties too."""
    d2r = np.pi / 180.0
    rhombus = tool.circle_rhombus(gimbal_to_steps_table, config.theta_max / d2r)
    np.testing.assert_allclose([rhombus.a * d2r, rhombus.b * d2r],
                               [config.half_diagonal_tip, config.half_diagonal_tilt], rtol=1e-8, atol=0.0)

    fit = tool.SplineFit(gimbal_to_steps_table, rhombus, config.intervals)
    for motor, fitted in ((1, fit.coeffs1), (2, fit.coeffs2)):
        committed = config.coefficients[motor - 1]
        # The CSV files keep ten significant digits
        np.testing.assert_allclose(committed, fitted.ravel(), atol=1e-8 * np.max(np.abs(committed)), rtol=0.0,
                                   err_msg=f"motor {motor}")


def test_round_trip_through_steps_to_gimbal_table(table_cell_requests, steps_to_gimbal_table):
    """The motor steps of the adapter, mapped back to gimbal angles with the motor-steps-to-gimbal table, give the
    requested gimbal angles within MAX_ROUND_TRIP_ERROR, at every table entry in the circle of radius thetaMax. The fit
    never uses the motor-steps-to-gimbal table, so this is an independent check of the accuracy of the module. Motor
    steps outside that table give NaN, which fails the test."""
    d2r = np.pi / 180.0
    (tip, tilt), (n1, n2) = table_cell_requests
    mapped_tip, mapped_tilt = steps_to_gimbal_table.gimbal_angles(n1, n2)  # [deg], bilinear interpolation of the table
    error = np.hypot(mapped_tip - tip / d2r, mapped_tilt - tilt / d2r)  # [deg]
    assert not np.any(np.isnan(error)), f"{np.count_nonzero(np.isnan(error))} outputs are outside the table"
    worst = np.argmax(error)
    assert error[worst] <= MAX_ROUND_TRIP_ERROR, (
        f"error {error[worst]:.4f} deg at tip {tip[worst] / d2r:.4f} deg, tilt {tilt[worst] / d2r:.4f} deg")


def test_coefficients_are_least_squares(config, gimbal_to_steps_table):
    """The committed coefficients minimize the sum of the squared errors at the table entries in the rhombus: the
    gradient g = A^T (A c - n) of half the sum is zero, to within the rounding of the coefficients in the CSV files.
    The files keep ten significant digits, so each coefficient is off by at most 5e-10 of its value. A change d of the
    coefficients changes the gradient by A^T A d, and the basis values in A are not negative, so the bound for each
    gradient component is A^T A (1e-9 |c|), twice the rounding for margin."""
    d2r = np.pi / 180.0
    u, v = config.uv(gimbal_to_steps_table.tip * d2r, gimbal_to_steps_table.tilt * d2r)
    in_rhombus = (np.abs(u) <= 1.0) & (np.abs(v) <= 1.0)
    matrix = tool.design_matrix(u[in_rhombus], v[in_rhombus], config.knots)
    measured = (gimbal_to_steps_table.motor1[in_rhombus], gimbal_to_steps_table.motor2[in_rhombus])  # [steps]

    for motor in (1, 2):
        coefficients = config.coefficients[motor - 1]
        gradient = matrix.T @ (matrix @ coefficients - measured[motor - 1])  # [steps]
        tolerance = matrix.T @ (matrix @ (1e-9 * np.abs(coefficients)))  # [steps]
        np.testing.assert_array_less(np.abs(gradient), tolerance, err_msg=f"motor {motor}")
