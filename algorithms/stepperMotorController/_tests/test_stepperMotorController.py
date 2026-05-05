import math

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fp32 import stepperMotorControllerF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def wrap_delta(delta, steps_per_rev):
    """Wrap a step delta to shortest path [-steps_per_rev/2, +steps_per_rev/2]."""
    half = steps_per_rev // 2
    delta = delta % steps_per_rev  # Python % always returns [0, N)
    if delta > half:
        delta -= steps_per_rev
    return delta


def angle_to_steps(angle, steps_per_rev):
    """Convert angle in radians to integer step position, matching C++ float round-half-away-from-zero."""
    # Use float32 to match C++ single-precision computation
    angle_f = np.float32(angle)
    two_pi = np.float32(2.0) * np.float32(math.pi)
    x = float(angle_f * np.float32(steps_per_rev) / two_pi)
    # C++ round() rounds half away from zero; Python round() uses banker's rounding
    return int(math.floor(x + 0.5)) if x >= 0 else int(math.ceil(x - 0.5))


def create_sim(process_rate_sec):
    """Create a simulation with a single process and task."""
    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(process_rate_sec)
    proc = sim.CreateNewProcess("TestProcess")
    proc.addTask(sim.CreateNewTask("unitTask", process_rate))
    return sim, process_rate


@pytest.mark.parametrize("step_angle", [math.radians(1.8), math.radians(1.0), math.radians(0.9)])
@pytest.mark.parametrize("theta_init_deg", [0.0, -45.0, 90.0])
@pytest.mark.parametrize("theta_ref_deg", [10.0, -30.0, 50.5])
def test_stepper_motor_controller_nominal(show_plots, step_angle, theta_init_deg, theta_ref_deg):
    r"""
    **Validation Test Description**

    Verify that the stepper motor controller commands the correct number of steps
    (via shortest path wrapping) when moving from an initial step position to a reference angle.

    **Test Parameters**

    Args:
        step_angle (float): [rad/step] Angle per motor step
        theta_init_deg (float): [deg] Initial motor angle (converted to steps for the adapter)
        theta_ref_deg (float): [deg] Reference motor angle
    """
    process_rate_sec = 0.1
    sim, process_rate = create_sim(process_rate_sec)

    steps_per_rev = round(2 * math.pi / step_angle)
    init_step = angle_to_steps(theta_init_deg * macros.D2R, steps_per_rev)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.controlFrequency = 1.0 / process_rate_sec
    motor_controller.motorFrequency = 100.0
    motor_controller.currentPositionTolerance = 0
    motor_controller.desiredPositionTolerance = 0
    motor_controller.initialAngle = theta_init_deg * macros.D2R
    sim.AddModelToTask("unitTask", motor_controller)

    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = theta_ref_deg * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)

    log = motor_controller.motorStepCommandOutMsg.recorder(process_rate)
    sim.AddModelToTask("unitTask", log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(process_rate)
    sim.ExecuteSimulation()

    # Compute expected steps via shortest path
    refSteps = angle_to_steps(theta_ref_deg * macros.D2R, steps_per_rev)
    expectedSteps = wrap_delta(refSteps - init_step, steps_per_rev)

    if expectedSteps != 0:
        np.testing.assert_allclose(log.stepsCommanded[0], expectedSteps, atol=0, rtol=0)
    else:
        np.testing.assert_allclose(log.stepsCommanded[0], 0, atol=0, rtol=0)


@pytest.mark.parametrize("theta_init_deg, theta_ref_deg, expected_sign", [
    (-162.0, 162.0, -1),  # -0.9pi to 0.9pi: shortest path is backward
    (170.0, -170.0, 1),   # near +pi to near -pi: shortest path is forward
    (1.0, 359.0, -1),     # 1 deg to 359 deg: shortest path is -2 steps backward
])
def test_stepper_motor_controller_shortest_path(show_plots, theta_init_deg, theta_ref_deg, expected_sign):
    r"""
    **Validation Test Description**

    Verify that the controller wraps step commands to always take the shortest path around the circle,
    rather than the naive (possibly longer) path.

    **Test Parameters**

    Args:
        theta_init_deg (float): [deg] Initial motor angle (converted to steps for the adapter)
        theta_ref_deg (float): [deg] Reference motor angle
        expected_sign (int): Expected sign of the step command (-1 or +1)
    """
    step_angle = math.radians(1.0)  # 1 deg/step (= 360 steps/rev)
    steps_per_rev = round(2 * math.pi / step_angle)
    process_rate_sec = 0.1
    sim, process_rate = create_sim(process_rate_sec)

    init_step = angle_to_steps(theta_init_deg * macros.D2R, steps_per_rev)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.controlFrequency = 1.0 / process_rate_sec
    motor_controller.motorFrequency = 100.0
    motor_controller.currentPositionTolerance = 0
    motor_controller.desiredPositionTolerance = 0
    motor_controller.initialAngle = theta_init_deg * macros.D2R
    sim.AddModelToTask("unitTask", motor_controller)

    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = theta_ref_deg * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)

    log = motor_controller.motorStepCommandOutMsg.recorder(process_rate)
    sim.AddModelToTask("unitTask", log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(process_rate)
    sim.ExecuteSimulation()

    # Verify direction
    np.testing.assert_equal(np.sign(log.stepsCommanded[0]), expected_sign)

    # Verify magnitude via truth computation
    refSteps = angle_to_steps(theta_ref_deg * macros.D2R, steps_per_rev)
    expectedSteps = wrap_delta(refSteps - init_step, steps_per_rev)
    np.testing.assert_allclose(log.stepsCommanded[0], expectedSteps, atol=0, rtol=0)

    # Verify the magnitude is at most half a revolution (shortest path guarantee)
    np.testing.assert_array_less(np.abs(log.stepsCommanded[0]), steps_per_rev // 2 + 1)


def test_stepper_motor_controller_no_move_within_tolerance(show_plots):
    r"""
    **Validation Test Description**

    Verify that no move command is issued when the reference angle is within the current-position tolerance
    of the current motor position.
    """
    step_angle = math.radians(1.0)  # 1 deg/step (= 360 steps/rev)
    steps_per_rev = round(2 * math.pi / step_angle)
    process_rate_sec = 0.1
    sim, process_rate = create_sim(process_rate_sec)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.controlFrequency = 1.0 / process_rate_sec
    motor_controller.motorFrequency = 100.0
    motor_controller.currentPositionTolerance = 5
    motor_controller.desiredPositionTolerance = 0
    motor_controller.initialAngle = 0.0
    sim.AddModelToTask("unitTask", motor_controller)

    # Reference is 3 deg from init = 3 steps, within tolerance of 5
    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = 3.0 * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)

    log = motor_controller.motorStepCommandOutMsg.recorder(process_rate)
    sim.AddModelToTask("unitTask", log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(process_rate)
    sim.ExecuteSimulation()

    # No MOVE issued, output message should have default zero value
    np.testing.assert_allclose(log.stepsCommanded[0], 0, atol=0, rtol=0)


def test_stepper_motor_controller_interrupt(show_plots):
    r"""
    **Validation Test Description**

    Verify that changing the reference angle mid-move causes the controller to stop, settle, and
    then issue a new move command to the updated reference from the interrupted position.

    The motor advances 1 step per tick (motorFrequency == controlFrequency) for predictable
    intermediate positions.
    """
    step_angle = math.radians(1.0)  # 1 deg/step (= 360 steps/rev)
    steps_per_rev = round(2 * math.pi / step_angle)
    process_rate_sec = 0.1
    control_freq = 1.0 / process_rate_sec  # 10 Hz
    motor_freq = control_freq              # 1 step per tick
    settle_ticks = 2
    sim, process_rate = create_sim(process_rate_sec)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.controlFrequency = control_freq
    motor_controller.motorFrequency = motor_freq
    motor_controller.currentPositionTolerance = 0
    motor_controller.desiredPositionTolerance = 0
    motor_controller.settleCountMax = settle_ticks
    motor_controller.initialAngle = 0.0
    sim.AddModelToTask("unitTask", motor_controller)

    # First reference: 20 deg = 20 steps from init
    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = 20.0 * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)

    log = motor_controller.motorStepCommandOutMsg.recorder(process_rate)
    sim.AddModelToTask("unitTask", log)

    # Run 7 ticks (t=0.0, 0.1, ... 0.6):
    #   Tick at t=0.0: IDLE -> MOVE(20), enter MOVING, then advance 1 step (current=1)
    #   Ticks at t=0.1-0.6: MOVING, advancing 1 step/tick -> current = 7 after t=0.6
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.6))
    sim.ExecuteSimulation()

    # Verify first MOVE command
    np.testing.assert_allclose(log.stepsCommanded[0], 20, atol=0, rtol=0)

    # Change reference to 10 deg = 10 steps from origin
    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = 10.0 * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data, sim.TotalSim.getCurrentNanos())
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)

    # Run enough ticks for: MOVING(detect change)->STOPPING->SETTLING(2 ticks)->IDLE->MOVE
    #   Tick at t=0.7: MOVING, sees changed ref -> STOP, commanded frozen at current=7
    #   Tick at t=0.8: STOPPING -> SETTLING (settleCount=0)
    #   Tick at t=0.9: SETTLING (settleCount=0 < 2 -> settleCount=1)
    #   Tick at t=1.0: SETTLING (settleCount=1 < 2 -> settleCount=2)
    #   Tick at t=1.1: SETTLING (settleCount=2 >= 2 -> IDLE)
    #   Tick at t=1.2: IDLE -> MOVE(wrap_delta(10 - 7, 360) = 3)
    sim.ConfigureStopTime(macros.sec2nano(1.3))
    sim.ExecuteSimulation()

    # The second MOVE command writes 3 (interrupted at 7, target 10) and is the last value held by the message.
    np.testing.assert_allclose(log.stepsCommanded[-1], 3, atol=0, rtol=0)


def test_stepper_motor_controller_fractional_step_accumulation(show_plots):
    r"""
    **Validation Test Description**

    Verify that the step accumulator correctly handles a non-integer motor/control frequency ratio.
    With motorFrequency=15 Hz and controlFrequency=10 Hz the ratio is 1.5 steps per tick, producing
    the pattern 1, 2, 1, 2, ... steps per tick. After commanding 9 steps the motor must land exactly
    on step 9 (not 8 or 10). This is verified by commanding a return to the origin and checking
    that the return command is exactly -9 steps.
    """
    step_angle = math.radians(1.0)  # 1 deg/step (= 360 steps/rev)
    steps_per_rev = round(2 * math.pi / step_angle)
    process_rate_sec = 0.1
    control_freq = 1.0 / process_rate_sec  # 10 Hz
    motor_freq = 15.0                      # 15 Hz -> 1.5 steps/tick
    sim, process_rate = create_sim(process_rate_sec)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.controlFrequency = control_freq
    motor_controller.motorFrequency = motor_freq
    motor_controller.currentPositionTolerance = 0
    motor_controller.desiredPositionTolerance = 0
    motor_controller.settleCountMax = 0
    motor_controller.initialAngle = 0.0
    sim.AddModelToTask("unitTask", motor_controller)

    # Command 9 steps (9 deg with 360 steps/rev)
    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = 9.0 * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)

    log = motor_controller.motorStepCommandOutMsg.recorder(process_rate)
    sim.AddModelToTask("unitTask", log)

    # Run enough ticks for the full cycle to complete
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify first MOVE command
    np.testing.assert_allclose(log.stepsCommanded[0], 9, atol=0, rtol=0)

    # Now command return to origin — if accumulator was correct, current position is exactly 9
    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = 0.0
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data, sim.TotalSim.getCurrentNanos())
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)

    # Run more ticks for IDLE -> MOVE
    sim.ConfigureStopTime(macros.sec2nano(1.2))
    sim.ExecuteSimulation()

    # Return command should be exactly -9 (proves motor landed on step 9, not 8 or 10)
    np.testing.assert_allclose(log.stepsCommanded[-1], -9, atol=0, rtol=0)


if __name__ == "__main__":
    test_stepper_motor_controller_nominal(False, 360, 0.0, 10.0)
    test_stepper_motor_controller_shortest_path(False, -162.0, 162.0, -1)
    test_stepper_motor_controller_no_move_within_tolerance(False)
    test_stepper_motor_controller_interrupt(False)
    test_stepper_motor_controller_fractional_step_accumulation(False)
