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


def make_stepper_motor_msg_payload(motor_position=0, is_motor_moving=False):
    """Create a StepperMotorMsg pre-written with the given feedback values."""
    payload = messaging.StepperMotorMsgPayload()
    payload.motorPosition = motor_position
    payload.isMotorMoving = is_motor_moving
    return messaging.StepperMotorMsg().write(payload)


def write_stepper_motor_msg(msg, motor_position, is_motor_moving, time_ns):
    """Overwrite a StepperMotorMsg with new feedback values at the given sim time."""
    payload = messaging.StepperMotorMsgPayload()
    payload.motorPosition = motor_position
    payload.isMotorMoving = is_motor_moving
    msg.write(payload, time_ns)


@pytest.mark.parametrize("step_angle", [math.radians(1.8), math.radians(1.0), math.radians(0.9)])
@pytest.mark.parametrize("theta_init_deg", [0.0, -45.0, 90.0])
@pytest.mark.parametrize("theta_ref_deg", [10.0, -30.0, 50.5])
def test_stepper_motor_controller_nominal(show_plots, step_angle, theta_init_deg, theta_ref_deg):
    r"""
    **Validation Test Description**

    Verify that the stepper motor controller commands the correct number of steps
    (via shortest path wrapping) when moving from an initial step position to a reference angle.
    The motor's initial position is supplied via the feedback message's ``motorPosition`` field.
    """
    process_rate_sec = 0.1
    sim, process_rate = create_sim(process_rate_sec)

    steps_per_rev = round(2 * math.pi / step_angle)
    init_step = angle_to_steps(theta_init_deg * macros.D2R, steps_per_rev)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.setMotorAngleRange(0.0, 2 * math.pi)
    motor_controller.minStepCommand = 1
    sim.AddModelToTask("unitTask", motor_controller)

    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = theta_ref_deg * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)
    feedback_msg = make_stepper_motor_msg_payload(motor_position=init_step)
    motor_controller.stepperMotorInMsg.subscribeTo(feedback_msg)

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
    """
    step_angle = math.radians(1.0)  # 1 deg/step (= 360 steps/rev)
    steps_per_rev = round(2 * math.pi / step_angle)
    process_rate_sec = 0.1
    sim, process_rate = create_sim(process_rate_sec)

    init_step = angle_to_steps(theta_init_deg * macros.D2R, steps_per_rev)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.setMotorAngleRange(0.0, 2 * math.pi)
    motor_controller.minStepCommand = 1
    sim.AddModelToTask("unitTask", motor_controller)

    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = theta_ref_deg * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)
    feedback_msg = make_stepper_motor_msg_payload(motor_position=init_step)
    motor_controller.stepperMotorInMsg.subscribeTo(feedback_msg)

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


def test_stepper_motor_controller_below_min_step_command(show_plots):
    r"""
    **Validation Test Description**

    Verify that no move command is issued when the step delta between the reference angle and
    the current motor position is below ``minStepCommand``.
    """
    step_angle = math.radians(1.0)  # 1 deg/step (= 360 steps/rev)
    process_rate_sec = 0.1
    sim, process_rate = create_sim(process_rate_sec)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.setMotorAngleRange(0.0, 2 * math.pi)
    motor_controller.minStepCommand = 5
    sim.AddModelToTask("unitTask", motor_controller)

    # Reference is 3 deg from init = 3 steps, below minStepCommand of 5
    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = 3.0 * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)
    feedback_msg = make_stepper_motor_msg_payload(motor_position=0)
    motor_controller.stepperMotorInMsg.subscribeTo(feedback_msg)

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

    Verify that changing the reference angle mid-move causes the controller to transition to
    STOPPING and wait for the motor to finish its original commanded steps, then re-plan from the
    final position. The Xmera adapter does not propagate ``STOP`` to the motor (the motor cannot
    stop mid-step), so the motor reaches the original target before the algorithm re-plans.

    The "motor" is simulated here by writing the feedback message directly with chosen
    ``motorPosition`` and ``isMotorMoving`` values, so this test does not depend on the stepper
    motor dynamics module.
    """
    step_angle = math.radians(1.0)  # 1 deg/step
    process_rate_sec = 0.1
    settle_ticks = 2
    sim, process_rate = create_sim(process_rate_sec)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.setMotorAngleRange(0.0, 2 * math.pi)
    motor_controller.minStepCommand = 1
    motor_controller.settleCountMax = settle_ticks
    sim.AddModelToTask("unitTask", motor_controller)

    # First reference: 20 deg = 20 steps from motorPosition=0
    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = 20.0 * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)

    # Feedback: stand in for "motor already moving" so the algorithm doesn't transition out of
    # MOVING during Phase 1's second tick (the test can't update the feedback between two ticks
    # of the same ExecuteSimulation; in a real sim the dynamics would set isMotorMoving=True as
    # soon as it received the MOVE).
    feedback_msg = make_stepper_motor_msg_payload(motor_position=0, is_motor_moving=True)
    motor_controller.stepperMotorInMsg.subscribeTo(feedback_msg)

    log = motor_controller.motorStepCommandOutMsg.recorder(process_rate)
    sim.AddModelToTask("unitTask", log)

    # Phase 1: controller emits MOVE(20) at tick 0; tick 0.1 stays in MOVING
    sim.InitializeSimulation()
    sim.ConfigureStopTime(process_rate)
    sim.ExecuteSimulation()
    np.testing.assert_allclose(log.stepsCommanded[0], 20, atol=0, rtol=0)

    # Phase 2: motor is moving (mid-actuation, motorPosition=5)
    write_stepper_motor_msg(feedback_msg, motor_position=5, is_motor_moving=True, time_ns=sim.TotalSim.getCurrentNanos())
    sim.ConfigureStopTime(macros.sec2nano(0.4))
    sim.ExecuteSimulation()

    # Phase 3: reference changes to 10 deg mid-move → algorithm transitions to STOPPING
    ref_msg_data.theta = 10.0 * macros.D2R
    ref_msg.write(ref_msg_data, sim.TotalSim.getCurrentNanos())
    sim.ConfigureStopTime(macros.sec2nano(0.6))
    sim.ExecuteSimulation()

    # Phase 4: motor finishes original move at motorPosition=20 and stops → STOPPING → SETTLING → IDLE → MOVE(-10)
    write_stepper_motor_msg(feedback_msg, motor_position=20, is_motor_moving=False, time_ns=sim.TotalSim.getCurrentNanos())
    sim.ConfigureStopTime(macros.sec2nano(1.5))
    sim.ExecuteSimulation()

    # Second MOVE must be -10 (motor reached 20 first, then re-planned to 10)
    np.testing.assert_allclose(log.stepsCommanded[-1], -10, atol=0, rtol=0)


def test_stepper_motor_controller_position_tracking(show_plots):
    r"""
    **Validation Test Description**

    Verify the controller correctly uses the absolute ``motorPosition`` feedback across
    successive MOVE commands. Commanding a forward move followed by a return to the origin must
    produce exactly opposite step counts, which is only true if the controller reads the motor's
    absolute position from the feedback message.
    """
    step_angle = math.radians(1.0)
    process_rate_sec = 0.1
    sim, process_rate = create_sim(process_rate_sec)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.setMotorAngleRange(0.0, 2 * math.pi)
    motor_controller.minStepCommand = 1
    motor_controller.settleCountMax = 0
    sim.AddModelToTask("unitTask", motor_controller)

    # First reference: 9 deg = 9 steps
    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = 9.0 * macros.D2R
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)

    feedback_msg = make_stepper_motor_msg_payload(motor_position=0, is_motor_moving=False)
    motor_controller.stepperMotorInMsg.subscribeTo(feedback_msg)

    log = motor_controller.motorStepCommandOutMsg.recorder(process_rate)
    sim.AddModelToTask("unitTask", log)

    # Tick 0: motor at rest → MOVE(9)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(process_rate)
    sim.ExecuteSimulation()
    np.testing.assert_allclose(log.stepsCommanded[0], 9, atol=0, rtol=0)

    # Simulate motor reaching motorPosition=9 and stopping
    write_stepper_motor_msg(feedback_msg, motor_position=9, is_motor_moving=False, time_ns=sim.TotalSim.getCurrentNanos())

    # Change reference back to origin
    ref_msg_data.theta = 0.0
    ref_msg.write(ref_msg_data, sim.TotalSim.getCurrentNanos())

    # Run a few ticks for SETTLING (settleCountMax=0) → IDLE → MOVE(-9)
    sim.ConfigureStopTime(macros.sec2nano(0.5))
    sim.ExecuteSimulation()

    # Return command must be exactly -9 (proves controller read absolute motorPosition=9 from feedback)
    np.testing.assert_allclose(log.stepsCommanded[-1], -9, atol=0, rtol=0)


@pytest.mark.parametrize("ref_deg", [-10.0, 200.0])
def test_stepper_motor_controller_out_of_range(show_plots, ref_deg):
    r"""
    **Validation Test Description**

    Verify that the controller commands zero steps when the reference angle is outside the
    configured motor angle range. The configured range is [45 deg, 90 deg]; references at
    -10 deg (below min) and 200 deg (above max) must both result in no commanded movement.
    """
    step_angle = math.radians(1.0)
    process_rate_sec = 0.1
    sim, process_rate = create_sim(process_rate_sec)

    motor_controller = stepperMotorControllerF32.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.stepAngle = step_angle
    motor_controller.setMotorAngleRange(math.radians(45.0), math.radians(90.0))
    motor_controller.minStepCommand = 1
    sim.AddModelToTask("unitTask", motor_controller)

    # Motor starts in-range at 60 deg = 60 steps
    init_step = angle_to_steps(math.radians(60.0), round(2 * math.pi / step_angle))

    ref_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    ref_msg_data.theta = math.radians(ref_deg)
    ref_msg = messaging.HingedRigidBodyMsgF32().write(ref_msg_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(ref_msg)
    feedback_msg = make_stepper_motor_msg_payload(motor_position=init_step)
    motor_controller.stepperMotorInMsg.subscribeTo(feedback_msg)

    log = motor_controller.motorStepCommandOutMsg.recorder(process_rate)
    sim.AddModelToTask("unitTask", log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(process_rate)
    sim.ExecuteSimulation()

    np.testing.assert_allclose(log.stepsCommanded[0], 0, atol=0, rtol=0)


if __name__ == "__main__":
    test_stepper_motor_controller_nominal(False, math.radians(1.0), 0.0, 10.0)
    test_stepper_motor_controller_shortest_path(False, -162.0, 162.0, -1)
    test_stepper_motor_controller_below_min_step_command(False)
    test_stepper_motor_controller_interrupt(False)
    test_stepper_motor_controller_position_tracking(False)
    test_stepper_motor_controller_out_of_range(False, 200.0)
