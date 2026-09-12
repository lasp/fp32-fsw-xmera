import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import axisToGimbalAnglesF32
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

accuracy = 1e-5


def gimbal_axis_M(angle1, angle2):
    """The gimbal thrust axis in mount frame coordinates: T(alpha, beta), proportional to
    [tan(beta), -tan(alpha), 1], scaled so it stays finite up to the 90 degree boundary. The mount frame's +z axis
    is the un-deflected thrust, so a neutral gimbal gives [0, 0, 1]."""
    axis = np.array([np.sin(angle2) * np.cos(angle1),
                     -np.cos(angle2) * np.sin(angle1),
                     np.cos(angle2) * np.cos(angle1)])
    return axis / np.linalg.norm(axis)


@pytest.mark.parametrize("angle1", [0.0, 12.0 * macros.D2R, -18.25 * macros.D2R])
@pytest.mark.parametrize("angle2", [0.0, -7.5 * macros.D2R, 9.75 * macros.D2R])
@pytest.mark.parametrize("mount_euler_angles", [np.zeros(3), np.array([5.0, 10.0, 0.0]) * macros.D2R])
@pytest.mark.parametrize("request_scale", [1.0, 137.0])
def test_axis_to_gimbal_angles(angle1, angle2, mount_euler_angles, request_scale):
    """Module Unit Test: the module maps a commanded body-frame thrust direction onto the two gimbal angles that
    place the gimbal thrust axis on it. Each angle is the inclination of that axis projected into one of the mount planes
    containing the un-deflected axis. The request is built from a known pair of angles and rotated into the body
    frame, so the module must return that same pair whatever the mounting orientation and whatever the length of
    the request."""
    # The mount frame is defined with its +z axis along the un-deflected gimbal thrust axis, so sigma_MB carries
    # the gimbal's mounting orientation on the hub.
    sigma_MB = np.array(rbk.euler1232MRP(mount_euler_angles))
    dcm_MB = rbk.MRP2C(sigma_MB)

    # Build the request from the angles under test, then express it in the body frame the module reads it in.
    thrust_hat_M = gimbal_axis_M(angle1, angle2)
    thrust_hat_B = request_scale * np.matmul(dcm_MB.transpose(), thrust_hat_M)

    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(1)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = axisToGimbalAnglesF32.AxisToGimbalAngles()
    module.modelTag = "axisToGimbalAngles"
    sim.AddModelToTask(task_name, module)

    module.sigma_MB = sigma_MB
    module.thetaMax = 60.0 * macros.D2R

    thrust_direction_message = messaging.BodyHeadingMsgF32Payload()
    thrust_direction_message.rHat_XB_B = thrust_hat_B
    thrust_direction_in_msg = messaging.BodyHeadingMsgF32().write(thrust_direction_message)
    module.thrustDirectionInMsg.subscribeTo(thrust_direction_in_msg)

    gimbal_log = module.twoAxisGimbalOutMsg.recorder()
    sim.AddModelToTask(task_name, gimbal_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1))
    sim.ExecuteSimulation()

    theta1 = gimbal_log.theta1[-1]
    theta2 = gimbal_log.theta2[-1]

    # The module recovers the angles the request was built from.
    np.testing.assert_allclose(theta1, angle1, rtol=accuracy, atol=accuracy, verbose=True)
    np.testing.assert_allclose(theta2, angle2, rtol=accuracy, atol=accuracy, verbose=True)

    # And those angles put the gimbal thrust axis back on the request.
    np.testing.assert_allclose(gimbal_axis_M(theta1, theta2), thrust_hat_M, rtol=accuracy, atol=accuracy,
                               verbose=True)


def _run_single_request(request_B, theta_max):
    """Run the module for one request and return the two gimbal angles."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(1)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = axisToGimbalAnglesF32.AxisToGimbalAngles()
    module.modelTag = "axisToGimbalAngles"
    sim.AddModelToTask(task_name, module)
    module.thetaMax = theta_max

    thrust_direction_message = messaging.BodyHeadingMsgF32Payload()
    thrust_direction_message.rHat_XB_B = request_B
    thrust_direction_in_msg = messaging.BodyHeadingMsgF32().write(thrust_direction_message)
    module.thrustDirectionInMsg.subscribeTo(thrust_direction_in_msg)

    gimbal_log = module.twoAxisGimbalOutMsg.recorder()
    sim.AddModelToTask(task_name, gimbal_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1))
    sim.ExecuteSimulation()

    return gimbal_log.theta1[-1], gimbal_log.theta2[-1]


def test_axis_to_gimbal_angles_request_without_direction():
    """A request of zero length carries no direction at all, so there is nothing to point at and nothing to
    limit. The gimbal stays at its neutral position."""
    accuracy = 1e-6
    theta1, theta2 = _run_single_request(np.zeros(3), 60.0 * macros.D2R)

    np.testing.assert_allclose(theta1, 0.0, rtol=accuracy, atol=accuracy, verbose=True)
    np.testing.assert_allclose(theta2, 0.0, rtol=accuracy, atol=accuracy, verbose=True)


@pytest.mark.parametrize("request_B", [np.array([1.0, 0.0, 0.0]),     # exactly 90 deg of deflection
                                       np.array([0.0, 1.0, 0.0]),     # exactly 90 deg, on the other axis
                                       np.array([0.0, 0.0, -1.0]),    # opposite the neutral axis
                                       np.array([1.0, 1.0, -1.0])])   # beyond the travel, off both axes
@pytest.mark.parametrize("theta_max", [15.0 * macros.D2R, 45.0 * macros.D2R])
def test_axis_to_gimbal_angles_limits_the_deflection(request_B, theta_max):
    """A request beyond the travel of the mechanism goes to the edge of the cone of half-angle thetaMax, rather
    than to a railed pair of angles or back to the neutral position."""
    accuracy = 1e-5
    theta1, theta2 = _run_single_request(request_B, theta_max)

    # The two angles rebuild a direction at exactly the travel limit.
    deflection = np.arccos(np.clip(gimbal_axis_M(theta1, theta2)[2], -1.0, 1.0))
    np.testing.assert_allclose(deflection, theta_max, rtol=accuracy, atol=accuracy, verbose=True)

    # Each angle also stays inside the travel.
    assert abs(theta1) <= theta_max + accuracy
    assert abs(theta2) <= theta_max + accuracy
