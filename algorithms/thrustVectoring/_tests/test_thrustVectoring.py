import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import thrustVectoringF32
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


@pytest.mark.parametrize("seed", list(np.linspace(1, 10, 10)))
@pytest.mark.parametrize("delta_cm", [0.1, 0.2, 0.3])
@pytest.mark.parametrize("torque_request", [0.0, 0.05])
@pytest.mark.parametrize("arm_length", [0.0, 0.4])
@pytest.mark.parametrize("theta_max", [np.pi / 2, np.pi / 36])
@pytest.mark.parametrize("accuracy", [1e-4])
def test_thrust_vectoring(delta_cm, arm_length, torque_request, theta_max, seed, accuracy):
    """Module Unit Test: the platform points the thruster so it delivers the requested torque about the center of
    mass, reducing to alignment through the center of mass when no torque is requested. The center-of-mass offset
    is randomized over seed, so each parameter combination is exercised on ten geometries."""
    # Seed numpy's generator, used for the random center-of-mass shift below, so the test is deterministic and
    # independent of execution order.
    np.random.seed(int(seed))

    # The mount frame is defined with its -z axis along the un-deflected thrust, so sigma_MB carries the
    # thruster's mounting orientation.
    euler_angles_123 = np.array([5.0 * macros.D2R, 10.0 * macros.D2R, 0.0])
    sigma_MB = np.array(rbk.euler1232MRP(euler_angles_123))
    r_MB_B = np.array([0.0, 0.1, 1.4])
    thrust = 10.0

    r_CB_B = np.random.rand(3)
    r_CB_B = r_CB_B / np.linalg.norm(r_CB_B) * delta_cm

    L_req_B = np.array([1.0, -0.5, 0.8])
    L_req_B = L_req_B / np.linalg.norm(L_req_B) * torque_request

    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(1)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = thrustVectoringF32.ThrustVectoring()
    module.modelTag = "thrustVectoring"
    sim.AddModelToTask(task_name, module)

    module.sigma_MB = sigma_MB
    module.r_MB_B = r_MB_B
    module.armLength = arm_length
    module.thetaMax = theta_max

    veh_config_message = messaging.VehicleConfigMsgF32Payload()
    veh_config_message.CoM_B = r_CB_B
    veh_config_in_msg = messaging.VehicleConfigMsgF32().write(veh_config_message)
    module.vehConfigInMsg.subscribeTo(veh_config_in_msg)

    # The thruster fires along the platform -z axis from a point on that axis, so the module requires exactly
    # this description and takes only the magnitude from it.
    thr_config_message = messaging.THRConfigMsgF32Payload()
    thr_config_message.rThrust_B = np.array([0.0, 0.0, 0.0])
    thr_config_message.tHatThrust_B = np.array([0.0, 0.0, -1.0])
    thr_config_message.maxThrust = thrust
    thr_config_in_msg = messaging.THRConfigMsgF32().write(thr_config_message)
    module.thrusterConfigFInMsg.subscribeTo(thr_config_in_msg)

    cmd_torque_message = messaging.CmdTorqueBodyMsgF32Payload()
    cmd_torque_message.torqueRequestBody = L_req_B
    cmd_torque_in_msg = messaging.CmdTorqueBodyMsgF32().write(cmd_torque_message)
    module.cmdTorqueInMsg.subscribeTo(cmd_torque_in_msg)

    body_heading_log = module.bodyHeadingOutMsg.recorder()
    sim.AddModelToTask(task_name, body_heading_log)
    thr_config_b_log = module.thrusterConfigBOutMsg.recorder()
    sim.AddModelToTask(task_name, thr_config_b_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1))
    sim.ExecuteSimulation()

    tHat_B = body_heading_log.rHat_XB_B[-1]
    rThrust_B = thr_config_b_log.rThrust_B[-1]
    tHatThrust_B = thr_config_b_log.tHatThrust_B[-1]
    maxThrust = thr_config_b_log.maxThrust[-1]

    # The reported body-frame thrust heading is a unit vector and the thrust magnitude is preserved.
    np.testing.assert_allclose(np.linalg.norm(tHat_B), 1.0, rtol=accuracy, atol=accuracy, verbose=True)
    np.testing.assert_allclose(maxThrust, thrust, rtol=accuracy, atol=accuracy, verbose=True)

    # Both output messages report the same thrust direction.
    np.testing.assert_allclose(tHatThrust_B, tHat_B, rtol=accuracy, atol=accuracy, verbose=True)

    # The thrust deflection from its neutral, un-rotated direction stays within the configured cone.
    dcm_MB = rbk.MRP2C(sigma_MB)
    neutral_B = np.matmul(dcm_MB.transpose(), np.array([0.0, 0.0, -1.0]))
    deflection = np.arccos(np.clip(np.dot(neutral_B, tHat_B), -1.0, 1.0))
    np.testing.assert_array_less(deflection, theta_max + accuracy, verbose=True)

    # The thruster sits armLength behind the joint, along the thrust.
    np.testing.assert_allclose(rThrust_B, r_MB_B - arm_length * np.array(tHat_B), rtol=accuracy, atol=accuracy,
                               verbose=True)

    # Whenever the cone does not clamp the solution, the thruster delivers the requested torque about the center
    # of mass, up to the component along r_MC that this geometry cannot produce.
    if deflection < theta_max - accuracy:
        r_TC_B = np.array(rThrust_B) - r_CB_B
        if torque_request == 0.0:
            # A zero request aligns the thruster through the center of mass, so the moment arm is parallel to the
            # thrust direction.
            offset = np.linalg.norm(np.cross(tHat_B, r_TC_B)) / np.linalg.norm(r_TC_B)
            np.testing.assert_allclose(offset, 0.0, rtol=accuracy, atol=accuracy, verbose=True)
        else:
            r_MC_B = r_MB_B - r_CB_B
            rHat_MC_B = r_MC_B / np.linalg.norm(r_MC_B)
            L_achieved_B = np.cross(r_TC_B, thrust * np.array(tHat_B))
            L_req_reachable_B = L_req_B - np.dot(rHat_MC_B, L_req_B) * rHat_MC_B
            np.testing.assert_allclose(L_achieved_B, L_req_reachable_B, rtol=1e-2, atol=accuracy, verbose=True)


def test_thrust_vectoring_latches_configuration_at_reset():
    """Module Unit Test: the vehicle and thruster configuration messages are read once, when reset() builds the
    module configuration, and not on every update. The center of mass is rewritten after the simulation has
    started, which must not change the output until reconfigure() re-reads it. The requested torque is held at
    zero so the pointing does not drift between cycles on its own."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(1)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = thrustVectoringF32.ThrustVectoring()
    module.modelTag = "thrustVectoring"
    sim.AddModelToTask(task_name, module)

    module.sigma_MB = np.array([0.0, 0.0, 0.0])
    module.r_MB_B = np.array([0.0, 0.1, 1.4])
    module.armLength = 0.1
    module.thetaMax = np.pi / 2

    veh_config_message = messaging.VehicleConfigMsgF32Payload()
    veh_config_message.CoM_B = np.array([0.05, 0.02, 0.1])
    veh_config_in_msg = messaging.VehicleConfigMsgF32().write(veh_config_message)
    module.vehConfigInMsg.subscribeTo(veh_config_in_msg)

    thr_config_message = messaging.THRConfigMsgF32Payload()
    thr_config_message.rThrust_B = np.array([0.0, 0.0, 0.0])
    thr_config_message.tHatThrust_B = np.array([0.0, 0.0, -1.0])
    thr_config_message.maxThrust = 10.0
    thr_config_in_msg = messaging.THRConfigMsgF32().write(thr_config_message)
    module.thrusterConfigFInMsg.subscribeTo(thr_config_in_msg)

    cmd_torque_message = messaging.CmdTorqueBodyMsgF32Payload()
    cmd_torque_message.torqueRequestBody = np.array([0.0, 0.0, 0.0])
    cmd_torque_in_msg = messaging.CmdTorqueBodyMsgF32().write(cmd_torque_message)
    module.cmdTorqueInMsg.subscribeTo(cmd_torque_in_msg)

    body_heading_log = module.bodyHeadingOutMsg.recorder()
    sim.AddModelToTask(task_name, body_heading_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1))
    sim.ExecuteSimulation()
    heading_at_reset = np.array(body_heading_log.rHat_XB_B[-1])

    # Move the center of mass after reset: the module must keep using the value it latched.
    veh_config_message.CoM_B = np.array([-0.05, 0.15, 0.1])
    veh_config_in_msg.write(veh_config_message)
    sim.ConfigureStopTime(macros.sec2nano(2))
    sim.ExecuteSimulation()
    np.testing.assert_allclose(body_heading_log.rHat_XB_B[-1], heading_at_reset, rtol=1e-6, atol=1e-6,
                               verbose=True)

    # reconfigure() re-reads the messages, so the new center of mass now takes effect.
    module.reconfigure()
    sim.ConfigureStopTime(macros.sec2nano(3))
    sim.ExecuteSimulation()
    assert np.linalg.norm(np.array(body_heading_log.rHat_XB_B[-1]) - heading_at_reset) > 1e-4


if __name__ == "__main__":
    test_thrust_vectoring(0.1, 0.4, 0.05, np.pi / 2, 1.0, 1e-4)
    test_thrust_vectoring_latches_configuration_at_reset()
