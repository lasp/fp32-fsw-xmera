import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import convertStPlatformToBodyF32
from xmera.utilities import macros
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging

def pack_angular_rate_quaternion(omega_CN_C):
    """Pack a rate into the micro-ASC rate quaternion [axis*sin(|w|/2), cos(|w|/2)] (scalar-last)."""
    angle = np.linalg.norm(omega_CN_C)
    if angle == 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    axis = omega_CN_C / angle
    return [*(axis * np.sin(angle / 2.0)), np.cos(angle / 2.0)]


def test_identity_mounting():
    """With identity mounting the output MRP and omega should match the case-frame values."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "convertStPlatformToBody"
    sim.AddModelToTask(task_name, module)

    # Zero mounting MRP (default)

    # Build a star tracker sensor input: 30-deg rotation about z, small omega
    angle = np.pi/6.0
    ep_CN = np.array([np.cos(angle/2), 0, 0, np.sin(angle/2)])
    omega_CN_C = np.array([0.01, -0.02, 0.03])

    sensor_data = messaging.AscPlatformAttitudeTmMsgF32Payload()
    sensor_data.solutionTimestamp = 100.0
    sensor_data.platformAttitude = [ep_CN[1], ep_CN[2], ep_CN[3], ep_CN[0]]
    sensor_data.platformAngularVelocity = pack_angular_rate_quaternion(omega_CN_C)
    sensor_in_msg = messaging.AscPlatformAttitudeTmMsgF32().write(sensor_data)

    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.3))
    sim.ExecuteSimulation()

    # With identity mounting, sigma_BN == sigma_CN and omega_BN_B == omega_CN_C
    sigma_CN = rbk.EP2MRP(ep_CN)
    sigma_BN = att_log.MRP_BdyInrtl[0]
    omega_BN_B = att_log.omega_BN_B[0]

    accuracy = 1e-6
    np.testing.assert_allclose(att_log.timeTag[0], 100.0, atol=accuracy,
                               err_msg="timeTag must be passed through in seconds")
    np.testing.assert_allclose(sigma_BN, sigma_CN, atol=accuracy,
                               err_msg="MRP mismatch with identity mounting")
    np.testing.assert_allclose(omega_BN_B, omega_CN_C, atol=accuracy,
                               err_msg="omega mismatch with identity mounting")


def test_rotated_mounting():
    """With a non-trivial mounting orientation, verify the full attitude + omega transformation."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "convertStPlatformToBody"
    sim.AddModelToTask(task_name, module)

    # 45-degree rotation about z for the mounting orientation
    mount_angle = np.pi/4.0
    ep_mount = np.array([np.cos(mount_angle/2), 0, 0, np.sin(mount_angle/2)])
    sigma_CB = rbk.EP2MRP(ep_mount)
    module.sigma_CB = sigma_CB

    # Star tracker reports 60-degree rotation about x, with some angular velocity
    st_angle = np.pi/3.0
    ep_CN = np.array([np.cos(st_angle/2), np.sin(st_angle/2), 0, 0])
    omega_CN_C = np.array([-0.015, 0.008, 0.022])

    sensor_data = messaging.AscPlatformAttitudeTmMsgF32Payload()
    sensor_data.solutionTimestamp = 200.0
    sensor_data.platformAttitude = [ep_CN[1], ep_CN[2], ep_CN[3], ep_CN[0]]
    sensor_data.platformAngularVelocity = pack_angular_rate_quaternion(omega_CN_C)
    sensor_in_msg = messaging.AscPlatformAttitudeTmMsgF32().write(sensor_data)

    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.3))
    sim.ExecuteSimulation()

    # Compute truth: sigma_BN = addMRP(sigma_CN, sigma_BC)
    sigma_CN = rbk.EP2MRP(ep_CN)
    dcm_BC = rbk.MRP2C(sigma_CB).T
    sigma_BC = rbk.C2MRP(dcm_BC)
    sigma_BN_truth = rbk.addMRP(sigma_CN, sigma_BC)
    omega_BN_B_truth = dcm_BC @ omega_CN_C

    sigma_BN = att_log.MRP_BdyInrtl[0]
    omega_BN_B = att_log.omega_BN_B[0]

    accuracy = 1e-6
    np.testing.assert_allclose(sigma_BN, sigma_BN_truth, atol=accuracy,
                               err_msg="MRP mismatch with rotated mounting")
    np.testing.assert_allclose(omega_BN_B, omega_BN_B_truth, atol=accuracy,
                               err_msg="omega mismatch with rotated mounting")


def test_never_written_input():
    """With a connected but never-written input, the output is sigma_BC with zero rate and a zero time tag."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(1.0)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "convertStPlatformToBody"
    sim.AddModelToTask(task_name, module)

    # 90-degree mounting rotation about z
    sigma_CB = np.array([0.0, 0.0, np.tan(np.pi/8.0)])
    module.sigma_CB = sigma_CB

    # Connected, but never written
    sensor_in_msg = messaging.AscPlatformAttitudeTmMsgF32()
    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    accuracy = 1e-6
    np.testing.assert_allclose(att_log.MRP_BdyInrtl[-1], -sigma_CB, atol=accuracy,
                               err_msg="sigma_BN must equal sigma_BC when the input was never written")
    np.testing.assert_allclose(att_log.omega_BN_B[-1], np.zeros(3), atol=accuracy,
                               err_msg="omega_BN_B must be zero when the input was never written")
    np.testing.assert_allclose(att_log.timeTag[-1], 0.0, atol=accuracy,
                               err_msg="timeTag must be zero when the input was never written")


def test_reconfigure():
    """reconfigure() applies a new mounting from the next update."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "convertStPlatformToBody"
    sim.AddModelToTask(task_name, module)

    # Identity attitude, rate about the case x-axis, identity mounting
    omega_CN_C = np.array([0.1, 0.0, 0.0])
    sensor_data = messaging.AscPlatformAttitudeTmMsgF32Payload()
    sensor_data.solutionTimestamp = 100.0
    sensor_data.platformAttitude = [0.0, 0.0, 0.0, 1.0]
    sensor_data.platformAngularVelocity = pack_angular_rate_quaternion(omega_CN_C)
    sensor_in_msg = messaging.AscPlatformAttitudeTmMsgF32().write(sensor_data)
    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.5))
    sim.ExecuteSimulation()

    accuracy = 1e-6
    np.testing.assert_allclose(att_log.omega_BN_B[-1], omega_CN_C, atol=accuracy,
                               err_msg="omega mismatch with identity mounting")

    # 90-degree mounting rotation about z, applied through reconfigure()
    sigma_CB = np.array([0.0, 0.0, np.tan(np.pi/8.0)])
    module.sigma_CB = sigma_CB
    module.reconfigure()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    omega_BN_B_truth = rbk.MRP2C(sigma_CB).T @ omega_CN_C
    np.testing.assert_allclose(att_log.MRP_BdyInrtl[-1], -sigma_CB, atol=accuracy,
                               err_msg="sigma_BN must be the inverse of the new mounting")
    np.testing.assert_allclose(att_log.omega_BN_B[-1], omega_BN_B_truth, atol=accuracy,
                               err_msg="omega_BN_B must be rotated by the new mounting")


def test_unlinked_input_raises():
    """reset() raises when stSensorInMsg is not connected."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(1.0)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "convertStPlatformToBody"
    sim.AddModelToTask(task_name, module)

    with pytest.raises(RuntimeError):
        sim.InitializeSimulation()


def test_sigma_pass_through():
    """Verify the mounting MRP setter/getter round-trips correctly through the wrapper."""
    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()

    # Set a 30-degree rotation about z
    angle = np.pi/6.0
    ep = np.array([np.cos(angle/2), 0, 0, np.sin(angle/2)])
    sigma_CB = rbk.EP2MRP(ep)
    module.sigma_CB = sigma_CB

    sigma_out = np.ravel(module.sigma_CB)
    np.testing.assert_allclose(sigma_out, sigma_CB, atol=1e-6,
                               err_msg="MRP round-trip mismatch")


if __name__ == "__main__":
    test_identity_mounting()
    test_rotated_mounting()
    test_never_written_input()
    test_reconfigure()
    test_unlinked_input_raises()
    test_sigma_pass_through()
