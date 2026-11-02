import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import stPlatformAttitudeF32
from xmera.utilities import macros
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging

def to_scalar_last(ep):
    """Reorder a scalar-first quaternion [q0, q1, q2, q3] to the scalar-last form [q1, q2, q3, q0] the sensor sends."""
    return [ep[1], ep[2], ep[3], ep[0]]


def pack_angular_rate_quaternion(omega_PN_P):
    """Pack a rate into the star tracker rate quaternion: omega as a PRV, converted to a scalar-last quaternion."""
    return to_scalar_last(rbk.PRV2EP(np.asarray(omega_PN_P, dtype=float)))


def test_identity_mounting():
    """With identity mounting the output MRP and omega should match the platform-frame values."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = stPlatformAttitudeF32.StPlatformAttitude()
    module.modelTag = "stPlatformAttitude"
    sim.AddModelToTask(task_name, module)

    # Zero mounting MRP (default)

    # Build a star tracker sensor input: 30-deg rotation about z, small omega
    angle = np.pi/6.0
    ep_PN = rbk.PRV2EP(angle * np.array([0.0, 0.0, 1.0]))
    omega_PN_P = np.array([0.01, -0.02, 0.03])

    sensor_data = messaging.StPlatformAttitudeMsgF32Payload()
    sensor_data.solutionTimestamp = 100.0
    sensor_data.platformAttitude = to_scalar_last(ep_PN)
    sensor_data.platformAngularVelocity = pack_angular_rate_quaternion(omega_PN_P)
    sensor_in_msg = messaging.StPlatformAttitudeMsgF32().write(sensor_data)

    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.3))
    sim.ExecuteSimulation()

    # With identity mounting, sigma_BN == sigma_PN and omega_BN_B == omega_PN_P
    sigma_PN = rbk.EP2MRP(ep_PN)
    sigma_BN = att_log.MRP_BdyInrtl[0]
    omega_BN_B = att_log.omega_BN_B[0]

    accuracy = 1e-6
    np.testing.assert_allclose(att_log.timeTag[0], 100.0, atol=accuracy,
                               err_msg="timeTag must be passed through in seconds")
    np.testing.assert_allclose(sigma_BN, sigma_PN, atol=accuracy,
                               err_msg="MRP mismatch with identity mounting")
    np.testing.assert_allclose(omega_BN_B, omega_PN_P, atol=accuracy,
                               err_msg="omega mismatch with identity mounting")


def test_rotated_mounting():
    """With a non-trivial mounting orientation, verify the full attitude + omega transformation."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = stPlatformAttitudeF32.StPlatformAttitude()
    module.modelTag = "stPlatformAttitude"
    sim.AddModelToTask(task_name, module)

    # 45-degree rotation about z for the mounting orientation
    mount_angle = np.pi/4.0
    sigma_PB = rbk.PRV2MRP(mount_angle * np.array([0.0, 0.0, 1.0]))
    module.sigma_PB = sigma_PB

    # Star tracker reports 60-degree rotation about x, with some angular velocity
    st_angle = np.pi/3.0
    ep_PN = rbk.PRV2EP(st_angle * np.array([1.0, 0.0, 0.0]))
    omega_PN_P = np.array([-0.015, 0.008, 0.022])

    sensor_data = messaging.StPlatformAttitudeMsgF32Payload()
    sensor_data.solutionTimestamp = 200.0
    sensor_data.platformAttitude = to_scalar_last(ep_PN)
    sensor_data.platformAngularVelocity = pack_angular_rate_quaternion(omega_PN_P)
    sensor_in_msg = messaging.StPlatformAttitudeMsgF32().write(sensor_data)

    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.3))
    sim.ExecuteSimulation()

    # Compute truth: sigma_BN = addMRP(sigma_PN, sigma_BP)
    sigma_PN = rbk.EP2MRP(ep_PN)
    dcm_BP = rbk.MRP2C(sigma_PB).T
    sigma_BP = rbk.C2MRP(dcm_BP)
    sigma_BN_truth = rbk.addMRP(sigma_PN, sigma_BP)
    omega_BN_B_truth = dcm_BP @ omega_PN_P

    sigma_BN = att_log.MRP_BdyInrtl[0]
    omega_BN_B = att_log.omega_BN_B[0]

    accuracy = 1e-6
    np.testing.assert_allclose(sigma_BN, sigma_BN_truth, atol=accuracy,
                               err_msg="MRP mismatch with rotated mounting")
    np.testing.assert_allclose(omega_BN_B, omega_BN_B_truth, atol=accuracy,
                               err_msg="omega mismatch with rotated mounting")


def test_never_written_input():
    """With a connected but never-written input, the output is sigma_BP with zero rate and a zero time tag."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(1.0)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = stPlatformAttitudeF32.StPlatformAttitude()
    module.modelTag = "stPlatformAttitude"
    sim.AddModelToTask(task_name, module)

    # 90-degree mounting rotation about z
    sigma_PB = rbk.PRV2MRP(np.pi/2.0 * np.array([0.0, 0.0, 1.0]))
    module.sigma_PB = sigma_PB

    # Connected, but never written
    sensor_in_msg = messaging.StPlatformAttitudeMsgF32()
    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    accuracy = 1e-6
    np.testing.assert_allclose(att_log.MRP_BdyInrtl[-1], -sigma_PB, atol=accuracy,
                               err_msg="sigma_BN must equal sigma_BP when the input was never written")
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

    module = stPlatformAttitudeF32.StPlatformAttitude()
    module.modelTag = "stPlatformAttitude"
    sim.AddModelToTask(task_name, module)

    # Identity attitude, rate about the platform x-axis, identity mounting
    omega_PN_P = np.array([0.1, 0.0, 0.0])
    sensor_data = messaging.StPlatformAttitudeMsgF32Payload()
    sensor_data.solutionTimestamp = 100.0
    sensor_data.platformAttitude = [0.0, 0.0, 0.0, 1.0]
    sensor_data.platformAngularVelocity = pack_angular_rate_quaternion(omega_PN_P)
    sensor_in_msg = messaging.StPlatformAttitudeMsgF32().write(sensor_data)
    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.5))
    sim.ExecuteSimulation()

    accuracy = 1e-6
    np.testing.assert_allclose(att_log.omega_BN_B[-1], omega_PN_P, atol=accuracy,
                               err_msg="omega mismatch with identity mounting")

    # 90-degree mounting rotation about z, applied through reconfigure()
    sigma_PB = rbk.PRV2MRP(np.pi/2.0 * np.array([0.0, 0.0, 1.0]))
    module.sigma_PB = sigma_PB
    module.reconfigure()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    omega_BN_B_truth = rbk.MRP2C(sigma_PB).T @ omega_PN_P
    np.testing.assert_allclose(att_log.MRP_BdyInrtl[-1], -sigma_PB, atol=accuracy,
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

    module = stPlatformAttitudeF32.StPlatformAttitude()
    module.modelTag = "stPlatformAttitude"
    sim.AddModelToTask(task_name, module)

    with pytest.raises(RuntimeError):
        sim.InitializeSimulation()


def test_sigma_pass_through():
    """Verify the mounting MRP setter/getter round-trips correctly through the wrapper."""
    module = stPlatformAttitudeF32.StPlatformAttitude()

    # Set a 30-degree rotation about z
    angle = np.pi/6.0
    sigma_PB = rbk.PRV2MRP(angle * np.array([0.0, 0.0, 1.0]))
    module.sigma_PB = sigma_PB

    sigma_out = np.ravel(module.sigma_PB)
    np.testing.assert_allclose(sigma_out, sigma_PB, atol=1e-6,
                               err_msg="MRP round-trip mismatch")


if __name__ == "__main__":
    test_identity_mounting()
    test_rotated_mounting()
    test_never_written_input()
    test_reconfigure()
    test_unlinked_input_raises()
    test_sigma_pass_through()
