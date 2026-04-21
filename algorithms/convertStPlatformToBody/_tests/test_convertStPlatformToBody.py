# MIT License
#
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import convertStPlatformToBodyF32
from xmera.utilities import macros
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging


def test_identity_dcm():
    """With identity mounting DCM the output MRP and omega should match the case-frame values."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "convertStPlatformToBody"
    sim.AddModelToTask(task_name, module)

    # Identity DCM (default)

    # Build a star tracker sensor input: 30-deg rotation about z, small omega
    angle = np.pi/6.0
    ep_CN = np.array([np.cos(angle/2), 0, 0, np.sin(angle/2)])
    omega_CN_C = np.array([0.01, -0.02, 0.03])

    sensor_data = messaging.STSensorMsgPayload()
    sensor_data.timeTag = 100.0
    sensor_data.qInrtl2Case = ep_CN.tolist()
    sensor_data.omega_CN_C = omega_CN_C.tolist()
    sensor_in_msg = messaging.STSensorMsg().write(sensor_data)

    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.3))
    sim.ExecuteSimulation()

    # With identity DCM, sigma_BN == sigma_CN and omega_BN_B == omega_CN_C
    sigma_CN = rbk.EP2MRP(ep_CN)
    sigma_BN = att_log.MRP_BdyInrtl[0]
    omega_BN_B = att_log.omega_BN_B[0]

    accuracy = 1e-6
    np.testing.assert_allclose(sigma_BN, sigma_CN, atol=accuracy,
                               err_msg="MRP mismatch with identity DCM")
    np.testing.assert_allclose(omega_BN_B, omega_CN_C, atol=accuracy,
                               err_msg="omega mismatch with identity DCM")


def test_rotated_dcm():
    """With a non-trivial mounting DCM, verify the full attitude + omega transformation."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "convertStPlatformToBody"
    sim.AddModelToTask(task_name, module)

    # 45-degree rotation about z for mounting DCM
    mount_angle = np.pi/4.0
    ep_mount = np.array([np.cos(mount_angle/2), 0, 0, np.sin(mount_angle/2)])
    dcm_CB = rbk.EP2C(ep_mount)
    module.setDcmCB(dcm_CB.tolist())

    # Star tracker reports 60-degree rotation about x, with some angular velocity
    st_angle = np.pi/3.0
    ep_CN = np.array([np.cos(st_angle/2), np.sin(st_angle/2), 0, 0])
    omega_CN_C = np.array([-0.015, 0.008, 0.022])

    sensor_data = messaging.STSensorMsgPayload()
    sensor_data.timeTag = 200.0
    sensor_data.qInrtl2Case = ep_CN.tolist()
    sensor_data.omega_CN_C = omega_CN_C.tolist()
    sensor_in_msg = messaging.STSensorMsg().write(sensor_data)

    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.3))
    sim.ExecuteSimulation()

    # Compute truth: sigma_BN = addMRP(sigma_CN, sigma_BC)
    sigma_CN = rbk.EP2MRP(ep_CN)
    dcm_BC = dcm_CB.T
    sigma_BC = rbk.C2MRP(dcm_BC)
    sigma_BN_truth = rbk.addMRP(sigma_CN, sigma_BC)
    omega_BN_B_truth = dcm_BC @ omega_CN_C

    sigma_BN = att_log.MRP_BdyInrtl[0]
    omega_BN_B = att_log.omega_BN_B[0]

    accuracy = 1e-6
    np.testing.assert_allclose(sigma_BN, sigma_BN_truth, atol=accuracy,
                               err_msg="MRP mismatch with rotated DCM")
    np.testing.assert_allclose(omega_BN_B, omega_BN_B_truth, atol=accuracy,
                               err_msg="omega mismatch with rotated DCM")


def test_zero_input():
    """With default (zeroed) input, module should not crash and output should be zero."""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(1.0)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, process_rate))

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "convertStPlatformToBody"
    sim.AddModelToTask(task_name, module)

    sensor_data = messaging.STSensorMsgPayload()
    sensor_in_msg = messaging.STSensorMsg().write(sensor_data)
    module.stSensorInMsg.subscribeTo(sensor_in_msg)

    att_log = module.stAttOutMsg.recorder()
    sim.AddModelToTask(task_name, att_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Should produce finite output without crashing
    sigma_BN = att_log.MRP_BdyInrtl[0]
    omega_BN_B = att_log.omega_BN_B[0]
    assert np.all(np.isfinite(sigma_BN)), "MRP output is not finite"
    assert np.all(np.isfinite(omega_BN_B)), "omega output is not finite"


def test_dcm_pass_through():
    """Verify the DCM setter/getter round-trips correctly through the wrapper."""
    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()

    # Set a 30-degree rotation about z
    angle = np.pi/6.0
    ep = np.array([np.cos(angle/2), 0, 0, np.sin(angle/2)])
    dcm_CB = rbk.EP2C(ep)
    module.setDcmCB(dcm_CB.tolist())

    dcm_out = np.array(module.getDcmCB())
    np.testing.assert_allclose(dcm_out, dcm_CB, atol=1e-6,
                               err_msg="DCM round-trip mismatch")


if __name__ == "__main__":
    test_identity_dcm()
    test_rotated_dcm()
    test_zero_input()
    test_dcm_pass_through()
