import numpy as np
from xmera.utilities import SimulationBaseClass
from xmera.fp32 import attTrackingErrorF32
from xmera.utilities import macros
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging

def test_att_tracking_error():
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Create instance of attTrackingError
    attitude_tracking_error = attTrackingErrorF32.AttTrackingError()
    attitude_tracking_error.modelTag = "attTrackingError"
    unit_test_sim.AddModelToTask(unit_task_name, attitude_tracking_error)

    # Create navigation message
    nav_state_out_data = messaging.NavAttMsgF32Payload()
    sigma_BN = [0.25, -0.45, 0.75]
    nav_state_out_data.sigma_BN = sigma_BN
    omega_BN_B = [-0.015, -0.012, 0.005]
    nav_state_out_data.omega_BN_B = omega_BN_B
    nav_state_in_msg = messaging.NavAttMsgF32().write(nav_state_out_data)

    # Create reference frame message
    ref_state_out_data = messaging.AttRefMsgF32Payload()
    sigma_RN = [0.35, -0.25, 0.15]
    ref_state_out_data.sigma_RN = sigma_RN
    omega_RN_N = [0.018, -0.032, 0.015]
    ref_state_out_data.omega_RN_N = omega_RN_N
    domega_RN_N = [0.048, -0.022, 0.025]
    ref_state_out_data.domega_RN_N = domega_RN_N
    ref_in_msg = messaging.AttRefMsgF32().write(ref_state_out_data)

    # Set up data logging
    att_guid_out_msg_log = attitude_tracking_error.attGuidOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, att_guid_out_msg_log)

    # Connect module input messages
    attitude_tracking_error.attNavInMsg.subscribeTo(nav_state_in_msg)
    attitude_tracking_error.attRefInMsg.subscribeTo(ref_in_msg)

    # Run the simulation
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.3))
    unit_test_sim.ExecuteSimulation()

    # Extract module output data from the message logger
    sigma_BR = att_guid_out_msg_log.sigma_BR[0]
    omega_BR_B = att_guid_out_msg_log.omega_BR_B[0]
    omega_RN_B = att_guid_out_msg_log.omega_RN_B[0]
    domega_RN_B = att_guid_out_msg_log.domega_RN_B[0]

    # Compute truth values for test check
    dcm_RN = rbk.MRP2C(np.array(sigma_RN))
    dcm_BN = rbk.MRP2C(np.array(sigma_BN))
    dcm_BR = np.dot(dcm_BN, dcm_RN.T)
    sigma_BR_truth = rbk.C2MRP(dcm_BR)
    omega_BR_B_truth = np.array(omega_BN_B) - np.dot(dcm_BN, np.array(omega_RN_N))
    omega_RN_B_truth = np.dot(dcm_BN, np.array(omega_RN_N))
    domega_RN_B_truth = np.dot(dcm_BN, np.array(domega_RN_N))

    # Verify module outputs match computed truth values
    tolerance = 1e-5
    np.testing.assert_allclose(sigma_BR_truth, sigma_BR, atol=tolerance, verbose=True)
    np.testing.assert_allclose(omega_BR_B_truth, omega_BR_B, atol=tolerance, verbose=True)
    np.testing.assert_allclose(omega_RN_B_truth, omega_RN_B, atol=tolerance, verbose=True)
    np.testing.assert_allclose(domega_RN_B_truth, domega_RN_B, atol=tolerance, verbose=True)

if __name__ == "__main__":
    test_att_tracking_error()
