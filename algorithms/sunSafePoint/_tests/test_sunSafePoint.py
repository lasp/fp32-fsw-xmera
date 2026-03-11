import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import sunSafePointF32
from xmera.architecture import messaging
from xmera.utilities import macros as mc

@pytest.mark.parametrize("case", [
     (1)        # sun is visible, vectors are not aligned
    ,(2)        # sun is not visible, vectors are not aligned
    ,(3)        # sun is visible, vectors are aligned
    ,(4)        # sun is visible, vectors are oppositely aligned
    ,(5)        # sun is visible, vectors are oppositely aligned, and command sc is b1
    ,(6)        # sun is not visible, vectors are not aligned, no specified omega_RN_B value
    ,(7)        # sun is visible, vectors not aligned, nominal spin rate specified about sun heading vector
])

def test_sun_safe_point(show_plots, case):

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = mc.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Create the sunSafePoint module
    sun_safe_point = sunSafePointF32.SunSafePoint()
    sun_safe_point.modelTag = "sunSafePoint"
    unit_test_sim.AddModelToTask(unit_task_name, sun_safe_point)

    # Initialize sunSafePoint module configuration data
    sun_safe_point.minUnitMag = 0.1
    sun_safe_point.smallAngle = 0.01 * mc.D2R

    sHat_cmd_B = []
    sun_vec_B = []
    if case == 1:  # Sun visible, vectors not aligned
        sHat_cmd_B = np.array([0.0, 0.0, 1.0])
        sun_vec_B = np.array([1.0, 1.0, 0.0])

    elif case == 2:  # Sun not visible, search rate specified
        sHat_cmd_B = np.array([0.0, 0.0, 1.0])
        sun_vec_B = np.array([0.0, sun_safe_point.minUnitMag / 2, 0.0])

        omega_RN_B_Search = np.array([0.0, 0.0, 0.1])
        sun_safe_point.omega_RN_B = omega_RN_B_Search

    elif case == 3:  # Sun visible, vectors aligned
        sHat_cmd_B = np.array([0.0, 0.0, 1.0])
        sun_vec_B = sHat_cmd_B

    elif case == 4:  # Sun visible, vectors oppositely aligned
        sHat_cmd_B = np.array([0.0, 0.0, 1.0])
        sun_vec_B = -sHat_cmd_B

    elif case == 5:  # Sun visible, vectors oppositely aligned, sHatCmd is along b1
        sHat_cmd_B = np.array([1.0, 0.0, 0.0])
        sun_vec_B = -sHat_cmd_B

    elif case == 6:  # Sun not visible, no search rate specified
        sHat_cmd_B = np.array([0.0, 0.0, 1.0])
        sun_vec_B = np.array([0.0, sun_safe_point.minUnitMag / 2, 0.0])

    else:  # Sun visible, spin rate about sun heading vector specified
        sHat_cmd_B = np.array([0.0, 0.0, 1.0])
        sun_vec_B = np.array([1.0, 1.0, 0.0])

        sun_safe_point.sunAxisSpinRate = 1.5*mc.D2R
        omega_RN_B_Search = sun_vec_B/np.linalg.norm(sun_vec_B) * sun_safe_point.sunAxisSpinRate

    sun_safe_point.sHatBdyCmd = sHat_cmd_B

    # Create sunSafePoint sun direction input messages
    input_sun_vec_data = messaging.NavAttMsgF32Payload()
    input_sun_vec_data.vehSunPntBdy = sun_vec_B
    sun_in_msg = messaging.NavAttMsgF32().write(input_sun_vec_data)

    # Create sunSafePoint IMU input message
    input_imu_data = messaging.NavAttMsgF32Payload()
    omega_BN_B = np.array([0.01, 0.50, -0.2])
    input_imu_data.omega_BN_B = omega_BN_B
    imu_in_msg = messaging.NavAttMsgF32().write(input_imu_data)

    # Set up data logging
    att_guid_out_msg_data_log = sun_safe_point.attGuidanceOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, att_guid_out_msg_data_log)

    # Connect messages
    sun_safe_point.sunDirectionInMsg.subscribeTo(sun_in_msg)
    sun_safe_point.imuInMsg.subscribeTo(imu_in_msg)

    # Run the simulation
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(1.))
    sun_safe_point.reset(0)
    unit_test_sim.ExecuteSimulation()

    # Check sigma_BR
    # Set the filtered output truth states
    if case == 1 or case == 7:
        eHat = np.cross(sun_vec_B, sHat_cmd_B)
        eHat = eHat / np.linalg.norm(eHat)
        phi = np.arccos(np.dot(sun_vec_B / np.linalg.norm(sun_vec_B), sHat_cmd_B))
        sigma_true = eHat * np.tan(phi / 4.0)
        sigma_BR_truth = [
                    sigma_true.tolist(),
                    sigma_true.tolist(),
                    sigma_true.tolist()
                   ]
    if case == 2 or case == 3 or case == 6:
        sigma_BR_truth = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]
    if case == 4:
        eHat = np.cross(sHat_cmd_B, np.array([1, 0, 0]))
        eHat = eHat / np.linalg.norm(eHat)
        phi = np.arccos(np.dot(sun_vec_B / np.linalg.norm(sun_vec_B), sHat_cmd_B))
        sigma_true = eHat * np.tan(phi / 4.0)
        sigma_BR_truth = [
                    sigma_true.tolist(),
                    sigma_true.tolist(),
                    sigma_true.tolist()
               ]
    if case == 5:
        eHat = np.cross(sHat_cmd_B, np.array([0, 1, 0]))
        eHat = eHat / np.linalg.norm(eHat)
        phi = np.arccos(np.dot(sun_vec_B / np.linalg.norm(sun_vec_B), sHat_cmd_B))
        sigma_true = eHat * np.tan(phi / 4.0)
        sigma_BR_truth = [
            sigma_true.tolist(),
            sigma_true.tolist(),
            sigma_true.tolist()
        ]

    # Compare the module results to the truth values
    tolerance = 1e-6
    np.testing.assert_allclose(sigma_BR_truth,
                               att_guid_out_msg_data_log.sigma_BR,
                               rtol=tolerance,
                               atol=tolerance,
                               verbose=True)

    # Check omega_BR_B
    # Set the filtered output truth states
    if case == 1 or case == 3 or case == 4 or case == 5 or case == 6:
        omega_BR_B_truth = [
            omega_BN_B.tolist(),
            omega_BN_B.tolist(),
            omega_BN_B.tolist()
        ]
    if case == 2 or case == 7:
        omega_BR_B_truth = [
            (omega_BN_B - omega_RN_B_Search).tolist(),
            (omega_BN_B - omega_RN_B_Search).tolist(),
            (omega_BN_B - omega_RN_B_Search).tolist()
        ]

    # Compare the module results to the truth values
    np.testing.assert_allclose(omega_BR_B_truth,
                               att_guid_out_msg_data_log.omega_BR_B,
                               rtol=tolerance,
                               atol=tolerance,
                               verbose=True)

    # Check omega_RN_B
    # Set the filtered output truth states
    if case == 1 or case == 3 or case == 4 or case == 5 or case == 6:
        omega_RN_B_truth = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]
    if case == 2 or case == 7:
        omega_RN_B_truth = [
            omega_RN_B_Search,
            omega_RN_B_Search,
            omega_RN_B_Search
        ]

    # Compare the module results to the truth values
    np.testing.assert_allclose(omega_RN_B_truth,
                               att_guid_out_msg_data_log.omega_RN_B,
                               rtol=tolerance,
                               atol=tolerance,
                               verbose=True)

    # Check domega_RN_B
    # Set the filtered output truth states
    domega_RN_B_truth = [
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0]
               ]

    # Compare the module results to the truth values
    np.testing.assert_allclose(domega_RN_B_truth,
                               att_guid_out_msg_data_log.domega_RN_B,
                               rtol=tolerance,
                               atol=tolerance,
                               verbose=True)

if __name__ == "__main__":
    test_sun_safe_point(False, 1)
