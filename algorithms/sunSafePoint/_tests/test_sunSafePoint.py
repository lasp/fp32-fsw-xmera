import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import sunSafePointF32
from xmera.architecture import messaging
from xmera.utilities import macros as mc

def unit_orthogonal(v):
    """Replicates Eigen's unitOrthogonal() for 3D vectors (OrthoMethods.h)."""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    # Eigen uses isMuchSmallerThan with dummy_precision (1e-5 for float)
    eps = 1e-5
    if abs(x) > abs(z) * eps or abs(y) > abs(z) * eps:
        invnm = 1.0 / np.sqrt(x**2 + y**2)
        return np.array([-y * invnm, x * invnm, 0.0])
    else:
        invnm = 1.0 / np.sqrt(y**2 + z**2)
        return np.array([0.0, -z * invnm, y * invnm])


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
    """Exercises the terminal POINT phase. The module always runs the search sequence first, so the
    test runs past the full sequence (forcing the POINT transition) and checks the steady-state
    (final) logged sample against the pointing truth."""
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
    omega_RN_B_Search = np.array([0.0, 0.0, 0.0])
    sunAxisSpinRate = 0.0
    if case == 1:  # Sun visible, vectors not aligned
        sHat_cmd_B = np.array([0.0, 0.0, 1.0])
        sun_vec_B = np.array([1.0, 1.0, 0.0])

    elif case == 2:  # Sun not visible, search rate specified
        sHat_cmd_B = np.array([0.0, 0.0, 1.0])
        sun_vec_B = np.array([0.0, 0.0, 0.0])

        omega_RN_B_Search = np.array([0.0, 0.0, 0.1])

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
        sun_vec_B = np.array([0.0, 0.0, 0.0])

    else:  # Sun visible, spin rate about sun heading vector specified
        sHat_cmd_B = np.array([0.0, 0.0, 1.0])
        sun_vec_B = np.array([1.0, 1.0, 0.0])

        sunAxisSpinRate = 1.5*mc.D2R
        omega_RN_B_Search = sun_vec_B/np.linalg.norm(sun_vec_B) * sunAxisSpinRate

    sun_safe_point.sHatBdyCmd = sHat_cmd_B
    sun_safe_point.omega_RN_B = omega_RN_B_Search
    sun_safe_point.sunAxisSpinRate = sunAxisSpinRate
    sun_safe_point.observationThreshold = 4

    # Configure a (no-op) search sequence; the run advances past it to force the POINT transition.
    for i in range(4):
        rotation = sunSafePointF32.RotationProperties()
        rotation.rotationDuration = 1.0
        rotation.rotationRate = 0.0
        rotation.rotationAxis = sunSafePointF32.RotationAxis_b1Hat_B
        sun_safe_point.setRotation(i, rotation)

    # Create sunSafePoint sun direction input messages
    input_sun_vec_data = messaging.NavAttMsgF32Payload()
    input_sun_vec_data.vehSunPntBdy = sun_vec_B
    sun_in_msg = messaging.NavAttMsgF32().write(input_sun_vec_data)

    # Create sunSafePoint body rate input message
    input_rate_data = messaging.NavAttMsgF32Payload()
    omega_BN_B = np.array([0.01, 0.50, -0.2])
    input_rate_data.omega_BN_B = omega_BN_B
    rate_in_msg = messaging.NavAttMsgF32().write(input_rate_data)

    # Create sunSafePoint filter residuals input message (CSS observation count)
    input_residuals_data = messaging.FilterResidualsMsgF32Payload()
    input_residuals_data.sizeOfObservations = 0
    residuals_in_msg = messaging.FilterResidualsMsgF32().write(input_residuals_data)

    # Set up data logging
    att_guid_out_msg_data_log = sun_safe_point.attGuidanceOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, att_guid_out_msg_data_log)

    # Connect messages
    sun_safe_point.sunDirectionInMsg.subscribeTo(sun_in_msg)
    sun_safe_point.rateInMsg.subscribeTo(rate_in_msg)
    sun_safe_point.filterResidualsInMsg.subscribeTo(residuals_in_msg)

    # Run the simulation past the 4 s search sequence so the final sample is in the POINT phase
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(5.))
    sun_safe_point.reset(0)
    unit_test_sim.ExecuteSimulation()

    # Build the pointing-phase truth for the final (steady-state) sample
    if case == 1 or case == 7:
        eHat = np.cross(sun_vec_B, sHat_cmd_B)
        eHat = eHat / np.linalg.norm(eHat)
        phi = np.arccos(np.dot(sun_vec_B / np.linalg.norm(sun_vec_B), sHat_cmd_B))
        sigma_BR_truth = eHat * np.tan(phi / 4.0)
    if case == 2 or case == 3 or case == 6:
        sigma_BR_truth = np.array([0.0, 0.0, 0.0])
    if case == 4 or case == 5:
        eHat = unit_orthogonal(sHat_cmd_B)
        phi = np.arccos(np.dot(sun_vec_B / np.linalg.norm(sun_vec_B), sHat_cmd_B))
        sigma_BR_truth = eHat * np.tan(phi / 4.0)

    if case == 1 or case == 3 or case == 4 or case == 5 or case == 6:
        omega_RN_B_truth = np.array([0.0, 0.0, 0.0])
        omega_BR_B_truth = omega_BN_B
    if case == 2 or case == 7:
        omega_RN_B_truth = omega_RN_B_Search
        omega_BR_B_truth = omega_BN_B - omega_RN_B_Search

    domega_RN_B_truth = np.array([0.0, 0.0, 0.0])

    # Compare the final (POINT) sample to the truth values
    tolerance = 1e-6
    np.testing.assert_allclose(att_guid_out_msg_data_log.sigma_BR[-1], sigma_BR_truth,
                               rtol=tolerance, atol=tolerance, verbose=True)
    np.testing.assert_allclose(att_guid_out_msg_data_log.omega_BR_B[-1], omega_BR_B_truth,
                               rtol=tolerance, atol=tolerance, verbose=True)
    np.testing.assert_allclose(att_guid_out_msg_data_log.omega_RN_B[-1], omega_RN_B_truth,
                               rtol=tolerance, atol=tolerance, verbose=True)
    np.testing.assert_allclose(att_guid_out_msg_data_log.domega_RN_B[-1], domega_RN_B_truth,
                               rtol=tolerance, atol=tolerance, verbose=True)

    # Parameter round-trips (scalar attributes + SWIG rotation-config binding)
    np.testing.assert_allclose(sun_safe_point.sunAxisSpinRate, sunAxisSpinRate, rtol=tolerance, atol=tolerance)
    np.testing.assert_allclose(np.array(sun_safe_point.omega_RN_B).flatten(), omega_RN_B_Search, rtol=tolerance, atol=tolerance)
    np.testing.assert_allclose(np.array(sun_safe_point.sHatBdyCmd).flatten(), sHat_cmd_B, rtol=tolerance, atol=tolerance)
    assert sun_safe_point.observationThreshold == 4
    read_rotation = sun_safe_point.getRotation(1)
    np.testing.assert_allclose(read_rotation.rotationDuration, 1.0, rtol=tolerance, atol=tolerance)
    np.testing.assert_allclose(read_rotation.rotationRate, 0.0, rtol=tolerance, atol=tolerance)
    assert read_rotation.rotationAxis == sunSafePointF32.RotationAxis_b1Hat_B

if __name__ == "__main__":
    test_sun_safe_point(False, 1)
