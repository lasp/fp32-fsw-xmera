import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import sunSearchPointF32
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

def test_sun_search_point(show_plots, case):
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

    # Create the sunSearchPoint module
    sun_search_point = sunSearchPointF32.SunSearchPoint()
    sun_search_point.modelTag = "sunSearchPoint"
    unit_test_sim.AddModelToTask(unit_task_name, sun_search_point)

    # Initialize sunSearchPoint module configuration data
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

    sun_search_point.sHatBdyCmd = sHat_cmd_B
    sun_search_point.omega_RN_B = omega_RN_B_Search
    sun_search_point.sunAxisSpinRate = sunAxisSpinRate
    sun_search_point.observationThreshold = 4

    # Configure a (no-op) search sequence; the run advances past it to force the POINT transition.
    for i in range(4):
        rotation = sunSearchPointF32.RotationProperties()
        rotation.rotationDuration = 1.0
        rotation.rotationRate = 0.0
        rotation.rotationAxis = sunSearchPointF32.RotationAxis_b1Hat_B
        sun_search_point.setRotation(i, rotation)

    # Create sunSearchPoint sun direction input messages
    input_sun_vec_data = messaging.NavAttMsgF32Payload()
    input_sun_vec_data.vehSunPntBdy = sun_vec_B
    sun_in_msg = messaging.NavAttMsgF32().write(input_sun_vec_data)

    # Create sunSearchPoint body rate input message
    input_rate_data = messaging.NavAttMsgF32Payload()
    omega_BN_B = np.array([0.01, 0.50, -0.2])
    input_rate_data.omega_BN_B = omega_BN_B
    rate_in_msg = messaging.NavAttMsgF32().write(input_rate_data)

    # Create sunSearchPoint filter residuals input message (CSS observation count)
    input_residuals_data = messaging.FilterResidualsMsgF32Payload()
    input_residuals_data.sizeOfObservations = 0
    residuals_in_msg = messaging.FilterResidualsMsgF32().write(input_residuals_data)

    # Set up data logging
    att_guid_out_msg_data_log = sun_search_point.attGuidanceOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, att_guid_out_msg_data_log)
    fault_out_msg_data_log = sun_search_point.sunSearchPointFaultOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, fault_out_msg_data_log)

    # Connect messages
    sun_search_point.sunDirectionInMsg.subscribeTo(sun_in_msg)
    sun_search_point.rateInMsg.subscribeTo(rate_in_msg)
    sun_search_point.filterResidualsInMsg.subscribeTo(residuals_in_msg)

    # Run the simulation past the 4 s search sequence so the final sample is in the POINT phase
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(5.))
    sun_search_point.reset(0)
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

    # The run never reaches the observation threshold, so the search fails and forces POINT,
    # latching the search-failure fault.
    assert fault_out_msg_data_log.faultDetected[-1] == True

    # Parameter round-trips (scalar attributes + SWIG rotation-config binding)
    np.testing.assert_allclose(sun_search_point.sunAxisSpinRate, sunAxisSpinRate, rtol=tolerance, atol=tolerance)
    np.testing.assert_allclose(np.array(sun_search_point.omega_RN_B).flatten(), omega_RN_B_Search, rtol=tolerance, atol=tolerance)
    np.testing.assert_allclose(np.array(sun_search_point.sHatBdyCmd).flatten(), sHat_cmd_B, rtol=tolerance, atol=tolerance)
    assert sun_search_point.observationThreshold == 4
    read_rotation = sun_search_point.getRotation(1)
    np.testing.assert_allclose(read_rotation.rotationDuration, 1.0, rtol=tolerance, atol=tolerance)
    np.testing.assert_allclose(read_rotation.rotationRate, 0.0, rtol=tolerance, atol=tolerance)
    assert read_rotation.rotationAxis == sunSearchPointF32.RotationAxis_b1Hat_B

def test_search_then_point(show_plots):
    """Adapter wiring: stepping callTime with the filter-residuals observation count drives the
    output from the search sequence into sun pointing. Confirms the adapter threads callTime through
    and reads sizeOfObservations from filterResidualsInMsg (it would fail if either were dropped)."""
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = mc.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    sun_search_point = sunSearchPointF32.SunSearchPoint()
    sun_search_point.modelTag = "sunSearchPoint"
    unit_test_sim.AddModelToTask(unit_task_name, sun_search_point)

    sHat_cmd_B = np.array([0.0, 0.0, 1.0])
    sun_vec_B = np.array([1.0, 1.0, 0.0])
    sun_search_point.sHatBdyCmd = sHat_cmd_B
    sun_search_point.sunAxisSpinRate = 0.0
    sun_search_point.observationThreshold = 4

    # Search sequence: rotation 1 spins at 0.2 rad/s about b1 for 1 s, remaining rotations no-op.
    rates = [0.2, 0.0, 0.0, 0.0]
    for i in range(4):
        rotation = sunSearchPointF32.RotationProperties()
        rotation.rotationDuration = 1.0
        rotation.rotationRate = rates[i]
        rotation.rotationAxis = sunSearchPointF32.RotationAxis_b1Hat_B
        sun_search_point.setRotation(i, rotation)

    input_sun_vec_data = messaging.NavAttMsgF32Payload()
    input_sun_vec_data.vehSunPntBdy = sun_vec_B
    sun_in_msg = messaging.NavAttMsgF32().write(input_sun_vec_data)

    input_rate_data = messaging.NavAttMsgF32Payload()
    input_rate_data.omega_BN_B = np.array([0.0, 0.0, 0.0])
    rate_in_msg = messaging.NavAttMsgF32().write(input_rate_data)

    # Observation count above the threshold: transition is allowed once rotation 1 completes.
    input_residuals_data = messaging.FilterResidualsMsgF32Payload()
    input_residuals_data.sizeOfObservations = 10
    residuals_in_msg = messaging.FilterResidualsMsgF32().write(input_residuals_data)

    att_guid_out_msg_data_log = sun_search_point.attGuidanceOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, att_guid_out_msg_data_log)
    fault_out_msg_data_log = sun_search_point.sunSearchPointFaultOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, fault_out_msg_data_log)

    sun_search_point.sunDirectionInMsg.subscribeTo(sun_in_msg)
    sun_search_point.rateInMsg.subscribeTo(rate_in_msg)
    sun_search_point.filterResidualsInMsg.subscribeTo(residuals_in_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(3.))
    sun_search_point.reset(0)
    unit_test_sim.ExecuteSimulation()

    times = att_guid_out_msg_data_log.times() * mc.NANO2SEC
    sigma_BR = att_guid_out_msg_data_log.sigma_BR
    omega_RN_B = att_guid_out_msg_data_log.omega_RN_B

    # Pointing truth (sun visible, spin 0): sigma from the eigen-axis, omega_RN_B = 0.
    eHat = np.cross(sun_vec_B, sHat_cmd_B)
    eHat = eHat / np.linalg.norm(eHat)
    phi = np.arccos(np.dot(sun_vec_B / np.linalg.norm(sun_vec_B), sHat_cmd_B))
    sigma_point_truth = eHat * np.tan(phi / 4.0)

    tolerance = 1e-6
    saw_search = False
    saw_point = False
    for i, t in enumerate(times):
        if t < 0.99:  # during rotation 1: search rate about b1, zero attitude error
            np.testing.assert_allclose(sigma_BR[i], [0.0, 0.0, 0.0], rtol=tolerance, atol=tolerance)
            np.testing.assert_allclose(omega_RN_B[i], [0.2, 0.0, 0.0], rtol=tolerance, atol=tolerance)
            saw_search = True
        elif t > 1.01:  # after rotation 1 with observations above threshold: pointing
            np.testing.assert_allclose(sigma_BR[i], sigma_point_truth, rtol=tolerance, atol=tolerance)
            np.testing.assert_allclose(omega_RN_B[i], [0.0, 0.0, 0.0], rtol=tolerance, atol=tolerance)
            saw_point = True

    assert saw_search, "expected search-phase samples before the transition"
    assert saw_point, "expected pointing-phase samples after the transition"

    # The sun was acquired (observations above threshold), so the search succeeded: no fault.
    assert fault_out_msg_data_log.faultDetected[-1] == False


def test_reconfigure_applies_params(show_plots):
    """Adapter reConfigure(): pushes an updated module parameter onto the live algorithm via
    setConfig() without re-initializing the search state machine. After the module is already
    pointing, change the spin rate and reConfigure(); the new spin rate takes effect while the
    module stays in the pointing phase (a reInitialize would have restarted the search)."""
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = mc.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    sun_search_point = sunSearchPointF32.SunSearchPoint()
    sun_search_point.modelTag = "sunSearchPoint"
    unit_test_sim.AddModelToTask(unit_task_name, sun_search_point)

    sHat_cmd_B = np.array([0.0, 0.0, 1.0])
    sun_vec_B = np.array([1.0, 1.0, 0.0])
    sun_search_point.sHatBdyCmd = sHat_cmd_B
    sun_search_point.sunAxisSpinRate = 0.0
    sun_search_point.observationThreshold = 4

    # No-op search sequence; the run advances past it to force the POINT transition.
    for i in range(4):
        rotation = sunSearchPointF32.RotationProperties()
        rotation.rotationDuration = 1.0
        rotation.rotationRate = 0.0
        rotation.rotationAxis = sunSearchPointF32.RotationAxis_b1Hat_B
        sun_search_point.setRotation(i, rotation)

    input_sun_vec_data = messaging.NavAttMsgF32Payload()
    input_sun_vec_data.vehSunPntBdy = sun_vec_B
    sun_in_msg = messaging.NavAttMsgF32().write(input_sun_vec_data)

    input_rate_data = messaging.NavAttMsgF32Payload()
    input_rate_data.omega_BN_B = np.array([0.0, 0.0, 0.0])
    rate_in_msg = messaging.NavAttMsgF32().write(input_rate_data)

    input_residuals_data = messaging.FilterResidualsMsgF32Payload()
    input_residuals_data.sizeOfObservations = 0
    residuals_in_msg = messaging.FilterResidualsMsgF32().write(input_residuals_data)

    att_guid_out_msg_data_log = sun_search_point.attGuidanceOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, att_guid_out_msg_data_log)

    sun_search_point.sunDirectionInMsg.subscribeTo(sun_in_msg)
    sun_search_point.rateInMsg.subscribeTo(rate_in_msg)
    sun_search_point.filterResidualsInMsg.subscribeTo(residuals_in_msg)

    # Run past the 4 s search so the module is in terminal POINT (spin rate 0 -> omega_RN_B = 0).
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(5.0))
    sun_search_point.reset(0)
    unit_test_sim.ExecuteSimulation()
    np.testing.assert_allclose(att_guid_out_msg_data_log.omega_RN_B[-1], [0.0, 0.0, 0.0], rtol=1e-6, atol=1e-6)

    # Change the spin rate and reConfigure (no reset): applies to the live algorithm without
    # re-initializing, so the module stays in POINT and now spins about the sun heading.
    spin_rate = 1.5 * mc.D2R
    sun_search_point.sunAxisSpinRate = spin_rate
    sun_search_point.reConfigure()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(6.0))
    unit_test_sim.ExecuteSimulation()

    expected_omega = sun_vec_B / np.linalg.norm(sun_vec_B) * spin_rate
    np.testing.assert_allclose(att_guid_out_msg_data_log.omega_RN_B[-1], expected_omega, rtol=1e-6, atol=1e-6)


def test_reinitialize_rearms_search(show_plots):
    """Adapter reInitialize(): pass-through to the algorithm's reInitialize(), which re-arms the
    search state machine without rebuilding the config. After the module has acquired the sun and
    entered the terminal POINT phase, reInitialize() restarts the scripted search sequence from the
    first rotation (a reConfigure would have left it pointing)."""
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = mc.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    sun_search_point = sunSearchPointF32.SunSearchPoint()
    sun_search_point.modelTag = "sunSearchPoint"
    unit_test_sim.AddModelToTask(unit_task_name, sun_search_point)

    sHat_cmd_B = np.array([0.0, 0.0, 1.0])
    sun_vec_B = np.array([1.0, 1.0, 0.0])
    sun_search_point.sHatBdyCmd = sHat_cmd_B
    sun_search_point.sunAxisSpinRate = 0.0
    sun_search_point.observationThreshold = 4

    # Search sequence with a non-zero first-rotation rate so the SEARCH output (omega_RN_B about
    # b1Hat_B) is distinguishable from the spin-0 POINT output (omega_RN_B == 0).
    search_rate = 0.5
    for i in range(4):
        rotation = sunSearchPointF32.RotationProperties()
        rotation.rotationDuration = 10.0
        rotation.rotationRate = search_rate
        rotation.rotationAxis = sunSearchPointF32.RotationAxis_b1Hat_B
        sun_search_point.setRotation(i, rotation)

    input_sun_vec_data = messaging.NavAttMsgF32Payload()
    input_sun_vec_data.vehSunPntBdy = sun_vec_B
    sun_in_msg = messaging.NavAttMsgF32().write(input_sun_vec_data)

    input_rate_data = messaging.NavAttMsgF32Payload()
    input_rate_data.omega_BN_B = np.array([0.0, 0.0, 0.0])
    rate_in_msg = messaging.NavAttMsgF32().write(input_rate_data)

    input_residuals_data = messaging.FilterResidualsMsgF32Payload()
    input_residuals_data.sizeOfObservations = 10
    residuals_in_msg = messaging.FilterResidualsMsgF32().write(input_residuals_data)

    att_guid_out_msg_data_log = sun_search_point.attGuidanceOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, att_guid_out_msg_data_log)

    sun_search_point.sunDirectionInMsg.subscribeTo(sun_in_msg)
    sun_search_point.rateInMsg.subscribeTo(rate_in_msg)
    sun_search_point.filterResidualsInMsg.subscribeTo(residuals_in_msg)

    # Run past the first rotation (10 s) with observations above threshold: the module acquires the
    # sun and enters the terminal POINT phase (spin rate 0 -> omega_RN_B == 0).
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(12.0))
    sun_search_point.reset(0)
    unit_test_sim.ExecuteSimulation()
    np.testing.assert_allclose(att_guid_out_msg_data_log.omega_RN_B[-1], [0.0, 0.0, 0.0], rtol=1e-6, atol=1e-6)

    # reInitialize re-arms the search machine: the next update restarts the sequence at the first
    # rotation, so omega_RN_B is the slot-0 search rate about b1Hat_B (not the POINT output).
    sun_search_point.reInitialize()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(13.0))
    unit_test_sim.ExecuteSimulation()
    np.testing.assert_allclose(
        att_guid_out_msg_data_log.omega_RN_B[-1], [search_rate, 0.0, 0.0], rtol=1e-6, atol=1e-6)


if __name__ == "__main__":
    test_sun_search_point(False, 1)
    test_search_then_point(False)
    test_reconfigure_applies_params(False)
    test_reinitialize_rearms_search(False)
