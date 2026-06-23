import inspect
import os

import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import mrpRotationF32
from xmera.utilities import macros as mc
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging


def compute_truth(sigma_RR0_init, omega_RR0_R, ref_state_in_data, dt, num_steps_pre_reset, num_steps_post_reset):
    """Each tick advances sigma_RR0 by the configured controlPeriod (dt) and emits the output frame.
    The algorithm integrates BEFORE computing the output, so tick k uses sigma_RR0(k+1) = sigma_RR0(k) + dt * sigmaDot.
    """
    sigma_R0N = np.array(ref_state_in_data.sigma_RN)
    R0N = rbk.MRP2C(sigma_R0N)
    omega_R0N_N = np.array(ref_state_in_data.omega_RN_N)
    domega_R0N_N = np.array(ref_state_in_data.domega_RN_N)
    omega_RR0_R = np.array(omega_RR0_R)

    truth_sigma = []
    truth_omega_RN_N = []
    truth_domega_RN_N = []

    def integrate_and_emit(sigma_RR0):
        # Forward-Euler MRP integration with mrpSwitch (shadow set when |sigma| > 1).
        B = rbk.BmatMRP(sigma_RR0)
        sigma_RR0_new = sigma_RR0 + dt * 0.25 * np.dot(B, omega_RR0_R)
        n_sq = float(np.dot(sigma_RR0_new, sigma_RR0_new))
        if n_sq > 1.0:
            sigma_RR0_new = -sigma_RR0_new / n_sq
        # Output uses the post-integration sigma_RR0.
        RR0 = rbk.MRP2C(sigma_RR0_new)
        RN = np.dot(RR0, R0N)
        sigma_RN = rbk.C2MRP(RN)
        omega_RR0_N = np.dot(RN.T, omega_RR0_R)
        omega_RN_N = omega_RR0_N + omega_R0N_N
        domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
        domega_RN_N = domega_RR0_N + domega_R0N_N
        return sigma_RR0_new, sigma_RN, omega_RN_N, domega_RN_N

    sigma_RR0 = np.array(sigma_RR0_init, dtype=float)
    for _ in range(num_steps_pre_reset):
        sigma_RR0, sigma_RN, omega_RN_N, domega_RN_N = integrate_and_emit(sigma_RR0)
        truth_sigma.append(sigma_RN.tolist())
        truth_omega_RN_N.append(omega_RN_N.tolist())
        truth_domega_RN_N.append(domega_RN_N.tolist())

    # reset() reconstructs the algorithm, re-seeding its runtime sigma_RR0 from the configured initial
    # values, so the post-reset integration restarts from sigma_RR0_init.
    if num_steps_post_reset > 0:
        sigma_RR0 = np.array(sigma_RR0_init, dtype=float)
        for _ in range(num_steps_post_reset):
            sigma_RR0, sigma_RN, omega_RN_N, domega_RN_N = integrate_and_emit(sigma_RR0)
            truth_sigma.append(sigma_RN.tolist())
            truth_omega_RN_N.append(omega_RN_N.tolist())
            truth_domega_RN_N.append(domega_RN_N.tolist())

    return truth_sigma, truth_omega_RN_N, truth_domega_RN_N


@pytest.mark.parametrize("test_reset", [False, True])
def test_mrp_rotation(show_plots, test_reset):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Test times
    update_time = 0.5
    total_test_sim_time = 1.5

    # Create test thread
    test_process_rate = mc.sec2nano(update_time)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = mrpRotationF32.MrpRotation()
    module.modelTag = "mrpRotation"

    unit_test_sim.AddModelToTask(unit_task_name, module)

    sigma_RR0 = np.array([0.3, .5, 0.0])
    module.sigma_RR0 = sigma_RR0
    omega_RR0_R = np.array([0.1, 0.0, 0.0]) * mc.D2R
    module.omega_RR0_R = omega_RR0_R
    module.controlPeriod = update_time

    ref_state_in_data = messaging.AttRefMsgF32Payload()
    sigma_R0N = np.array([0.1, 0.2, 0.3])
    ref_state_in_data.sigma_RN = sigma_R0N
    omega_R0N_N = np.array([0.1, 0.0, 0.0])
    ref_state_in_data.omega_RN_N = omega_R0N_N
    domega_R0N_N = np.array([0.0, 0.0, 0.0])
    ref_state_in_data.domega_RN_N = domega_R0N_N
    att_ref_msg = messaging.AttRefMsgF32().write(ref_state_in_data)
    module.attRefInMsg.subscribeTo(att_ref_msg)

    data_log = module.attRefOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(total_test_sim_time))
    unit_test_sim.ExecuteSimulation()

    if test_reset:
        module.reset(1)
        unit_test_sim.ConfigureStopTime(mc.sec2nano(total_test_sim_time + 1.0))
        unit_test_sim.ExecuteSimulation()

    # Tick count: tasks at rate update_time fire at t=0, dt, 2*dt, ..., total_test_sim_time → that's
    # (total_test_sim_time / update_time) + 1 ticks. Reset adds 1.0s of additional sim → 1.0/update_time more.
    num_steps_pre_reset = int(round(total_test_sim_time / update_time)) + 1
    num_steps_post_reset = int(round(1.0 / update_time)) if test_reset else 0

    sigma_RN_true, omega_RN_true, dOmega_RN_true = compute_truth(
        sigma_RR0, omega_RR0_R, ref_state_in_data, update_time, num_steps_pre_reset, num_steps_post_reset
    )

    accuracy = 1e-6

    np.testing.assert_allclose(data_log.sigma_RN, sigma_RN_true, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(data_log.omega_RN_N, omega_RN_true, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(data_log.domega_RN_N, dOmega_RN_true, atol=accuracy, rtol=0, verbose=True)


if __name__ == "__main__":
    test_mrp_rotation(False, True)
