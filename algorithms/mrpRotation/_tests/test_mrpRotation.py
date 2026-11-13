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


def compute_truth(sigma_RcR0, segments, omega_RRp_Rp, ref_state_in_data, dt):
    """Each tick advances sigma_RRp by the configured controlPeriod (dt) and emits the output frame
    [RN] = [RRp][RpRc][RcR0][R0N]. The algorithm integrates BEFORE computing the output. The rate is
    given in Rp axes.

    ``segments`` is a list of (sigma_RpRc, num_steps, restart) tuples. ``restart`` is True when reset()
    rebuilt the algorithm before the segment (sigma_RRp restarts at identity); reconfigure() keeps sigma_RRp.
    """
    sigma_R0N = np.array(ref_state_in_data.sigma_RN)
    R0N = rbk.MRP2C(sigma_R0N)
    RcR0 = rbk.MRP2C(np.array(sigma_RcR0))
    omega_R0N_N = np.array(ref_state_in_data.omega_RN_N)
    domega_R0N_N = np.array(ref_state_in_data.domega_RN_N)
    omega_RRp_Rp = np.array(omega_RRp_Rp)

    truth_sigma = []
    truth_omega_RN_N = []
    truth_domega_RN_N = []

    sigma_RRp = np.zeros(3)
    for sigma_RpRc, num_steps, restart in segments:
        if restart:
            sigma_RRp = np.zeros(3)
        RpRc = rbk.MRP2C(np.array(sigma_RpRc))
        for _ in range(num_steps):
            # Forward-Euler MRP integration with mrpSwitch (shadow set when |sigma| > 1).
            B = rbk.BmatMRP(sigma_RRp)
            omega_RRp_R = rbk.MRP2C(sigma_RRp) @ omega_RRp_Rp
            sigma_RRp = sigma_RRp + dt * 0.25 * np.dot(B, omega_RRp_R)
            n_sq = float(np.dot(sigma_RRp, sigma_RRp))
            if n_sq > 1.0:
                sigma_RRp = -sigma_RRp / n_sq
            RpN = RpRc @ RcR0 @ R0N
            RN = rbk.MRP2C(sigma_RRp) @ RpN
            omega_RRp_N = RpN.T @ omega_RRp_Rp
            truth_sigma.append(rbk.C2MRP(RN).tolist())
            truth_omega_RN_N.append((omega_RRp_N + omega_R0N_N).tolist())
            truth_domega_RN_N.append((np.cross(omega_R0N_N, omega_RRp_N) + domega_R0N_N).tolist())

    return truth_sigma, truth_omega_RN_N, truth_domega_RN_N


@pytest.mark.parametrize("second_phase", ["none", "reset", "reconfigure"])
def test_mrp_rotation(show_plots, second_phase):
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

    sigma_RcR0 = np.array([0.3, .5, 0.0])
    module.sigma_RcR0 = sigma_RcR0
    sigma_RpRc_1 = np.array([0.0, 0.0, 0.05])
    module.sigma_RpRc = sigma_RpRc_1
    omega_RRp_Rp = np.array([0.1, 0.0, 0.0]) * mc.D2R
    module.omega_RRp_Rp = omega_RRp_Rp
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

    # Tick count: tasks at rate update_time fire at t=0, dt, 2*dt, ..., total_test_sim_time → that's
    # (total_test_sim_time / update_time) + 1 ticks. The second phase adds 1.0s → 1.0/update_time more.
    segments = [(sigma_RpRc_1, int(round(total_test_sim_time / update_time)) + 1, True)]

    if second_phase != "none":
        if second_phase == "reset":
            module.reset(1)
            segments.append((sigma_RpRc_1, int(round(1.0 / update_time)), True))
        else:
            sigma_RpRc_2 = np.array([0.04, -0.02, 0.0])
            module.sigma_RpRc = sigma_RpRc_2
            module.reconfigure()
            segments.append((sigma_RpRc_2, int(round(1.0 / update_time)), False))
        unit_test_sim.ConfigureStopTime(mc.sec2nano(total_test_sim_time + 1.0))
        unit_test_sim.ExecuteSimulation()

    sigma_RN_true, omega_RN_true, dOmega_RN_true = compute_truth(
        sigma_RcR0, segments, omega_RRp_Rp, ref_state_in_data, update_time
    )

    accuracy = 1e-6

    np.testing.assert_allclose(data_log.sigma_RN, sigma_RN_true, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(data_log.omega_RN_N, omega_RN_true, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(data_log.domega_RN_N, dOmega_RN_true, atol=accuracy, rtol=0, verbose=True)


if __name__ == "__main__":
    test_mrp_rotation(False, "reconfigure")
