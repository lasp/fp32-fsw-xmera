# SPDX-License-Identifier: ISC
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import hillPointF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import astroFunctions as af
from xmera.utilities import macros


@pytest.mark.parametrize("cel_msg_set", [True, False])
def test_hill_point(cel_msg_set):
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = hillPointF32.HillPoint()
    module.modelTag = "hillPoint"
    sim.AddModelToTask(task_name, module)

    # circular equatorial orbit at 2.8 Earth radii, true anomaly 60 deg
    a = af.E_radius * 2.8
    e = 0.0
    i = 0.0
    Omega = 0.0
    omega = 0.0
    f = 60 * af.D2R
    (r, v) = af.OE2RV(af.mu_E, a, e, i, Omega, omega, f)

    nav_state_out_data = messaging.NavTransMsgF32Payload()
    nav_state_out_data.r_BN_N = r
    nav_state_out_data.v_BN_N = v
    nav_msg = messaging.NavTransMsgF32().write(nav_state_out_data)
    module.transNavInMsg.subscribeTo(nav_msg)

    if cel_msg_set:
        cel_body_data = messaging.EphemerisMsgF32Payload()
        cel_body_data.r_BdyZero_N = np.array([0.0, 0.0, 0.0])
        cel_body_data.v_BdyZero_N = np.array([0.0, 0.0, 0.0])
        cel_body_msg = messaging.EphemerisMsgF32().write(cel_body_data)
        module.celBodyInMsg.subscribeTo(cel_body_msg)

    data_log = module.attRefOutMsg.recorder()
    sim.AddModelToTask(task_name, data_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # FP32 tolerance: ~7 sig fig => 1e-6 absolute is comfortable for these magnitudes.
    accuracy = 1e-6

    sigma_truth = [0.0, 0.0, 0.267949192431]
    for sample in data_log.sigma_RN:
        np.testing.assert_allclose(sigma_truth, sample, atol=accuracy, verbose=True)

    omega_truth = [0.0, 0.0, 0.000264539877]
    for sample in data_log.omega_RN_N:
        np.testing.assert_allclose(omega_truth, sample, atol=accuracy, verbose=True)

    # for a circular orbit (e=0) the analytical d(omega)/dt is zero;
    # any nonzero output is float-precision noise.
    domega_truth = [0.0, 0.0, 0.0]
    for sample in data_log.domega_RN_N:
        np.testing.assert_allclose(domega_truth, sample, atol=accuracy, verbose=True)


if __name__ == "__main__":
    test_hill_point(True)
