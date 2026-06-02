# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

import pytest
import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.fp32 import triadF32
from xmera.utilities import macros
from xmera.architecture import messaging


def reference_triad(eh_N, sh_N, a1_B, h1_B):
    r2 = h1_B
    r3 = np.cross(a1_B, h1_B) / np.linalg.norm(np.cross(a1_B, h1_B))
    r1 = np.cross(r2, r3)

    n2 = eh_N
    n1 = np.cross(sh_N, eh_N) / np.linalg.norm(np.cross(sh_N, eh_N))
    n3 = np.cross(n1, n2)

    ND = (np.vstack((n1, n2, n3))).T
    RD = (np.vstack((r1, r2, r3))).T

    RN = RD @ ND.T
    return RN


@pytest.mark.parametrize("case", [
    1,  # SPE below 90 degrees, x-axis orthogonal to sun
    2,  # SPE below 90 degrees, y-axis aligned to earth
    3,  # SPE above 90 degrees, x-axis orthogonal to sun
    4,  # SPE above 90 degrees, y-axis aligned to earth
])
def test_triad(show_plots, case):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    if case == 1 or case == 2:
        # SPE = 7.16
        sh_N = [2.12024926e+11, 2.12239088e+11, 6.60583756e-01]
        r_EN_N = [3.43401015e+11, 2.76561597e+11, 2.78825040e+10]
    if case == 3 or case == 4:
        # SPE = 129.23
        sh_N = [-7.47993852e+10, -3.03274801e+08, -6.16397545e-01]
        r_EN_N = [5.65767033e+10, 6.40192339e+10, 2.78825040e+10]

    sh_N = sh_N / np.linalg.norm(sh_N)
    r_BN_N = np.array([0, 0, 0])

    eh_N = r_EN_N - r_BN_N
    r_EN_N = r_EN_N / np.linalg.norm(r_EN_N)
    eh_N = eh_N / np.linalg.norm(eh_N)

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = triadF32.Triad()
    module.modelTag = "triad"

    a1_B = np.array([1, 0, 0])
    h1_B = np.array([0, 1, 0])

    unit_test_sim.AddModelToTask(unit_task_name, module)
    module.sadaHat_B = a1_B
    module.thrustReqHat_N = eh_N
    module.signOfZHat_N = 1.0

    sigma_BN = np.array([0.1, -0.2, 0.1])
    BN = rbk.MRP2C(sigma_BN)
    rS_B = np.matmul(BN, sh_N)
    nav_att_data = messaging.NavAttMsgF32Payload()
    nav_att_data.sigma_BN = sigma_BN
    nav_att_data.vehSunPntBdy = rS_B
    nav_att_msg = messaging.NavAttMsgF32().write(nav_att_data)
    module.attNavInMsg.subscribeTo(nav_att_msg)

    thrust_body_msg_data = messaging.BodyHeadingMsgF32Payload()
    thrust_body_msg_data.rHat_XB_B = h1_B
    thrust_body_msg = messaging.BodyHeadingMsgF32().write(thrust_body_msg_data)
    module.bodyHeadingInMsg.subscribeTo(thrust_body_msg)

    data_log = module.attRefOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))
    unit_test_sim.ExecuteSimulation()

    module_output = data_log.sigma_RN
    sigma_RN = module_output[0]
    RN = rbk.MRP2C(sigma_RN)

    if case == 1 or case == 3:
        check = np.dot(np.array(RN[0, :]), sh_N)
        np.testing.assert_allclose(check, 0.0, atol=1e-3,
                                   err_msg=f"X-axis is not orthogonal to sun")
    elif case == 2 or case == 4:
        check = np.dot(RN[1, :], eh_N)
        np.testing.assert_allclose(check, 1.0, atol=1e-3,
                                   err_msg=f"Y-axis not aligned to earth")


if __name__ == "__main__":
    test_triad(False, 1)
