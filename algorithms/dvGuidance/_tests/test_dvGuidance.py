import numpy as np

from xmera.architecture import messaging
from xmera.fp32 import dvGuidanceF32
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def test_dv_guidance():
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = dvGuidanceF32.DvGuidance()
    module.modelTag = "dvGuidance"
    sim.AddModelToTask(task_name, module)

    cmd_dv_N = np.array([5.0, 5.0, 5.0])
    r_hat_pref_N = np.array([1.0, 0.0, 0.0])
    module.cmdDv_N = cmd_dv_N
    module.rHatPref_N = r_hat_pref_N

    data_log = module.attRefOutMsg.recorder()
    sim.AddModelToTask(task_name, data_log)

    # Run at t = 0.0, 0.5, and 1.0 s. The burn frame is inertially fixed, so every sample is the same.
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Absolute tolerance for FP32 guidance outputs.
    accuracy = 1e-6

    # Expected burn frame R: 3rd axis along the delta-V command, 1st axis perpendicular to rHatPref_N and
    # the delta-V command, 2nd axis completes the right-handed triad.
    r3_N = cmd_dv_N / np.linalg.norm(cmd_dv_N)
    r1_N = np.cross(r_hat_pref_N, r3_N)
    r1_N = r1_N / np.linalg.norm(r1_N)
    r2_N = np.cross(r3_N, r1_N)
    true_sigma = rbk.C2MRP(np.array([r1_N, r2_N, r3_N]))
    true_omega = [0.0, 0.0, 0.0]
    true_domega = [0.0, 0.0, 0.0]

    for i in range(len(data_log.sigma_RN)):
        np.testing.assert_allclose(true_sigma, data_log.sigma_RN[i], atol=accuracy, verbose=True)
        np.testing.assert_allclose(true_omega, data_log.omega_RN_N[i], atol=accuracy, verbose=True)
        np.testing.assert_allclose(true_domega, data_log.domega_RN_N[i], atol=accuracy, verbose=True)


if __name__ == "__main__":
    test_dv_guidance()
