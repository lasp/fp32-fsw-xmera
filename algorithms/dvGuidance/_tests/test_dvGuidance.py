import numpy as np

from xmera.architecture import messaging
from xmera.fp32 import dvGuidanceF32
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

    burn_cmd = messaging.DvBurnCmdMsgF32Payload()
    burn_cmd.dvInrtlCmd = [5.0, 5.0, 5.0]
    burn_cmd.dvRotVecUnit = [1.0, 0.0, 0.0]
    burn_cmd.dvRotVecMag = 0.5
    burn_cmd.burnStartTime = macros.sec2nano(0.5)
    burn_in_msg = messaging.DvBurnCmdMsgF32().write(burn_cmd)
    module.burnDataInMsg.subscribeTo(burn_in_msg)

    data_log = module.attRefOutMsg.recorder()
    sim.AddModelToTask(task_name, data_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # FP32 tolerance: ~7 sig fig => 1e-6 absolute is comfortable for these magnitudes.
    accuracy = 1e-6

    true_sigma = [
        [5.69822629e-01, 1.99143700e-01, 2.72649472e-01],
        [6.12361487e-01, 1.31298090e-01, 3.16981631e-01],
        [6.50967464e-01, 5.62624705e-02, 3.61117890e-01],
    ]
    true_omega = [
        [4.08248290e-01, -2.04124145e-01, -2.04124145e-01],
        [4.08248290e-01, -2.04124145e-01, -2.04124145e-01],
        [4.08248290e-01, -2.04124145e-01, -2.04124145e-01],
    ]
    true_domega = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]

    for i in range(len(true_sigma)):
        np.testing.assert_allclose(true_sigma[i], data_log.sigma_RN[i], atol=accuracy, verbose=True)
        np.testing.assert_allclose(true_omega[i], data_log.omega_RN_N[i], atol=accuracy, verbose=True)
        np.testing.assert_allclose(true_domega[i], data_log.domega_RN_N[i], atol=accuracy, verbose=True)


if __name__ == "__main__":
    test_dv_guidance()
