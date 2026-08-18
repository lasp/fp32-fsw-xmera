import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import thrMomentumManagementF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


@pytest.mark.parametrize("hs_min_check", [0, 1])
def test_thr_momentum_management(hs_min_check):
    """Module Unit Test"""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = thrMomentumManagementF32.ThrMomentumManagement()
    module.modelTag = "thrMomentumManagement"
    sim.AddModelToTask(task_name, module)

    # hs_min_check == 1 puts the threshold above the RW cluster momentum, so no dumping is requested
    if hs_min_check:
        module.hsMin = 1000.0 / 6000.0 * 100.0  # Nms
    else:
        module.hsMin = 100.0 / 6000.0 * 100.0  # Nms

    # wheelSpeeds message
    rw_speed_message = messaging.RWSpeedMsgF32Payload()
    rw_speed_message.wheelSpeeds = [10.0, -25.0, 50.0, 100.0]
    rw_speed_in_msg = messaging.RWSpeedMsgF32().write(rw_speed_message)

    # wheelConfigData message
    js = 0.1
    rw_config_params = messaging.RWArrayConfigMsgF32Payload()
    rw_config_params.GsMatrix_B = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.5773502691896258, 0.5773502691896258, 0.5773502691896258,
    ]
    rw_config_params.JsList = [js] * 4
    rw_config_params.numRW = 4
    rw_config_in_msg = messaging.RWArrayConfigMsgF32().write(rw_config_params)

    data_log = module.deltaHOutMsg.recorder()
    sim.AddModelToTask(task_name, data_log)

    module.rwSpeedsInMsg.subscribeTo(rw_speed_in_msg)
    module.rwConfigDataInMsg.subscribeTo(rw_config_in_msg)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.5))
    sim.ExecuteSimulation()

    # Truth values carried over from the double-precision Xmera unit test. The module writes the
    # request every update; the wheel speeds are constant, so both logged steps carry the same value.
    if hs_min_check == 1:
        true_vector = [0.0, 0.0, 0.0]
    else:
        true_vector = [-5.914369484146579, -2.858300248464629, -9.407020039211664]

    # FP32 tolerance: the observed error against the double truth is ~4e-7 on magnitudes of order 10,
    # i.e. at float epsilon, so 1e-6 absolute is the tightest defensible bound.
    accuracy = 1e-6

    assert len(data_log.torqueRequestBody) == 2
    for sample in data_log.torqueRequestBody:
        np.testing.assert_allclose(true_vector, sample, atol=accuracy, rtol=accuracy, verbose=True)
