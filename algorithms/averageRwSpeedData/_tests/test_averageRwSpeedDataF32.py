import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import averageRwSpeedDataF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

_NUM_RW = messaging.RW_EFF_CNT
_SAMPLE_PERIOD_SEC = 0.2  # 5 Hz RW speed rate
_MAX_AVERAGING_WINDOW_SEC = 2.0  # mirrors kMaxAveragingWindowSec in averageRwSpeedDataAlgorithm.h


def _make_sim(window_sec):
    unit_task_name = "unitTask"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess("TestProcess")
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, macros.sec2nano(_SAMPLE_PERIOD_SEC)))

    module = averageRwSpeedDataF32.AverageRwSpeedData()
    module.modelTag = "averageRwSpeedData"
    module.rwSpeedAveragingWindow = window_sec

    rw_speed_msg = messaging.RWSpeedMsgF32().write(messaging.RWSpeedMsgF32Payload(), time=0)
    module.rwSpeedInMsg.subscribeTo(rw_speed_msg)

    unit_test_sim.AddModelToTask(unit_task_name, module)
    data_log = module.rwSpeedOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)
    unit_test_sim.InitializeSimulation()
    return unit_test_sim, module, data_log


def test_average_rw_speed_data():
    """The output wheel speeds are the mean of the samples inside the window; the wheel angles pass through."""
    window_sec = 1.0
    samples_in_window = round(window_sec / _SAMPLE_PERIOD_SEC) + 1  # the window includes both ends
    num_steps = 15

    unit_test_sim, module, data_log = _make_sim(window_sec)

    rng = np.random.default_rng(seed=7)
    input_speeds = np.zeros((num_steps, _NUM_RW), dtype=np.float32)
    input_thetas = np.zeros((num_steps, _NUM_RW), dtype=np.float32)
    expected_speeds = np.zeros((num_steps, _NUM_RW), dtype=np.float32)

    for step in range(num_steps):
        input_speeds[step] = rng.normal(loc=100.0, scale=20.0, size=_NUM_RW).astype(np.float32)
        input_thetas[step] = rng.uniform(low=-np.pi, high=np.pi, size=_NUM_RW).astype(np.float32)
        first_in_window = max(0, step + 1 - samples_in_window)
        expected_speeds[step] = input_speeds[first_in_window : step + 1].mean(axis=0)

        payload = messaging.RWSpeedMsgF32Payload()
        payload.wheelSpeeds = input_speeds[step].tolist()
        payload.wheelThetas = input_thetas[step].tolist()
        sim_time_ns = macros.sec2nano((step + 1) * _SAMPLE_PERIOD_SEC)
        rw_speed_msg = messaging.RWSpeedMsgF32().write(payload, time=sim_time_ns)
        module.rwSpeedInMsg.subscribeTo(rw_speed_msg)
        unit_test_sim.ConfigureStopTime(sim_time_ns)
        unit_test_sim.ExecuteSimulation()

    output_speeds = np.array(data_log.wheelSpeeds)[1:, :_NUM_RW]
    output_thetas = np.array(data_log.wheelThetas)[1:, :_NUM_RW]

    np.testing.assert_allclose(output_speeds, expected_speeds, rtol=1e-6, atol=1e-4)

    # The averaging changes the wheel speeds, but the wheel angles stay as they were in the input.
    assert not np.allclose(output_speeds[1:], input_speeds[1:])
    np.testing.assert_array_equal(output_thetas, input_thetas)


def test_average_rw_speed_data_requires_input_message():
    """reset() rejects an unconnected input message."""
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess("TestProcess")
    test_proc.addTask(unit_test_sim.CreateNewTask("unitTask", macros.sec2nano(_SAMPLE_PERIOD_SEC)))

    module = averageRwSpeedDataF32.AverageRwSpeedData()
    module.rwSpeedAveragingWindow = 1.0
    unit_test_sim.AddModelToTask("unitTask", module)

    with pytest.raises(RuntimeError, match="rwSpeed input message name was not linked"):
        unit_test_sim.InitializeSimulation()


@pytest.mark.parametrize("window_sec", [-0.1, _MAX_AVERAGING_WINDOW_SEC + 0.01])
def test_average_rw_speed_data_rejects_invalid_window(window_sec):
    """reset() rejects an averaging window outside the accepted range."""
    with pytest.raises(RuntimeError, match="rwSpeedAveragingWindow"):
        _make_sim(window_sec)


if __name__ == "__main__":
    test_average_rw_speed_data()
    test_average_rw_speed_data_requires_input_message()
    test_average_rw_speed_data_rejects_invalid_window(-0.1)
