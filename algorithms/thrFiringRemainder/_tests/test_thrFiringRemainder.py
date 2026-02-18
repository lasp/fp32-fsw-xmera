import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import thrFiringRemainderF32
from xmera.utilities import macros

from xmera.architecture.messaging import (
    THRArrayConfigMsgF32,
    THRArrayConfigMsgF32Payload,
    THRArrayCmdForceMsgF32,
    THRArrayCmdForceMsgF32Payload,
)
from xmera.architecture.messaging.THRArrayCmdForceMsgF32Payload import MAX_EFF_CNT
import numpy as np


def generate_random_thrusters(rng: np.random.Generator, num_thrusters: int, max_thrust: float) -> list[dict]:
    """
    Generate random thruster configurations.

    Args:
        rng: numpy random generator for reproducibility
        num_thrusters: number of thrusters to generate (clamped to MAX_EFF_CNT)
        max_thrust: maximum thrust value for all thrusters

    Returns:
        List of thruster config dicts with keys: rThrust_B, tHatThrust_B, maxThrust
    """
    num_thrusters = min(num_thrusters, MAX_EFF_CNT)
    thrusters = []
    for _ in range(num_thrusters):
        r_thrust_B = rng.uniform(-2.0, 2.0, size=3)
        direction = rng.standard_normal(size=3)
        tHat_thrust_B = direction / np.linalg.norm(direction)
        thrusters.append({
            "rThrust_B": r_thrust_B.tolist(),
            "tHatThrust_B": tHat_thrust_B.tolist(),
            "maxThrust": max_thrust,
        })
    return thrusters


def create_thruster_array_config_msg(thrusters: list[dict]) -> THRArrayConfigMsgF32:
    """
    Create a thruster array config message from a list of thruster dicts.

    Each dict must have keys: rThrust_B, tHatThrust_B, maxThrust
    """
    payload = THRArrayConfigMsgF32Payload()
    for i, thr in enumerate(thrusters):
        payload.thrusters[i].rThrust_B = thr["rThrust_B"]
        payload.thrusters[i].tHatThrust_B = thr["tHatThrust_B"]
        payload.thrusters[i].maxThrust = thr["maxThrust"]
    payload.numThrusters = len(thrusters)

    msg = THRArrayConfigMsgF32().write(payload)
    msg.this.disown()
    return msg


@pytest.mark.parametrize("thrust_pulsing_regime", [thrFiringRemainderF32.ThrustPulsingRegime_OFF_PULSING,
                                                   thrFiringRemainderF32.ThrustPulsingRegime_ON_PULSING])
def test_thrFiringRemainderF32(show_plots, thrust_pulsing_regime):
    """Exercise the Python/SWIG interface for ThrFiringRemainder.

    Verifies that the module can be configured, connected, and run
    within the simulation framework, and that it produces non-trivial output.
    Correctness is validated by the C++ unit and fuzz tests.
    """
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    fsw_rate = 0.5
    control_period = fsw_rate
    test_process_rate = macros.sec2nano(fsw_rate)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Configure the module
    module = thrFiringRemainderF32.ThrFiringRemainder()
    module.modelTag = "thrFiringRemainder"
    module.thrMinFireTime = 0.2
    module.controlPeriod = control_period
    module.thrustPulsingRegime = thrust_pulsing_regime

    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Generate random thruster configuration
    rng = np.random.default_rng(seed=42)
    num_thrusters = rng.integers(1, MAX_EFF_CNT + 1)
    max_thrust = 0.5
    thrusters = generate_random_thrusters(rng, num_thrusters, max_thrust)
    thr_config_msg = create_thruster_array_config_msg(thrusters)

    # Generate random thrust force commands
    thr_message_data = THRArrayCmdForceMsgF32Payload()
    if thrust_pulsing_regime == thrFiringRemainderF32.ThrustPulsingRegime_OFF_PULSING:
        thr_force = rng.uniform(-max_thrust, 0.0, size=num_thrusters)
    else:
        thr_force = rng.uniform(0.0, max_thrust, size=num_thrusters)
    thr_message_data.thrForce = thr_force.tolist()
    thr_force_msg = THRArrayCmdForceMsgF32().write(thr_message_data)

    # Connect messages
    module.thrConfInMsg.subscribeTo(thr_config_msg)
    module.thrForceInMsg.subscribeTo(thr_force_msg)

    # Record output
    data_log = module.onTimeOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Run simulation
    unit_test_sim.InitializeSimulation()
    final_time = 3.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(final_time))
    unit_test_sim.ExecuteSimulation()

    on_time_requests = data_log.onTimeRequest[:, :num_thrusters]

    # Verify output shape and basic invariants
    expected_steps = int(final_time / fsw_rate) + 1
    assert on_time_requests.shape == (expected_steps, num_thrusters)

    # All on-times must be non-negative
    assert np.all(on_time_requests >= 0.0)

    # All on-times must not exceed the oversaturation bound
    assert np.all(on_time_requests <= 1.1 * control_period)

    # Non-zero on-times must be >= thrMinFireTime
    non_zero = on_time_requests[on_time_requests > 0.0]
    assert np.all(non_zero >= module.thrMinFireTime)


if __name__ == "__main__":
    test_thrFiringRemainderF32(True, thrFiringRemainderF32.ThrustPulsingRegime_ON_PULSING)
