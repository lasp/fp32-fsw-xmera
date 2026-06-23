import numpy
import pytest
from xmera.fp32 import thrFiringSchmittF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

from xmera.architecture.messaging import (
    THRArrayConfigMsgF32,
    THRArrayConfigMsgF32Payload,
    THRArrayCmdForceMsgF32,
    THRArrayCmdForceMsgF32Payload,
)


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


@pytest.mark.parametrize("reset_check, thrust_pulsing_regime", [
    (False, thrFiringSchmittF32.ThrustPulsingRegime_ON_PULSING),
    (True, thrFiringSchmittF32.ThrustPulsingRegime_ON_PULSING),
    (False, thrFiringSchmittF32.ThrustPulsingRegime_OFF_PULSING),
    (True, thrFiringSchmittF32.ThrustPulsingRegime_OFF_PULSING),
])
def test_thr_firing_schmitt(show_plots, reset_check, thrust_pulsing_regime):
    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    fsw_rate = 0.5
    # Create test thread
    test_process_rate = macros.sec2nano(fsw_rate)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = thrFiringSchmittF32.ThrFiringSchmitt()
    module.modelTag = "thrFiringSchmitt"

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    control_period = fsw_rate
    thr_min_fire_time = 0.2
    level_on = 0.75
    level_off = 0.25
    on_time_saturation_factor = 1.1

    # Initialize the test module configuration data
    module.thrMinFireTime = thr_min_fire_time
    module.thrustPulsingRegime = thrust_pulsing_regime
    module.setLevelsOnOff(level_on, level_off)
    module.controlPeriod = control_period
    module.onTimeSaturationFactor = on_time_saturation_factor

    # setup thruster cluster message
    thrusters = [
        {"rThrust_B": [-0.86360, -0.82550,  1.79070], "tHatThrust_B": [ 1.0,  0.0, 0.0], "maxThrust": 0.5},
        {"rThrust_B": [-0.82550, -0.86360,  1.79070], "tHatThrust_B": [ 0.0,  1.0, 0.0], "maxThrust": 0.5},
        {"rThrust_B": [ 0.82550,  0.86360,  1.79070], "tHatThrust_B": [ 0.0, -1.0, 0.0], "maxThrust": 0.5},
        {"rThrust_B": [ 0.86360,  0.82550,  1.79070], "tHatThrust_B": [-1.0,  0.0, 0.0], "maxThrust": 0.5},
        {"rThrust_B": [-0.86360, -0.82550, -1.79070], "tHatThrust_B": [-1.0,  0.0, 0.0], "maxThrust": 0.5},
        {"rThrust_B": [-0.82550, -0.86360, -1.79070], "tHatThrust_B": [ 0.0, -1.0, 0.0], "maxThrust": 0.5},
        {"rThrust_B": [ 0.82550,  0.86360, -1.79070], "tHatThrust_B": [ 0.0,  1.0, 0.0], "maxThrust": 0.5},
        {"rThrust_B": [ 0.86360,  0.82550, -1.79070], "tHatThrust_B": [ 1.0,  0.0, 0.0], "maxThrust": 0.5},
    ]
    num_thrusters = len(thrusters)
    thr_conf_msg = create_thruster_array_config_msg(thrusters)
    module.thrConfInMsg.subscribeTo(thr_conf_msg)

    # setup thruster impulse request message
    input_message_data = THRArrayCmdForceMsgF32Payload()
    thr_cmd_msg = THRArrayCmdForceMsgF32()
    module.thrForceInMsg.subscribeTo(thr_cmd_msg)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.onTimeOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.

    if thrust_pulsing_regime == thrFiringSchmittF32.ThrustPulsingRegime_OFF_PULSING:
        eff_req1 = [0.0, -0.1, -0.2, -0.3, -0.349, -0.351, -0.451, -0.5]
        eff_req2 = [0.0, -0.1, -0.2, -0.3, -0.351, -0.351, -0.451, -0.5]
        eff_req3 = [0.0, -0.1, -0.2, -0.3, -0.5, -0.351, -0.451, -0.5]
        eff_req4 = [0.0, -0.1, -0.2, -0.3, -0.351, -0.351, -0.451, -0.5]

    else:
        eff_req1 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.49]
        eff_req2 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.11]
        eff_req3 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.01]
        eff_req4 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.11]

    input_message_data.thrForce = eff_req1
    thr_cmd_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    input_message_data.thrForce = eff_req2
    thr_cmd_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    input_message_data.thrForce = eff_req3
    thr_cmd_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.5))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    input_message_data.thrForce = eff_req4
    thr_cmd_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(3.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    if reset_check:
        # reset the module to test this functionality
        module.reset(macros.sec2nano(3.0))     # this module reset function needs a time input (in NanoSeconds)

        # run the module again for an additional 1.0 seconds
        unit_test_sim.ConfigureStopTime(macros.sec2nano(5.5))        # seconds to stop simulation
        unit_test_sim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    on_time_requests = data_log.onTimeRequest[:, :num_thrusters]

    # set the filtered output truth states
    if reset_check:
        if thrust_pulsing_regime == thrFiringSchmittF32.ThrustPulsingRegime_OFF_PULSING:
            true_vector = [
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   ]
        else:
            true_vector = [
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   ]

    else:
        if thrust_pulsing_regime == thrFiringSchmittF32.ThrustPulsingRegime_OFF_PULSING:
            true_vector = [
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   ]
        else:
            true_vector = [
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   ]

    numpy.testing.assert_allclose(on_time_requests, true_vector, atol=1e-12, err_msg="onTimeRequest")

    # All on-times must be non-negative
    assert numpy.all(on_time_requests >= 0.0)

    # All on-times must not exceed the oversaturation bound
    assert numpy.all(on_time_requests <= module.onTimeSaturationFactor * control_period + 1e-6)

    # Non-zero on-times must be >= thrMinFireTime
    non_zero = on_time_requests[on_time_requests > 0.0]
    assert numpy.all(non_zero >= module.thrMinFireTime)

    # Getter/Setter roundtrip checks
    numpy.testing.assert_allclose(module.thrMinFireTime, thr_min_fire_time, atol=1e-6)
    numpy.testing.assert_equal(module.thrustPulsingRegime, thrust_pulsing_regime)
    numpy.testing.assert_allclose(module.controlPeriod, control_period, atol=1e-6)
    numpy.testing.assert_allclose(module.onTimeSaturationFactor, on_time_saturation_factor, atol=1e-6)
    levels = module.getLevelsOnOff()
    numpy.testing.assert_allclose(levels[0], level_on, atol=1e-6)
    numpy.testing.assert_allclose(levels[1], level_off, atol=1e-6)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thr_firing_schmitt(False, True, thrFiringSchmittF32.ThrustPulsingRegime_ON_PULSING)
