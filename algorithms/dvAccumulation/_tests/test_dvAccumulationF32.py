"""
Module Name:        dvAccumulation

Smoke / interface test for the fp32 dvAccumulation module. dvAccumulation integrates a single
body-frame acceleration sample per update (an IMUSensorBodyMsgF32 input), using the module call time
for the integration step. This test exercises every interface end-to-end through the SysModel adapter
-- link the input message, run several steps, and confirm the module executes and produces a finite,
correctly-signed accumulated Delta-V.
"""

import numpy as np
from xmera.architecture import messaging
from xmera.fp32 import dvAccumulationF32
from xmera.utilities import SimulationBaseClass, macros


def test_dv_accumulation():
    """End-to-end smoke test: drive a constant body-frame acceleration through the adapter over
    several control steps and confirm the accumulated Delta-V is finite and grows as expected."""

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess("TestProcess")
    test_proc.addTask(unit_test_sim.CreateNewTask("unitTask", test_process_rate))

    module = dvAccumulationF32.DvAccumulation()
    module.modelTag = "dvAccumulation"
    unit_test_sim.AddModelToTask("unitTask", module)

    data_log = module.dvAccumulationOutMsg.recorder()
    unit_test_sim.AddModelToTask("unitTask", data_log)

    # Constant body-frame acceleration [m/s^2] on the input IMU message.
    accel_body = [0.1, -0.2, 0.3]
    input_data = messaging.IMUSensorBodyMsgF32Payload()
    input_data.AccelBody = accel_body
    in_msg = messaging.IMUSensorBodyMsgF32().write(input_data)
    module.imuInMsg.subscribeTo(in_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))
    unit_test_sim.ExecuteSimulation()

    accum_dv = np.array(data_log.vehAccumDV)
    time_tag = np.array(data_log.timeTag)

    # Every interface ran and produced finite output.
    assert accum_dv.shape[0] > 0
    assert np.all(np.isfinite(accum_dv))
    assert np.all(np.isfinite(time_tag))

    # The first call only sets the time reference (no integration), so the final accumulated Delta-V is
    # positive time * accel: it must be non-zero and share the sign of the input acceleration.
    final_dv = accum_dv[-1]
    assert np.linalg.norm(final_dv) > 0.0
    assert np.sign(final_dv[0]) == np.sign(accel_body[0])
    assert np.sign(final_dv[1]) == np.sign(accel_body[1])
    assert np.sign(final_dv[2]) == np.sign(accel_body[2])


if __name__ == "__main__":
    test_dv_accumulation()
