"""
Module Name:        dvAccumulation

Smoke / interface test for the fp32 dvAccumulation module. dvAccumulation integrates one
body-frame acceleration sample in each update (an IMUSensorBodyMsgF32 input). It uses the
configured control period as the integration step.
"""

import numpy as np
from xmera.architecture import messaging
from xmera.fp32 import dvAccumulationF32
from xmera.utilities import SimulationBaseClass, macros


def test_dv_accumulation():
    """End-to-end smoke test. It puts a constant body-frame acceleration through the adapter during
    more than one control step. Then it makes sure that the accumulated Delta-V is finite and that
    it increases."""

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess("TestProcess")
    test_proc.addTask(unit_test_sim.CreateNewTask("unitTask", test_process_rate))

    module = dvAccumulationF32.DvAccumulation()
    module.modelTag = "dvAccumulation"
    module.controlPeriod = macros.NANO2SEC * test_process_rate
    # This is a test of the swig_eigen typemap: a python list must convert to the Eigen::Vector3f
    # property.
    module.accelBias_B = [0.0, 0.0, 0.0]
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

    # Each interface operated and gave finite output.
    assert accum_dv.shape[0] > 0
    assert np.all(np.isfinite(accum_dv))
    assert np.all(np.isfinite(time_tag))

    # The first call starts the accumulation window and does not integrate. Thus the last accumulated
    # Delta-V is time * accel. It must not be zero, and its sign must be the same as the sign of the
    # input acceleration.
    final_dv = accum_dv[-1]
    assert np.linalg.norm(final_dv) > 0.0
    assert np.sign(final_dv[0]) == np.sign(accel_body[0])
    assert np.sign(final_dv[1]) == np.sign(accel_body[1])
    assert np.sign(final_dv[2]) == np.sign(accel_body[2])


if __name__ == "__main__":
    test_dv_accumulation()
