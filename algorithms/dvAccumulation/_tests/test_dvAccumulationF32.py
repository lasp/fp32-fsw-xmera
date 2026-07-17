"""
Module Name:        dvAccumulation

Regression test for the fp32 dvAccumulation port. Faithful port of the original Xmera
dvAccumulation unit test: drives two accelerometer snapshots through the SysModel adapter and
checks the accumulated body-frame Delta-V (and time-tag) against the original test's hardcoded
expected values. The fp32 port reproduces the original double-precision oracle to within float32
tolerance, which is the point of the test.
"""

import numpy as np
from numpy import random
from xmera.architecture import messaging
from xmera.fp32 import dvAccumulationF32
from xmera.utilities import SimulationBaseClass, macros


def generateAccData(num_packets=120):
    """Return a list of `num_packets` AccPktDataMsgF32Payload with random measTime / accel_B,
    generated identically to the original Xmera test (legacy numpy global RNG)."""
    accPktList = []
    for _ in range(num_packets):
        packet = messaging.AccPktDataMsgF32Payload()
        packet.measTime = abs(int(random.normal(5e7, 1e7)))
        packet.accel_B = random.normal(0.1, 0.2, 3)  # [m/s^2]; stored as float32 in the F32 payload
        accPktList.append(packet)
    return accPktList


def test_dv_accumulation():
    """End-to-end: drive two snapshots through the adapter and compare against the original
    Xmera test's hardcoded expected Delta-V and time-tag series."""

    random.seed(12345)

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess("TestProcess")
    testProc.addTask(unitTestSim.CreateNewTask("unitTask", testProcessRate))

    module = dvAccumulationF32.DvAccumulation()
    module.modelTag = "dvAccumulation"
    unitTestSim.AddModelToTask("unitTask", module)

    dataLog = module.dvAcumOutMsg.recorder()
    unitTestSim.AddModelToTask("unitTask", dataLog)

    # First snapshot
    firstInput = messaging.AccDataMsgF32Payload()
    firstInput.accPkts = generateAccData()
    inMsg = messaging.AccDataMsgF32()
    module.accPktInMsg.subscribeTo(inMsg)

    unitTestSim.InitializeSimulation()
    inMsg.write(firstInput)

    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # Second snapshot — same module, fresh accel data (continues the seeded RNG stream)
    secondInput = messaging.AccDataMsgF32Payload()
    secondInput.accPkts = generateAccData()
    inMsg.write(secondInput)

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))
    unitTestSim.ExecuteSimulation()

    # Expected values from the original Xmera dvAccumulation unit test. reset() runs before the
    # first message is written, so it seeds previousTime = 0; the first update integrates the first
    # snapshot (skipping the earliest packet via the dvInitialized latch), and the second update adds
    # the packets newer than the first snapshot's latest measTime.
    trueDVVector = np.array(
        [[4.82820079e-03, 7.81971465e-03, 2.29605663e-03]] * 3
        + [[6.44596343e-03, 9.00203561e-03, 2.60580728e-03]] * 2
    )
    trueTime = np.array([7.2123026e07] * 3 + [7.6667436e07] * 2) * macros.NANO2SEC

    np.testing.assert_allclose(dataLog.vehAccumDV, trueDVVector, rtol=0, atol=1e-6)
    np.testing.assert_allclose(dataLog.timeTag, trueTime, rtol=0, atol=1e-6)


if __name__ == "__main__":
    test_dv_accumulation()
