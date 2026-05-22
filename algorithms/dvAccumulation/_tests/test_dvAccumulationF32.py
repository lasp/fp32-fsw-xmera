"""
Module Name:        dvAccumulation

Regression test for the fp32 dvAccumulation port. Writes a deterministic 120-packet
accelerometer snapshot through the SysModel adapter and checks that the running Delta-V
accumulator matches an independent numpy reference that mirrors the algorithm's behavior
(sort-by-measTime, integrate every packet strictly newer than the previously-seen latest
measTime).
"""

import inspect
import os

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fp32 import dvAccumulationF32
from xmera.utilities import SimulationBaseClass, macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


def generateAccData(rng, num_packets=120):
    """Return a list of `num_packets` AccPktDataMsgF32Payload with random measTime / accel_B."""
    accPktList = []
    for _ in range(num_packets):
        packet = messaging.AccPktDataMsgF32Payload()
        packet.measTime = abs(int(rng.normal(5e7, 1e7)))
        packet.accel_B = rng.normal(0.1, 0.2, 3).astype(np.float32)
        accPktList.append(packet)
    return accPktList


def referenceUpdate(packets, prev_time):
    """Reference: sort packets by measTime ascending, integrate every packet strictly newer than
    prev_time, return (vehAccumDV_B, new_prev_time)."""
    sorted_packets = sorted(packets, key=lambda p: p.measTime)
    vehAccumDV_B = np.zeros(3, dtype=np.float32)
    for p in sorted_packets:
        if p.measTime > prev_time:
            dt = np.float32(p.measTime - prev_time) * np.float32(1e-9)
            accel = np.array(list(p.accel_B), dtype=np.float32)
            vehAccumDV_B = vehAccumDV_B + dt * accel
            prev_time = p.measTime
    return vehAccumDV_B, prev_time


def test_dv_accumulation():
    """End-to-end: drive two snapshots through the adapter and compare against a numpy reference."""

    rng = np.random.default_rng(12345)

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
    firstPackets = generateAccData(rng)
    firstInput = messaging.AccDataMsgF32Payload()
    firstInput.accPkts = firstPackets
    inMsg = messaging.AccDataMsgF32()
    module.accPktInMsg.subscribeTo(inMsg)

    unitTestSim.InitializeSimulation()
    inMsg.write(firstInput)

    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # Second snapshot — same module, fresh accel data
    secondPackets = generateAccData(rng)
    secondInput = messaging.AccDataMsgF32Payload()
    secondInput.accPkts = secondPackets
    inMsg.write(secondInput)

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))
    unitTestSim.ExecuteSimulation()

    # Compute reference. reset() seeds previousTime to the latest measTime in the first snapshot,
    # so the first update integrates nothing new (no packets > latest). The second snapshot then
    # adds whatever is strictly newer.
    seededPrev = max((p.measTime for p in firstPackets if p.measTime > 0), default=0)
    accumAfterSecond, _ = referenceUpdate(secondPackets, seededPrev)

    outputDV = np.array(dataLog.vehAccumDV)
    # Output series is recorded at each task step. The last sample reflects the post-second-update
    # state. Compare just that.
    np.testing.assert_allclose(outputDV[-1], accumAfterSecond, rtol=0, atol=1e-6)


if __name__ == "__main__":
    test_dv_accumulation()
