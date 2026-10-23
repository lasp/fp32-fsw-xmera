import numpy as np
import pytest

from xmera.architecture import messaging
from xmera.fp32 import momentumManagementF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


# Double-precision RW cluster momentum for the wheel speeds and spin axes configured below.
CLUSTER_MOMENTUM = np.array([6.773502691896258, 3.273502691896258, 10.773502691896258])  # [Nms]

NOMINAL_HS_MIN = 100.0 / 6000.0 * 100.0  # [Nms]
HIGH_HS_MIN = 1000.0 / 6000.0 * 100.0  # [Nms] above the cluster momentum


def plane_projector(undumpable_axis):
    """Orthogonal projector onto the plane perpendicular to ``undumpable_axis``."""
    axis = np.array(undumpable_axis) / np.linalg.norm(undumpable_axis)
    return np.identity(3) - np.outer(axis, axis)


# Every case sets each configuration property explicitly, so every value is passed through the Python interface.
@pytest.mark.parametrize("hs_min, dumpable_projection, hs_desired", [
    # The whole cluster momentum is dumped.
    (NOMINAL_HS_MIN, np.identity(3), [0.0, 0.0, 0.0]),
    # The threshold is above the cluster momentum, so no dumping is requested.
    (HIGH_HS_MIN, np.identity(3), [0.0, 0.0, 0.0]),
    # A direction the effectors cannot dump about, off a body axis as a gimbaled thruster's moment arm is.
    # The whole cluster momentum lies in that direction, so nothing is left to dump.
    (0.0, plane_projector(CLUSTER_MOMENTUM), [0.0, 0.0, 0.0]),
    # A part of the cluster momentum is undumpable; the rest is dumped.
    (NOMINAL_HS_MIN, plane_projector([0.0, 0.2716, -0.9624]), [0.0, 0.0, 0.0]),
    # The cluster momentum is dumped toward a non-zero desired momentum rather than toward zero.
    (NOMINAL_HS_MIN, np.identity(3), [2.0, -1.0, 5.0]),
    # A desired momentum and an undumpable direction together.
    (NOMINAL_HS_MIN, plane_projector([0.0, 0.2716, -0.9624]), [2.0, -1.0, 5.0]),
    # A desired momentum smaller than the threshold: the cluster is still driven all the way to it.
    (NOMINAL_HS_MIN, np.identity(3), [1.0, 0.0, 0.0]),
    # The cluster momentum is above the threshold, but its difference to the desired momentum is not.
    (5.0, np.identity(3), [6.0, 3.0, 10.0]),
])
def test_momentum_management(hs_min, dumpable_projection, hs_desired):
    """Module Unit Test"""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = sim.CreateNewProcess(process_name)
    test_proc.addTask(sim.CreateNewTask(task_name, test_process_rate))

    module = momentumManagementF32.MomentumManagement()
    module.modelTag = "momentumManagement"
    sim.AddModelToTask(task_name, module)

    module.hsMin = hs_min  # [Nms]

    # [1/s] feedback gain mapping the stored momentum onto the requested torque. Sized so the ~13 Nms
    # cluster momentum is dumped over a few hundred seconds, i.e. a torque of order 0.5 Nm.
    k_gain = 0.05
    module.K = k_gain

    # The integral term is switched off here so the logged request is a pure function of the wheel speeds and
    # can be checked against the double-precision truth values below; the integral path is covered by the C++
    # unit tests. Only the integral consumes integralLimit and controlPeriod, so both may stay at zero, but
    # controlPeriod is set to the task rate anyway to mirror a realistic configuration.
    module.Ki = 0.0
    module.integralLimit = 0.0
    module.controlPeriod = macros.NANO2SEC * test_process_rate

    # The projector keeps only the momentum the effectors can dump; the identity keeps all of it.
    module.dumpableProjection_B = dumpable_projection

    # The momentum the dump drives toward; zero dumps the whole cluster momentum.
    module.hsDesired_B = hs_desired

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

    data_log = module.cmdTorqueOutMsg.recorder()
    sim.AddModelToTask(task_name, data_log)

    module.rwSpeedsInMsg.subscribeTo(rw_speed_in_msg)
    module.rwConfigDataInMsg.subscribeTo(rw_config_in_msg)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.5))
    sim.ExecuteSimulation()

    # The threshold gates the law: above it the whole dumpable momentum difference is dumped, below it nothing
    # is. The requested torque is the dumpable difference between the double-precision cluster momentum and the
    # desired momentum, scaled by -K. The module writes the request every update; the wheel speeds are constant,
    # so both logged steps carry the same value.
    dumpable_difference = dumpable_projection @ (CLUSTER_MOMENTUM - np.array(hs_desired))
    if np.linalg.norm(dumpable_difference) >= hs_min:
        true_vector = -k_gain * dumpable_difference
    else:
        true_vector = np.zeros(3)

    # FP32 tolerance: the observed error against the double truth is ~4e-7 on cluster momenta of order 10,
    # i.e. at float epsilon. The absolute error scales with the gain; the relative error does not, and must
    # stay above float32 epsilon (~1.2e-7).
    atol = k_gain * 1e-6
    rtol = 1e-6

    assert len(data_log.torqueRequestBody) == 2
    for sample in data_log.torqueRequestBody:
        np.testing.assert_allclose(true_vector, sample, atol=atol, rtol=rtol, verbose=True)
