import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.architecture.messaging import (
    THRArrayConfigMsgF32,
    THRArrayConfigMsgF32Payload,
)
from xmera.fp32 import forceTorqueThrForceMappingF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

rcs_location_data_1 = [[-0.86360, -0.82550, 1.79070],
                       [-0.82550, -0.86360, 1.79070],
                       [0.82550, 0.86360, 1.79070],
                       [0.86360, 0.82550, 1.79070],
                       [-0.86360, -0.82550, -1.79070],
                       [-0.82550, -0.86360, -1.79070],
                       [0.82550, 0.86360, -1.79070],
                       [0.86360, 0.82550, -1.79070]]

rcs_location_data_2 = [[-1, -1, 1],
                       [-1, -1, 1],
                       [-1, -1, 1],
                       [1, 1, 1],
                       [1, 1, 1],
                       [1, 1, 1],
                       [1, 1, -1],
                       [1, 1, -1],
                       [1, 1, -1],
                       [-1, -1, -1],
                       [-1, -1, -1],
                       [-1, -1, -1]]

rcs_direction_data_1 = [[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0]]

rcs_direction_data_2 = [[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [0.0, 0.0, -1.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]]

np.random.seed(42)

num_thr_rand = np.random.randint(1, messaging.MAX_EFF_CNT)
rcs_location_data_rand = np.round(np.random.randn(num_thr_rand, 3), 3).tolist()
randomized_directions = np.round(np.random.randn(num_thr_rand, 3), 3)
rcs_direction_data_rand = (randomized_directions / np.linalg.norm(randomized_directions, axis=1)[:, None]).tolist()
torque_rand = np.random.rand(3)
force_rand = np.random.rand(3)

r"""
Test 1: Ensures that the forceTorqueThrForce module can compute a valid solution for cases where there is a direction
        where no thrusters point - ensures matrix invertibility is handled.
Test 2: Ensures that the forceTorqueThrForce module can compute a valid solution for the case where there is zero
        requested torque in a connected input message, but a requested non-zero force.
Test 3: Ensures that the forceTorqueThrForce module can compute a valid solution for the case where there is no torque
        input message, but a requested non-zero force.
Test 4: Ensures that the forceTorqueThrForce module can compute a valid solution for the case where Thrusters point in
        each direction.
"""

@pytest.mark.parametrize("rcs_location, rcs_direction, requested_torque, requested_force, torque_in_msg_flag",
                         [(rcs_location_data_1, rcs_direction_data_1, [0.4, 0.2, 0.4], [0.9, 1.1, 0.], True),
                          (rcs_location_data_1, rcs_direction_data_1, [0.0, 0.0, 0.0], [0.9, 1.1, 0.], True),
                          (rcs_location_data_1, rcs_direction_data_1, [0.0, 0.0, 0.0], [0.9, 1.1, 0.], False),
                          (rcs_location_data_2, rcs_direction_data_2, [0.0, 0.0, 0.0], [0.9, 1.1, 1.], True),
                          (rcs_location_data_rand, rcs_direction_data_rand, torque_rand, force_rand, True)])

def test_force_torque_thr_force_mapping(rcs_location, rcs_direction, requested_torque, requested_force,
                                        torque_in_msg_flag):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # setup module to be tested
    module = forceTorqueThrForceMappingF32.ForceTorqueThrForceMapping()
    module.modelTag = "forceTorqueThrForceMappingTag"
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Configure blank module input messages
    cmd_torque_in_msg_data = messaging.CmdTorqueBodyMsgF32Payload()
    cmd_torque_in_msg_data.torqueRequestBody = requested_torque
    cmd_torque_in_msg = messaging.CmdTorqueBodyMsgF32().write(cmd_torque_in_msg_data)

    cmd_force_in_msg_data = messaging.CmdForceBodyMsgF32Payload()
    cmd_force_in_msg_data.forceRequestBody = requested_force
    cmd_force_in_msg = messaging.CmdForceBodyMsgF32().write(cmd_force_in_msg_data)

    num_thrusters = len(rcs_location)
    max_thrust = 3.0  # N

    thr_config_payload = THRArrayConfigMsgF32Payload()
    thr_config_payload.numThrusters = num_thrusters
    for i in range(num_thrusters):
        thr_config_payload.thrusters[i].rThrust_B = rcs_location[i]
        thr_config_payload.thrusters[i].tHatThrust_B = rcs_direction[i]
        thr_config_payload.thrusters[i].maxThrust = max_thrust
    thr_config_in_msg = THRArrayConfigMsgF32().write(thr_config_payload)

    CoM_B = np.array([0.1, 0.1, 0.1])

    veh_config_in_msg_data = messaging.VehicleConfigMsgF32Payload()
    veh_config_in_msg_data.CoM_B = CoM_B
    veh_config_in_msg = messaging.VehicleConfigMsgF32().write(veh_config_in_msg_data)

    # subscribe input messages to module
    if torque_in_msg_flag:
        module.cmdTorqueInMsg.subscribeTo(cmd_torque_in_msg)
    module.cmdForceInMsg.subscribeTo(cmd_force_in_msg)
    module.thrConfigInMsg.subscribeTo(thr_config_in_msg)
    module.vehConfigInMsg.subscribeTo(veh_config_in_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.5))
    unit_test_sim.ExecuteSimulation()

    truth = compute_thrust_mapping_truth(rcs_location, rcs_direction, requested_torque, requested_force, CoM_B)

    accuracy = 1e-5
    np.testing.assert_allclose(np.array([module.thrForceCmdOutMsg.read().thrForce[0:len(rcs_location)]]).flatten(), truth,
                               atol=accuracy, rtol=accuracy, verbose=True)


def compute_thrust_mapping_truth(rcs_location, rcs_direction, requested_torque, requested_force, CoM_B):
    """Independent fp64 truth that mirrors the algorithm's truncated-SVD pseudo-inverse exactly.

    Two details must match the algorithm so the only remaining disagreement is fp32 round-off:
      1. DG has the same shape (6 x MAX_EFF_CNT, trailing zero columns) as the algorithm's matrix,
         so the SVD operates on the same operator.
      2. The truncation cutoff uses fp32 epsilon scaled by max(6, MAX_EFF_CNT) — the algorithm's
         noise floor — instead of fp64 epsilon. Otherwise the truth would keep singular values in
         the [eps_d, eps_f] gap that the algorithm correctly drops as fp32 noise, and 1/sv would
         blow up.
    """
    num_thrusters = len(rcs_location)
    max_eff_cnt = messaging.MAX_EFF_CNT
    ft = np.concatenate([requested_torque, requested_force]).astype(np.float64)
    CoM_B = np.array(CoM_B, dtype=np.float64)

    DG = np.zeros((6, max_eff_cnt), dtype=np.float64)
    for i in range(num_thrusters):
        r = np.array(rcs_location[i], dtype=np.float64)
        g = np.array(rcs_direction[i], dtype=np.float64)
        DG[0:3, i] = np.cross(r - CoM_B, g)
        DG[3:6, i] = g

    U, sv, Vt = np.linalg.svd(DG, full_matrices=False)
    eps_f = np.finfo(np.float32).eps
    tol = sv[0] * eps_f * max(6, max_eff_cnt)
    inv_sv = np.divide(1.0, sv, out=np.zeros_like(sv), where=sv > tol)
    thr_forces = Vt.T @ np.diag(inv_sv) @ U.T @ ft

    # min-shift over the active head only, matching the algorithm.
    thr_forces[0:num_thrusters] -= thr_forces[0:num_thrusters].min()
    return thr_forces[0:num_thrusters]


if __name__ == "__main__":
    test_force_torque_thr_force_mapping(rcs_location_data_1,
                                        rcs_direction_data_1,
                                        [0.4, 0.2, 0.4],
                                        [0.9, 1.1, 0.],
                                        True)
