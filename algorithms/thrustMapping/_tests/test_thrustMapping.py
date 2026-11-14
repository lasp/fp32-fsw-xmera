import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.architecture.messaging import (
    THRArrayAvailabilityMsgF32,
    THRArrayAvailabilityMsgF32Payload,
    THRArrayConfigMsgF32,
    THRArrayConfigMsgF32Payload,
)
from xmera.fp32 import thrustMappingF32
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
                       [1, 1, 1],
                       [1, 1, 1],
                       [1, 1, -1],
                       [1, 1, -1],
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

rcs_direction_data_2 = [[0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [0.0, 0.0, -1.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]]

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

# Per-layout axis selections for desiredControlAxes_B. Only the selected rows of DG enter the solve,
# and each selected axis must be controllable. Layout 1's directions all lie in the body x-y plane, so
# torque xyz + force xy are controllable but force_z is not. Layout 2 and the random layout are
# full-rank, so they select every axis.
desired_control_axes_layout_1 = [True, True, True, True, True, False]
desired_control_axes_layout_2 = [True, True, True, True, True, True]
desired_control_axes_all = [True, True, True, True, True, True]


@pytest.mark.parametrize("rcs_location, rcs_direction, requested_torque, requested_force, "
                         "torque_in_msg_flag, desired_control_axes",
                         [(rcs_location_data_1, rcs_direction_data_1, [0.4, 0.2, 0.4], [0.9, 1.1, 0.], True,
                           desired_control_axes_layout_1),
                          (rcs_location_data_1, rcs_direction_data_1, [0.0, 0.0, 0.0], [0.9, 1.1, 0.], True,
                           desired_control_axes_layout_1),
                          (rcs_location_data_1, rcs_direction_data_1, [0.0, 0.0, 0.0], [0.9, 1.1, 0.], False,
                           desired_control_axes_layout_1),
                          (rcs_location_data_2, rcs_direction_data_2, [0.0, 0.0, 0.0], [0.9, 1.1, 1.], True,
                           desired_control_axes_layout_2),
                          (rcs_location_data_rand, rcs_direction_data_rand, torque_rand, force_rand, True,
                           desired_control_axes_all)])

def test_thrust_mapping(rcs_location, rcs_direction, requested_torque, requested_force,
                                        torque_in_msg_flag, desired_control_axes):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # setup module to be tested
    module = thrustMappingF32.ThrustMapping()
    module.modelTag = "thrustMappingTag"
    module.desiredControlAxes_B = desired_control_axes
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

    truth = compute_thrust_mapping_truth(rcs_location, rcs_direction, requested_torque, requested_force, CoM_B,
                                         desired_control_axes)

    accuracy = 1e-5
    np.testing.assert_allclose(np.array([module.thrForceCmdOutMsg.read().thrForce[0:len(rcs_location)]]).flatten(), truth,
                               atol=accuracy, rtol=accuracy, verbose=True)


def compute_thrust_mapping_truth(rcs_location, rcs_direction, requested_torque, requested_force, CoM_B,
                                 desired_control_axes, thruster_availability=None):
    """Independent fp64 truth that mirrors the algorithm's truncated-SVD pseudo-inverse exactly.

    Three details must match the algorithm so the only remaining disagreement is fp32 round-off:
      1. DG has the same shape (6 x MAX_EFF_CNT, trailing zero columns) as the algorithm's matrix,
         so the SVD operates on the same operator.
      2. The truncation cutoff uses fp32 epsilon scaled by max(6, MAX_EFF_CNT) — the algorithm's
         noise floor — instead of fp64 epsilon. Otherwise the truth would keep singular values in
         the [eps_d, eps_f] gap that the algorithm correctly drops as fp32 noise, and 1/sv would
         blow up.
      3. The rows of the axes outside desired_control_axes are zeroed, as the algorithm zeroes them,
         so both solve the same reduced problem.
      4. A null-space shift removes the negative entries, as the algorithm does. The reference does
         not subtract the minimum.
      5. An unavailable thruster keeps a zero column of DG, as the algorithm keeps one. Its force is
         thus always zero, and the solve uses only the thrusters that remain.
    """
    num_thrusters = len(rcs_location)
    max_eff_cnt = messaging.MAX_EFF_CNT
    ft = np.concatenate([requested_torque, requested_force]).astype(np.float64)
    CoM_B = np.array(CoM_B, dtype=np.float64)

    if thruster_availability is None:
        thruster_availability = [True] * num_thrusters

    DG = np.zeros((6, max_eff_cnt), dtype=np.float64)
    for i in range(num_thrusters):
        if not thruster_availability[i]:
            continue
        r = np.array(rcs_location[i], dtype=np.float64)
        g = np.array(rcs_direction[i], dtype=np.float64)
        DG[0:3, i] = np.cross(r - CoM_B, g)
        DG[3:6, i] = g
    for axis, selected in enumerate(desired_control_axes):
        if not selected:
            DG[axis, :] = 0.0

    U, sv, Vt = np.linalg.svd(DG, full_matrices=False)
    eps_f = np.finfo(np.float32).eps
    tol = sv[0] * eps_f * max(6, max_eff_cnt)
    inv_sv = np.divide(1.0, sv, out=np.zeros_like(sv), where=sv > tol)
    thr_forces = Vt.T @ np.diag(inv_sv) @ U.T @ ft

    # Shift direction: the part of the all-ones vector lying in the null space of the kept row space.
    # Null space dimension is num_thrusters - rank. With no dimension, there is no shift.
    rank = int(np.count_nonzero(sv > tol))
    num_available = int(sum(1 for i in range(num_thrusters) if thruster_availability[i]))
    ones = np.zeros(max_eff_cnt, dtype=np.float64)
    for i in range(num_thrusters):
        if thruster_availability[i]:
            ones[i] = 1.0
    null_shift = np.zeros(max_eff_cnt, dtype=np.float64)
    if rank < num_available:
        null_shift = ones.copy()
        row_space = Vt.T[:, 0:rank]
        null_shift -= row_space @ (row_space.T @ ones)
        null_shift[num_thrusters:] = 0.0

    # Largest per-entry step, over the entries the shift reaches.
    null_space_tol = 1e-6
    shift_reach_tol = 1e-1
    shift_scale = np.abs(null_shift[0:num_thrusters]).max()
    if shift_scale > null_space_tol:
        step = 0.0
        for j in range(num_thrusters):
            if null_shift[j] > shift_reach_tol * shift_scale:
                step = max(step, -thr_forces[j] / null_shift[j])
        thr_forces[0:num_thrusters] += step * null_shift[0:num_thrusters]

    return np.maximum(thr_forces[0:num_thrusters], 0.0)

r"""
Test 6: An unavailable thruster takes no part in the mapping. The module commands it zero thrust, and
        it re-solves the allocation across the thrusters that remain.
"""


@pytest.mark.parametrize("dead_thruster", [0, 3, 7])
def test_thrust_mapping_availability(dead_thruster):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = thrustMappingF32.ThrustMapping()
    module.modelTag = "thrustMappingTag"
    module.desiredControlAxes_B = desired_control_axes_layout_1
    unit_test_sim.AddModelToTask(unit_task_name, module)

    requested_torque = [0.4, 0.2, 0.4]
    requested_force = [0.9, 1.1, 0.0]

    cmd_torque_in_msg_data = messaging.CmdTorqueBodyMsgF32Payload()
    cmd_torque_in_msg_data.torqueRequestBody = requested_torque
    cmd_torque_in_msg = messaging.CmdTorqueBodyMsgF32().write(cmd_torque_in_msg_data)

    cmd_force_in_msg_data = messaging.CmdForceBodyMsgF32Payload()
    cmd_force_in_msg_data.forceRequestBody = requested_force
    cmd_force_in_msg = messaging.CmdForceBodyMsgF32().write(cmd_force_in_msg_data)

    num_thrusters = len(rcs_location_data_1)
    thr_config_payload = THRArrayConfigMsgF32Payload()
    thr_config_payload.numThrusters = num_thrusters
    for i in range(num_thrusters):
        thr_config_payload.thrusters[i].rThrust_B = rcs_location_data_1[i]
        thr_config_payload.thrusters[i].tHatThrust_B = rcs_direction_data_1[i]
        thr_config_payload.thrusters[i].maxThrust = 3.0
    thr_config_in_msg = THRArrayConfigMsgF32().write(thr_config_payload)

    availability_payload = THRArrayAvailabilityMsgF32Payload()
    availability = [messaging.DEVICE_AVAILABLE] * messaging.MAX_EFF_CNT
    availability[dead_thruster] = messaging.DEVICE_UNAVAILABLE
    availability_payload.thrusterAvailability = availability
    thr_avail_in_msg = THRArrayAvailabilityMsgF32().write(availability_payload)

    CoM_B = np.array([0.1, 0.1, 0.1])
    veh_config_in_msg_data = messaging.VehicleConfigMsgF32Payload()
    veh_config_in_msg_data.CoM_B = CoM_B
    veh_config_in_msg = messaging.VehicleConfigMsgF32().write(veh_config_in_msg_data)

    module.cmdTorqueInMsg.subscribeTo(cmd_torque_in_msg)
    module.cmdForceInMsg.subscribeTo(cmd_force_in_msg)
    module.thrConfigInMsg.subscribeTo(thr_config_in_msg)
    module.thrAvailInMsg.subscribeTo(thr_avail_in_msg)
    module.vehConfigInMsg.subscribeTo(veh_config_in_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.5))
    unit_test_sim.ExecuteSimulation()

    thr_force = np.array(module.thrForceCmdOutMsg.read().thrForce[0:num_thrusters])

    # The unavailable thruster is never commanded, and no command is negative.
    np.testing.assert_allclose(thr_force[dead_thruster], 0.0, atol=1e-6, rtol=1e-6)
    assert np.all(thr_force >= -1e-6)

    # The truth mirrors the algorithm: the unavailable thruster keeps a zero column of DG.
    availability_mask = [i != dead_thruster for i in range(num_thrusters)]
    truth = compute_thrust_mapping_truth(rcs_location_data_1,
                                         rcs_direction_data_1,
                                         requested_torque,
                                         requested_force,
                                         CoM_B,
                                         desired_control_axes_layout_1,
                                         availability_mask)
    accuracy = 1e-5
    np.testing.assert_allclose(thr_force, truth, atol=accuracy, rtol=accuracy, verbose=True)


if __name__ == "__main__":
    test_thrust_mapping(rcs_location_data_1,
                                        rcs_direction_data_1,
                                        [0.4, 0.2, 0.4],
                                        [0.9, 1.1, 0.],
                                        True,
                                        desired_control_axes_layout_1)
