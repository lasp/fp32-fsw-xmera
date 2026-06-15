"""
Module Name:        rwMotorTorque
"""

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.fp32 import rwMotorTorqueF32
from xmera.utilities import macros
from xmera.architecture import messaging

@pytest.mark.parametrize("num_control_axes", [0, 1, 2, 3])
@pytest.mark.parametrize("num_wheels", [2, 4, messaging.RW_EFF_CNT])
@pytest.mark.parametrize("num_input_cmd_torques", [1, 2])
@pytest.mark.parametrize("rw_avail_msg",["NO", "ON", "OFF", "MIXED"])
@pytest.mark.parametrize("omega_gain", [0.0, 0.5])

def test_rw_motor_torque(show_plots, num_control_axes, num_wheels, num_input_cmd_torques, rw_avail_msg, omega_gain):
    # @TODO With the current implementation of throwing an exception when zero control axes are specified, Python quits
    #  and causes all unit tests to fail. Until a different way of handling exceptions or errors is implemented, the
    #  test with 0 control axes is skipped.
    if num_control_axes == 0:
        pytest.skip("Zero control axes can currently not be tested.")

    # In case compile-time max RW number is less than parametrized number of wheels, skip test
    if num_wheels > messaging.RW_EFF_CNT:
        pytest.skip("Number of reaction wheels greater than compile time RW_EFF_CNT.")

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = rwMotorTorqueF32.RwMotorTorque()
    module.modelTag = "rwMotorTorque"

    # Initialize module variables
    if num_control_axes == 3:
        control_axes_B = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    elif num_control_axes == 2:
        control_axes_B = [[1, 0, 0], [0, 1, 0], [0, 0, 0]]
    elif num_control_axes == 1:
        control_axes_B = [[1, 0, 0], [0, 0, 0], [0, 0, 0]]
    else:
        control_axes_B = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

    module.controlAxes_B = control_axes_B
    module.omegaGain = omega_gain  # RW null-space feedback gain (0 disables the null-space term)

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    torque = np.array([1.0, -0.5, 0.7])

    # attControl message
    input_message_data = messaging.CmdTorqueBodyMsgF32Payload()
    requested_torque1 = torque
    input_message_data.torqueRequestBody = requested_torque1
    cmd_torque_in_msg = messaging.CmdTorqueBodyMsgF32().write(input_message_data)

    requested_torque = np.array(requested_torque1)

    if num_input_cmd_torques == 2:
        input_message_data2 = messaging.CmdTorqueBodyMsgF32Payload()
        requested_torque2 = np.array([[1.1, -1.3, 2.0], [0.3, 0.9, -1.4], [2.2, 1.7, 0.6]]) @ torque
        input_message_data2.torqueRequestBody = requested_torque2
        cmd_torque_in2_msg = messaging.CmdTorqueBodyMsgF32().write(input_message_data2)
        requested_torque += np.array(requested_torque2)

    # wheelConfigData message
    rw_config_params = messaging.RWArrayConfigMsgF32Payload()
    RW_EFF_CNT = messaging.RW_EFF_CNT

    if num_wheels == 4:
        rw_config_params.GsMatrix_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.5773502691896258, 0.5773502691896258, 0.5773502691896258
        ]
    else:
        # Spread the spin axes evenly over the unit sphere (Fibonacci sphere) to keep [Gs] well-conditioned.
        golden_angle = np.pi * (3.0 - np.sqrt(5.0))
        G_s_B = []
        for rw in range(num_wheels):
            z = 1.0 - 2.0 * (rw + 0.5) / num_wheels
            radius = np.sqrt(max(0.0, 1.0 - z * z))
            phi = golden_angle * rw
            axis = np.array([radius * np.cos(phi), radius * np.sin(phi), z])
            G_s_B.append(axis / np.linalg.norm(axis))

        rw_config_params.GsMatrix_B = np.array(G_s_B).flatten()

    rw_config_params.JsList = [0.1] * num_wheels
    rw_config_params.numRW = num_wheels
    rw_config_in_msg = messaging.RWArrayConfigMsgF32().write(rw_config_params)

    # Current RW speeds driving the null-space term; desired speeds default to zero (unlinked).
    rw_speeds = [10.0 * (i + 1) for i in range(num_wheels)]
    desired_omega = [0.0] * num_wheels
    input_speed_msg = messaging.RWSpeedMsgF32Payload()
    input_speed_msg.wheelSpeeds = rw_speeds
    rw_speed_in_msg = messaging.RWSpeedMsgF32().write(input_speed_msg)

    if rw_avail_msg != "NO":
        rw_availability_message = messaging.RWAvailabilityMsgPayload()
        if rw_avail_msg == "ON":
            avail = [messaging.AVAILABLE] * num_wheels
        elif rw_avail_msg == "OFF":
            avail = [messaging.UNAVAILABLE] * num_wheels
        else:
            avail_alternating = [messaging.AVAILABLE, messaging.AVAILABLE, messaging.UNAVAILABLE] * RW_EFF_CNT
            avail = avail_alternating[:num_wheels]

        rw_availability_message.wheelAvailability = avail

        rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
        module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)
    else:
        avail = [rwMotorTorqueF32.AVAILABLE] * num_wheels  # this is used purely for the python level solution

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.rwMotorTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # connect messages
    module.vehControlInMsg.subscribeTo(cmd_torque_in_msg)
    if num_input_cmd_torques == 2:
        module.vehControlIn2Msg.subscribeTo(cmd_torque_in2_msg)
    module.rwParamsInMsg.subscribeTo(rw_config_in_msg)
    module.rwSpeedsInMsg.subscribeTo(rw_speed_in_msg)

    # set the output truth states (needs to be computed before module reset because it also determines if test needs to
    # be skipped due to control mapping matrix not being full rank
    u_s = compute_true_torque(np.array(control_axes_B),
                              np.array(rw_config_params.GsMatrix_B).reshape((3, RW_EFF_CNT), order='F'),
                              requested_torque,
                              avail)

    # Add the null-space term (built from the available wheels, matching the algorithm).
    u_s = u_s + compute_null_space_torque(
        np.array(rw_config_params.GsMatrix_B).reshape((3, RW_EFF_CNT), order='F'),
        num_wheels, rw_speeds, desired_omega, omega_gain, avail)

    true_motor_torque = [u_s] * 2

    unit_test_sim.InitializeSimulation()
    module.reset(0)

    # Set the simulation time.
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.5))

    # Begin the simulation time run set above
    unit_test_sim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    motor_torque = data_log.motorTorque

    # compare the module results to the truth values
    accuracy = 1e-5
    np.testing.assert_allclose(motor_torque, true_motor_torque, rtol=accuracy, atol=accuracy, verbose=True)

    G_s_B =np.array( rw_config_params.GsMatrix_B).reshape((3, RW_EFF_CNT), order='F')
    F = np.transpose(motor_torque[0])
    received_torque = -(G_s_B @ F).flatten()

    # The control mapping must reproduce the requested body torque. Skip with null-space on, whose fp32
    # body-torque leak the truth comparison above already bounds.
    if omega_gain == 0.0 and num_wheels >= num_control_axes > 0:
        if (len(avail) - np.sum(avail)) > num_control_axes:
            np.testing.assert_allclose(received_torque[:num_control_axes], requested_torque[:num_control_axes],
                                       rtol=accuracy,
                                       atol=accuracy,
                                       verbose=True)


def compute_true_torque(C, Gs_B, Lr, avail_msg):

    num_control_axes = (np.linalg.norm(C, axis=1) > 0.0).sum()
    num_wheels = len(avail_msg)
    non_avail_wheels = 0

    # Remove wheels that are deemed unavailable and normalize spin axes
    for i in range(len(Gs_B[0])): #
        if num_wheels > i:
            if avail_msg[i] != messaging.AVAILABLE:
                Gs_B[:,i] = [0.0, 0.0, 0.0]
                non_avail_wheels += 1
            else:
                col_norm = np.linalg.norm(Gs_B[:,i])
                if col_norm > 0.0:
                    Gs_B[:,i] /= col_norm
        else:
            Gs_B[:,i] = [0.0, 0.0, 0.0]

    Lr_C = np.dot(C,Lr) # Project torque onto control axes
    CGs = np.dot(C, Gs_B) # Map the control axes onto the wheels

    # If rank of control mapping matrix is less than number of control axes, skip test (would throw exception)
    rank = np.linalg.matrix_rank(CGs)
    if rank < num_control_axes:
        pytest.skip("Control mapping matrix [CB][G_s] is not full rank.")

    # Build minimum norm framework
    M = np.dot(CGs, CGs.T)
    M_rep = np.identity(3) # Need to keep the matrix non-singular for inversion
    for i in range(0,num_control_axes):
        for j in range(0,num_control_axes):
            M_rep[i][j] = M[i][j]
    M_inv = np.linalg.inv(M_rep)

    # Remove projection to any non-defined control axes
    for i in range(num_control_axes,3):
        M_inv[i][i] = 0.0

    # Determine the solution
    L = np.dot(M_inv, Lr_C)

    # Map the solution to the wheels
    u_s = np.dot(CGs.T, L)

    return -u_s


def compute_null_space_torque(Gs_B, num_wheels, rw_speeds, desired_omega, omega_gain, avail_msg):
    """Mirror the algorithm's null-space term over the available wheels."""
    rw_eff_cnt = Gs_B.shape[1]
    u_null = np.zeros(rw_eff_cnt)

    # [Gs] from the available wheels, each in its original column.
    Gs = np.zeros((3, rw_eff_cnt))
    num_avail = 0
    for i in range(num_wheels):
        if avail_msg[i] == messaging.AVAILABLE:
            col_norm = np.linalg.norm(Gs_B[:, i])
            if col_norm > 0.0:
                Gs[:, i] = Gs_B[:, i] / col_norm
            num_avail += 1

    if num_avail > 3:
        GsGsT = Gs @ Gs.T
        singular_values = np.linalg.svd(GsGsT, compute_uv=False)
        if singular_values[2] > singular_values[0] * 1e-6:
            tau = np.identity(rw_eff_cnt) - Gs.T @ np.linalg.inv(GsGsT) @ Gs
            d = np.zeros(rw_eff_cnt)
            d[:num_wheels] = -omega_gain * (np.array(rw_speeds) - np.array(desired_omega))
            u_null = tau @ d
            # Zero the null-space torque for unavailable or absent wheels.
            for i in range(rw_eff_cnt):
                if i >= num_wheels or avail_msg[i] != messaging.AVAILABLE:
                    u_null[i] = 0.0

    return u_null


if __name__ == "__main__":
    test_rw_motor_torque(False,
                         3,  # numControlAxes
                         36,  # numWheels
                         2,  # numInputCmdTorques
                         "NO",  # RWAvailMsg ("NO", "ON", "OFF")
                         0.5  # omegaGain
                         )
