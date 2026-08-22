import inspect
import os
import math
import numpy as np
import pandas as pd
import pytest
from xmera.architecture import messaging
from xmera.fp32 import gimbalAnglesToMotorAnglesF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

motor_angle_home = 103.2242 * macros.D2R

def load_interpolation_tables():
    gimbal_to_motor_1_angle_data = pd.read_csv(os.path.join(path, "gimbalToMotor1Angles.csv"), header=None)
    gimbal_to_motor_2_angle_data = pd.read_csv(os.path.join(path, "gimbalToMotor2Angles.csv"), header=None)
    row_start_stride_indices_data = pd.read_csv(os.path.join(path, "rowStartStrideIndices.csv"), header=None)
    row_start_col_indices_data = pd.read_csv(os.path.join(path, "rowStartColIndices.csv"), header=None)

    gimbal_to_motor_1_angle_data_array = np.deg2rad(gimbal_to_motor_1_angle_data.to_numpy().flatten()).tolist()
    gimbal_to_motor_2_angle_data_array = np.deg2rad(gimbal_to_motor_2_angle_data.to_numpy().flatten()).tolist()
    row_start_stride_indices_data_array = [int(idx) for idx in row_start_stride_indices_data.to_numpy().flatten()]
    row_start_col_indices_data_array = [int(idx) for idx in row_start_col_indices_data.to_numpy().flatten()]

    return (gimbal_to_motor_1_angle_data_array,
            gimbal_to_motor_2_angle_data_array,
            row_start_stride_indices_data_array,
            row_start_col_indices_data_array)

@pytest.mark.parametrize("gimbal_angle_1_ref", [-12.0 * macros.D2R])
@pytest.mark.parametrize("gimbal_angle_2_ref", [-10.0 * macros.D2R])
def test_no_interpolation_outside_boundary_returns_home(gimbal_angle_1_ref, gimbal_angle_2_ref):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Load the interpolation table data
    (gimbal_to_motor_1_angle_data_array,
     gimbal_to_motor_2_angle_data_array,
     row_start_stride_indices_data_array,
     row_start_col_indices_data_array) = load_interpolation_tables()

    gimbal_to_motor_angles = gimbalAnglesToMotorAnglesF32.GimbalAnglesToMotorAngles()
    gimbal_to_motor_angles.modelTag = "gimbalAnglesToMotorAngles"
    gimbal_to_motor_angles.minAngle = 0.0
    gimbal_to_motor_angles.maxAngle = 2 * math.pi
    gimbal_to_motor_angles.gimbalToMotor1AngleData = gimbal_to_motor_1_angle_data_array
    gimbal_to_motor_angles.gimbalToMotor2AngleData = gimbal_to_motor_2_angle_data_array
    gimbal_to_motor_angles.rowStartStrideIndices = row_start_stride_indices_data_array
    gimbal_to_motor_angles.rowStartColIndices = row_start_col_indices_data_array
    gimbal_to_motor_angles.tipColIdxOffset = 37
    gimbal_to_motor_angles.tiltRowIdxOffset = 54
    gimbal_to_motor_angles.tableStepAngle = 0.5 * macros.D2R
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_to_motor_angles)

    # Create gimbalAnglesToMotorAngles gimbal reference input message
    gimbal_reference_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_reference_message_data.theta1 = gimbal_angle_1_ref
    gimbal_reference_message_data.theta2 = gimbal_angle_2_ref
    gimbal_reference_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_reference_message_data)
    gimbal_to_motor_angles.twoAxisGimbalInMsg.subscribeTo(gimbal_reference_message)

    # Set up data logging for the module output
    motor_1_angle_data = gimbal_to_motor_angles.motor1AngleOutMsg.recorder()
    motor_2_angle_data = gimbal_to_motor_angles.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_angle_data)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_angle_data)

    # Run the simulation
    sim_time = 1.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    motor_1_angles_sim = motor_1_angle_data.theta  # [rad]
    motor_2_angles_sim = motor_2_angle_data.theta  # [rad]

    np.testing.assert_allclose(motor_angle_home, motor_1_angles_sim[-1], atol=1e-5, verbose=True)
    np.testing.assert_allclose(motor_angle_home, motor_2_angles_sim[-1], atol=1e-5, verbose=True)

@pytest.mark.parametrize("gimbal_angle_1_ref", [5.0 * macros.D2R])
@pytest.mark.parametrize("gimbal_angle_2_ref", [-21.3 * macros.D2R])
def test_linear_interpolation_outside_boundary_returns_home(gimbal_angle_1_ref, gimbal_angle_2_ref):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Load the interpolation table data
    (gimbal_to_motor_1_angle_data_array,
     gimbal_to_motor_2_angle_data_array,
     row_start_stride_indices_data_array,
     row_start_col_indices_data_array) = load_interpolation_tables()

    gimbal_to_motor_angles = gimbalAnglesToMotorAnglesF32.GimbalAnglesToMotorAngles()
    gimbal_to_motor_angles.modelTag = "gimbalAnglesToMotorAngles"
    gimbal_to_motor_angles.minAngle = 0.0
    gimbal_to_motor_angles.maxAngle = 2 * math.pi
    gimbal_to_motor_angles.gimbalToMotor1AngleData = gimbal_to_motor_1_angle_data_array
    gimbal_to_motor_angles.gimbalToMotor2AngleData = gimbal_to_motor_2_angle_data_array
    gimbal_to_motor_angles.rowStartStrideIndices = row_start_stride_indices_data_array
    gimbal_to_motor_angles.rowStartColIndices = row_start_col_indices_data_array
    gimbal_to_motor_angles.tipColIdxOffset = 37
    gimbal_to_motor_angles.tiltRowIdxOffset = 54
    gimbal_to_motor_angles.tableStepAngle = 0.5 * macros.D2R
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_to_motor_angles)

    # Create gimbalAnglesToMotorAngles gimbal reference input message
    gimbal_reference_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_reference_message_data.theta1 = gimbal_angle_1_ref
    gimbal_reference_message_data.theta2 = gimbal_angle_2_ref
    gimbal_reference_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_reference_message_data)
    gimbal_to_motor_angles.twoAxisGimbalInMsg.subscribeTo(gimbal_reference_message)

    # Set up data logging for the module output
    motor_1_angle_data = gimbal_to_motor_angles.motor1AngleOutMsg.recorder()
    motor_2_angle_data = gimbal_to_motor_angles.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_angle_data)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_angle_data)

    # Run the simulation
    sim_time = 1.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    motor_1_angles_sim = motor_1_angle_data.theta  # [rad]
    motor_2_angles_sim = motor_2_angle_data.theta  # [rad]

    np.testing.assert_allclose(motor_angle_home, motor_1_angles_sim[-1], atol=1e-5, verbose=True)
    np.testing.assert_allclose(motor_angle_home, motor_2_angles_sim[-1], atol=1e-5, verbose=True)

@pytest.mark.parametrize("gimbal_angle_1_ref", [8.37 * macros.D2R])
@pytest.mark.parametrize("gimbal_angle_2_ref", [16.63 * macros.D2R])
def test_bilinear_interpolation_outside_boundary_returns_home(gimbal_angle_1_ref, gimbal_angle_2_ref):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Load the interpolation table data
    (gimbal_to_motor_1_angle_data_array,
     gimbal_to_motor_2_angle_data_array,
     row_start_stride_indices_data_array,
     row_start_col_indices_data_array) = load_interpolation_tables()

    gimbal_to_motor_angles = gimbalAnglesToMotorAnglesF32.GimbalAnglesToMotorAngles()
    gimbal_to_motor_angles.modelTag = "gimbalAnglesToMotorAngles"
    gimbal_to_motor_angles.minAngle = 0.0
    gimbal_to_motor_angles.maxAngle = 2 * math.pi
    gimbal_to_motor_angles.gimbalToMotor1AngleData = gimbal_to_motor_1_angle_data_array
    gimbal_to_motor_angles.gimbalToMotor2AngleData = gimbal_to_motor_2_angle_data_array
    gimbal_to_motor_angles.rowStartStrideIndices = row_start_stride_indices_data_array
    gimbal_to_motor_angles.rowStartColIndices = row_start_col_indices_data_array
    gimbal_to_motor_angles.tipColIdxOffset = 37
    gimbal_to_motor_angles.tiltRowIdxOffset = 54
    gimbal_to_motor_angles.tableStepAngle = 0.5 * macros.D2R
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_to_motor_angles)

    # Create gimbalAnglesToMotorAngles gimbal reference input message
    gimbal_reference_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_reference_message_data.theta1 = gimbal_angle_1_ref
    gimbal_reference_message_data.theta2 = gimbal_angle_2_ref
    gimbal_reference_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_reference_message_data)
    gimbal_to_motor_angles.twoAxisGimbalInMsg.subscribeTo(gimbal_reference_message)

    # Set up data logging for the module output
    motor_1_angle_data = gimbal_to_motor_angles.motor1AngleOutMsg.recorder()
    motor_2_angle_data = gimbal_to_motor_angles.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_angle_data)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_angle_data)

    # Run the simulation
    sim_time = 1.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    motor_1_angles_sim = motor_1_angle_data.theta  # [rad]
    motor_2_angles_sim = motor_2_angle_data.theta  # [rad]

    np.testing.assert_allclose(motor_angle_home, motor_1_angles_sim[-1], atol=1e-5, verbose=True)
    np.testing.assert_allclose(motor_angle_home, motor_2_angles_sim[-1], atol=1e-5, verbose=True)

@pytest.mark.parametrize("gimbal_angle_1_ref", [-0.71 * macros.D2R])
@pytest.mark.parametrize("gimbal_angle_2_ref", [25.5 * macros.D2R])
def test_linear_interpolation_along_boundary_returns_home(gimbal_angle_1_ref, gimbal_angle_2_ref):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Load the interpolation table data
    (gimbal_to_motor_1_angle_data_array,
     gimbal_to_motor_2_angle_data_array,
     row_start_stride_indices_data_array,
     row_start_col_indices_data_array) = load_interpolation_tables()

    gimbal_to_motor_angles = gimbalAnglesToMotorAnglesF32.GimbalAnglesToMotorAngles()
    gimbal_to_motor_angles.modelTag = "gimbalAnglesToMotorAngles"
    gimbal_to_motor_angles.minAngle = 0.0
    gimbal_to_motor_angles.maxAngle = 2 * math.pi
    gimbal_to_motor_angles.gimbalToMotor1AngleData = gimbal_to_motor_1_angle_data_array
    gimbal_to_motor_angles.gimbalToMotor2AngleData = gimbal_to_motor_2_angle_data_array
    gimbal_to_motor_angles.rowStartStrideIndices = row_start_stride_indices_data_array
    gimbal_to_motor_angles.rowStartColIndices = row_start_col_indices_data_array
    gimbal_to_motor_angles.tipColIdxOffset = 37
    gimbal_to_motor_angles.tiltRowIdxOffset = 54
    gimbal_to_motor_angles.tableStepAngle = 0.5 * macros.D2R
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_to_motor_angles)

    # Create gimbalAnglesToMotorAngles gimbal reference input message
    gimbal_reference_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_reference_message_data.theta1 = gimbal_angle_1_ref
    gimbal_reference_message_data.theta2 = gimbal_angle_2_ref
    gimbal_reference_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_reference_message_data)
    gimbal_to_motor_angles.twoAxisGimbalInMsg.subscribeTo(gimbal_reference_message)

    # Set up data logging for the module output
    motor_1_angle_data = gimbal_to_motor_angles.motor1AngleOutMsg.recorder()
    motor_2_angle_data = gimbal_to_motor_angles.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_angle_data)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_angle_data)

    # Run the simulation
    sim_time = 1.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    motor_1_angles_sim = motor_1_angle_data.theta  # [rad]
    motor_2_angles_sim = motor_2_angle_data.theta  # [rad]

    np.testing.assert_allclose(motor_angle_home, motor_1_angles_sim[-1], atol=1e-5, verbose=True)
    np.testing.assert_allclose(motor_angle_home, motor_2_angles_sim[-1], atol=1e-5, verbose=True)

@pytest.mark.parametrize("gimbal_angle_1_ref", [1.61 * macros.D2R])
@pytest.mark.parametrize("gimbal_angle_2_ref", [25.81 * macros.D2R])
def test_bilinear_interpolation_along_boundary_returns_home(gimbal_angle_1_ref, gimbal_angle_2_ref):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Load the interpolation table data
    (gimbal_to_motor_1_angle_data_array,
     gimbal_to_motor_2_angle_data_array,
     row_start_stride_indices_data_array,
     row_start_col_indices_data_array) = load_interpolation_tables()

    gimbal_to_motor_angles = gimbalAnglesToMotorAnglesF32.GimbalAnglesToMotorAngles()
    gimbal_to_motor_angles.modelTag = "gimbalAnglesToMotorAngles"
    gimbal_to_motor_angles.minAngle = 0.0
    gimbal_to_motor_angles.maxAngle = 2 * math.pi
    gimbal_to_motor_angles.gimbalToMotor1AngleData = gimbal_to_motor_1_angle_data_array
    gimbal_to_motor_angles.gimbalToMotor2AngleData = gimbal_to_motor_2_angle_data_array
    gimbal_to_motor_angles.rowStartStrideIndices = row_start_stride_indices_data_array
    gimbal_to_motor_angles.rowStartColIndices = row_start_col_indices_data_array
    gimbal_to_motor_angles.tipColIdxOffset = 37
    gimbal_to_motor_angles.tiltRowIdxOffset = 54
    gimbal_to_motor_angles.tableStepAngle = 0.5 * macros.D2R
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_to_motor_angles)

    # Create gimbalAnglesToMotorAngles gimbal reference input message
    gimbal_reference_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_reference_message_data.theta1 = gimbal_angle_1_ref
    gimbal_reference_message_data.theta2 = gimbal_angle_2_ref
    gimbal_reference_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_reference_message_data)
    gimbal_to_motor_angles.twoAxisGimbalInMsg.subscribeTo(gimbal_reference_message)

    # Set up data logging for the module output
    motor_1_angle_data = gimbal_to_motor_angles.motor1AngleOutMsg.recorder()
    motor_2_angle_data = gimbal_to_motor_angles.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_angle_data)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_angle_data)

    # Run the simulation
    sim_time = 1.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    motor_1_angles_sim = motor_1_angle_data.theta  # [rad]
    motor_2_angles_sim = motor_2_angle_data.theta  # [rad]

    np.testing.assert_allclose(motor_angle_home, motor_1_angles_sim[-1], atol=1e-5, verbose=True)
    np.testing.assert_allclose(motor_angle_home, motor_2_angles_sim[-1], atol=1e-5, verbose=True)

@pytest.mark.parametrize("gimbal_angle_1_ref", [1.5 * macros.D2R])
@pytest.mark.parametrize("gimbal_angle_2_ref", [4.0 * macros.D2R])
def test_no_interpolation_inside_boundary_returns_exact(gimbal_angle_1_ref, gimbal_angle_2_ref):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Load the interpolation table data
    (gimbal_to_motor_1_angle_data_array,
     gimbal_to_motor_2_angle_data_array,
     row_start_stride_indices_data_array,
     row_start_col_indices_data_array) = load_interpolation_tables()

    gimbal_to_motor_angles = gimbalAnglesToMotorAnglesF32.GimbalAnglesToMotorAngles()
    gimbal_to_motor_angles.modelTag = "gimbalAnglesToMotorAngles"
    gimbal_to_motor_angles.minAngle = 0.0
    gimbal_to_motor_angles.maxAngle = 2 * math.pi
    gimbal_to_motor_angles.gimbalToMotor1AngleData = gimbal_to_motor_1_angle_data_array
    gimbal_to_motor_angles.gimbalToMotor2AngleData = gimbal_to_motor_2_angle_data_array
    gimbal_to_motor_angles.rowStartStrideIndices = row_start_stride_indices_data_array
    gimbal_to_motor_angles.rowStartColIndices = row_start_col_indices_data_array
    gimbal_to_motor_angles.tipColIdxOffset = 37
    gimbal_to_motor_angles.tiltRowIdxOffset = 54
    gimbal_to_motor_angles.tableStepAngle = 0.5 * macros.D2R
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_to_motor_angles)

    # Create gimbalAnglesToMotorAngles gimbal reference input message
    gimbal_reference_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_reference_message_data.theta1 = gimbal_angle_1_ref
    gimbal_reference_message_data.theta2 = gimbal_angle_2_ref
    gimbal_reference_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_reference_message_data)
    gimbal_to_motor_angles.twoAxisGimbalInMsg.subscribeTo(gimbal_reference_message)

    # Set up data logging for the module output
    motor_1_angle_data = gimbal_to_motor_angles.motor1AngleOutMsg.recorder()
    motor_2_angle_data = gimbal_to_motor_angles.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_angle_data)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_angle_data)

    # Run the simulation
    sim_time = 1.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    motor_1_angles_sim = motor_1_angle_data.theta  # [rad]
    motor_2_angles_sim = motor_2_angle_data.theta  # [rad]

    # Truth data
    motor_1_angle_truth = 84.06074422 * macros.D2R
    motor_2_angle_truth = 72.42640668 * macros.D2R

    np.testing.assert_allclose(motor_1_angle_truth, motor_1_angles_sim[-1], atol=1e-5, verbose=True)
    np.testing.assert_allclose(motor_2_angle_truth, motor_2_angles_sim[-1], atol=1e-5, verbose=True)

@pytest.mark.parametrize("gimbal_angle_1_ref", [2.2 * macros.D2R])
@pytest.mark.parametrize("gimbal_angle_2_ref", [6.5 * macros.D2R])
def test_linear_interpolation_inside_boundary_along_tip_returns_exact(gimbal_angle_1_ref, gimbal_angle_2_ref):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Load the interpolation table data
    (gimbal_to_motor_1_angle_data_array,
     gimbal_to_motor_2_angle_data_array,
     row_start_stride_indices_data_array,
     row_start_col_indices_data_array) = load_interpolation_tables()

    gimbal_to_motor_angles = gimbalAnglesToMotorAnglesF32.GimbalAnglesToMotorAngles()
    gimbal_to_motor_angles.modelTag = "gimbalAnglesToMotorAngles"
    gimbal_to_motor_angles.minAngle = 0.0
    gimbal_to_motor_angles.maxAngle = 2 * math.pi
    gimbal_to_motor_angles.gimbalToMotor1AngleData = gimbal_to_motor_1_angle_data_array
    gimbal_to_motor_angles.gimbalToMotor2AngleData = gimbal_to_motor_2_angle_data_array
    gimbal_to_motor_angles.rowStartStrideIndices = row_start_stride_indices_data_array
    gimbal_to_motor_angles.rowStartColIndices = row_start_col_indices_data_array
    gimbal_to_motor_angles.tipColIdxOffset = 37
    gimbal_to_motor_angles.tiltRowIdxOffset = 54
    gimbal_to_motor_angles.tableStepAngle = 0.5 * macros.D2R
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_to_motor_angles)

    # Create gimbalAnglesToMotorAngles gimbal reference input message
    gimbal_reference_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_reference_message_data.theta1 = gimbal_angle_1_ref
    gimbal_reference_message_data.theta2 = gimbal_angle_2_ref
    gimbal_reference_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_reference_message_data)
    gimbal_to_motor_angles.twoAxisGimbalInMsg.subscribeTo(gimbal_reference_message)

    # Set up data logging for the module output
    motor_1_angle_data = gimbal_to_motor_angles.motor1AngleOutMsg.recorder()
    motor_2_angle_data = gimbal_to_motor_angles.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_angle_data)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_angle_data)

    # Run the simulation
    sim_time = 1.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    motor_1_angles_sim = motor_1_angle_data.theta  # [rad]
    motor_2_angles_sim = motor_2_angle_data.theta  # [rad]

    # Gimbal bounds
    gimbal_angle_1_lower_bound = 2.0 * macros.D2R
    gimbal_angle_1_upper_bound = 2.5 * macros.D2R

    # Motor bounds
    motor_1_lower_bound = 86.6590305 * macros.D2R
    motor_1_upper_bound = 85.59218715 * macros.D2R
    motor_2_lower_bound = 67.5806477 * macros.D2R
    motor_2_upper_bound = 66.39986032 * macros.D2R

    # Compute the truth values
    motor_1_angle_truth = linear_interpolation(gimbal_angle_1_lower_bound,
                                              gimbal_angle_1_upper_bound,
                                              motor_1_lower_bound,
                                              motor_1_upper_bound,
                                              gimbal_angle_1_ref)
    motor_2_angle_truth = linear_interpolation(gimbal_angle_1_lower_bound,
                                              gimbal_angle_1_upper_bound,
                                              motor_2_lower_bound,
                                              motor_2_upper_bound,
                                              gimbal_angle_1_ref)

    np.testing.assert_allclose(motor_1_angle_truth, motor_1_angles_sim[-1], atol=1e-5, verbose=True)
    np.testing.assert_allclose(motor_2_angle_truth, motor_2_angles_sim[-1], atol=1e-5, verbose=True)

@pytest.mark.parametrize("gimbal_angle_1_ref", [0.5 * macros.D2R])
@pytest.mark.parametrize("gimbal_angle_2_ref", [-2.7 * macros.D2R])
def test_linear_interpolation_inside_boundary_along_tilt_returns_exact(gimbal_angle_1_ref, gimbal_angle_2_ref):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Load the interpolation table data
    (gimbal_to_motor_1_angle_data_array,
     gimbal_to_motor_2_angle_data_array,
     row_start_stride_indices_data_array,
     row_start_col_indices_data_array) = load_interpolation_tables()

    gimbal_to_motor_angles = gimbalAnglesToMotorAnglesF32.GimbalAnglesToMotorAngles()
    gimbal_to_motor_angles.modelTag = "gimbalAnglesToMotorAngles"
    gimbal_to_motor_angles.minAngle = 0.0
    gimbal_to_motor_angles.maxAngle = 2 * math.pi
    gimbal_to_motor_angles.gimbalToMotor1AngleData = gimbal_to_motor_1_angle_data_array
    gimbal_to_motor_angles.gimbalToMotor2AngleData = gimbal_to_motor_2_angle_data_array
    gimbal_to_motor_angles.rowStartStrideIndices = row_start_stride_indices_data_array
    gimbal_to_motor_angles.rowStartColIndices = row_start_col_indices_data_array
    gimbal_to_motor_angles.tipColIdxOffset = 37
    gimbal_to_motor_angles.tiltRowIdxOffset = 54
    gimbal_to_motor_angles.tableStepAngle = 0.5 * macros.D2R
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_to_motor_angles)

    # Create gimbalAnglesToMotorAngles gimbal reference input message
    gimbal_reference_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_reference_message_data.theta1 = gimbal_angle_1_ref
    gimbal_reference_message_data.theta2 = gimbal_angle_2_ref
    gimbal_reference_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_reference_message_data)
    gimbal_to_motor_angles.twoAxisGimbalInMsg.subscribeTo(gimbal_reference_message)

    # Set up data logging for the module output
    motor_1_angle_data = gimbal_to_motor_angles.motor1AngleOutMsg.recorder()
    motor_2_angle_data = gimbal_to_motor_angles.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_angle_data)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_angle_data)

    # Run the simulation
    sim_time = 1.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    motor_1_angles_sim = motor_1_angle_data.theta  # [rad]
    motor_2_angles_sim = motor_2_angle_data.theta  # [rad]

    # Gimbal bounds
    gimbal_angle_2_lower_bound = -3.0 * macros.D2R
    gimbal_angle_2_upper_bound = -2.5 * macros.D2R

    # Motor bounds
    motor_1_lower_bound = 76.06429902 * macros.D2R
    motor_1_upper_bound = 76.78640286 * macros.D2R
    motor_2_lower_bound = 84.72342209 * macros.D2R
    motor_2_upper_bound = 83.99860858 * macros.D2R

    # Compute the truth values
    motor_1_angle_truth = linear_interpolation(gimbal_angle_2_lower_bound,
                                              gimbal_angle_2_upper_bound,
                                              motor_1_lower_bound,
                                              motor_1_upper_bound,
                                              gimbal_angle_2_ref)
    motor_2_angle_truth = linear_interpolation(gimbal_angle_2_lower_bound,
                                              gimbal_angle_2_upper_bound,
                                              motor_2_lower_bound,
                                              motor_2_upper_bound,
                                              gimbal_angle_2_ref)

    np.testing.assert_allclose(motor_1_angle_truth, motor_1_angles_sim[-1], atol=1e-5, verbose=True)
    np.testing.assert_allclose(motor_2_angle_truth, motor_2_angles_sim[-1], atol=1e-5, verbose=True)

@pytest.mark.parametrize("gimbal_angle_1_ref", [1.7 * macros.D2R])
@pytest.mark.parametrize("gimbal_angle_2_ref", [-2.1 * macros.D2R])
def test_bilinear_interpolation_inside_boundary_returns_exact(gimbal_angle_1_ref, gimbal_angle_2_ref):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Load the interpolation table data
    (gimbal_to_motor_1_angle_data_array,
     gimbal_to_motor_2_angle_data_array,
     row_start_stride_indices_data_array,
     row_start_col_indices_data_array) = load_interpolation_tables()

    gimbal_to_motor_angles = gimbalAnglesToMotorAnglesF32.GimbalAnglesToMotorAngles()
    gimbal_to_motor_angles.modelTag = "gimbalAnglesToMotorAngles"
    gimbal_to_motor_angles.minAngle = 0.0
    gimbal_to_motor_angles.maxAngle = 2 * math.pi
    gimbal_to_motor_angles.gimbalToMotor1AngleData = gimbal_to_motor_1_angle_data_array
    gimbal_to_motor_angles.gimbalToMotor2AngleData = gimbal_to_motor_2_angle_data_array
    gimbal_to_motor_angles.rowStartStrideIndices = row_start_stride_indices_data_array
    gimbal_to_motor_angles.rowStartColIndices = row_start_col_indices_data_array
    gimbal_to_motor_angles.tipColIdxOffset = 37
    gimbal_to_motor_angles.tiltRowIdxOffset = 54
    gimbal_to_motor_angles.tableStepAngle = 0.5 * macros.D2R
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_to_motor_angles)

    # Create gimbalAnglesToMotorAngles gimbal reference input message
    gimbal_reference_message_data = messaging.TwoAxisGimbalMsgF32Payload()
    gimbal_reference_message_data.theta1 = gimbal_angle_1_ref
    gimbal_reference_message_data.theta2 = gimbal_angle_2_ref
    gimbal_reference_message = messaging.TwoAxisGimbalMsgF32().write(gimbal_reference_message_data)
    gimbal_to_motor_angles.twoAxisGimbalInMsg.subscribeTo(gimbal_reference_message)

    # Set up data logging for the module output
    motor_1_angle_data = gimbal_to_motor_angles.motor1AngleOutMsg.recorder()
    motor_2_angle_data = gimbal_to_motor_angles.motor2AngleOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, motor_1_angle_data)
    unit_test_sim.AddModelToTask(unit_task_name, motor_2_angle_data)

    # Run the simulation
    sim_time = 1.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    motor_1_angles_sim = motor_1_angle_data.theta  # [rad]
    motor_2_angles_sim = motor_2_angle_data.theta  # [rad]

    # Gimbal bounds
    gimbal_angle_1_lower_bound = 1.5 * macros.D2R
    gimbal_angle_1_upper_bound = 2.0 * macros.D2R
    gimbal_angle_2_lower_bound = -2.5 * macros.D2R
    gimbal_angle_2_upper_bound = -2.0 * macros.D2R

    # Motor bounds
    motor_1_lower_lower_bound = 74.61881907 * macros.D2R
    motor_1_lower_upper_bound = 75.34739993 * macros.D2R
    motor_1_upper_lower_bound = 73.51549152 * macros.D2R
    motor_1_upper_upper_bound = 74.24682682 * macros.D2R
    motor_2_lower_lower_bound = 81.87856171 * macros.D2R
    motor_2_lower_upper_bound = 81.15338313 * macros.D2R
    motor_2_upper_lower_bound = 80.8092383 * macros.D2R
    motor_2_upper_upper_bound = 80.07973765 * macros.D2R

    # Compute the truth values
    motor_1_angle_truth = bilinear_interpolation(gimbal_angle_1_lower_bound,
                                              gimbal_angle_1_upper_bound,
                                              gimbal_angle_2_lower_bound,
                                              gimbal_angle_2_upper_bound,
                                              motor_1_lower_lower_bound,
                                              motor_1_lower_upper_bound,
                                              motor_1_upper_lower_bound,
                                              motor_1_upper_upper_bound,
                                              gimbal_angle_1_ref,
                                              gimbal_angle_2_ref)
    motor_2_angle_truth = bilinear_interpolation(gimbal_angle_1_lower_bound,
                                              gimbal_angle_1_upper_bound,
                                              gimbal_angle_2_lower_bound,
                                              gimbal_angle_2_upper_bound,
                                              motor_2_lower_lower_bound,
                                              motor_2_lower_upper_bound,
                                              motor_2_upper_lower_bound,
                                              motor_2_upper_upper_bound,
                                              gimbal_angle_1_ref,
                                              gimbal_angle_2_ref)

    np.testing.assert_allclose(motor_1_angle_truth, motor_1_angles_sim[-1], atol=1e-5, verbose=True)
    np.testing.assert_allclose(motor_2_angle_truth, motor_2_angles_sim[-1], atol=1e-5, verbose=True)

def linear_interpolation(x1, x2, y1, y2, x):
    return y1 * (x2 - x) / (x2 - x1) + y2 * (x - x1) / (x2 - x1)

def bilinear_interpolation(x1, x2, y1, y2, z11, z12, z21, z22, x, y):
    return (1.0 / ((x2 - x1) * (y2 - y1))
            * (z11 * (x2 - x) * (y2 - y) + z21 * (x - x1) * (y2 - y)
               + z12 * (x2 - x) * (y - y1) + z22 * (x - x1) * (y - y1)))

if __name__ == "__main__":
    test_no_interpolation_outside_boundary_returns_home(-12.0 * macros.D2R, -10.0 * macros.D2R)
    test_linear_interpolation_outside_boundary_returns_home(5.0 * macros.D2R, -21.3 * macros.D2R)
    test_bilinear_interpolation_outside_boundary_returns_home(8.37 * macros.D2R, 16.63 * macros.D2R)
    test_linear_interpolation_along_boundary_returns_home(-0.71 * macros.D2R, 25.5 * macros.D2R)
    test_bilinear_interpolation_along_boundary_returns_home(1.61 * macros.D2R, 25.81 * macros.D2R)
    test_no_interpolation_inside_boundary_returns_exact(1.5 * macros.D2R, 4.0 * macros.D2R)
    test_linear_interpolation_inside_boundary_along_tip_returns_exact(2.2 * macros.D2R, 6.5 * macros.D2R)
    test_linear_interpolation_inside_boundary_along_tilt_returns_exact(0.5 * macros.D2R, -2.7 * macros.D2R)
    test_bilinear_interpolation_inside_boundary_returns_exact(1.7 * macros.D2R, -2.1 * macros.D2R)
