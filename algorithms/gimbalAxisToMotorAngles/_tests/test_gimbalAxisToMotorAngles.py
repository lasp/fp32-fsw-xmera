import inspect
import os

import numpy as np
import pandas as pd
import pytest
from xmera.architecture import messaging
from xmera.fp32 import gimbalAxisToMotorAnglesF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


def load_interpolation_tables():
    """Load the gimbal-to-motor interpolation tables, convert to radians, and mark empty cells with -1."""
    gimbal_to_motor_1_angle = pd.read_csv(os.path.join(path, "gimbal_to_motor1_angle.csv"), header=None)
    gimbal_to_motor_2_angle = pd.read_csv(os.path.join(path, "gimbal_to_motor2_angle.csv"), header=None)

    gimbal_to_motor_1_angle_table = np.deg2rad(gimbal_to_motor_1_angle.to_numpy())
    gimbal_to_motor_2_angle_table = np.deg2rad(gimbal_to_motor_2_angle.to_numpy())

    gimbal_to_motor_1_angle_table = np.nan_to_num(gimbal_to_motor_1_angle_table, nan=-1)
    gimbal_to_motor_2_angle_table = np.nan_to_num(gimbal_to_motor_2_angle_table, nan=-1)

    return gimbal_to_motor_1_angle_table, gimbal_to_motor_2_angle_table


def to_swig_table(np_table):
    """Convert a (111, 76) numpy table into the module's wrapped GimbalMotorTable type."""
    table = gimbalAxisToMotorAnglesF32.GimbalMotorTable2D()
    for row_index in range(np_table.shape[0]):
        table[row_index] = np_table[row_index].astype(float).tolist()
    return table


@pytest.mark.parametrize("gimbal_tip_angle_ref", [0.0 * macros.D2R, 10.1 * macros.D2R, -10.9 * macros.D2R])
@pytest.mark.parametrize("gimbal_tilt_angle_ref", [0.0 * macros.D2R, 10.1 * macros.D2R, -10.9 * macros.D2R])
def test_gimbal_axis_to_motor_angles(gimbal_tip_angle_ref, gimbal_tilt_angle_ref):
    r"""
    **Validation Test Description**

    This unit test ensures that the gimbal flight software module gimbalAxisToMotorAngles correctly
    determines the gimbal sequential tip and tilt angles corresponding to the reference body-frame thrust direction
    vector.

    Args:
        gimbal_tip_angle_ref (float):  Gimbal tip reference angle
        gimbal_tilt_angle_ref (float): Gimbal tilt reference angle
    """

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    test_time_step_sec = 0.1
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Specify the commanded thrust direction (in hub-frame components) for the reference gimbal angles
    # fmt:off
    thrust_dir_hat_ref_b = np.array([np.sin(gimbal_tilt_angle_ref),
                                     -np.cos(gimbal_tilt_angle_ref) * np.sin(gimbal_tip_angle_ref),
                                     np.cos(gimbal_tilt_angle_ref) * np.cos(gimbal_tip_angle_ref)])
    # fmt:on

    # Specify the gimbal mount frame hub-relative attitude
    dcm_mb = np.identity(3)

    # Create the thrust direction reference message
    thrust_direction_message_data = messaging.BodyHeadingMsgF32Payload()
    thrust_direction_message_data.rHat_XB_B = thrust_dir_hat_ref_b.tolist()
    thrust_direction_message = messaging.BodyHeadingMsgF32().write(thrust_direction_message_data)

    # Load the gimbal interpolation tables
    gimbal_to_motor_1_table, gimbal_to_motor_2_table = load_interpolation_tables()

    # Create and configure the module under test
    gimbal_controller = gimbalAxisToMotorAnglesF32.GimbalAxisToMotorAngles()
    gimbal_controller.modelTag = "twoAxisGimbalController"
    gimbal_controller.gimbalToMotor1Data = to_swig_table(gimbal_to_motor_1_table)
    gimbal_controller.gimbalToMotor2Data = to_swig_table(gimbal_to_motor_2_table)
    gimbal_controller.dcm_MB = dcm_mb.tolist()
    gimbal_controller.thrustDirectionInMsg.subscribeTo(thrust_direction_message)
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_controller)

    # Set up data logging for the module output
    gimbal_angle_data = gimbal_controller.twoAxisGimbalOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, gimbal_angle_data)

    # Run the simulation
    sim_time = 5.0
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data for comparison
    gimbal_tip_angles_sim = macros.R2D * gimbal_angle_data.theta1  # [deg]
    gimbal_tilt_angles_sim = macros.R2D * gimbal_angle_data.theta2  # [deg]

    # Check that the module-determined angles match the reference values (FP32 tolerance)
    np.testing.assert_allclose(macros.R2D * gimbal_tip_angle_ref, gimbal_tip_angles_sim[-1], atol=1e-4, verbose=True)
    np.testing.assert_allclose(macros.R2D * gimbal_tilt_angle_ref, gimbal_tilt_angles_sim[-1], atol=1e-4, verbose=True)


if __name__ == "__main__":
    test_gimbal_axis_to_motor_angles(14.6 * macros.D2R, -4.8 * macros.D2R)
