import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import mrpPDF32
from xmera.utilities import macros
from xmera.architecture import messaging

@pytest.mark.parametrize("set_external_torque", [False, True])
def test_mrpPD(show_plots, set_external_torque):
    r"""
    **Validation Test Description**

    The unit test for this module verifies that the module output control torque vector matches the expected value.
    Given the spacecraft vehicle configuration input message containing the spacecraft inertia, an input attitude
    guidance message, and an optional known external torque vector, the module-computed control torque command vector
    is logged and compared with the computed truth value.

    **Test Parameters**

    The unit test verifies that the module output control torque vector matches the expected value. The test
    method parameters include the following.

    :param show_plots: flag to show the test run plots
    :param set_external_torque: flag to set the knownTorquePntB_B variable
    :return: void
    """

    task_name = "unit_test_task"
    process_name = "unit_test_process"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    process_rate = macros.sec2nano(0.5)  # Update process rate update time
    process = unit_test_sim.CreateNewProcess(process_name)
    process.addTask(unit_test_sim.CreateNewTask(task_name, process_rate))

    # Create the mrpPD module
    mrp_pd = mrpPDF32.MrpPD()
    mrp_pd.modelTag = "mrpPD"
    mrp_pd.P = 150.0
    mrp_pd.K = 0.15
    knownTorquePntB_B = np.array([0.0, 0.0, 0.0])
    if set_external_torque:
        knownTorquePntB_B = np.array([0.1, 0.2, 0.3])
        mrp_pd.knownTorquePntB_B = knownTorquePntB_B
    unit_test_sim.AddModelToTask(task_name, mrp_pd)

    # Create the mrpPD module attitude guidance input message
    guidance_cmd_data = messaging.AttGuidMsgF32Payload()
    guidance_cmd_data.sigma_BR = np.array([0.3, -0.5, 0.7])
    guidance_cmd_data.omega_BR_B = np.array([0.010, -0.020, 0.015])  # [rad/s]
    guidance_cmd_data.omega_RN_B = np.array([-0.02, -0.01, 0.005])  # [rad/s]
    guidance_cmd_data.domega_RN_B = np.array([0.0002, 0.0003, 0.0001])  # [rad/s^2]
    guidInMsg = messaging.AttGuidMsgF32().write(guidance_cmd_data)
    mrp_pd.guidInMsg.subscribeTo(guidInMsg)

    # Create the mrpPD module vehicle configuration input FSW message:
    inertia = [1000., 0., 0., 0., 800., 0., 0., 0., 800.]  # [kg*m^2]
    vehicle_config_payload = messaging.VehicleConfigMsgF32Payload()
    vehicle_config_payload.ISCPntB_B = inertia
    vehicle_msg = messaging.VehicleConfigMsgF32().write(vehicle_config_payload)
    mrp_pd.vehConfigInMsg.subscribeTo(vehicle_msg)

    # Set up data logging
    cmd_torque_data_log = mrp_pd.cmdTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(task_name, cmd_torque_data_log)

    # Run the simulation for 3*process rate, 4 total steps including zero
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))
    unit_test_sim.ExecuteSimulation()

    # Compute the truth control torque vector
    truth_torque = compute_true_torque(mrp_pd, guidance_cmd_data, np.array(inertia).reshape(3, 3), knownTorquePntB_B)  # [Nm]

    # Compare the module-computed command torque to the truth value
    accuracy = 1e-6
    np.testing.assert_allclose(truth_torque,
                               cmd_torque_data_log.torqueRequestBody[-1],
                               atol=accuracy,
                               rtol=0,
                               verbose=True)

def compute_true_torque(mrp_pd, guidance_cmd_data, ISCPntB_B, knownTorquePntB_B):
    # Compute hub inertial angular velocity in B-frame components
    omega_BR_B = np.array(guidance_cmd_data.omega_BR_B)
    omega_RN_B = np.array(guidance_cmd_data.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B

    K = mrp_pd.K
    P = mrp_pd.P
    sigma_BR = np.array(guidance_cmd_data.sigma_BR)
    domega_RN_B = np.array(guidance_cmd_data.domega_RN_B)

    # Compute required attitude control torque
    Lr = (- K * sigma_BR - P * omega_BR_B + np.cross(omega_RN_B, ISCPntB_B @ omega_BN_B)
          + ISCPntB_B @ (domega_RN_B - np.cross(omega_BN_B, omega_RN_B)) - knownTorquePntB_B)  # [Nm]

    return Lr

if __name__ == "__main__":
    test_mrpPD(False, False)
