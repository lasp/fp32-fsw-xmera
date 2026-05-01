import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import rateControlF32
from xmera.utilities import macros
from xmera.architecture import messaging

@pytest.mark.parametrize("set_ext_torque", [False, True])

def test_rateControl(set_ext_torque):
    r"""
    **Validation Test Description**

    This test confirms the functionality of the rateControl fsw algorithm. The spacecraft inertia tensor message is set,
    as well as a guidance message. An external torque can be toggled on or off. The module is then run for a few time
    steps and the determined control torque output is compared to the computed truth value.

    **Test Parameters**

    The unit test verifies that the module output torque message vector matches expected values. The test
    method parameters include the following.

    :param set_ext_torque: flag to set the known_torque_PntB_N variable
    :return: void
    """

    unit_task_name = "unitTask"
    unit_process_name = "test_process"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Create an instance of the rateControl module
    rate_cntrl = rateControlF32.RateControl()
    rate_cntrl.setDerivativeGainP(150.0)
    rate_cntrl.modelTag = "rateControl"
    unit_test_sim.AddModelToTask(unit_task_name, rate_cntrl)

    # Set the external torque
    known_torque_PntB_N = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    if set_ext_torque:
        known_torque_PntB_N = np.array([0.1, 0.2, 0.3], dtype=np.float32)
        rate_cntrl.setKnownTorquePntB_B(known_torque_PntB_N)

    # Create the attitude guidance input message
    att_guidance_message_data = messaging.AttGuidMsgF32Payload()
    att_guidance_message_data.sigma_BR = np.array([0.3, -0.5, 0.7], dtype=np.float32)
    att_guidance_message_data.omega_BR_B = np.array([0.010, -0.020, 0.015], dtype=np.float32)
    att_guidance_message_data.omega_RN_B = np.array([-0.02, -0.01, 0.005], dtype=np.float32)
    att_guidance_message_data.domega_RN_B = np.array([0.0002, 0.0003, 0.0001], dtype=np.float32)
    att_guidance_message = messaging.AttGuidMsgF32().write(att_guidance_message_data)

    # Create the vehicleConfig fsw message
    vehicle_config_in = messaging.VehicleConfigMsgF32Payload()
    vehicle_config_in.ISCPntB_B = [1000., 0., 0.,
                                  0., 800., 0.,
                                  0., 0., 800.]
    vc_in_msg = messaging.VehicleConfigMsgF32().write(vehicle_config_in)

    # Set up data logging
    cmd_torque_outmsg_datalog = rate_cntrl.cmdTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, cmd_torque_outmsg_datalog)

    # Connect messages
    rate_cntrl.vehConfigInMsg.subscribeTo(vc_in_msg)
    rate_cntrl.guidInMsg.subscribeTo(att_guidance_message)

    # Execute the simulation
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))
    unit_test_sim.ExecuteSimulation()

    # Extract logged data for test check
    cmd_torque_truth = np.array([findTrueTorques(rate_cntrl, att_guidance_message_data, vehicle_config_in, known_torque_PntB_N)]*3
                              , dtype=np.float32)

    # Compare the module results to the computed truth value
    accuracy = 1e-7
    np.testing.assert_allclose(cmd_torque_truth,
                               cmd_torque_outmsg_datalog.torqueRequestBody,
                               atol=accuracy,
                               rtol=0,
                               verbose=True)

def findTrueTorques(rate_cntrl, att_guidance_message_data, vehicle_config_out, known_torque_PntB_N):
    P = rate_cntrl.getDerivativeGainP()
    omega_BR_B = np.array(att_guidance_message_data.omega_BR_B, dtype=np.float32)
    domega_RN_B = np.array(att_guidance_message_data.domega_RN_B, dtype=np.float32)

    ISCPntB_B = np.identity(3, dtype=np.float32)
    ISCPntB_B[0][0] = vehicle_config_out.ISCPntB_B[0]
    ISCPntB_B[1][1] = vehicle_config_out.ISCPntB_B[4]
    ISCPntB_B[2][2] = vehicle_config_out.ISCPntB_B[8]
    return (- P * omega_BR_B
            + np.dot(ISCPntB_B, domega_RN_B)
            - known_torque_PntB_N)

if __name__ == "__main__":
    test_rateControl(False)
