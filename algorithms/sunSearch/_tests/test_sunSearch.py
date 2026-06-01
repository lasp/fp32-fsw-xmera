import inspect
import os

import numpy as np
import pytest
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


from xmera.utilities import SimulationBaseClass
from xmera.fp32 import sunSearchF32
from xmera.utilities import macros
from xmera.architecture import messaging
from xmera.architecture import sim_model


def compute_kinematic_properties(theta_R, T_R, u_M, I, omega_M):

    alpha_M = u_M / I

    # Computing the fastest bang-bang slew with no coasting arc
    alpha = 4 * theta_R / T_R**2
    omega = 2 * theta_R / T_R
    T = T_R
    t_c = T_R / 2

    # If angular acceleration exceeds limit, decrease acceleration and increase slew time
    if alpha > alpha_M:
        alpha = alpha_M
        T = 2 * (theta_R / alpha)**0.5
        t_c = T / 2
        omega = alpha * t_c

    # If angular rate exceeds limit, increase slew time adding a coasting arc
    if omega > omega_M:
        omega = omega_M
        T = theta_R / omega + omega / alpha
        t_c = omega / alpha

    return alpha, omega, T, t_c


@pytest.mark.parametrize("axis_1", [1, 2, 3])
@pytest.mark.parametrize("axis_2", [1, 2, 3])
@pytest.mark.parametrize("axis_3", [1, 2, 3])
@pytest.mark.parametrize("omega_BN_B", [[0, 0, 0], [0.01, -0.02, 0.03]])
def test_sun_search(show_plots, axis_1, axis_2, axis_3, omega_BN_B):

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    sim_model.setDefaultLogLevel(sim_model.BSK_WARNING)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(1.1)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    theta_1 = np.pi/2
    theta_2 = np.pi
    theta_3 = 2*np.pi
    T_R = 1
    u_M = 1
    omega_M = np.pi / 18

    # Construct algorithm and associated C++ container
    module = sunSearchF32.SunSearch()
    module.modelTag = "sunSearch"

    slew_prop_1 = sunSearchF32.SlewProperties()
    slew_prop_1.slewTime = T_R
    slew_prop_1.slewAngle = theta_1
    slew_prop_1.slewMaxRate = omega_M
    slew_prop_1.slewMaxTorque = u_M
    slew_prop_1.slewRotAxis = axis_1

    slew_prop_2 = sunSearchF32.SlewProperties()
    slew_prop_2.slewTime = T_R
    slew_prop_2.slewAngle = theta_2
    slew_prop_2.slewMaxRate = omega_M
    slew_prop_2.slewMaxTorque = u_M
    slew_prop_2.slewRotAxis = axis_2

    slew_prop_3 = sunSearchF32.SlewProperties()
    slew_prop_3.slewTime = T_R
    slew_prop_3.slewAngle = theta_3
    slew_prop_3.slewMaxRate = omega_M
    slew_prop_3.slewMaxTorque = u_M
    slew_prop_3.slewRotAxis = axis_3

    module.setSlewProperties(slew_prop_1)
    module.setSlewProperties(slew_prop_2)
    module.setSlewProperties(slew_prop_3)

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Initialize the test module configuration data
    # These will eventually become input messages

    # Create input navigation message
    nav_att_data = messaging.NavAttMsgF32Payload()
    nav_att_data.omega_BN_B = omega_BN_B
    nav_att_msg = messaging.NavAttMsgF32().write(nav_att_data)
    module.attNavInMsg.subscribeTo(nav_att_msg)

    I = [100, 200, 300]

    # Create input vehicle configuration message
    veh_conf_data = messaging.VehicleConfigMsgF32Payload()
    veh_conf_data.ISCPntB_B = [I[0],  0.0,  0.0,
                                0.0, I[1],  0.0,
                                0.0,  0.0, I[2]]
    veh_conf_msg = messaging.VehicleConfigMsgF32().write(veh_conf_data)
    module.vehConfigInMsg.subscribeTo(veh_conf_msg)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.attGuidOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    alpha_1, omega_1, T_1, tc_1 = compute_kinematic_properties(theta_1, T_R, u_M, I[axis_1-1], omega_M)
    alpha_2, omega_2, T_2, tc_2 = compute_kinematic_properties(theta_2, T_R, u_M, I[axis_2-1], omega_M)
    alpha_3, omega_3, T_3, tc_3 = compute_kinematic_properties(theta_3, T_R, u_M, I[axis_3-1], omega_M)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unit_test_sim.ConfigureStopTime(macros.sec2nano(T_1+T_2+T_3))

    # Begin the simulation time run set above
    unit_test_sim.ExecuteSimulation()

    time = data_log.times() * macros.NANO2SEC
    omega_BR_B = data_log.omega_BR_B
    omega_RN_B = data_log.omega_RN_B
    omega_dot_RN_B = data_log.domega_RN_B

    time_vector = [0, tc_1, T_1-tc_1, T_1, T_1+tc_2, T_1+T_2-tc_2, T_1+T_2, T_1+T_2+tc_3, T_1+T_2+T_3-tc_3, T_1+T_2+T_3]

    omega_BR_B_truth = np.zeros((len(time), 3))
    omega_RN_B_truth = np.zeros((len(time), 3))
    omega_dot_RN_B_truth = np.zeros((len(time), 3))
    for i in range(len(time)):
        t = time[i]
        if t < time_vector[1]:
            omega_RN_B_truth[i, axis_1-1] = omega_1 * t / tc_1
            omega_dot_RN_B_truth[i, axis_1-1] = alpha_1
        elif t < time_vector[2]:
            omega_RN_B_truth[i, axis_1-1] = omega_1
        elif t < time_vector[3]:
            omega_RN_B_truth[i, axis_1-1] = omega_1 * (T_1-t) / tc_1
            omega_dot_RN_B_truth[i, axis_1-1] = -alpha_1
        elif t < time_vector[4]:
            omega_RN_B_truth[i, axis_2-1] = omega_2 * (t-T_1) / tc_2
            omega_dot_RN_B_truth[i, axis_2-1] = alpha_2
        elif t < time_vector[5]:
            omega_RN_B_truth[i, axis_2-1] = omega_2
        elif t < time_vector[6]:
            omega_RN_B_truth[i, axis_2-1] = omega_2 * (T_1+T_2-t) / tc_2
            omega_dot_RN_B_truth[i, axis_2-1] = -alpha_2
        elif t < time_vector[7]:
            omega_RN_B_truth[i, axis_3-1] = omega_3 * (t-T_1-T_2) / tc_3
            omega_dot_RN_B_truth[i, axis_3-1] = alpha_3
        elif t < time_vector[8]:
            omega_RN_B_truth[i, axis_3-1] = omega_3
        elif t < time_vector[9]:
            omega_RN_B_truth[i, axis_3-1] = omega_3 * (T_1+T_2+T_3-t) / tc_3
            omega_dot_RN_B_truth[i, axis_3-1] = -alpha_3
        omega_BR_B_truth[i] = omega_BN_B - omega_RN_B_truth[i]

    accuracy = 1e-6

    # set the filtered output truth states
    np.testing.assert_allclose(omega_BR_B, omega_BR_B_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omega_RN_B, omega_RN_B_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omega_dot_RN_B, omega_dot_RN_B_truth, rtol=0, atol=accuracy, verbose=True)

    return



if __name__ == "__main__":
    test_sun_search(False, 1, 2, 3, [0, 0, 0])
