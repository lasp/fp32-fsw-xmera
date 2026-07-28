import pytest
import random
import numpy as np


# Import all the modules that are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.fp32 import thrusterPlatformReferenceF32
from xmera.utilities import macros
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging
from xmera.architecture import sim_model


# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
@pytest.mark.parametrize("seed", list(np.linspace(1, 10, 10)))
@pytest.mark.parametrize("delta_cm", [0.1, 0.2, 0.3])
@pytest.mark.parametrize("k", [0, 1, 5, 10])
@pytest.mark.parametrize("theta_max", [-1, np.pi / 36])
@pytest.mark.parametrize("accuracy", [1e-4])
def test_thruster_platform_reference(show_plots, delta_cm, k, theta_max, seed, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the correctness of the tip and tilt reference angles computed by
    :ref:`thrusterPlatformReference`. The correctness of the output is determined based on whether the thruster
    is aligned with the system's center of mass, when the momentum dumping control gain :math:`\kappa = 0`.
    Moreover, the other module output messages, ``bodyHeadingOutMsg`` and ``thrusterTorqueOutMsg`` are checked
    versus equivalent python code.

    **Test Parameters**

    This test randomizes the position of the center of mass and runs the test 10 times for any other combination
    of test parameters.

    Args:
        delta_cm (m): magnitude of the center of mass shift, whose direction is generated randomly
        k (Hz): proportional gain of the momentum dumping control law
        seed (-): seed is varied to randomly change the shift in the center of mass
        accuracy (float): accuracy within which results are considered to match the truth values.

    **Description of Variables Being Tested**

    For :math:`\kappa = 0`, the correctness of the result is assessed based on the norm of the
    cross product between the thrust direction vector :math:`{}^\mathcal{F}\boldsymbol{t}` and the relative position
    of the center of mass with respect to the thruster application point :math:`T`. For :math:`\kappa \neq 0` this
    test is not performed, as the thruster is not aligned with the center of mass. This script does not test the
    integral feedback term, which would require running a simulation for an extended period of time.

    The python code also computes equivalently the thrust direction in body frame coordinates :math:`{}^\mathcal{B}\boldsymbol{t}`
    and the net torque on the system :math:`{}^\mathcal{B}\boldsymbol{L}`, and compares them to the respective output
    messages for all values of :math:`\kappa = 0` tested.

    **General Documentation Comments**

    The offset vectors provided as input parameters ensure that a solution exists, such that the Unit Test can correctly
    assess the alignment of the thruster. This is, in general, not guaranteed.
    """
    thruster_platform_reference_test_function(show_plots, delta_cm, k, theta_max, seed, accuracy)


def thruster_platform_reference_test_function(show_plots, delta_cm, k, theta_max, seed, accuracy):

    random.seed(seed)

    euler_angles_123 = np.array([5.0 * macros.D2R, 10.0 * macros.D2R, 0.0])
    sigma_MB = np.array(rbk.euler1232MRP(euler_angles_123))
    r_BM_M = np.array([0.0, 0.1, 1.4])
    r_FM_F = np.array([0.0, 0.0, -0.1])
    r_TF_F = np.array([-0.01, 0.03, 0.02])
    T_F = np.array([1.0, 1.0, 10.0])

    r_CB_B = np.array([0, 0, 0]) + np.random.rand(3)
    r_CB_B = r_CB_B / np.linalg.norm(r_CB_B) * delta_cm

    unit_task_name = "unitTask"          # arbitrary name (don't change)
    unit_process_name = "TestProcess"    # arbitrary name (don't change)
    sim_model.setDefaultLogLevel(sim_model.BSK_WARNING)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(1)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    platform = thrusterPlatformReferenceF32.ThrusterPlatformReference()
    platform.modelTag = "platformReference"

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, platform)

    # Initialize the test module configuration data
    platform.sigma_MB = sigma_MB
    platform.r_BM_M = r_BM_M
    platform.r_FM_F = r_FM_F
    platform.K = k
    platform.Ki = 0
    platform.controlPeriod = 1.0
    platform.theta1Max = theta_max
    platform.theta2Max = theta_max

    # Create input vehicle configuration msg
    input_veh_config_msg_data = messaging.VehicleConfigMsgF32Payload()
    input_veh_config_msg_data.CoM_B = r_CB_B
    input_veh_config_msg = messaging.VehicleConfigMsgF32().write(input_veh_config_msg_data)
    platform.vehConfigInMsg.subscribeTo(input_veh_config_msg)

    # Create input THR Config Msg
    thr_config = messaging.THRConfigMsgF32Payload()
    thr_config.rThrust_B = r_TF_F
    thr_config.maxThrust = np.linalg.norm(T_F)
    thr_config.tHatThrust_B = T_F / thr_config.maxThrust
    thr_config_f_msg = messaging.THRConfigMsgF32().write(thr_config)
    platform.thrusterConfigFInMsg.subscribeTo(thr_config_f_msg)

    # Create input RW configuration msg
    input_rw_config_msg_data = messaging.RWArrayConfigMsgF32Payload()
    input_rw_config_msg_data.GsMatrix_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    input_rw_config_msg_data.JsList = [0.01, 0.01, 0.01]
    input_rw_config_msg_data.numRW = 3
    input_rw_config_msg_data.uMax = [0.001, 0.001, 0.001]
    input_rw_config_msg = messaging.RWArrayConfigMsgF32().write(input_rw_config_msg_data)
    platform.rwConfigDataInMsg.subscribeTo(input_rw_config_msg)

    # Create input RW speeds msg
    input_rw_speeds_msg_data = messaging.RWSpeedMsgF32Payload()
    input_rw_speeds_msg_data.wheelSpeeds = [100, 100, 100]
    input_rw_speeds_msg = messaging.RWSpeedMsgF32().write(input_rw_speeds_msg_data)
    platform.rwSpeedsInMsg.subscribeTo(input_rw_speeds_msg)

    # Setup logging on the test module output messages so that we get all the writes to it
    ref1_log = platform.hingedRigidBodyRef1OutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, ref1_log)
    ref2_log = platform.hingedRigidBodyRef2OutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, ref2_log)
    body_heading_log = platform.bodyHeadingOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, body_heading_log)
    thruster_torque_log = platform.thrusterTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, thruster_torque_log)
    thr_config_b_log = platform.thrusterConfigBOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, thr_config_b_log)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unit_test_sim.ExecuteSimulation()

    theta1 = ref1_log.theta[0]
    theta2 = ref2_log.theta[0]

    FM = rbk.euler1232C([theta1, theta2, 0.0])
    MB = rbk.MRP2C(sigma_MB)

    r_CB_M = np.matmul(MB, r_CB_B)
    r_CM_M = r_CB_M + r_BM_M
    r_CM_F = np.matmul(FM, r_CM_M)
    r_CT_F = r_CM_F - r_FM_F - r_TF_F

    offset = np.linalg.norm(np.cross(r_CT_F, T_F) / np.linalg.norm(np.array(r_CT_F)) / np.linalg.norm(np.array(T_F)))

    # check if the CM offset is zero if control gain k is also 0
    if k == 0 and theta_max < 0:
        np.testing.assert_allclose(offset, 0.0, rtol=accuracy, atol=accuracy, verbose=True)

    T_B_hat_sim = body_heading_log.rHat_XB_B[0]             # simulation result
    FB = np.matmul(FM, MB)
    T_B = np.matmul(FB.transpose(), T_F)
    T_B_hat = T_B / np.linalg.norm(T_B)                     # truth value

    # compare the module results to the python computation for body-frame thruster direction
    np.testing.assert_allclose(T_B_hat_sim, T_B_hat, rtol=accuracy, atol=accuracy, verbose=True)

    L_B_sim = thruster_torque_log.torqueRequestBody[0]     # simulation result
    L_F = np.cross(r_CT_F, T_F)
    L_B = np.matmul(FB.transpose(), L_F)

    # compare the module results to the python computation for body-frame cmd torque
    np.testing.assert_allclose(L_B_sim, L_B, rtol=accuracy, atol=accuracy, verbose=True)

    # compare the module results to the python computation for thruster configuration in B frame
    r_TB_B = r_CB_B - np.matmul(FB.transpose(), r_CT_F)
    r_TB_B_sim = thr_config_b_log.rThrust_B[0]
    tHat_B_sim = thr_config_b_log.tHatThrust_B[0]
    tMax_sim = thr_config_b_log.maxThrust[0]
    np.testing.assert_allclose(r_TB_B_sim, r_TB_B, rtol=accuracy, atol=accuracy, verbose=True)
    np.testing.assert_allclose(tHat_B_sim, T_B_hat, rtol=accuracy, atol=accuracy, verbose=True)
    np.testing.assert_allclose(tMax_sim, np.linalg.norm(T_B), rtol=accuracy, atol=accuracy, verbose=True)

    # compare the output reference angle
    if theta_max > 0:
        np.testing.assert_array_less(theta1, theta_max + accuracy, verbose=True)
        np.testing.assert_array_less(theta2, theta_max + accuracy, verbose=True)

    return


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thruster_platform_reference(
        False,                   # show_plots
        0.1,                     # delta_cm
        0,                       # k
        -1,                      # theta_max
        np.random.rand(1)[0],    # seed
        1e-4                     # accuracy
    )
