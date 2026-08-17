import pytest
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
@pytest.mark.parametrize("k", [1, 5, 10])
@pytest.mark.parametrize("wheel_speed", [0.0, 100.0])
@pytest.mark.parametrize("theta_max", [np.pi / 2, np.pi / 36])
@pytest.mark.parametrize("accuracy", [1e-4])
def test_thruster_platform_reference(show_plots, delta_cm, k, wheel_speed, theta_max, seed, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the correctness of the platform reference orientation computed by
    :ref:`thrusterPlatformReference`. The correctness of the output is determined based on whether the thruster
    line of action is aligned with the system's center of mass, when the reaction wheels carry no momentum to dump.
    Moreover, the other module output messages, ``bodyHeadingOutMsg`` and ``thrusterConfigBOutMsg``, are checked
    versus equivalent python code.

    **Test Parameters**

    This test randomizes the position of the center of mass and runs the test 10 times for any other combination
    of test parameters.

    Args:
        delta_cm (m): magnitude of the center of mass shift, whose direction is generated randomly
        k (Hz): proportional gain of the momentum dumping control law
        wheel_speed (r/s): speed of each reaction wheel, which sets the momentum to be dumped
        theta_max (rad): half-angle of the thrust-deflection cone
        seed (-): seed is varied to randomly change the shift in the center of mass
        accuracy (float): accuracy within which results are considered to match the truth values.

    **Description of Variables Being Tested**

    For zero wheel speeds the requested dumping torque vanishes, and the correctness of the result is assessed based
    on the norm of the cross product between the body-frame thrust direction and the relative position of the center
    of mass with respect to the thruster application point :math:`T`, which must vanish when the thruster is aligned
    with the center of mass. For non-zero wheel speeds this alignment test is not performed, as the thruster is
    intentionally offset from the center of mass. This script does not test the integral feedback term, which would
    require running a simulation for an extended period of time.

    For all wheel speeds, the thruster configuration output message is checked to be self-consistent with the
    body-frame thrust heading and magnitude, and the thrust deflection is checked to stay within the configured cone.

    **General Documentation Comments**

    The offset vectors provided as input parameters ensure that a solution exists, such that the Unit Test can
    correctly assess the alignment of the thruster. This is, in general, not guaranteed.
    """
    thruster_platform_reference_test_function(show_plots, delta_cm, k, wheel_speed, theta_max, seed, accuracy)


def thruster_platform_reference_test_function(show_plots, delta_cm, k, wheel_speed, theta_max, seed, accuracy):

    # seed numpy's generator (used for the random center-of-mass shift below) so the test is deterministic and
    # independent of execution order
    np.random.seed(int(seed))

    euler_angles_123 = np.array([5.0 * macros.D2R, 10.0 * macros.D2R, 0.0])
    sigma_MB = np.array(rbk.euler1232MRP(euler_angles_123))
    r_MB_B = np.array([0.0, -0.1, -1.4])
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
    platform.r_MB_B = r_MB_B
    platform.r_FM_F = r_FM_F
    platform.K = k
    platform.Ki = 0.01
    platform.integralLimit = 10.0
    platform.controlPeriod = 1.0
    platform.thetaMax = theta_max

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
    input_rw_speeds_msg_data.wheelSpeeds = [wheel_speed, wheel_speed, wheel_speed]
    input_rw_speeds_msg = messaging.RWSpeedMsgF32().write(input_rw_speeds_msg_data)
    platform.rwSpeedsInMsg.subscribeTo(input_rw_speeds_msg)

    # Setup logging on the test module output messages so that we get all the writes to it
    body_heading_log = platform.bodyHeadingOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, body_heading_log)
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

    thrust = np.linalg.norm(T_F)
    tHat_B_sim = body_heading_log.rHat_XB_B[0]
    r_TB_B_sim = thr_config_b_log.rThrust_B[0]
    tHat_B_cfg_sim = thr_config_b_log.tHatThrust_B[0]
    tMax_sim = thr_config_b_log.maxThrust[0]

    # the reported body-frame thrust heading is a unit vector and the thrust magnitude is preserved
    np.testing.assert_allclose(np.linalg.norm(tHat_B_sim), 1.0, rtol=accuracy, atol=accuracy, verbose=True)
    np.testing.assert_allclose(tMax_sim, thrust, rtol=accuracy, atol=accuracy, verbose=True)

    # the body-heading and thruster-configuration messages report the same thrust direction
    np.testing.assert_allclose(tHat_B_cfg_sim, tHat_B_sim, rtol=accuracy, atol=accuracy, verbose=True)

    # the thrust deflection from its neutral (un-rotated) direction stays within the configured cone
    MB = rbk.MRP2C(sigma_MB)
    tHat_F = T_F / thrust
    neutral_B = np.matmul(MB.transpose(), tHat_F)
    deflection = np.arccos(np.clip(np.dot(neutral_B, tHat_B_sim), -1.0, 1.0))
    np.testing.assert_array_less(deflection, theta_max + accuracy, verbose=True)

    # with no wheel momentum to dump the thruster aligns through the center of mass whenever the cone does not clamp
    # it, so the moment arm is parallel to the thrust direction (zero offset).
    if wheel_speed == 0.0 and deflection < theta_max - accuracy:
        r_TC_B = np.array(r_TB_B_sim) - r_CB_B
        offset = np.linalg.norm(np.cross(tHat_B_sim, r_TC_B)) / np.linalg.norm(r_TC_B)
        np.testing.assert_allclose(offset, 0.0, rtol=accuracy, atol=accuracy, verbose=True)

    return


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thruster_platform_reference(
        False,                   # show_plots
        0.1,                     # delta_cm
        1,                       # k
        0.0,                     # wheel_speed
        np.pi / 2,               # theta_max
        np.random.rand(1)[0],    # seed
        1e-4                     # accuracy
    )
