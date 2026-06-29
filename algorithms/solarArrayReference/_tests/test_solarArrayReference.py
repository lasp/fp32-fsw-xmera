"""
Module Name:        solarArrayReference
"""

import pytest
import numpy as np

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.fp32 import solarArrayReferenceF32           # import the module that is to be tested
from xmera.utilities import macros
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging                      # import the message definitions
from xmera.architecture import sim_model


# this python function computes the same reference angle as the tested module
def compute_rotation_angle(sigma_RN, rHat_SB_N, a1Hat_B, a2Hat_B, theta0):

    RN = rbk.MRP2C(sigma_RN)
    rS_R = np.matmul(RN, rHat_SB_N)

    a2_R = []
    dotP = np.dot(rS_R, a1Hat_B)
    for n in range(3):
        a2_R.append(rS_R[n] - dotP * a1Hat_B[n])
    a2_R = np.array(a2_R)
    a2_R_norm = np.linalg.norm(a2_R)
    if a2_R_norm > 1e-6:
        a2_R = a2_R / a2_R_norm
        # arccos returns [0, pi]; negating gives [-pi, 0], so output is naturally in [-pi, pi]
        theta = np.arccos(min(max(np.dot(a2Hat_B, a2_R),-1),1))
        if np.dot(a1Hat_B, np.cross(a2Hat_B, a2_R)) < 0:
            theta = -theta
    else:
        # wrap current theta0 to [-pi, pi]
        theta = np.arctan2(np.sin(theta0), np.cos(theta0))

    return theta


@pytest.mark.parametrize("rHat_SB_N", [[1, 0, 0],
                                  [0, 0, 1]])
@pytest.mark.parametrize("sigma_BN", [[0.1, 0.2, 0.3],
                                      [0.5, 0.4, 0.3]])
@pytest.mark.parametrize("sigma_RN", [[0.3, 0.2, 0.1],
                                      [0.9, 0.7, 0.8]])
@pytest.mark.parametrize("accuracy", [1e-6])


def test_solarArrayReference(show_plots, rHat_SB_N, sigma_BN, sigma_RN, accuracy):
    r"""
    **Validation Test Description**

    This unit test verifies the correctness of the output reference angle computed by the :ref:`solarArrayReference`.
    The inputs provided are the inertial Sun direction, current attitude of the hub, and reference frame. Based on
    current attitude, the sun direction vector is mapped into body frame coordinates and passed into the Attitude
    Navigation Message.

    **Test Parameters**

    Args:
        rHat_SB_N[3] (double): Sun direction vector, in inertial frame components;
        sigma_BN[3] (double): spacecraft hub attitude with respect to the inertial frame, in MRP;
        sigma_RN[3] (double): reference frame attitude with respect to the inertial frame, in MRP;
        attitudeFrame (int): 0 to calculate reference rotation angle w.r.t. reference frame, 1 to calculate it w.r.t the current spacecraft attitude;
        accuracy (float): absolute accuracy value used in the validation tests.

    **Description of Variables Being Tested**

    This unit test checks the correctness of the output attitude reference message

    - ``solarArrayRefOutMsg``

    in all its parts. The reference angle ``theta`` is checked versus the value computed by a python function that computes the same angle.
    The reference angle derivative ``thetaDot`` is checked versus zero, as the module is run for only one Update call.
    """
    a1Hat_B = np.array([1, 0, 0])
    a2Hat_B = np.array([0, 1, 0])
    BN = rbk.MRP2C(sigma_BN)
    rHat_SB_B = np.matmul(BN, rHat_SB_N)
    thetaC = 0
    thetaDotC = 0

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    sim_model.setDefaultLogLevel(sim_model.BSK_WARNING)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(1)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct tested module and associated C container
    solar_array = solarArrayReferenceF32.SolarArrayReference()
    solar_array.modelTag = "solarArrayReference"

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, solar_array)

    # Initialize the test module configuration data
    solar_array.driveAxis = a1Hat_B
    solar_array.surfaceNormal = a2Hat_B

    # Create input attitude navigation message
    nav_att_in_msg_data = messaging.NavAttMsgF32Payload()
    nav_att_in_msg_data.sigma_BN = sigma_BN
    nav_att_in_msg_data.vehSunPntBdy = rHat_SB_B
    nav_att_in_msg = messaging.NavAttMsgF32().write(nav_att_in_msg_data)
    solar_array.attNavInMsg.subscribeTo(nav_att_in_msg)

    # Create input attitude reference message
    att_ref_in_msg_data = messaging.AttRefMsgF32Payload()
    att_ref_in_msg_data.sigma_RN = sigma_RN
    att_ref_in_msg = messaging.AttRefMsgF32().write(att_ref_in_msg_data)
    solar_array.attRefInMsg.subscribeTo(att_ref_in_msg)

    # Create input hinged rigid body body message
    hinged_rigid_body_in_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    hinged_rigid_body_in_msg_data.theta = thetaC
    hinged_rigid_body_in_msg_data.thetaDot = thetaDotC
    hinged_rigid_body_in_msg = messaging.HingedRigidBodyMsgF32().write(hinged_rigid_body_in_msg_data)
    solar_array.hingedRigidBodyInMsg.subscribeTo(hinged_rigid_body_in_msg)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = solar_array.solarArrayRefOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Set the simulation time.
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.5))

    # Begin the simulation time run set above
    unit_test_sim.ExecuteSimulation()

    thetaR = compute_rotation_angle(sigma_RN, rHat_SB_N, a1Hat_B, a2Hat_B, thetaC)

    # compare the module results to the truth values
    np.testing.assert_allclose(data_log.theta[0], thetaR, atol=accuracy, rtol=accuracy)


@pytest.mark.parametrize("specifiedAngle, offsetAngle", [
    (0.0, 0.0),
    (0.5, 0.0),
    (-1.2, 0.0),
    (3.0, 0.0),
    (-3.0, 0.0),
    (0.5, 0.3),     # offset shifts result
    (2.0, 2.0),     # sum past pi -> wraps to negative
    (-2.0, -2.0),   # sum past -pi -> wraps to positive
])
@pytest.mark.parametrize("accuracy", [1e-6])
def test_solarArrayReference_specifiedAngle(show_plots, specifiedAngle, offsetAngle, accuracy):
    r"""
    Verifies that in SPECIFIED_ANGLE tracking mode the output reference angle equals
    (specifiedAngle + offsetAngle) wrapped to [-pi, pi], regardless of attitude or sun inputs.
    """
    a1Hat_B = np.array([1, 0, 0])
    a2Hat_B = np.array([0, 1, 0])

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    sim_model.setDefaultLogLevel(sim_model.BSK_WARNING)

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(1)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    solar_array = solarArrayReferenceF32.SolarArrayReference()
    solar_array.modelTag = "solarArrayReference"
    unit_test_sim.AddModelToTask(unit_task_name, solar_array)

    solar_array.driveAxis = a1Hat_B
    solar_array.surfaceNormal = a2Hat_B
    solar_array.trackingMode = solarArrayReferenceF32.SPECIFIED_ANGLE
    solar_array.specifiedArrayAngle = specifiedAngle
    solar_array.offsetAngle = offsetAngle

    # Inputs are arbitrary in this mode — the algorithm should ignore them.
    nav_att_in_msg_data = messaging.NavAttMsgF32Payload()
    nav_att_in_msg_data.sigma_BN = [0.1, 0.2, 0.3]
    nav_att_in_msg_data.vehSunPntBdy = [1.0, 0.0, 0.0]
    nav_att_in_msg = messaging.NavAttMsgF32().write(nav_att_in_msg_data)
    solar_array.attNavInMsg.subscribeTo(nav_att_in_msg)

    att_ref_in_msg_data = messaging.AttRefMsgF32Payload()
    att_ref_in_msg_data.sigma_RN = [0.3, 0.2, 0.1]
    att_ref_in_msg = messaging.AttRefMsgF32().write(att_ref_in_msg_data)
    solar_array.attRefInMsg.subscribeTo(att_ref_in_msg)

    hinged_rigid_body_in_msg_data = messaging.HingedRigidBodyMsgF32Payload()
    hinged_rigid_body_in_msg_data.theta = 0.0
    hinged_rigid_body_in_msg_data.thetaDot = 0.0
    hinged_rigid_body_in_msg = messaging.HingedRigidBodyMsgF32().write(hinged_rigid_body_in_msg_data)
    solar_array.hingedRigidBodyInMsg.subscribeTo(hinged_rigid_body_in_msg)

    data_log = solar_array.solarArrayRefOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.5))
    unit_test_sim.ExecuteSimulation()

    summed = specifiedAngle + offsetAngle
    expected = np.arctan2(np.sin(summed), np.cos(summed))
    np.testing.assert_allclose(data_log.theta[0], expected, atol=accuracy, rtol=accuracy)


if __name__ == "__main__":
    test_solarArrayReference(
                 False,
                 np.array([1, 0, 0]),
                 np.array([0.1, 0.2, 0.3]),
                 np.array([0.3, 0.2, 0.1]),
                 0,
                 1e-12
               )
