import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fp32 import celestialTwoBodyPointF32  # module that is to be tested
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import SimulationBaseClass
from xmera.utilities import astroFunctions as af
from xmera.utilities import macros
from xmera.utilities import unitTestSupport  # general support file with common unit test functions
from numpy import linalg as la


def compute_celestial_two_body_point(R_P1, v_P1, a_P1, R_P2, v_P2, a_P2):

    # Beforehand computations
    R_n = np.cross(R_P1, R_P2)
    v_n = np.cross(v_P1, R_P2) + np.cross(R_P1, v_P2)
    a_n = np.cross(a_P1, R_P2) + np.cross(R_P1, a_P2) + 2 * np.cross(v_P1, v_P2)

    # Reference Frame generation
    r1_hat = R_P1/la.norm(R_P1)
    r3_hat = R_n/la.norm(R_n)
    r2_hat = np.cross(r3_hat, r1_hat)
    RN = np.array([r1_hat, r2_hat, r3_hat])
    sigma_RN = rbk.C2MRP(RN)

    # Reference base-vectors first time-derivative
    I_33 = np.identity(3)
    C1 = I_33 - np.outer(r1_hat, r1_hat)
    dr1_hat = 1.0 / la.norm(R_P1) * np.dot(C1, v_P1)
    C3 = I_33 - np.outer(r3_hat, r3_hat)
    dr3_hat = 1.0 / la.norm(R_n) * np.dot(C3, v_n)
    dr2_hat = np.cross(dr3_hat, r1_hat) + np.cross(r3_hat, dr1_hat)

    # Angular Velocity computation
    omega_RN_R = np.array([
        np.dot(r3_hat, dr2_hat),
        np.dot(r1_hat, dr3_hat),
        np.dot(r2_hat, dr1_hat)
    ])
    omega_RN_N = np.dot(RN.T, omega_RN_R)

    # Reference base-vectors second time-derivative
    temp33_1 = 2 * np.outer(dr1_hat, r1_hat) + np.outer(r1_hat, dr1_hat)
    ddr1_hat = 1.0 / la.norm(R_P1) * (np.dot(C1, a_P1) - np.dot(temp33_1, v_P1))
    temp33_3 = 2 * np.outer(dr3_hat, r3_hat) + np.outer(r3_hat, dr3_hat)
    ddr3_hat = 1.0 / la.norm(R_n) * (np.dot(C3, a_n) - np.dot(temp33_3, v_n))
    ddr2_hat = np.cross(ddr3_hat, r1_hat) + np.cross(ddr1_hat, r3_hat) + 2 * np.cross(dr3_hat, dr1_hat)

    # Angular Acceleration computation
    domega_RN_R = np.array([
        np.dot(dr3_hat, dr2_hat) + np.dot(r3_hat, ddr2_hat) - np.dot(omega_RN_R, dr1_hat),
        np.dot(dr1_hat, dr3_hat) + np.dot(r1_hat, ddr3_hat) - np.dot(omega_RN_R, dr2_hat),
        np.dot(dr2_hat, dr1_hat) + np.dot(r2_hat, ddr1_hat) - np.dot(omega_RN_R, dr3_hat)

    ])
    domega_RN_N = np.dot(RN.T, domega_RN_R)

    return sigma_RN, omega_RN_N, domega_RN_N


def test_celestial_two_body_point_test_function():
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = celestialTwoBodyPointF32.CelestialTwoBodyPoint()
    module.modelTag = "celestialTwoBodyPoint"
    module.singularityThreshold = 1.0 * af.D2R
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Previous Computation of Initial Conditions for the test
    a = af.E_radius * 2.8
    e = 0.0
    i = 0.0
    Omega = 0.0
    omega = 0.0
    f = 60 * af.D2R
    (r, v) = af.OE2RV(af.mu_E, a, e, i, Omega, omega, f)
    r_BN_N = np.array([0., 0., 0.])
    v_BN_N = np.array([0., 0., 0.])
    cel_position_vec = r
    cel_velocity_vec = v

    # Navigation Input Message
    nav_state_out_data = messaging.NavTransMsgF32Payload()  # Create a structure for the input message
    nav_state_out_data.r_BN_N = r_BN_N
    nav_state_out_data.v_BN_N = v_BN_N
    nav_msg = messaging.NavTransMsgF32().write(nav_state_out_data)

    # Spice Input Message of Primary Body
    cel_body_data = messaging.EphemerisMsgF32Payload()
    cel_body_data.r_BdyZero_N = cel_position_vec
    cel_body_data.v_BdyZero_N = cel_velocity_vec
    cel_body_msg = messaging.EphemerisMsgF32().write(cel_body_data)

    data_log = module.attRefOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    module.transNavInMsg.subscribeTo(nav_msg)
    module.primaryCelBodyInMsg.subscribeTo(cel_body_msg)


    sec_body_data = messaging.EphemerisMsgF32Payload()
    sec_position_vec = [500., 500., 500.]
    sec_body_data.r_BdyZero_N = sec_position_vec
    sec_velocity_vec = [100., -10., 20.]
    sec_body_data.v_BdyZero_N = sec_velocity_vec
    cel2nd_body_msg = messaging.EphemerisMsgF32().write(sec_body_data)

    module.secondaryCelBodyInMsg.subscribeTo(cel2nd_body_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    ## truth values
    a = af.E_radius * 2.8
    e = 0.0
    i = 0.0
    Omega = 0.0
    omega = 0.0
    f = 60 * af.D2R
    (r, v) = af.OE2RV(af.mu_E, a, e, i, Omega, omega, f)
    r_BN_N = np.array([0., 0., 0.])
    v_BN_N = np.array([0., 0., 0.])
    cel_position_vec = r
    cel_velocity_vec = v

    R_P1 = cel_position_vec - r_BN_N
    v_P1 = cel_velocity_vec - v_BN_N
    a_P1 = np.array([0., 0., 0.])

    R_P2 = np.cross(R_P1, v_P1)
    v_P2 = np.cross(R_P1, a_P1)
    a_P2 = np.cross(v_P1, a_P1)

    sigma_RN, omega_RN_N, domega_RN_N = compute_celestial_two_body_point(R_P1, v_P1, a_P1, R_P2, v_P2, a_P2)

    true_sigma_RN = [sigma_RN] * 3
    true_omega_RN_N = [omega_RN_N] * 3
    true_domega_RN_N = [domega_RN_N] * 3

    ## module output
    sigma_RN = data_log.sigma_RN
    omega_RN_N = data_log.omega_RN_N
    domega_RN_N = data_log.domega_RN_N

    # compare the module results to the truth values
    accuracy = 1e-6

    np.testing.assert_allclose(sigma_RN, true_sigma_RN, rtol=0, atol=accuracy, err_msg='sigma_RN', verbose=True)
    np.testing.assert_allclose(omega_RN_N, true_omega_RN_N, rtol=0, atol=accuracy, err_msg='omega_RN_N', verbose=True)
    np.testing.assert_allclose(domega_RN_N, true_domega_RN_N, rtol=0, atol=accuracy, err_msg='domega_RN_N', verbose=True)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_celestial_two_body_point_test_function(True)
