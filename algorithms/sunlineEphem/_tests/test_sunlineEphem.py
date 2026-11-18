import numpy as np

from xmera.architecture import messaging
from xmera.fp32 import sunlineEphemF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import RigidBodyKinematics as rbk

def test_sunline_ephem():
    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    sunline_ephem_obj = sunlineEphemF32.SunlineEphem()
    sunline_ephem_obj.modelTag = "sunlineEphem"           # update python name of test module

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sunline_ephem_obj)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    veh_att_data = messaging.NavAttMsgF32Payload()
    veh_pos_data = messaging.NavTransMsgF32Payload()
    sun_data = messaging.EphemerisMsgF32Payload()

    # Artificially put sun at the origin.
    sun_data.r_BdyZero_N = [0.0, 0.0, 0.0]

    # Non-trivial attitude so the body-frame and inertial-frame outputs differ
    sigma_BN = [0.1, 0.2, -0.3]
    veh_att_data.sigma_BN = sigma_BN
    veh_att_in_msg = messaging.NavAttMsgF32().write(veh_att_data)

    # Place spacecraft unit length away on each coordinate axis
    test_vectors = [[-1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0],
                   [0.0, 0.0, -1.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [0.0, 0.0, 0.0]] # test if the space pos vector aligns with the sun pos vector

    est_vector_B = np.zeros((len(test_vectors), 3))
    est_vector_N = np.zeros((len(test_vectors), 3))

    veh_pos_in_msg = messaging.NavTransMsgF32()
    sun_data_in_msg = messaging.EphemerisMsgF32().write(sun_data)
    sunline_ephem_obj.sunPositionInMsg.subscribeTo(sun_data_in_msg)
    sunline_ephem_obj.scPositionInMsg.subscribeTo(veh_pos_in_msg)
    sunline_ephem_obj.scAttitudeInMsg.subscribeTo(veh_att_in_msg)

    data_log = sunline_ephem_obj.navStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)
    heading_log = sunline_ephem_obj.sunHeadingOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, heading_log)

    for i in range(len(test_vectors)):
        test_vec = test_vectors[i]
        veh_pos_data.r_BN_N = test_vec
        veh_pos_in_msg.write(veh_pos_data)

        # Need to call the self-init and cross-init methods
        unit_test_sim.InitializeSimulation()
        unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
        unit_test_sim.ExecuteSimulation()
        est_vector_B[i] = data_log.vehSunPntBdy[-1]
        est_vector_N[i] = heading_log.rHat_XN_N[-1]

        # reset the module to test this functionality
        sunline_ephem_obj.reset(1)

    # set the inertial-frame truth states; the body-frame truth is the same direction rotated by [BN]
    true_vector_N = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
                [0.0, 0.0, 0.0],
                ]

    dcm_BN = np.array(rbk.MRP2C(sigma_BN))
    true_vector_B = [dcm_BN @ np.array(v) for v in true_vector_N]

    for i in range(len(true_vector_N)):
        np.testing.assert_allclose(est_vector_N[i], true_vector_N[i], rtol=1e-6, atol=1e-6, verbose=False)
        np.testing.assert_allclose(est_vector_B[i], true_vector_B[i], rtol=1e-6, atol=1e-6, verbose=False)


if __name__ == "__main__":
    test_sunline_ephem()
