import numpy as np
import pytest

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.fp32 import navAggregateF32
from xmera.utilities import macros
from xmera.architecture import messaging


@pytest.mark.parametrize("num_att_nav, num_trans_nav", [
      (0, 0)
    , (1, 1)
    , (0, 1)
    , (1, 0)
    , (2, 2)
    , (1, 2)
    , (0, 2)
    , (2, 1)
    , (2, 0)
    , (3, 3)
    , (3, 2)
    , (3, 1)
    , (3, 0)
    , (2, 3)
    , (1, 3)
    , (0, 3)
])

def test_nav_aggregate(show_plots, num_att_nav, num_trans_nav):
    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct an instance of the module being tested
    module = navAggregateF32.NavAggregate()
    module.modelTag = "navAggregate"

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Create input messages
    nav_att1_msg = messaging.NavAttMsgF32Payload()
    nav_att1_msg.timeTag = 11.11
    nav_att1_msg.sigma_BN = [0.1, 0.01, -0.1]
    nav_att1_msg.omega_BN_B = [1., 1., -1.]
    nav_att1_msg.vehSunPntBdy = [-0.1, 0.1, 0.1]
    nav_att1_in_msg = messaging.NavAttMsgF32().write(nav_att1_msg)
    nav_att2_msg = messaging.NavAttMsgF32Payload()
    nav_att2_msg.timeTag = 22.22
    nav_att2_msg.sigma_BN = [0.2, 0.02, -0.2]
    nav_att2_msg.omega_BN_B = [2., 2., -2.]
    nav_att2_msg.vehSunPntBdy = [-0.2, 0.2, 0.2]
    nav_att2_in_msg = messaging.NavAttMsgF32().write(nav_att2_msg)

    nav_trans1_msg = messaging.NavTransMsgF32Payload()
    nav_trans1_msg.timeTag = 11.1
    nav_trans1_msg.r_BN_N = [1000.0, 100.0, -1000.0]
    nav_trans1_msg.v_BN_N = [1., 1., -1.]
    nav_trans1_msg.vehAccumDV = [-10.1, 10.1, 10.1]
    nav_trans1_in_msg = messaging.NavTransMsgF32().write(nav_trans1_msg)
    nav_trans2_msg = messaging.NavTransMsgF32Payload()
    nav_trans2_msg.timeTag = 22.2
    nav_trans2_msg.r_BN_N = [2000.0, 200.0, -2000.0]
    nav_trans2_msg.v_BN_N = [2., 2., -2.]
    nav_trans2_msg.vehAccumDV = [-20.2, 20.2, 20.2]
    nav_trans2_in_msg = messaging.NavTransMsgF32().write(nav_trans2_msg)

    # create input navigation message containers
    nav_att1 = navAggregateF32.AggregateAttInput()
    nav_att2 = navAggregateF32.AggregateAttInput()
    nav_trans1 = navAggregateF32.AggregateTransInput()
    nav_trans2 = navAggregateF32.AggregateTransInput()

    module.setAttMsgCount(num_att_nav)
    if num_att_nav == 3:       # here the index asks to read from an empty (zero) message
        module.setAttMsgCount(2)

    module.setTransMsgCount(num_trans_nav)
    if num_trans_nav == 3:     # here the index asks to read from an empty (zero) message
        module.setTransMsgCount(2)

    if num_att_nav <= navAggregateF32.MAX_AGG_NAV_MSG:
        module.attMsgs = [nav_att1, nav_att2]
        module.attMsgs[0].navAttInMsg.subscribeTo(nav_att1_in_msg)
        module.attMsgs[1].navAttInMsg.subscribeTo(nav_att2_in_msg)
    else:
        module.attMsgs = [nav_att1] * navAggregateF32.MAX_AGG_NAV_MSG
        for i in range(navAggregateF32.MAX_AGG_NAV_MSG):
            module.attMsgs[i].navAttInMsg.subscribeTo(nav_att1_in_msg)
    if num_trans_nav <= navAggregateF32.MAX_AGG_NAV_MSG:
        module.transMsgs = [nav_trans1, nav_trans2]
        module.transMsgs[0].navTransInMsg.subscribeTo(nav_trans1_in_msg)
        module.transMsgs[1].navTransInMsg.subscribeTo(nav_trans2_in_msg)
    else:
        module.transMsgs = [nav_trans1] * navAggregateF32.MAX_AGG_NAV_MSG
        for i in range(navAggregateF32.MAX_AGG_NAV_MSG):
            module.transMsgs[i].navTransInMsg.subscribeTo(nav_trans1_in_msg)

    if num_att_nav > 1:       # always read from the last message counter
        module.setAttTimeIdx(num_att_nav - 1)
        module.setAttIdx(num_att_nav - 1)
        module.setRateIdx(num_att_nav - 1)
        module.setSunIdx(num_att_nav - 1)
    if num_trans_nav > 1:     # always read from the last message counter
        module.setTransTimeIdx(num_trans_nav - 1)
        module.setPosIdx(num_trans_nav - 1)
        module.setVelIdx(num_trans_nav - 1)
        module.setDvIdx(num_trans_nav - 1)

    # Setup logging on the test module output message so that we get all the writes to it
    data_att_log = module.navAttOutMsg.recorder()
    data_trans_log = module.navTransOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_att_log)
    unit_test_sim.AddModelToTask(unit_task_name, data_trans_log)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unit_test_sim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    att_time_tag = np.transpose([data_att_log.timeTag])
    att_sigma = data_att_log.sigma_BN
    att_omega = data_att_log.omega_BN_B
    att_sun_vector = data_att_log.vehSunPntBdy

    trans_time_tag = np.transpose([data_trans_log.timeTag])
    trans_pos = data_trans_log.r_BN_N
    trans_vel = data_trans_log.v_BN_N
    trans_accum = data_trans_log.vehAccumDV

    # set the filtered output truth states
    if num_att_nav == 0 or num_att_nav == 3:
        true_att_time_tag = [[0.0]]*3
        true_att_sigma = [[0., 0., 0.]]*3
        true_att_omega = [[0., 0., 0.]]*3
        true_att_sun_vector = [[0., 0., 0.]]*3

    if num_trans_nav == 0 or num_trans_nav == 3:
        true_trans_time_tag = [[0.0]]*3
        true_trans_pos = [[0.0, 0.0, 0.0]]*3
        true_trans_vel = [[0.0, 0.0, 0.0]]*3
        true_trans_accum = [[0.0, 0.0, 0.0]]*3

    if num_att_nav == 1:
        true_att_time_tag = [[nav_att1_msg.timeTag]]*3
        true_att_sigma = [nav_att1_msg.sigma_BN]*3
        true_att_omega = [nav_att1_msg.omega_BN_B]*3
        true_att_sun_vector = [nav_att1_msg.vehSunPntBdy]*3

    if num_trans_nav == 1:
        true_trans_time_tag = [[nav_trans1_msg.timeTag]]*3
        true_trans_pos = [nav_trans1_msg.r_BN_N]*3
        true_trans_vel = [nav_trans1_msg.v_BN_N]*3
        true_trans_accum = [nav_trans1_msg.vehAccumDV]*3

    if num_att_nav == 2:
        true_att_time_tag = [[nav_att2_msg.timeTag]] * 3
        true_att_sigma = [nav_att2_msg.sigma_BN] * 3
        true_att_omega = [nav_att2_msg.omega_BN_B] * 3
        true_att_sun_vector = [nav_att2_msg.vehSunPntBdy] * 3

    if num_trans_nav == 2:
        true_trans_time_tag = [[nav_trans2_msg.timeTag]]*3
        true_trans_pos = [nav_trans2_msg.r_BN_N]*3
        true_trans_vel = [nav_trans2_msg.v_BN_N]*3
        true_trans_accum = [nav_trans2_msg.vehAccumDV]*3

    # compare the module results to the truth values
    accuracy = 1e-7

    np.testing.assert_allclose(att_time_tag, true_att_time_tag, atol=accuracy, rtol=0, err_msg="att_time_tag")
    np.testing.assert_allclose(att_sigma, true_att_sigma, atol=accuracy, rtol=0, err_msg="att_sigma")
    np.testing.assert_allclose(att_omega, true_att_omega, atol=accuracy, rtol=0, err_msg="att_omega")
    np.testing.assert_allclose(att_sun_vector, true_att_sun_vector, atol=accuracy, rtol=0, err_msg="att_sun_vector")
    np.testing.assert_allclose(trans_time_tag, true_trans_time_tag, atol=accuracy, rtol=0, err_msg="trans_time_tag")
    np.testing.assert_allclose(trans_pos, true_trans_pos, atol=accuracy, rtol=0, err_msg="trans_pos")
    np.testing.assert_allclose(trans_vel, true_trans_vel, atol=accuracy, rtol=0, err_msg="trans_vel")
    np.testing.assert_allclose(trans_accum, true_trans_accum, atol=accuracy, rtol=0, err_msg="trans_accum")


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_nav_aggregate(False, 2, 2)
