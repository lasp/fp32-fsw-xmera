# MIT License
#
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fp32 import mrpSteeringF32  # import the module that is to be tested
from xmera.utilities import RigidBodyKinematics
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

@pytest.mark.parametrize("K1", [0.15, 0])
@pytest.mark.parametrize("K3", [1.0, 0])
@pytest.mark.parametrize("omega_max", [1.5 * macros.D2R, 0.001])
@pytest.mark.parametrize("ignore_feed_forward", [True, False])

def test_mrp_steering_tracking_integrated(show_plots, K1, K3, omega_max, ignore_feed_forward):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    rate = 0.5
    test_process_rate = macros.sec2nano(rate)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = mrpSteeringF32.MrpSteering()
    module.modelTag = "mrpSteering"

    unit_test_sim.AddModelToTask(unit_task_name, module)

    module.K1 = K1
    module.K3 = K3
    module.omegaMax = omega_max
    module.ignoreOuterLoopFeedforward = ignore_feed_forward
    module.Ki = 0.01
    module.P = 150.0
    module.integralLimit = 2. / module.Ki * 0.1
    module.knownTorquePntB_B = [0., 0., 0.]
    module.controlPeriod = rate

    # attGuidOut Message:
    guid_cmd_data = messaging.AttGuidMsgF32Payload()  # Create a structure for the input message
    guid_cmd_data.sigma_BR = [0.3, -0.5, 0.7]
    guid_cmd_data.omega_BR_B = [0.010, -0.020, 0.015]
    guid_cmd_data.omega_RN_B = [-0.02, -0.01, 0.005]
    guid_cmd_data.domega_RN_B = [0.0002, 0.0003, 0.0001]
    guid_in_msg = messaging.AttGuidMsgF32().write(guid_cmd_data)

    # vehicleConfigData Message:
    vehicle_config_out = messaging.VehicleConfigMsgF32Payload()
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicle_config_out.ISCPntB_B = I
    vc_in_msg = messaging.VehicleConfigMsgF32().write(vehicle_config_out)

    # wheelSpeeds Message
    rw_speed_message = messaging.RWSpeedMsgF32Payload()
    omega = [10.0, 25.0, 50.0, 100.0]
    rw_speed_message.wheelSpeeds = omega
    rw_in_msg = messaging.RWSpeedMsgF32().write(rw_speed_message)

    # wheelConfigData message
    rw_config_params = messaging.RWArrayConfigMsgF32Payload()
    rw_config_params.GsMatrix_B = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.5773502691896258, 0.5773502691896258, 0.5773502691896258
    ]
    rw_config_params.JsList = [0.1, 0.1, 0.1, 0.1]
    rw_config_params.numRW = 4
    rw_param_in_msg = messaging.RWArrayConfigMsgF32().write(rw_config_params)

    # wheelAvailability message
    rw_avail_list = []
    rw_availability_message = messaging.RWAvailabilityMsgPayload()
    rw_avail = [messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE]
    rw_availability_message.wheelAvailability = rw_avail
    rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
    rw_avail_list.append(rw_avail)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.cmdTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # connect messages
    module.guidInMsg.subscribeTo(guid_in_msg)
    module.vehConfigInMsg.subscribeTo(vc_in_msg)
    module.rwParamsInMsg.subscribeTo(rw_param_in_msg)
    module.rwSpeedsInMsg.subscribeTo(rw_in_msg)
    module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    module.reset(1)  # this module reset function needs a time input (in NanoSeconds)

    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    # Compute true values
    true_vals = find_true_torques(module, guid_cmd_data, rw_speed_message, vehicle_config_out, rw_avail_list, rw_config_params)

    # compare the module results to the truth values
    accuracy = 1e-6

    np.testing.assert_allclose(data_log.torqueRequestBody, true_vals, atol=accuracy, rtol=accuracy, verbose=True)


def find_true_values(guid_cmd_data, module):

    omega_max = module.omegaMax
    sigma = np.asarray(guid_cmd_data.sigma_BR)
    K1 = np.asarray(module.K1)
    K3 = np.asarray(module.K3)
    B = RigidBodyKinematics.BmatMRP(sigma)
    omega_ast = []
    omega_ast_p = []

    for i in range(len(sigma)):
        steer_rate = -1*(2*omega_max/np.pi)*np.arctan((K1*sigma[i]+K3*sigma[i]*sigma[i]*sigma[i])*np.pi/(2*omega_max))
        omega_ast.append(steer_rate)

    if not module.ignoreOuterLoopFeedforward:
        sigma_p = 0.25*B.dot(omega_ast)
        for i in range(len(sigma)):
            omega_ast_rate = (K1+3*K3*sigma[i]**2)/(1+((K1*sigma[i]+K3*sigma[i]**3)**2)*(np.pi/(2*omega_max))**2)*sigma_p[i]
            omega_ast_p.append(-omega_ast_rate)
    else:
        omega_ast_p = np.asarray([0, 0, 0])

    return omega_ast, omega_ast_p

def find_true_torques(module, guid_cmd_data, rw_speed_message, vehicle_config_out, rw_avail_msg, rw_config_params):
    Lr = []

    #Read in variables
    num_rw = rw_config_params.numRW
    L = np.asarray(module.knownTorquePntB_B).flatten()
    dt = module.controlPeriod
    steps = [dt] * 5
    omega_BR_B = np.asarray(guid_cmd_data.omega_BR_B)
    omega_RN_B = np.asarray(guid_cmd_data.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B #find body rate
    domega_RN_B = np.asarray(guid_cmd_data.domega_RN_B)

    omega_BastR_B, omegap_BastR_B = find_true_values(guid_cmd_data, module)

    omega_BastN_B = omega_BastR_B+omega_RN_B
    omega_BBast_B = omega_BN_B - omega_BastN_B

    Isc = np.asarray(vehicle_config_out.ISCPntB_B)
    Isc = np.reshape(Isc, (3, 3))
    Ki = module.Ki
    P = module.P
    jsVec = rw_config_params.JsList[0:num_rw]
    GsMatrix = (rw_config_params.GsMatrix_B)
    GsMatrix_B_array = np.reshape(GsMatrix[0:num_rw * 3], (num_rw, 3))

    # Compute toruqes
    for i in range(len(steps)):
        if i == 0 or i == 3:  # at beginning of sim and after reset, zVec is zero
            zVec = np.asarray([0, 0, 0])

        #evaluate integral term
        if Ki > 0 and abs(module.integralLimit) > 0: #if integral feedback is on
            zVec = dt * omega_BBast_B + zVec  # z = integral(del_omega)
            # Make sure each component is less than the integral limit
            for i in range(3):
                if zVec[i] > module.integralLimit:
                        zVec[i] = zVec[i]/abs(zVec[i])*module.integralLimit

        else: # integral gain turned off/negative setting
            zVec = np.asarray([0, 0, 0])

        #compute torque Lr
        Lr0 = Ki * zVec  # +K*sigmaBR
        Lr1 = Lr0 + P * omega_BBast_B  # +P*deltaOmega

        GsHs = np.array([0,0,0])

        if num_rw > 0:
            for i in range(num_rw):
                if rw_avail_msg[0][i] == 0:  # Make RW availability check
                    GsHs = GsHs + np.dot(GsMatrix_B_array[i, :], jsVec[i] * (np.dot(omega_BN_B, GsMatrix_B_array[i, :]) + rw_speed_message.wheelSpeeds[i]))
                    # J_s*(dot(omegaBN_B,Gs_vec)+Omega_wheel)

        Lr2 = Lr1 - np.cross(omega_BastN_B, (Isc.dot(omega_BN_B)+GsHs))  #  - omega_BastN x ([I]omega + [Gs]h_s)

        Lr3 = Lr2 - Isc.dot(omegap_BastR_B + domega_RN_B - np.cross(omega_BN_B, omega_RN_B))
        # - [I](d(omega_B^ast/R)/dt + d(omega_r)/dt - omega x omega_r)
        Lr4 = Lr3 + L
        Lr4 = -Lr4
        Lr.append(np.ndarray.tolist(Lr4))

    return Lr

if __name__ == "__main__":
    test_mrp_steering_tracking_integrated(False, 0.15, 1.0, 0.025, False)
