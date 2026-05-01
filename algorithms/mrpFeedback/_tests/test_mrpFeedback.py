import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fp32 import mrpFeedbackF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


@pytest.mark.parametrize("int_gain", [0.01, 0])
@pytest.mark.parametrize("rw_num", [4, 0])
@pytest.mark.parametrize("integral_limit", [0, 20])
@pytest.mark.parametrize("ctrl_law", [0, 1])
@pytest.mark.parametrize("use_rw_availability", ["NO", "ON", "OFF"])
def test_mrp_feedback(int_gain, rw_num, integral_limit, ctrl_law, use_rw_availability):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = mrpFeedbackF32.MrpFeedback()
    module.modelTag = "mrpFeedback"
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Phase 1 config: assign public properties before reset() runs.
    module.K = 0.15
    module.Ki = int_gain
    module.P = 150.0
    module.integralLimit = integral_limit
    module.controlLawType = (
        mrpFeedbackF32.ControlLawType_NORMAL if ctrl_law == 0 else mrpFeedbackF32.ControlLawType_SIMPLE_INTEGRAL
    )
    module.knownTorquePntB_B = [1.0, 1.0, 1.0]

    # Attitude guidance input
    guid_cmd_data = messaging.AttGuidMsgF32Payload()
    sigma_BR = [0.3, -0.5, 0.7]
    guid_cmd_data.sigma_BR = sigma_BR
    omega_BR_B = [0.010, -0.020, 0.015]
    guid_cmd_data.omega_BR_B = omega_BR_B
    omega_RN_B = [-0.02, -0.01, 0.005]
    guid_cmd_data.omega_RN_B = omega_RN_B
    domega_RN_B = [0.0002, 0.0003, 0.0001]
    guid_cmd_data.domega_RN_B = domega_RN_B
    guid_in_msg = messaging.AttGuidMsgF32().write(guid_cmd_data)

    # Vehicle inertia
    vehicle_config = messaging.VehicleConfigMsgF32Payload()
    inertia = [
        1000.0, 0.0, 0.0,
        0.0, 800.0, 0.0,
        0.0, 0.0, 800.0,
    ]
    vehicle_config.ISCPntB_B = inertia
    vc_in_msg = messaging.VehicleConfigMsgF32().write(vehicle_config)

    # RW speeds
    rw_speed_message = messaging.RWSpeedMsgF32Payload()
    Omega = [10.0, 25.0, 50.0, 100.0]
    rw_speed_message.wheelSpeeds = Omega
    rw_speed_in_msg = messaging.RWSpeedMsgF32().write(rw_speed_message)

    # RW config
    js_list = []
    G_s_B = []
    if rw_num > 0:
        rw_config_params = messaging.RWArrayConfigMsgF32Payload()
        G_s_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.577350269190, 0.577350269190, 0.577350269190,
        ]
        js_list = [0.1, 0.1, 0.1, 0.1]
        rw_config_params.GsMatrix_B = G_s_B
        rw_config_params.JsList = js_list
        rw_config_params.numRW = rw_num
        rw_param_in_msg = messaging.RWArrayConfigMsgF32().write(rw_config_params)

    # RW availability
    rw_availability_message = messaging.RWAvailabilityMsgPayload()
    if use_rw_availability != "NO":
        if use_rw_availability == "ON":
            rw_availability_message.wheelAvailability = [
                messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE,
            ]
        else:
            rw_availability_message.wheelAvailability = [
                messaging.UNAVAILABLE, messaging.UNAVAILABLE, messaging.UNAVAILABLE, messaging.UNAVAILABLE,
            ]
        rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
    else:
        # default availability for the truth-builder (algorithm itself ignores when not connected)
        rw_availability_message.wheelAvailability = [
            messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE,
        ]

    Lr_true = find_true_torques(
        module, guid_cmd_data, rw_speed_message, vehicle_config, js_list, rw_num, G_s_B, rw_availability_message,
        ctrl_law,
    )

    data_log = module.cmdTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    module.guidInMsg.subscribeTo(guid_in_msg)
    module.vehConfigInMsg.subscribeTo(vc_in_msg)
    if rw_num > 0:
        module.rwParamsInMsg.subscribeTo(rw_param_in_msg)
        module.rwSpeedsInMsg.subscribeTo(rw_speed_in_msg)
    if use_rw_availability != "NO":
        module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))
    unit_test_sim.ExecuteSimulation()

    module.reset(1)

    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))
    unit_test_sim.ExecuteSimulation()

    Lr = data_log.torqueRequestBody

    accuracy = 1e-6
    np.testing.assert_allclose(Lr, Lr_true, atol=accuracy, rtol=accuracy, verbose=True)


def find_true_torques(module, guid_cmd_data, rw_speed_message, vehicle_config_out, js_list, num_rw, G_s_B, rw_avail_msg,
                      ctrl_law):
    Lr = []

    L = np.asarray(module.knownTorquePntB_B).flatten()
    steps = [0, 0, 0.5, 0, 0.5]
    omega_BR_B = np.asarray(guid_cmd_data.omega_BR_B)
    omega_RN_B = np.asarray(guid_cmd_data.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B
    domega_RN_B = np.asarray(guid_cmd_data.domega_RN_B)
    sigma_BR = np.asarray(guid_cmd_data.sigma_BR)
    Isc = np.reshape(np.asarray(vehicle_config_out.ISCPntB_B), (3, 3))
    Ki = module.Ki
    K = module.K
    P = module.P
    js_vec = js_list
    G_s_B_array = np.asarray(G_s_B)
    G_s_B_array = np.reshape(G_s_B_array[0:num_rw * 3], (num_rw, 3))
    sigma_int = np.asarray([0.0, 0.0, 0.0])

    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            sigma_int = np.asarray([0.0, 0.0, 0.0])

        if Ki > 0:
            sigma_int = K * dt * sigma_BR + sigma_int
            for n in range(3):
                if abs(sigma_int[n]) > module.integralLimit:
                    sigma_int[n] *= module.integralLimit / sigma_int[n]
            z_vec = sigma_int + Isc.dot(omega_BR_B)
        else:
            z_vec = np.asarray([0.0, 0.0, 0.0])

        Lr0 = K * sigma_BR
        Lr1 = Lr0 + P * omega_BR_B
        Lr2 = Lr1 + P * Ki * z_vec
        GsHs = np.array([0.0, 0.0, 0.0])

        if num_rw > 0:
            for j in range(num_rw):
                if rw_avail_msg.wheelAvailability[j] == 0:
                    GsHs = GsHs + np.dot(
                        G_s_B_array[j, :],
                        js_vec[j] * (np.dot(omega_BN_B, G_s_B_array[j, :]) + rw_speed_message.wheelSpeeds[j]),
                    )

        if ctrl_law == 0:
            Lr3 = Lr2 - np.cross((omega_RN_B + Ki * z_vec), (Isc.dot(omega_BN_B) + GsHs))
        else:
            Lr3 = Lr2 - np.cross(omega_BN_B, (Isc.dot(omega_BN_B) + GsHs))

        Lr4 = Lr3 + Isc.dot(-domega_RN_B + np.cross(omega_BN_B, omega_RN_B))
        Lr5 = -(Lr4 + L)
        Lr.append(np.ndarray.tolist(Lr5))
    return np.array(Lr)


if __name__ == "__main__":
    test_mrp_feedback(0.01, 0, 0.0, 1, "NO")
