import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import bodyRateMiscompareF32
from xmera.utilities import macros
from xmera.architecture import messaging


def test_body_rate_miscompare_nominal():
    """Nominal Unit Test"""
    body_rate_threshold_deg_per_sec = 1.0
    body_rate_threshold_rad_per_sec = np.deg2rad(body_rate_threshold_deg_per_sec)

    angular_velocity_mimu = np.array([-0.1, 0.2, -0.3])

    # Create a random array that will always be below the threshold for the nominal case
    random_array = np.random.uniform(
        low=-body_rate_threshold_rad_per_sec / np.sqrt(3), high=body_rate_threshold_rad_per_sec / np.sqrt(3), size=3
    )

    # Add this random array to the star tracker output
    angular_velocity_star_tracker = angular_velocity_mimu + random_array

    # Since this should never miscompare, the expected output is the star tracker output
    expected_output_angular_velocity = angular_velocity_star_tracker
    expected_output_fault = False

    run_test(
        body_rate_threshold_rad_per_sec,
        angular_velocity_mimu,
        angular_velocity_star_tracker,
        expected_output_angular_velocity,
        expected_output_fault,
    )


def test_body_rate_miscompare_off_nominal():
    """Off Nominal Unit Test"""
    body_rate_threshold_deg_per_sec = 2.0
    body_rate_threshold_rad_per_sec = np.deg2rad(body_rate_threshold_deg_per_sec)

    angular_velocity_mimu = np.array([-0.1, 0.2, -0.3])

    # Have the star tracker rate be too large
    star_tracker_rate_error = np.array(
        [body_rate_threshold_rad_per_sec * 2, body_rate_threshold_rad_per_sec * 2, body_rate_threshold_rad_per_sec * 2]
    )

    # Add this random array to the star tracker output
    angular_velocity_star_tracker = angular_velocity_mimu + star_tracker_rate_error

    # This will miscompare, so the module should output angular velocity of the mimus
    expected_output_angular_velocity = angular_velocity_mimu
    expected_output_fault = True

    run_test(
        body_rate_threshold_rad_per_sec,
        angular_velocity_mimu,
        angular_velocity_star_tracker,
        expected_output_angular_velocity,
        expected_output_fault,
    )


def run_test(
    body_rate_threshold_rad_per_sec,
    angular_velocity_mimu,
    angular_velocity_star_tracker,
    expected_output_angular_velocity,
    expected_output_fault,
):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    sim_time = 0.5
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = bodyRateMiscompareF32.BodyRateMiscompare()
    module.modelTag = "bodyRateMiscompare"

    module.bodyRateThreshold = body_rate_threshold_rad_per_sec

    # Initialize Imu rate from majority vote
    input_message_data = messaging.IMUSensorBodyMsgF32Payload()
    input_message_data.AngVelBody = angular_velocity_mimu.tolist()
    in_msg = messaging.IMUSensorBodyMsgF32().write(input_message_data, time=0)

    # Initialize star tracker rate from star tracker output
    star_tracker_message_data = messaging.STAttMsgPayload()
    star_tracker_message_data.omega_BN_B = angular_velocity_star_tracker.tolist()
    st_msg = messaging.STAttMsg().write(star_tracker_message_data, time=0)

    unit_test_sim.AddModelToTask(unit_task_name, module)

    module.imuSensorBodyInMsg.subscribeTo(in_msg)
    module.stBodyInMsg.subscribeTo(st_msg)

    rate_data_log = module.navAttOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, rate_data_log)
    fault_data_log = module.rateFaultOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, fault_data_log)

    unit_test_sim.InitializeSimulation()

    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))

    unit_test_sim.ExecuteSimulation()

    module_output_angular_velocity = rate_data_log.omega_BN_B

    np.testing.assert_allclose(
        module_output_angular_velocity[-1], expected_output_angular_velocity, rtol=0, atol=1e-7, verbose=True
    )
    np.testing.assert_allclose(fault_data_log.faultDetected[-1], expected_output_fault)
    np.testing.assert_allclose(
        module.bodyRateThreshold, body_rate_threshold_rad_per_sec, rtol=0, atol=1e-7, verbose=True
    )


if __name__ == "__main__":
    test_body_rate_miscompare_off_nominal()
