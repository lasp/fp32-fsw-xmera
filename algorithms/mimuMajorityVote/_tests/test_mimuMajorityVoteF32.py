import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import mimuMajorityVoteF32
from xmera.utilities import macros
from xmera.architecture import messaging


def test_mimu_majority_vote_nominal():
    """Nominal Unit Test"""
    omega_threshold_deg_per_sec = 1.0
    omega_threshold_rad_per_sec = np.deg2rad(omega_threshold_deg_per_sec)

    angular_velocity_1 = np.array([-0.1, 0.25, 0.3])

    expected_angular_velocity = np.copy(angular_velocity_1)

    # Create uniform random array that will always the keep the magnitude difference below the threshold
    random_array = np.random.uniform(
        low=-omega_threshold_rad_per_sec / np.sqrt(3), high=omega_threshold_rad_per_sec / np.sqrt(3), size=3
    )

    angular_velocity_2 = angular_velocity_1 + random_array

    expected_angular_velocity += angular_velocity_2

    random_array = np.random.uniform(
        low=-omega_threshold_rad_per_sec / np.sqrt(3), high=omega_threshold_rad_per_sec / np.sqrt(3), size=3
    )

    angular_velocity_3 = angular_velocity_1 + random_array

    # The nominal test should just be a normal average with no fault
    expected_angular_velocity += angular_velocity_3
    expected_angular_velocity /= 3.0
    expected_output_fault = False
    expected_output_fault_index = -1

    run_test(
        angular_velocity_1,
        angular_velocity_2,
        angular_velocity_3,
        omega_threshold_rad_per_sec,
        expected_angular_velocity,
        expected_output_fault,
        expected_output_fault_index,
    )


def test_mimu_majority_vote_off_nominal():
    """Nominal Unit Test"""
    omega_threshold_deg_per_sec = 2.0  # Angular velocity threshold in deg per sec
    omega_threshold_rad_per_sec = np.deg2rad(omega_threshold_deg_per_sec)

    angular_velocity_1 = np.array([-0.1, 0.25, 0.3])
    expected_angular_velocity = np.copy(angular_velocity_1)

    # Have the second measurement be too large so that it is rejected
    array = np.array(
        [omega_threshold_rad_per_sec * 2, omega_threshold_rad_per_sec * 2, omega_threshold_rad_per_sec * 2]
    )

    angular_velocity_2 = angular_velocity_1 + array

    # Create uniform random array that will always the keep the magnitude difference below the threshold
    random_array = np.random.uniform(
        low=-omega_threshold_rad_per_sec / np.sqrt(3), high=omega_threshold_rad_per_sec / np.sqrt(3), size=3
    )

    angular_velocity_3 = angular_velocity_1 + random_array

    # This off nominal case should reject the second measurement, fault detected and mark the second mimu faulted
    expected_angular_velocity += angular_velocity_3
    expected_angular_velocity /= 2.0
    expected_output_fault = True
    expected_output_fault_index = 1

    run_test(
        angular_velocity_1,
        angular_velocity_2,
        angular_velocity_3,
        omega_threshold_rad_per_sec,
        expected_angular_velocity,
        expected_output_fault,
        expected_output_fault_index,
    )


def run_test(
    angular_velocity_1,
    angular_velocity_2,
    angular_velocity_3,
    omega_threshold_rad_per_sec,
    expected_angular_velocity,
    expected_output_fault,
    expected_output_fault_index,
):

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    sim_time = 0.5
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = mimuMajorityVoteF32.MimuMajorityVote()
    module.modelTag = "mimuMajorityVote"

    module.omegaThreshold = omega_threshold_rad_per_sec

    unit_test_sim.AddModelToTask(unit_task_name, module)

    input_message_data_1 = messaging.IMUSensorBodyMsgF32Payload()
    input_message_data_1.AngVelBody = angular_velocity_1.tolist()
    in_msg_1 = messaging.IMUSensorBodyMsgF32().write(input_message_data_1)

    imu_message_1 = mimuMajorityVoteF32.ImuMessage()
    imu_message_1.imuSensorBodyInMsg.subscribeTo(in_msg_1)
    module.addImuInput(imu_message_1)

    input_message_data_2 = messaging.IMUSensorBodyMsgF32Payload()
    input_message_data_2.AngVelBody = angular_velocity_2.tolist()
    in_msg_2 = messaging.IMUSensorBodyMsgF32().write(input_message_data_2)

    imu_message_2 = mimuMajorityVoteF32.ImuMessage()
    imu_message_2.imuSensorBodyInMsg.subscribeTo(in_msg_2)
    module.addImuInput(imu_message_2)

    input_message_data_3 = messaging.IMUSensorBodyMsgF32Payload()
    input_message_data_3.AngVelBody = angular_velocity_3.tolist()
    in_msg_3 = messaging.IMUSensorBodyMsgF32().write(input_message_data_3)

    imu_message_3 = mimuMajorityVoteF32.ImuMessage()
    imu_message_3.imuSensorBodyInMsg.subscribeTo(in_msg_3)
    module.addImuInput(imu_message_3)

    rate_data_log = module.imuSensorBodyOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, rate_data_log)
    fault_data_log = module.mimuFaultMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, fault_data_log)

    unit_test_sim.InitializeSimulation()

    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))

    unit_test_sim.ExecuteSimulation()

    module_output_angular_velocity = rate_data_log.AngVelBody
    module_output_fault = fault_data_log.faultDetected
    module_output_fault_index = fault_data_log.mimuIndexFaulted

    np.testing.assert_allclose(
        module_output_angular_velocity[-1], expected_angular_velocity, rtol=0, atol=1e-7, verbose=True
    )
    np.testing.assert_allclose(module_output_fault[-1], expected_output_fault, verbose=True)
    np.testing.assert_allclose(module_output_fault_index[-1], expected_output_fault_index, verbose=True)
    np.testing.assert_allclose(module.omegaThreshold, omega_threshold_rad_per_sec, rtol=0, atol=1e-7, verbose=True)


if __name__ == "__main__":
    test_mimu_majority_vote_off_nominal()
