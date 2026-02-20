import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import averageMimuDataF32
from xmera.utilities import macros
from xmera.architecture import messaging
from xmera.utilities import RigidBodyKinematics


def test_average_mimu_data():
    """Module Unit Test"""
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    fsw_frequency = 10  # Set fsw frequency to 10 Hz
    mimu_frequency = 100  # Set mimu sensor frequency to 100 Hz
    num_fsw_steps = 12
    samples_per_fsw = int(mimu_frequency / fsw_frequency)
    max_acc_buf_pkt = 120  # matches MAX_ACC_BUF_PKT in C++
    delta_time_sim = 1 / fsw_frequency  # The averageMimuData module will run at the fsw frequency
    test_process_rate = macros.sec2nano(delta_time_sim)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module = averageMimuDataF32.AverageMimuData()
    module.modelTag = "averageMimuData"

    # Define parameters
    dcm_pltf_to_body = RigidBodyKinematics.euler3212C([0.01, -0.04, 0.06])
    module.setDcmPltfToBdy(dcm_pltf_to_body)
    duration_window = 1.0e10
    module.setAveragingWindow(duration_window)

    # Initialize message for message connection
    acc_data = messaging.AccDataMsgF32Payload()
    acc_data_msg = messaging.AccDataMsgF32().write(acc_data, time=0)

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.imuOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Connect messages
    module.accDataInMsg.subscribeTo(acc_data_msg)

    # Initialize simulation
    unit_test_sim.InitializeSimulation()

    # Write accData message
    delta_time = delta_time_sim / 10
    meas_time = 0
    sim_time = 0
    counter = 0
    gyro_sum = np.zeros(3)
    accel_sum = np.zeros(3)
    average_imu_output = np.zeros([num_fsw_steps, 3])
    average_acc_output = np.zeros([num_fsw_steps, 3])
    for index in range(num_fsw_steps):
        for _ in range(samples_per_fsw):
            meas_time += delta_time
            acc_data.accPkts[counter].measTime = macros.sec2nano(meas_time)
            random_array = np.random.normal(loc=2, scale=1, size=3)
            gyro_sum += random_array
            acc_data.accPkts[counter].gyro_B = random_array.tolist()
            random_array = np.random.normal(loc=2, scale=1, size=3)
            accel_sum += random_array
            acc_data.accPkts[counter].accel_B = random_array.tolist()
            counter += 1
        gyro_average = gyro_sum / max_acc_buf_pkt
        gyro_average = np.dot(dcm_pltf_to_body, gyro_average)
        accel_average = accel_sum / max_acc_buf_pkt
        accel_average = np.dot(dcm_pltf_to_body, accel_average)
        average_imu_output[index, :] = gyro_average
        average_acc_output[index, :] = accel_average
        sim_time += 1 / fsw_frequency
        acc_data_msg = messaging.AccDataMsgF32().write(acc_data, time=macros.sec2nano(sim_time))
        module.accDataInMsg.subscribeTo(acc_data_msg)
        unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
        unit_test_sim.ExecuteSimulation()

    # Test the getters
    np.testing.assert_allclose(duration_window, module.getAveragingWindow(), rtol=1e-8, atol=1e-6, verbose=True)
    np.testing.assert_allclose(dcm_pltf_to_body, module.getDcmPltfToBdy(), rtol=1e-8, atol=1e-6, verbose=True)

    # Pull logged data
    module_output_accel = data_log.AccelBody
    module_output_angular_velocity = data_log.AngVelBody

    # compare the module results to the truth values
    np.testing.assert_allclose(
        module_output_angular_velocity[1:, :], average_imu_output, rtol=1e-8, atol=1e-6, verbose=True
    )
    np.testing.assert_allclose(module_output_accel[1:, :], average_acc_output, rtol=1e-8, atol=1e-6, verbose=True)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_average_mimu_data()
