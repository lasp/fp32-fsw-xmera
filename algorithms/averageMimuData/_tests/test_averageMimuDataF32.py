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

    fsw_frequency = 10  # Set fsw frequency to 10 Hz
    num_fsw_steps = 12
    max_mimu_pkt = 4
    max_mimu_samples_per_pkt = 10  # matches MAX_MIMU_SAMPLES_PER_PKT in C++
    delta_time_sim = 1 / fsw_frequency
    test_process_rate = macros.sec2nano(delta_time_sim)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = averageMimuDataF32.AverageMimuData()
    module.modelTag = "averageMimuData"

    dcm_pltf_to_body = RigidBodyKinematics.euler3212C([0.01, -0.04, 0.06]).astype(np.float32)
    module.setDcmPltfToBdy(dcm_pltf_to_body)
    duration_window = 1.0e10  # huge window so the average covers every fresh sample
    module.setAveragingWindow(duration_window)

    mimu_pkt = messaging.MimuPacketF32Payload()
    mimu_pkt_msg = messaging.MimuPacketF32().write(mimu_pkt, time=0)

    unit_test_sim.AddModelToTask(unit_task_name, module)
    data_log = module.imuOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)
    module.mimuPacketInMsg.subscribeTo(mimu_pkt_msg)
    unit_test_sim.InitializeSimulation()

    total_samples_per_cycle = max_mimu_pkt * max_mimu_samples_per_pkt
    delta_time = delta_time_sim / total_samples_per_cycle
    meas_time = 0.0
    sim_time = 0.0
    expected_gyro = np.zeros([num_fsw_steps, 3], dtype=np.float32)
    expected_accel = np.zeros([num_fsw_steps, 3], dtype=np.float32)
    for index in range(num_fsw_steps):
        gyro_sum = np.zeros(3, dtype=np.float32)
        accel_sum = np.zeros(3, dtype=np.float32)
        for p in range(max_mimu_pkt):
            for s in range(max_mimu_samples_per_pkt):
                meas_time += delta_time
                mimu_pkt.packets[p].samples[s].measTime = macros.sec2nano(meas_time)
                gyro_sample = np.random.normal(loc=2, scale=1, size=3).astype(np.float32)
                gyro_sum += gyro_sample
                mimu_pkt.packets[p].samples[s].gyro_B = gyro_sample.tolist()
                accel_sample = np.random.normal(loc=2, scale=1, size=3).astype(np.float32)
                accel_sum += accel_sample
                mimu_pkt.packets[p].samples[s].accel_B = accel_sample.tolist()
        mimu_pkt.isValid = [True] * max_mimu_pkt
        gyro_average = np.dot(dcm_pltf_to_body, gyro_sum / total_samples_per_cycle)
        accel_average = np.dot(dcm_pltf_to_body, accel_sum / total_samples_per_cycle)
        expected_gyro[index, :] = gyro_average
        expected_accel[index, :] = accel_average
        sim_time += 1 / fsw_frequency
        mimu_pkt_msg = messaging.MimuPacketF32().write(mimu_pkt, time=macros.sec2nano(sim_time))
        module.mimuPacketInMsg.subscribeTo(mimu_pkt_msg)
        unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
        unit_test_sim.ExecuteSimulation()

    np.testing.assert_allclose(duration_window, module.getAveragingWindow(), rtol=1e-8, atol=1e-6, verbose=True)
    np.testing.assert_allclose(dcm_pltf_to_body, module.getDcmPltfToBdy(), rtol=1e-8, atol=1e-6, verbose=True)

    module_output_accel = data_log.AccelBody
    module_output_angular_velocity = data_log.AngVelBody

    np.testing.assert_allclose(
        module_output_angular_velocity[1:, :], expected_gyro, rtol=1e-8, atol=1e-6, verbose=True
    )
    np.testing.assert_allclose(module_output_accel[1:, :], expected_accel, rtol=1e-8, atol=1e-6, verbose=True)


if __name__ == "__main__":
    test_average_mimu_data()
