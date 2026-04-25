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


def test_average_mimu_data_buffer_fill():
    """Walk a 4-packet ring buffer from empty to full, then wrap.

    Each packet carries MAX_MIMU_SAMPLES_PER_PKT individual samples. The
    module should skip packets with isValid=False and skip samples with
    measTime=0 within fresh packets. The averaged output should match the
    hand-computed mean of the fresh samples at each cycle.
    """
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    fsw_frequency = 10
    max_mimu_pkt = 4
    max_mimu_samples_per_pkt = 10
    delta_time_sim = 1 / fsw_frequency
    test_process_rate = macros.sec2nano(delta_time_sim)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = averageMimuDataF32.AverageMimuData()
    module.modelTag = "averageMimuData"

    dcm_pltf_to_body = np.eye(3, dtype=np.float32)
    module.setDcmPltfToBdy(dcm_pltf_to_body)
    module.setAveragingWindow(1.0e10)  # wide window so only staleness matters

    mimu_pkt = messaging.MimuPacketF32Payload()
    mimu_pkt_msg = messaging.MimuPacketF32().write(mimu_pkt, time=0)

    unit_test_sim.AddModelToTask(unit_task_name, module)
    data_log = module.imuOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)
    module.mimuPacketInMsg.subscribeTo(mimu_pkt_msg)
    unit_test_sim.InitializeSimulation()

    # Deterministic per-(packet, sample) gyro/accel grid and timestamps.
    gyros = np.zeros((max_mimu_pkt, max_mimu_samples_per_pkt, 3), dtype=np.float32)
    accels = np.zeros((max_mimu_pkt, max_mimu_samples_per_pkt, 3), dtype=np.float32)
    times_s = np.zeros((max_mimu_pkt, max_mimu_samples_per_pkt), dtype=np.float64)
    for p in range(max_mimu_pkt):
        for s in range(max_mimu_samples_per_pkt):
            gyros[p, s] = np.array([float(p), float(s), float(p + s)], dtype=np.float32)
            accels[p, s] = np.array([float(p + 1), float(s + 1), float(p - s)], dtype=np.float32)
            # 1 ms spacing keeps samples ordered and within the wide window.
            times_s[p, s] = 0.10 + 0.001 * (p * max_mimu_samples_per_pkt + s)

    # Pre-load every packet with its samples once; cycles only flip isValid.
    for p in range(max_mimu_pkt):
        for s in range(max_mimu_samples_per_pkt):
            mimu_pkt.packets[p].samples[s].measTime = macros.sec2nano(float(times_s[p, s]))
            mimu_pkt.packets[p].samples[s].gyro_B = gyros[p, s].tolist()
            mimu_pkt.packets[p].samples[s].accel_B = accels[p, s].tolist()

    cycles = 6
    expected_gyro = np.zeros((cycles, 3), dtype=np.float32)
    expected_accel = np.zeros((cycles, 3), dtype=np.float32)
    sim_time = 0.0
    valid_flags = [False] * max_mimu_pkt

    for cycle in range(cycles):
        if cycle == 0:
            # Empty buffer.
            valid_flags = [False] * max_mimu_pkt
            expected_gyro[cycle] = 0.0
            expected_accel[cycle] = 0.0
        elif cycle <= max_mimu_pkt:
            # Mark the first `cycle` packets valid.
            valid_flags = [i < cycle for i in range(max_mimu_pkt)]
            valid_pkts = cycle
            sample_count = valid_pkts * max_mimu_samples_per_pkt
            expected_gyro[cycle] = gyros[:valid_pkts].reshape(-1, 3).sum(axis=0) / sample_count
            expected_accel[cycle] = accels[:valid_pkts].reshape(-1, 3).sum(axis=0) / sample_count
        else:
            # Wrap: overwrite packet 0 sample 0 with a much newer sample. With
            # the wide window, every fresh sample still qualifies; the mean
            # simply swaps the old packet[0][0] for the new one.
            new_gyro = np.array([-1.0, -2.0, -3.0], dtype=np.float32)
            new_accel = np.array([-4.0, -5.0, -6.0], dtype=np.float32)
            mimu_pkt.packets[0].samples[0].measTime = macros.sec2nano(1.0)
            mimu_pkt.packets[0].samples[0].gyro_B = new_gyro.tolist()
            mimu_pkt.packets[0].samples[0].accel_B = new_accel.tolist()
            sample_count = max_mimu_pkt * max_mimu_samples_per_pkt
            gyro_sum = gyros.reshape(-1, 3).sum(axis=0) - gyros[0, 0] + new_gyro
            accel_sum = accels.reshape(-1, 3).sum(axis=0) - accels[0, 0] + new_accel
            expected_gyro[cycle] = gyro_sum / sample_count
            expected_accel[cycle] = accel_sum / sample_count

        mimu_pkt.isValid = list(valid_flags)
        sim_time += delta_time_sim
        mimu_pkt_msg = messaging.MimuPacketF32().write(mimu_pkt, time=macros.sec2nano(sim_time))
        module.mimuPacketInMsg.subscribeTo(mimu_pkt_msg)
        unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
        unit_test_sim.ExecuteSimulation()

    module_output_gyro = data_log.AngVelBody[1:, :]
    module_output_accel = data_log.AccelBody[1:, :]

    np.testing.assert_allclose(module_output_gyro, expected_gyro, rtol=1e-6, atol=1e-6)
    np.testing.assert_allclose(module_output_accel, expected_accel, rtol=1e-6, atol=1e-6)


if __name__ == "__main__":
    test_average_mimu_data()
    test_average_mimu_data_buffer_fill()
