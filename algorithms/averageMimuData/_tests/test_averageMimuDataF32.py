import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import averageMimuDataF32
from xmera.utilities import macros
from xmera.architecture import messaging
from xmera.utilities import RigidBodyKinematics


# Algorithm-side compile-time constants. Mirrors averageMimuDataAlgorithm.h.
_MIMU_SAMPLE_PERIOD_NS = 10_000_000  # 100 Hz device sample rate
_MAX_AVG_WINDOW_SEC = 2.0
_RING_CAPACITY_PACKETS = 20  # ceil(100 Hz * 2 s / 10 samples_per_pkt)
_MAX_MIMU_PKT = 4
_MAX_MIMU_SAMPLES_PER_PKT = 10
_PACKET_SPAN_NS = _MIMU_SAMPLE_PERIOD_NS * _MAX_MIMU_SAMPLES_PER_PKT  # 100 ms / packet


def test_average_mimu_data():
    """Module Unit Test: cumulative averaging across cycles with ring eviction."""
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    fsw_frequency = 10  # 10 Hz FSW
    num_fsw_steps = 12
    delta_time_sim = 1 / fsw_frequency
    test_process_rate = macros.sec2nano(delta_time_sim)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = averageMimuDataF32.AverageMimuData()
    module.modelTag = "averageMimuData"

    dcm_pltf_to_body = RigidBodyKinematics.euler3212C([0.01, -0.04, 0.06]).astype(np.float32)
    module.setDcmPltfToBdy(dcm_pltf_to_body)
    module.setGyroAveragingWindow(_MAX_AVG_WINDOW_SEC)  # 2 s window covers the full ring
    module.setAccelAveragingWindow(_MAX_AVG_WINDOW_SEC)  # accel uses the same full-ring window here

    mimu_pkt = messaging.MimuPacketF32Payload()
    mimu_pkt_msg = messaging.MimuPacketF32().write(mimu_pkt, time=0)

    unit_test_sim.AddModelToTask(unit_task_name, module)
    data_log = module.imuOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)
    module.mimuPacketInMsg.subscribeTo(mimu_pkt_msg)
    unit_test_sim.InitializeSimulation()

    expected_gyro = np.zeros([num_fsw_steps, 3], dtype=np.float32)
    expected_accel = np.zeros([num_fsw_steps, 3], dtype=np.float32)

    # Each cycle ingests _MAX_MIMU_PKT new packets (always strictly newer than
    # the prior cycle's max). Once (k+1)*_MAX_MIMU_PKT exceeds _RING_CAPACITY,
    # the oldest packets get overwritten on insert. cycles_per_ring is the
    # number of whole cycles' worth of packets the ring holds.
    cycles_per_ring = _RING_CAPACITY_PACKETS // _MAX_MIMU_PKT  # 5
    cycle_gyro_sums = []
    cycle_accel_sums = []

    # Monotonic packet-time counter. Each packet advances by _PACKET_SPAN_NS.
    next_packet_time_ns = macros.sec2nano(0.001)
    sim_time = 0.0

    for index in range(num_fsw_steps):
        gyro_sum = np.zeros(3, dtype=np.float32)
        accel_sum = np.zeros(3, dtype=np.float32)
        for p in range(_MAX_MIMU_PKT):
            mimu_pkt.packets[p].measTime = next_packet_time_ns
            next_packet_time_ns += _PACKET_SPAN_NS
            for s in range(_MAX_MIMU_SAMPLES_PER_PKT):
                gyro_sample = np.random.normal(loc=2, scale=1, size=3).astype(np.float32)
                accel_sample = np.random.normal(loc=2, scale=1, size=3).astype(np.float32)
                mimu_pkt.packets[p].samples[s].gyro_B = gyro_sample.tolist()
                mimu_pkt.packets[p].samples[s].accel_B = accel_sample.tolist()
                gyro_sum += gyro_sample
                accel_sum += accel_sample
        mimu_pkt.isValid = [True] * _MAX_MIMU_PKT
        cycle_gyro_sums.append(gyro_sum)
        cycle_accel_sums.append(accel_sum)

        cycles_in_ring = min(index + 1, cycles_per_ring)
        first_retained = (index + 1) - cycles_in_ring
        cumulative_gyro = np.sum(cycle_gyro_sums[first_retained : index + 1], axis=0)
        cumulative_accel = np.sum(cycle_accel_sums[first_retained : index + 1], axis=0)
        sample_count = cycles_in_ring * _MAX_MIMU_PKT * _MAX_MIMU_SAMPLES_PER_PKT

        expected_gyro[index, :] = np.dot(dcm_pltf_to_body, cumulative_gyro / sample_count)
        expected_accel[index, :] = np.dot(dcm_pltf_to_body, cumulative_accel / sample_count)
        sim_time += delta_time_sim
        mimu_pkt_msg = messaging.MimuPacketF32().write(mimu_pkt, time=macros.sec2nano(sim_time))
        module.mimuPacketInMsg.subscribeTo(mimu_pkt_msg)
        unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
        unit_test_sim.ExecuteSimulation()

    np.testing.assert_allclose(_MAX_AVG_WINDOW_SEC, module.getGyroAveragingWindow(), rtol=1e-8, atol=1e-9, verbose=True)
    np.testing.assert_allclose(dcm_pltf_to_body, module.getDcmPltfToBdy(), rtol=1e-8, atol=1e-6, verbose=True)

    module_output_accel = data_log.AccelBody
    module_output_angular_velocity = data_log.AngVelBody

    np.testing.assert_allclose(
        module_output_angular_velocity[1:, :], expected_gyro, rtol=1e-6, atol=1e-6, verbose=True
    )
    np.testing.assert_allclose(module_output_accel[1:, :], expected_accel, rtol=1e-6, atol=1e-6, verbose=True)


def test_average_mimu_data_buffer_fill():
    """Walk the algorithm-owned ring buffer from empty to wrap.

    Cycle 0: no valid packets -> zero output.
    Cycles 1..4: progressively mark packets 0..(cycle-1) valid. Each cycle
                 only the newest packet (one not yet in the ring) is ingested
                 due to strict-monotonic by packet measTime.
    Cycle 5:     wrap. Packet 0's measTime jumps far ahead; only that packet
                 is ingested as a new ring slot. Other packets are dropped.
    """
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    fsw_frequency = 10
    delta_time_sim = 1 / fsw_frequency
    test_process_rate = macros.sec2nano(delta_time_sim)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = averageMimuDataF32.AverageMimuData()
    module.modelTag = "averageMimuData"

    dcm_pltf_to_body = np.eye(3, dtype=np.float32)
    module.setDcmPltfToBdy(dcm_pltf_to_body)
    module.setGyroAveragingWindow(_MAX_AVG_WINDOW_SEC)  # 2 s covers the full ring
    module.setAccelAveragingWindow(_MAX_AVG_WINDOW_SEC)  # accel uses the same full-ring window here

    mimu_pkt = messaging.MimuPacketF32Payload()
    mimu_pkt_msg = messaging.MimuPacketF32().write(mimu_pkt, time=0)

    unit_test_sim.AddModelToTask(unit_task_name, module)
    data_log = module.imuOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)
    module.mimuPacketInMsg.subscribeTo(mimu_pkt_msg)
    unit_test_sim.InitializeSimulation()

    # Deterministic per-(packet, sample) gyro/accel grid.
    gyros = np.zeros((_MAX_MIMU_PKT, _MAX_MIMU_SAMPLES_PER_PKT, 3), dtype=np.float32)
    accels = np.zeros((_MAX_MIMU_PKT, _MAX_MIMU_SAMPLES_PER_PKT, 3), dtype=np.float32)
    for p in range(_MAX_MIMU_PKT):
        for s in range(_MAX_MIMU_SAMPLES_PER_PKT):
            gyros[p, s] = np.array([float(p), float(s), float(p + s)], dtype=np.float32)
            accels[p, s] = np.array([float(p + 1), float(s + 1), float(p - s)], dtype=np.float32)

    # Pre-load every packet's gyro/accel. measTimes spaced by packet span so
    # each subsequent packet is strictly newer (one packet's tail = next's head).
    base_packet_time_ns = macros.sec2nano(0.001)
    for p in range(_MAX_MIMU_PKT):
        mimu_pkt.packets[p].measTime = base_packet_time_ns + (p * _PACKET_SPAN_NS)
        for s in range(_MAX_MIMU_SAMPLES_PER_PKT):
            mimu_pkt.packets[p].samples[s].gyro_B = gyros[p, s].tolist()
            mimu_pkt.packets[p].samples[s].accel_B = accels[p, s].tolist()

    cycles = 6
    expected_gyro = np.zeros((cycles, 3), dtype=np.float32)
    expected_accel = np.zeros((cycles, 3), dtype=np.float32)
    sim_time = 0.0
    valid_flags = [False] * _MAX_MIMU_PKT

    for cycle in range(cycles):
        if cycle == 0:
            valid_flags = [False] * _MAX_MIMU_PKT
            expected_gyro[cycle] = 0.0
            expected_accel[cycle] = 0.0
        elif cycle <= _MAX_MIMU_PKT:
            # Mark the first `cycle` packets valid. Strict-monotonic ingest
            # means only the newest (packet `cycle-1`) is added each call;
            # the earlier packets were already ingested on prior cycles.
            valid_flags = [i < cycle for i in range(_MAX_MIMU_PKT)]
            valid_pkts = cycle
            sample_count = valid_pkts * _MAX_MIMU_SAMPLES_PER_PKT
            expected_gyro[cycle] = gyros[:valid_pkts].reshape(-1, 3).sum(axis=0) / sample_count
            expected_accel[cycle] = accels[:valid_pkts].reshape(-1, 3).sum(axis=0) / sample_count
        else:
            # Wrap: packet 0's measTime jumps far ahead. Only this packet is
            # ingested; the others (still at their original measTimes) are
            # rejected by the strict-monotonic gate. Ring grows from 4 to 5
            # slots; each of those 10-sample groups uses the derived
            # schedule for staleness so all 50 samples qualify.
            wrap_gyros = gyros[0].copy()
            wrap_accels = accels[0].copy()
            wrap_gyros[0] = np.array([-1.0, -2.0, -3.0], dtype=np.float32)
            wrap_accels[0] = np.array([-4.0, -5.0, -6.0], dtype=np.float32)
            mimu_pkt.packets[0].measTime = macros.sec2nano(1.0)
            for s in range(_MAX_MIMU_SAMPLES_PER_PKT):
                mimu_pkt.packets[0].samples[s].gyro_B = wrap_gyros[s].tolist()
                mimu_pkt.packets[0].samples[s].accel_B = wrap_accels[s].tolist()

            sample_count = 5 * _MAX_MIMU_SAMPLES_PER_PKT
            gyro_sum = gyros.reshape(-1, 3).sum(axis=0) + wrap_gyros.sum(axis=0)
            accel_sum = accels.reshape(-1, 3).sum(axis=0) + wrap_accels.sum(axis=0)
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
