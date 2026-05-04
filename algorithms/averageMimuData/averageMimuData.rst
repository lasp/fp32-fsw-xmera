.. raw:: latex

    {\LARGE \textbf{averageMimuData}}

Executive Summary
-----------------
The ``averageMimuData`` algorithm computes a rolling average of recent gyro and accelerometer measurements held in an
internal ring that the algorithm itself owns and grows across update cycles. Each call to ``update()`` takes a snapshot
of up to ``MAX_MIMU_PKT_C`` (4) packets, each carrying a per-packet ``measTime`` (the timestamp of the first sample in
that group) plus ``MAX_MIMU_SAMPLES_PER_PKT_C`` (10) gyro/accel sample pairs. Phase 1 ingests any newly-arrived packets
- those whose ``measTime`` is strictly greater than the largest first-sample time ever ingested - into the next slot of
the ring, overwriting the oldest slot when capacity is reached. Phase 2 averages every sample currently in the ring
whose derived measurement time (``slot.measTime + s * kMimuSamplePeriodNs``) is within ``averagingWindow`` of the
newest stored sample's tail time, and rotates the result into the body frame using the user-provided DCM ``dcm_BP``.
Each contributing sample weighs equally. If the ring is empty or holds no fresh sample within the window, the output
is zero. The ring is sized at compile time to hold exactly ``kMaxAveragingWindowSec`` (2.0 s) of samples at the MIMU
device's compile-time sample rate ``kMimuSampleRateHz`` (100 Hz).

Message Connection Descriptions
-------------------------------
The following table lists the algorithm input and output data structures. The input is a 4-packet snapshot where each
packet carries a first-sample ``measTime`` plus ``MAX_MIMU_SAMPLES_PER_PKT_C`` gyro/accel samples; the output is the
averaged body-frame angular velocity and acceleration.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Variable Name
      - Type
      - Description
    * - mimuPacketInMsg
      - ``MimuPacketF32Payload``
      - Input snapshot: ``packets[MAX_MIMU_PKT]`` of ``MimuPacketGroupF32Payload`` plus ``isValid[MAX_MIMU_PKT]``.
        Each ``MimuPacketGroupF32Payload`` has a per-group ``measTime`` for the first sample.
    * - imuOutMsg
      - ``IMUSensorBodyMsgF32Payload``
      - Output averages in body frame: ``AccelBody`` and ``AngVelBody``

Module Parameters
-----------------
The following table lists all parameters that can be set. Parameters are optional unless indicated
(if not specified, the default is used).

.. list-table:: Module Parameters
    :widths: 22 18 12 16 22 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Setter / Getter
      - Description
    * - averagingWindow
      - double
      - [s]
      - 0.0
      - ``setAveragingWindow()`` / ``getAveragingWindow()``
      - Rolling time-window. Must be in ``[0.0, kMaxAveragingWindowSec]``; the setter throws otherwise.
    * - dcm_BP
      - Eigen::Matrix3f
      - [-]
      - Identity
      - ``setDcmPltfToBdy()`` / ``getDcmPltfToBdy()``
      - DCM mapping to body-frame vectors; the setter throws if not orthonormal with det=+1.

Module Assumptions and Limitations
----------------------------------
- ``measTime`` is in nanoseconds. The packet's ``measTime`` is the first sample's timestamp; subsequent samples are
  assumed to be at ``measTime + s * kMimuSamplePeriodNs`` for ``s = 1 .. N-1``. Upstream is responsible for ensuring
  this device-rate-derived schedule matches reality.
- Packet ``measTime`` values are assumed strictly monotonic across successive ``update()`` calls. A snapshot whose
  packets have ``measTime`` less than or equal to the prior call's max is dropped at ingest (no out-of-order ingestion).
- ``isValid`` gates a packet wholesale. When true, every sample in the packet is treated as real. There is no
  per-sample fill flag.
- A packet with ``isValid == true`` but ``measTime == 0`` is dropped at ingest. Empty ring slots remain
  ``isValid == false`` until ingested and therefore cannot pollute the average during warm-up.
- Among the samples currently in the ring, a measurement at derived time ``slot.measTime + s * kMimuSamplePeriodNs``
  is included in the average if ``(maxTimeTag - sampleMeasTime) <= averagingWindowNs``, where ``maxTimeTag`` is the
  newest stored slot's tail (``maxSlotMeasTime + (N - 1) * kMimuSamplePeriodNs``) and ``averagingWindowNs`` is the
  configured ``averagingWindow`` (seconds, in ``[0.0, kMaxAveragingWindowSec]``) converted once to nanoseconds. The
  comparison is done in ``uint64_t`` nanoseconds to avoid the precision loss that would occur casting a multi-second
  ``uint64_t`` delta through ``float``.
- ``setAveragingWindow`` throws if the argument is negative or exceeds ``kMaxAveragingWindowSec`` (the compile-time
  maximum). The ring is sized for exactly this maximum at the MIMU sample rate; larger windows can't be supported
  by a fixed-size buffer.
- ``setDcmPltfToBdy`` throws if the supplied matrix is not a proper DCM (orthonormal, right-handed, det=+1). The
  previously-stored DCM is preserved on rejection.
- If the ring is empty or holds no fresh sample within the averaging window, the output vectors are zero.
- The algorithm performs an unweighted arithmetic mean across measurements (no weighting, outlier rejection, or bias
  correction). Each contributing sample weighs equally regardless of which packet it came from.
- The internal ring is never reset. Setters (``setAveragingWindow``, ``setDcmPltfToBdy``) take effect on the next
  ``update()`` but do not clear stored data; callers that need a clean state must construct a new instance.

Initialization
--------------
Production wiring uses the SysModel-style ``AverageMimuData`` wrapper, which owns the algorithm and reads/writes
its messages::

    AverageMimuData mod;
    mod.setAveragingWindow(0.05F);           // float at the adapter; <= kMaxAveragingWindowSec
    mod.setDcmPltfToBdy(dcm_BP);             // Eigen::Matrix3f (platform-to-body)
    mod.mimuPacketInMsg.subscribeTo(...);    // required - reset() throws if not linked

Each cycle, the scheduler calls ``mod.updateState(callTime)``, which copies the latest ``MimuPacketF32Payload`` into
the algorithm and writes the body-frame averages to ``mod.imuOutMsg``.

For unit-test or standalone use, the algorithm class can be driven directly::

    AverageMimuDataAlgorithm alg;
    alg.setAveragingWindow(0.05);            // double at the algorithm layer
    alg.setDcmPltfToBdy(dcm_BP);
    OutputAverageAccelAngleVel out = alg.update(localPkts);

Detailed Module Description
---------------------------
Inputs
^^^^^^
The input payload carries ``packets[MAX_MIMU_PKT]`` (a 4-slot snapshot from one MIMU source) along with a parallel
``isValid[MAX_MIMU_PKT]`` array. Each slot is a ``MimuPacketGroupF32Payload`` carrying:

- ``measTime``: timestamp of the first sample in the packet (assumed nanoseconds)
- ``samples[MAX_MIMU_SAMPLES_PER_PKT]``: per-sample gyro/accel pairs. (Each ``AccPktDataMsgF32Payload`` still carries
  its own ``measTime`` field for legacy consumers, but the algorithm ignores it and uses the group-level ``measTime``
  + ``kMimuSamplePeriodNs`` schedule.)

The algorithm expects the inputs ``gyro_P`` and ``accel_P``, where the subscript P denotes the platform frame.
At the module interface, however, these signals are read from ``AccPktDataMsgF32Payload``, whose field names use the
legacy suffix B (``gyro_B`` and ``accel_B``). In this context, ``gyro_B`` and ``accel_B`` should therefore be
interpreted as platform-frame quantities, despite the field naming.

Configuration
^^^^^^^^^^^^^
1. Set ``averagingWindow`` to define how far back (from the newest fresh sample) to include samples in the average.
2. Set ``dcm_BP`` to map platform-frame vectors into the body frame.

Algorithm
^^^^^^^^^^^^^^^^
Given an input snapshot ``localPkts``, the algorithm executes in two phases.

Phase 1 - ingest:

a. Snapshot the prior-call ``lastIngestedMaxMeasTime`` as ``priorMax`` so that all packets in this snapshot are
   evaluated against the same boundary (allowing several new packets in one snapshot to be ingested together).
b. For each input packet ``packet``:

   - skip if ``packet.isValid == false``;
   - let ``firstSampleTime = packet.measTime``;
   - skip if ``firstSampleTime`` is ``0`` or not strictly greater than ``priorMax``;
   - otherwise copy the packet's samples and ``measTime`` into the next ring slot, advance the insert index modulo the
     ring capacity (overwriting the oldest slot when full), and update ``lastIngestedMaxMeasTime`` to the max of
     itself and ``firstSampleTime``.

Phase 2 - average over the ring:

0. Find the newest stored slot's ``measTime`` (``maxSlotMeasTime``) across all valid ring slots. If no slot is valid,
   return a zero output.
1. Compute ``maxTimeTag = maxSlotMeasTime + (N - 1) * kMimuSamplePeriodNs`` -- the tail sample's time of the newest
   stored packet.
2. For each valid ring slot and each ``s`` in ``0 .. N - 1``, compute the derived sample time
   ``sampleMeasTime = slot.measTime + s * kMimuSamplePeriodNs`` and include the sample iff
   ``(maxTimeTag - sampleMeasTime) <= averagingWindowNs``.
3. Average the selected gyro and accelerometer vectors (in the platform frame), weighting each sample equally.
4. Transform the averaged vectors to the body frame using ``dcm_BP`` and return them.

Recommended Practices
^^^^^^^^^^^^^^^^^^^^^
- Choose ``averagingWindow`` based on the desired smoothing. Larger values increase smoothing but introduce more
  latency. The setter rejects values outside ``[0.0, kMaxAveragingWindowSec]``.
- ``dcm_BP`` shall be a proper DCM (orthonormal, right-handed).
- Set ``packet.measTime`` to the timestamp of the first sample in each packet. Subsequent samples are scheduled at
  ``measTime + s * kMimuSamplePeriodNs`` regardless of what the per-sample ``AccPktDataMsgF32Payload.measTime`` says.
- The caller is responsible for maintaining ``isValid[p]``. Producers should clear ``isValid[p]`` for any packet that
  has not been written this cycle; the algorithm will treat the whole packet as stale.

Outputs
^^^^^^^
The algorithm returns::

    struct OutputAverageAccelAngleVel {
        Eigen::Vector3f accel_B = Eigen::Vector3f::Zero();       // body-frame averaged acceleration
        Eigen::Vector3f gyroOmega_B = Eigen::Vector3f::Zero();   // body-frame averaged angular velocity
    };

If no sample qualifies under the window filter, both output vectors are returned as zeros.

API Reference
-------------
Class Interface
^^^^^^^^^^^^^^^
The algorithm is implemented by::

    class AverageMimuDataAlgorithm {
       public:
        // Compile-time MIMU device sample rate + derived period (ns).
        static constexpr float         kMimuSampleRateHz     = 100.0F;
        static constexpr std::uint64_t kMimuSamplePeriodNs   = 10'000'000U;

        // Compile-time cap on the configured averaging window. Ring capacity is
        // sized to hold exactly kMaxAveragingWindowSec of samples at the MIMU rate.
        static constexpr float kMaxAveragingWindowSec = 2.0F;

        static constexpr std::size_t kRingCapacity =
            average_mimu_detail::ceilDivSamplesToPackets(
                kMimuSampleRateHz, kMaxAveragingWindowSec, MAX_MIMU_SAMPLES_PER_PKT_C);

        void setAveragingWindow(double averagingWindowIn);
        double getAveragingWindow() const;

        void setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn);
        Eigen::Matrix3f getDcmPltfToBdy() const;

        OutputAverageAccelAngleVel update(InputPktsData const& localPkts);
    };

Component (SysModel) Wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The SysModel-style wrapper class is ``AverageMimuData`` (declared in ``averageMimuData.h``). It owns a single
``AverageMimuDataAlgorithm`` instance and is the production entry point. Notable behaviors layered on top of the
algorithm:

- ``reset(callTime)`` throws ``std::invalid_argument`` if ``mimuPacketInMsg`` is not linked.
- ``updateState(callTime)`` reads the latest ``MimuPacketF32Payload``, copies it into the algorithm's
  ``InputPktsData``, calls ``algorithm.update(...)``, and writes ``AccelBody`` / ``AngVelBody`` on
  ``imuOutMsg``. The ``DVFrameBody`` and ``DRFrameBody`` fields on the output payload are left zero - the
  algorithm does not compute integrated DVs/DRs.
- If ``mimuPacketInMsg.timeWritten()`` has not changed since the previous ``updateState``, the call is skipped,
  ``staleDataCount`` is incremented, and no new output is written. The algorithm's internal ring is unchanged on a
  skipped cycle.
- ``setAveragingWindow`` on the adapter takes ``float`` and delegates to the algorithm's ``double`` setter; this
  is the only public-API type difference between the two layers.

Input Type
^^^^^^^^^^
The algorithm-internal input types are::

    struct Sample {
        Eigen::Vector3f gyro_P;
        Eigen::Vector3f accel_P;
    };

    struct InputPacket {
        bool          isValid;
        std::uint64_t measTime;   // First sample's measurement time (firstSampleTime)
        std::array<Sample, MAX_MIMU_SAMPLES_PER_PKT_C> samples;
    };

    struct InputPktsData {
        std::array<InputPacket, MAX_MIMU_PKT_C> packets;
    };

Notes
-----
- The averaging window is anchored to the newest sample's derived time, not to the current system time.
- The output is deterministic given the input snapshot history, ``averagingWindow``, and ``dcm_BP``.
