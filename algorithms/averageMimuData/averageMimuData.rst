.. raw:: latex

    {\LARGE \textbf{averageMimuData}}

Executive Summary
-----------------
The ``averageMimuData`` algorithm computes a rolling average of recent gyro and accelerometer measurements held in an
internal ring that the algorithm itself owns and grows across update cycles. Each call to ``update()`` takes a snapshot
of up to ``MAX_MIMU_PKT_C`` (4) packets, each carrying a per-packet ``measTime`` (the timestamp of the first sample in
that group) plus ``MAX_MIMU_SAMPLES_PER_PKT_C`` (10) gyro/accel sample pairs. Phase 1 ingests each packet that is valid
and has a nonzero ``measTime`` (including out-of-order packets) into the next slot of the ring, overwriting the oldest
slot when capacity is reached. Phase 2 averages the ring, but gyro and accelerometer are filtered over **independent**
time windows anchored to the newest stored sample's tail time (``maxTimeTag``): a sample (derived measurement time
``slot.measTime + s * kMimuSamplePeriodNs``) contributes to the angular-rate mean when its age relative to
``maxTimeTag`` is within ``gyroAveragingWindow``, and to the acceleration mean when within ``accelAveragingWindow``.
Each mean is rotated into the body frame using the configured DCM ``dcm_BC`` that maps the Camera Head Unit (CHU)
frame to the body frame. Each contributing sample weighs equally.
A modality with no sample inside its window (or an empty ring) yields a zero vector for that modality. The ring is
sized at compile time to
hold exactly ``kMaxAveragingWindowSec`` (2.0 s) of samples at the MIMU device's compile-time sample rate
``kMimuSampleRateHz`` (100 Hz); both windows share this ring and are each capped at ``kMaxAveragingWindowSec``.

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
All parameters are supplied through the validated ``AverageMimuDataConfig`` (built by ``AverageMimuDataConfig::create``,
which throws ``fsw::invalid_argument`` on an invalid value). On the ``AverageMimuData`` wrapper they are exposed as
public member variables set before ``reset()``; ``reset()`` freezes them into the config.

.. list-table:: Module Parameters
    :widths: 22 18 12 16 32
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description / Validation
    * - gyroAveragingWindow
      - float
      - [s]
      - 0.0
      - Rolling time-window for angular rate. Must be in ``[0.0, kMaxAveragingWindowSec]``.
    * - accelAveragingWindow
      - float
      - [s]
      - 0.0
      - Rolling time-window for acceleration. Must be in ``[0.0, kMaxAveragingWindowSec]``.
    * - dcm_BC
      - Eigen::Matrix3f
      - [-]
      - Identity
      - DCM mapping CHU-frame vectors to the body frame. Must be orthonormal with det=+1.

Module Assumptions and Limitations
----------------------------------
- The averaging window ends at ``maxTimeTag``, the tail-sample time of the newest stored packet
  (``max(slot.measTime) + (N - 1) * kMimuSamplePeriodNs``). A sample contributes only when its age relative to
  ``maxTimeTag`` is within the modality's window.
- Packets may arrive out of order; a packet is ingested whenever it is valid and has a nonzero ``measTime``. There is no
  duplicate detection, so the caller must not present the same packet in more than one snapshot.
- ``measTime`` is in nanoseconds. The packet's ``measTime`` is the first sample's timestamp; subsequent samples are
  assumed to be at ``measTime + s * kMimuSamplePeriodNs`` for ``s = 1 .. N-1``. Upstream is responsible for ensuring
  this device-rate-derived schedule matches reality.
- A packet with ``isValid == true`` but ``measTime == 0`` is dropped at ingest. Empty ring slots remain
  ``isValid == false`` until ingested and therefore cannot pollute the average during warm-up.
- If the ring is empty, or there is no sample within a window, the output vector is zero.
- The algorithm performs an unweighted arithmetic mean across measurements.
- The internal ring is seeded empty at construction and cleared by ``reInitialize()`` (invoked from the wrapper's
  ``reset()`` / ``reInitialize()``).

Initialization
--------------
Production wiring uses the SysModel-style ``AverageMimuData`` wrapper, which owns the algorithm and reads/writes its
messages. Configuration parameters are public properties set before ``reset()``::

    AverageMimuData mod;
    mod.gyroAveragingWindow = 0.05;          // <= kMaxAveragingWindowSec
    mod.accelAveragingWindow = 0.10;         // independent of the gyro window
    mod.dcm_BC = dcm_BC;                      // Eigen::Matrix3f (CHU-to-body)
    mod.mimuPacketInMsg.subscribeTo(...);    // required - reset() throws if not linked

``reset(callTime)`` validates the linked message, freezes the public properties into an ``AverageMimuDataConfig`` (which
throws on an invalid value), and constructs the algorithm. Each cycle, the scheduler calls ``mod.updateState(callTime)``,
which copies the latest ``MimuPacketF32Payload`` into the algorithm and writes the body-frame averages to
``mod.imuOutMsg``. To change parameters after ``reset()``, update the public properties and call ``reconfigure()``;
``reInitialize()`` clears the ring without discarding the configuration.

For unit-test or standalone use, the algorithm class is constructed from a validated config and driven directly::

    AverageMimuDataConfig cfg = AverageMimuDataConfig::create(0.05, 0.10, dcm_BC);
    AverageMimuDataAlgorithm alg(cfg);
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

The algorithm expects the inputs ``gyro_C`` and ``accel_C``, where the subscript C denotes the CHU frame.
At the module interface, however, these signals are read from ``AccPktDataMsgF32Payload``, whose field names use the
legacy suffix B (``gyro_B`` and ``accel_B``). In this context, ``gyro_B`` and ``accel_B`` should therefore be
interpreted as CHU-frame quantities, despite the field naming.

Configuration
^^^^^^^^^^^^^
1. Set ``gyroAveragingWindow`` and ``accelAveragingWindow`` to define, independently per modality, how far back (from
   the newest stored sample) to include samples in the average.
2. Set ``dcm_BC`` to map CHU-frame vectors into the body frame.

Algorithm
^^^^^^^^^^^^^^^^
Given an input snapshot ``localPkts``, the algorithm executes in two phases.

Phase 1 - ingest:

For each input packet ``packet``:

- skip if ``packet.isValid == false`` or ``packet.measTime == 0``;
- otherwise copy the packet's samples and ``measTime`` into the next ring slot and advance the insert index modulo the
  ring capacity (overwriting the oldest slot when full).

Out-of-order packets (a ``measTime`` older than others already stored) are ingested; staleness is handled entirely by
the Phase 2 window rather than by rejecting them at ingest. There is no duplicate detection at ingest.

Phase 2 - average over the ring:

0. Find the newest stored slot's ``measTime`` (``maxSlotMeasTime``) across all valid ring slots. If no slot is valid,
   return a zero output.
1. Compute ``maxTimeTag = maxSlotMeasTime + (N - 1) * kMimuSamplePeriodNs`` -- the tail sample's time of the newest
   stored packet.
2. For each valid ring slot and each ``s`` in ``0 .. N - 1``, compute the derived sample time
   ``sampleMeasTime = slot.measTime + s * kMimuSamplePeriodNs`` and its age ``maxTimeTag - sampleMeasTime``. Add the
   gyro vector to the gyro accumulator (and bump the gyro count) iff the age is ``<= gyroAveragingWindowNs``; add the
   accel vector to the accel accumulator (and bump the accel count) iff the age is ``<= accelAveragingWindowNs``.
3. Divide each accumulator by its own count to form the CHU-frame means, weighting each sample equally. A count of
   zero leaves that modality's mean at zero.
4. Transform each averaged vector to the body frame using ``dcm_BC`` and return them.

Recommended Practices
^^^^^^^^^^^^^^^^^^^^^
- Choose ``gyroAveragingWindow`` and ``accelAveragingWindow`` independently based on the desired smoothing for each
  modality. Larger values increase smoothing but introduce more latency. ``create()`` rejects values outside
  ``[0.0, kMaxAveragingWindowSec]``.
- ``dcm_BC`` shall be a proper DCM (orthonormal, right-handed).
- Set ``packet.measTime`` to the timestamp of the first sample in each packet. Subsequent samples are scheduled at
  ``measTime + s * kMimuSamplePeriodNs`` regardless of what the per-sample ``AccPktDataMsgF32Payload.measTime`` says.
- The caller is responsible for maintaining ``isValid[p]``. Producers should clear ``isValid[p]`` for any packet that
  has not been written this cycle; the algorithm will skip the whole packet.

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
The configuration and algorithm classes are::

    class AverageMimuDataConfig {
       public:
        // Validates the parameters and throws fsw::invalid_argument on a bad value.
        static AverageMimuDataConfig create(float gyroAveragingWindow, float accelAveragingWindow,
                                            Eigen::Matrix3f const& dcm_BC);

        static bool isValidGyroAveragingWindow(float window);
        static bool isValidAccelAveragingWindow(float window);
        static bool isValidDcmChuToBody(Eigen::Matrix3f const& dcm_BC);

        float getGyroAveragingWindow() const;
        float getAccelAveragingWindow() const;
        Eigen::Matrix3f const& getDcmChuToBody() const;
    };

    class AverageMimuDataAlgorithm {
       public:
        // Compile-time MIMU device sample rate + derived period (ns).
        static constexpr double        kMimuSampleRateHz   = 100.0;
        static constexpr std::uint64_t kMimuSamplePeriodNs =
            static_cast<std::uint64_t>(kSec2Nano / kMimuSampleRateHz);   // 1e7 ns (10 ms)

        // Compile-time cap on the configured averaging window. Ring capacity is
        // sized to hold exactly kMaxAveragingWindowSec of samples at the MIMU rate.
        static constexpr float       kMaxAveragingWindowSec = 2.0F;
        static constexpr std::size_t kRingCapacity          = /* ceil(rate * window / samples-per-pkt) */;

        explicit AverageMimuDataAlgorithm(AverageMimuDataConfig const& config);
        void setConfig(AverageMimuDataConfig const& config);   // replace config; runtime state untouched
        void reInitialize();                                   // clear the ring

        OutputAverageAccelAngleVel update(InputPktsData const& localPkts);
    };

Component (SysModel) Wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The SysModel-style wrapper class is ``AverageMimuData`` (declared in ``averageMimuData.h``). It holds the algorithm in a
``std::unique_ptr`` (constructed on ``reset()``) and is the production entry point. Notable behaviors layered on top of
the algorithm:

- The configuration parameters (``gyroAveragingWindow``, ``accelAveragingWindow``, ``dcm_BC``) are public member
  variables set before ``reset()``.
- ``reset(callTime)`` throws ``std::invalid_argument`` if ``mimuPacketInMsg`` is not linked, clears ``prevInMsgTime``,
  and constructs the algorithm from a validated config (``create()`` throws on a bad value).
- ``updateState(callTime)`` reads the latest ``MimuPacketF32Payload``, copies it into the algorithm's ``InputPktsData``,
  calls ``algorithm->update(...)``, and writes ``AccelBody`` / ``AngVelBody`` on ``imuOutMsg``. The
  ``DVFrameBody`` and ``DRFrameBody`` fields on the output payload are left zero - the algorithm does not compute
  integrated DVs/DRs.
- If ``mimuPacketInMsg.timeWritten()`` has not changed since the previous ``updateState``, the call is skipped and no
  new output is written. The algorithm's internal ring is unchanged on a skipped cycle.
- ``reconfigure()`` re-freezes the public properties into the running algorithm (config only, runtime state untouched);
  ``reInitialize()`` clears the algorithm's ring. Both throw ``XmeraLifecycleException`` if called before ``reset()``.

Input Type
^^^^^^^^^^
The algorithm-internal input types are::

    struct Sample {
        Eigen::Vector3f gyro_C;
        Eigen::Vector3f accel_C;
    };

    struct InputPacket {
        bool          isValid;
        std::uint64_t measTime;   // First sample's measurement time
        std::array<Sample, MAX_MIMU_SAMPLES_PER_PKT_C> samples;
    };

    struct InputPktsData {
        std::array<InputPacket, MAX_MIMU_PKT_C> packets;
    };

Notes
-----
- Both averaging windows are anchored to the newest stored sample's derived time (``maxTimeTag``), not to the current
  system time.
- The output is deterministic given the input snapshot history, ``gyroAveragingWindow``, ``accelAveragingWindow``, and
  ``dcm_BC``.
