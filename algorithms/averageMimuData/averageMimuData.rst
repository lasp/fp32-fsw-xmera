.. raw:: latex

    {\LARGE \textbf{averageMimuData}}

Executive Summary
-----------------
The ``averageMimuData`` algorithm computes a rolling average of recent gyro and accelerometer measurements drawn from a
small ring of MIMU packets. Each input slot holds a ``MimuPacketGroupF32Payload`` of up to ``MAX_MIMU_SAMPLES_PER_PKT``
time-tagged samples; a parallel ``isValid[MAX_MIMU_PKT]`` array marks which packets the caller has filled this cycle.
The algorithm skips any packet that is not valid and any sample with ``measTime == 0`` (treated as unfilled), uses the
newest remaining sample's time tag as the reference, averages every fresh sample whose age is within ``averagingWindow``,
and outputs the result rotated into the body frame using the user-provided DCM ``dcm_BP``. Each contributing sample
weighs equally. If no fresh sample qualifies, the output is zero.

Message Connection Descriptions
-------------------------------
The following table lists the algorithm input and output data structures. The input is a 4-packet ring-buffer snapshot
where each packet carries up to ``MAX_MIMU_SAMPLES_PER_PKT`` time-tagged measurements; the output is the averaged
body-frame angular velocity and acceleration.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Variable Name
      - Type
      - Description
    * - mimuPacketInMsg
      - ``MimuPacketF32Payload``
      - Input snapshot: ``packets[MAX_MIMU_PKT]`` of ``MimuPacketGroupF32Payload`` plus ``isValid[MAX_MIMU_PKT]``.
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
      - float
      - [s]
      - 0.0
      - ``setAveragingWindow()`` / ``getAveragingWindow()``
      - Rolling time-window.
    * - dcm_BP
      - Eigen::Matrix3f
      - [-]
      - Identity
      - ``setDcmPltfToBdy()`` / ``getDcmPltfToBdy()``
      - DCM mapping to body-frame vectors

Module Assumptions and Limitations
----------------------------------
- Sample time tags ``measTime`` are assumed to be in nanoseconds.
- A sample is considered fresh iff ``isValid[p] == true`` **and** ``samples[p][s].measTime != 0``. Stale samples are
  skipped both when choosing the newest time tag and when accumulating the average. The caller is responsible for
  clearing ``isValid[p]`` for any packet that has not yet been written this cycle, and for leaving unfilled samples
  within a fresh packet at ``measTime == 0`` so the algorithm does not include them during ring-buffer warm-up.
- Among the fresh samples, a measurement is included in the average if
  ``(maxTimeTag - measTime) <= averagingWindowNs``, where ``averagingWindowNs`` is the configured
  ``averagingWindow`` (seconds, ``>= 0.0``) converted once to nanoseconds. The comparison is done in
  ``uint64_t`` nanoseconds to avoid the precision loss that would occur casting a multi-second
  ``uint64_t`` delta through ``float``.
- If no sample is fresh, or if no fresh sample falls within the averaging window, the output vectors are zero.
- The algorithm performs an unweighted arithmetic mean across measurements (no weighting, outlier rejection, or bias
  correction). Each contributing sample weighs equally regardless of which packet it came from.

Initialization
--------------
Configure the time window and the platform-to-body DCM::

    AverageMimuDataAlgorithm alg;
    alg.setAveragingWindow(0.05f);          // seconds
    alg.setDcmPltfToBdy(dcm_BP);            // Eigen::Matrix3f (platform-to-body)

Then call the update method each cycle::

    OutputAverageAccelAngleVel out = alg.update(localPkts);

Detailed Module Description
---------------------------
Inputs
^^^^^^
The input payload carries ``packets[MAX_MIMU_PKT]`` (4 ring-buffer slots) along with a parallel ``isValid[MAX_MIMU_PKT]``
array. Each slot is a ``MimuPacketGroupF32Payload`` containing up to ``MAX_MIMU_SAMPLES_PER_PKT`` (10) individual
samples, where each sample provides:

- ``measTime``: measurement time tag (assumed nanoseconds; ``0`` denotes an unfilled slot)
- ``gyro_P``: 3-axis angular rate sample in platform frame (Eigen::Vector3f)
- ``accel_P``: 3-axis acceleration sample in platform frame (Eigen::Vector3f)

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
Given an input ring snapshot ``localPkts``, the algorithm:

0. Builds the set of fresh samples: those in packets with ``isValid[p] == true`` and with ``samples[p][s].measTime != 0``.
1. Finds the newest sample time tag ``maxTimeTag`` across the fresh samples. If no fresh sample exists, returns a zero
   output.
2. Includes each fresh sample whose ``(maxTimeTag - samples[p][s].measTime) <= averagingWindowNs``,
   i.e. whose age (in nanoseconds) does not exceed the configured window converted once to nanoseconds.
3. Averages the selected gyro and accelerometer vectors (assumed to be in the platform frame), weighting each sample
   equally.
4. Transforms the averaged vectors to the body frame using ``dcm_BP`` and returns them.

Recommended Practices
^^^^^^^^^^^^^^^^^^^^^
- Choose ``averagingWindow`` based on the expected sample rate and desired smoothing. Larger values increase smoothing
  but introduce more latency.
- ``dcm_BP`` shall be a proper DCM (orthonormal, right-handed).
- The caller is responsible for maintaining ``isValid[p]``. Ring-buffer producers should clear ``isValid[p]`` for any
  packet that has not been written this cycle; the algorithm will treat the whole packet as stale.

Outputs
^^^^^^^
The algorithm returns::

    struct OutputAverageAccelAngleVel {
        Eigen::Vector3f accel_B = Eigen::Vector3f::Zero();       // body-frame averaged acceleration
        Eigen::Vector3f gyroOmega_B = Eigen::Vector3f::Zero();   // body-frame averaged angular velocity
    };

If no fresh sample is in the window, both output vectors are returned as zeros.

API Reference
-------------
Class Interface
^^^^^^^^^^^^^^^
The algorithm is implemented by::

    class AverageMimuDataAlgorithm {
       public:
        void setAveragingWindow(float averagingWindowIn);
        float getAveragingWindow() const;

        void setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn);
        Eigen::Matrix3f getDcmPltfToBdy() const;

        OutputAverageAccelAngleVel update(InputPktsData const& localPkts) const;
    };

Input Type
^^^^^^^^^^
The algorithm-internal input type is an array-of-structs of ``Sample``::

    struct Sample {
        std::uint64_t   measTime;
        Eigen::Vector3f gyro_P;
        Eigen::Vector3f accel_P;
    };

    struct InputPktsData {
        std::array<bool, MAX_MIMU_PKT> isValid;
        std::array<std::array<Sample, MAX_MIMU_SAMPLES_PER_PKT>, MAX_MIMU_PKT> samples;
    };

Notes
-----
- The averaging window is anchored to the newest time tag among fresh samples, not to the current system time.
- The output is deterministic given the input ring snapshot, ``averagingWindow``, and ``dcm_BP``.
