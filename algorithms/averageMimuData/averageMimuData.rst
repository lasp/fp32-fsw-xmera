.. raw:: latex

    {\LARGE \textbf{averageMimuData}}

Executive Summary
-----------------
The ``averageMimuData`` algorithm computes a rolling average of recent gyro and accelerometer samples from a MIMU
packet buffer. It uses the newest packet time tag as the reference time, selects all packets whose measurement timeTag with respect to the reference time is within a
user-specified window (``averagingWindow``), averages the selected measurements, and outputs the averaged values expressed in
the body frame using a user-provided direction cosine matrix (DCM) ``dcm_BP``.

Message Connection Descriptions
-------------------------------
The following table lists the algorithm input and output data structures. The input is a packet buffer containing
time-tagged measurements, and the output is the averaged body-frame angular velocity and acceleration.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Variable Name
      - Type
      - Description
    * - localPkts
      - ``InputPktsData``
      - Input packet buffer containing arrays of ``(measTime, gyro_P, accel_P)``.
    * - out
      - ``OutputAverageAccelAnglevel``
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
- The packet time tags ``measTime`` are assumed to be in nanoseconds.
- Packets are included if ``(maxTimeTag - measTime) * NANO2SEC <= averagingWindow``, where ``averagingWindow >= 0.0``.
- If no packets fall within the time window, the output vectors are zero.
- The algorithm performs an unweighted arithmetic mean (no weighting, outlier rejection, or bias correction).

Initialization
--------------
Configure the time window and the platform-to-body DCM::

    AverageMimuDataAlgorithm alg;
    alg.setAveragingWindow(0.05f);          // seconds
    alg.setDcmPltfToBdy(dcm_BP);      // Eigen::Matrix3f (platform-to-body)

Then call the update method each cycle::

    OutData out = alg.update(localPkts);

Detailed Module Description
---------------------------
Inputs
^^^^^^
The input payload contains an array of packets. Each packet provides:

- ``measTime``: measurement time tag (assumed nanoseconds)
- ``gyro_P``: 3-axis angular rate sample in platform frame (Eigen::Vector3f)
- ``accel_P``: 3-axis acceleration sample in platform frame (Eigen::Vector3f)

The algorithm expects the inputs ``gyro_P`` and ``accel_P``, where the subscript P denotes the platform frame.
At the module interface, however, these signals are read from ``AccPktDataMsgF32Payload``, whose field names use the legacy suffix B (``gyro_B`` and ``accel_B``).
In this context, ``gyro_B`` and ``accel_B`` should therefore be interpreted as platform-frame quantities, despite the field naming.

Configuration
^^^^^^^^^^^^^
1. Set ``averagingWindow`` to define how far back (from the newest measurement) to include samples in the average.
2. Set ``dcm_BP`` to map platform-frame vectors into the body frame.

Algorithm
^^^^^^^^^^^^^^^^
Given an input packet buffer ``localPkts``, the algorithm:

1. Finds the newest packet time tag ``maxTimeTag`` across the buffer.
2. Includes i-th packet whose ``ΔT_i=(maxTimeTag-measTime_i)*NANO2SEC<=averagingWindow``
3. Averages the selected gyro and accelerometer vectors (assumed to be in the platform frame).
4. Transforms the averaged vectors to the body frame using ``dcm_BP`` and returns them.

We use a simple example to illustrate steps 1 and 2 with three packets:

.. code-block:: text

    time  ------------------------------------------------------------->

             |<-- averagingWindow -->|
    t1            t2                 t3 = maxTimeTag          t_latest
    o-------------o------------------o------------------------o
    |<------------- ΔT1 ------------>|
                  |<------ ΔT2 ----->|

    1. maxTimeTag is the measurement time tag t3
    2. packet at t1 is not included since ΔT1 > averagingWindow.
       packet at t2 is included since ΔT2 <= averagingWindow.
       packet at t3 is included since ΔT3 = 0 <= averagingWindow.

Recommended Practices
^^^^^^^^^^^^^^^^^^^^^
- Choose ``averagingWindow`` based on the expected packet update rate and desired smoothing. Larger values increase smoothing
  but introduce more latency.
- ``dcm_BP`` shall be a proper DCM (orthonormal, right-handed).

Outputs
^^^^^^^
The algorithm returns::

    struct OutData {
        Eigen::Vector3f AccelBody;   // body-frame averaged acceleration
        Eigen::Vector3f AngVelBody;  // body-frame averaged angular velocity
    };

If no packets fall within the window, both output vectors are returned as zeros.

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

        OutData update(AccDataMsgF32Payload const& localPkts) const;
    };

Return Type
^^^^^^^^^^^
The return type is::

    struct OutData {
        Eigen::Vector3f AccelBody = Eigen::Vector3f::Zero();
        Eigen::Vector3f AngVelBody = Eigen::Vector3f::Zero();
    };

Notes
-----
- The averaging window is anchored to the newest time tag present in the buffer, not to the current system time.
- The output is deterministic given the input packet buffer, ``averagingWindow``, and ``dcm_BP``.
