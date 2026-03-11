```rst
.. raw:: latex

    {\LARGE \textbf{averageMimuData}}

Executive Summary
-----------------
The ``averageMimuData`` algorithm computes a rolling average of recent gyro and accelerometer samples from a MIMU
packet buffer. It uses the newest packet timestamp as the reference time, selects all packets whose age is within a
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
      - :ref:`AccDataMsgF32Payload`
      - Input packet buffer containing samples ``(measTime, gyro_B, accel_B)``
    * - out
      - ``OutData``
      - Output averages in body frame: ``AngVelBody`` and ``AccelBody`` (both ``Eigen::Vector3f``)

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
      - Rolling-window width measured backward from the newest packet timestamp. Packets with age ``< averagingWindow`` are included
    * - dcm_BP
      - Eigen::Matrix3f
      - [-]
      - Identity
      - ``setDcmPltfToBdy()`` / ``getDcmPltfToBdy()``
      - DCM mapping to body-frame vectors

Module Assumptions and Limitations
----------------------------------
- The packet timestamps ``measTime`` are assumed to be in nanoseconds and converted using ``NANO2SEC``.
- Packets are included if ``(maxTimeTag - measTime) * NANO2SEC < averagingWindow`` (strict inequality).
- If ``averagingWindow`` is 0, no packets satisfy the strict inequality and the output is zero.
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
General Function
^^^^^^^^^^^^^^^^
Given an input packet buffer ``localPkts.accPkts``, the algorithm:

1. Finds the newest packet time tag ``maxTimeTag`` across the buffer.
2. Iterates over each packet and computes its age relative to ``maxTimeTag``.
3. Includes packets whose age is strictly less than ``averagingWindow`` seconds.
4. Sums the selected gyro and accelerometer vectors (as 3-vectors).
5. Divides by the number of selected packets to form the average in the platform frame.
6. Transforms the averaged vectors to the body frame using ``dcm_BP`` and returns them.

Algorithm
^^^^^^^^^
Let :math:`t_{\max}` be the newest packet time in the buffer. A packet with timestamp :math:`t_i` is *kept* if its age
(relative to :math:`t_{\max}`) is within the time window:

.. math::
    (t_{\max} - t_i)\,\texttt{NANO2SEC} < \texttt{averagingWindow}.

Let :math:`\mathcal{I}` be the set of indices of all kept packets, and let :math:`M = |\mathcal{I}|` be the number of
kept packets.

If :math:`M = 0`, return zeros:

.. math::
    ^B\omega_{\text{avg}} = \mathbf{0}, \qquad ^Ba_{\text{avg}} = \mathbf{0}.

Otherwise, average in the platform frame:

.. math::
    ^P\omega_{\text{avg}} = \frac{1}{M}\sum_{i\in\mathcal{I}}\, ^P\omega_i,
    \qquad
    ^Pa_{\text{avg}} = \frac{1}{M}\sum_{i\in\mathcal{I}}\, ^P a_i.

Finally, rotate the averages into the body frame:

.. math::
    ^B\omega_{\text{avg}} = \texttt{dcm\_BP}\,^P\omega_{\text{avg}},
    \qquad
    ^Ba_{\text{avg}} = \texttt{dcm\_BP}\,^Pa_{\text{avg}}.


User Guide
----------
Inputs
^^^^^^
The input payload contains an array of packets. Each packet provides:

- ``measTime``: measurement timestamp (assumed nanoseconds)
- ``gyro_B``: 3-axis angular rate sample (stored as a 3-element array)
- ``accel_B``: 3-axis acceleration sample (stored as a 3-element array)

Although the packet fields are named with ``_B`` in the payload, this algorithm treats the raw packet vectors as being
in the platform frame for averaging (consistent with the internal variable naming ``gyroSum_P`` and ``accelSum_P``),
and then applies ``dcm_BP`` to express the averages in the body frame.

Configuration
^^^^^^^^^^^^^
1. Set ``averagingWindow`` to define how far back (from the newest measurement) to include samples in the average.
2. Set ``dcm_BP`` to map platform-frame vectors into the body frame.

Recommended Practices
^^^^^^^^^^^^^^^^^^^^^
- Choose ``averagingWindow`` based on the expected packet update rate and desired smoothing. Larger values increase smoothing
  but introduce more latency.
- Ensure ``dcm_BP`` is a proper DCM (orthonormal, right-handed) consistent with your frame definitions.

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
        void setAveragingWindow(float window);
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
- The averaging window is anchored to the newest timestamp present in the buffer, not to the current system time.
- The output is deterministic given the input packet buffer, ``averagingWindow``, and ``dcm_BP``.
```
