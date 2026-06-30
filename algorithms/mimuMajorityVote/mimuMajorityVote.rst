==================
MIMU Majority Vote
==================

-----------------
Executive Summary
-----------------

The MIMU Majority Vote module combines the measurements from exactly three Inertial Measurement
Units (IMUs) into single best-estimate body-frame quantities. It runs two **independent** majority
votes — one on the angular velocity (gyro) and one on the apparent acceleration (accelerometer) —
each with its own threshold and fault persistence limit. In each vote a single faulty sensor is
identified by comparing each measurement against the full-sensor average, and the remaining two
sensors are averaged to form the best estimate. Because the votes are independent, an IMU may be
valid on one quantity but rejected on the other.
The module outputs the best-estimate averages, per-IMU validity arrays, and the per-sensor
difference magnitudes for both quantities — providing downstream fault management with the
information needed to take further action.

-------------------------------
Module Input/Output Messages
-------------------------------

The following table lists the Xmera message connections for the ``MimuMajorityVote`` module.

.. list-table:: Module I/O Messages
    :widths: 25 35 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ``imuSensorBodyInMsg`` (per sensor)
      - :ref:`IMUSensorBodyMsgF32Payload`
      - Input message for each connected IMU. One ``ImuMessage`` object is created
        per sensor and added via ``addImuInput()``.
    * - ``imuSensorBodyOutMsg``
      - :ref:`IMUSensorBodyMsgF32Payload`
      - Output message containing the majority-voted angular velocity (``AngVelBody``) and
        apparent acceleration (``AccelBody``) estimates in the spacecraft body frame.
    * - ``mimuFaultMsg``
      - :ref:`MimuFaultMsgPayload`
      - Output message containing the gyro and accel fault status (``gyroFaultDetected`` /
        ``accelFaultDetected``), per-IMU validity flags (``gyroImuValid[3]`` /
        ``accelImuValid[3]``), and per-IMU difference magnitudes (``gyroImuDifferenceMag[3]`` /
        ``accelImuDifferenceMag[3]``).

----------------------------
Algorithm Description
----------------------------

The same majority vote runs **independently** on two quantities: the angular velocity
(``omegaThreshold`` / ``gyroFaultPersistenceLimit``) and the apparent acceleration
(``accelThreshold`` / ``accelFaultPersistenceLimit``). Each vote keeps its own persistence counters,
so the two faults are decided separately. The description below is written for a generic measurement
:math:`\boldsymbol{m}_i \in \mathbb{R}^3` and applies to each quantity in turn.

The vote operates on :math:`n = 3` IMU measurements :math:`\boldsymbol{m}_i`, a scalar threshold
:math:`T > 0`, and a persistence limit :math:`P \geq 1` (number of consecutive detections required
to trigger a fault).

**Outlier detection and averaging**

Compute the average of all :math:`n` measurements:

.. math::

   \bar{\boldsymbol{m}} = \frac{1}{n} \sum_{i=0}^{n-1} \boldsymbol{m}_i

Compute the Euclidean distance of each measurement from the full average:

.. math::

   \delta_i = \lVert \boldsymbol{m}_i - \bar{\boldsymbol{m}} \rVert_2, \quad i = 0, \ldots, n-1

Identify the worst offender:

.. math::

   k = \arg\max_i \; \delta_i

**Fault persistence**

Each IMU maintains an independent consecutive-detection counter :math:`c_i`. Each update:

- If :math:`\delta_k \geq T`, increment :math:`c_k`; reset all other :math:`c_i` to zero.
- If :math:`\delta_k < T`, reset all :math:`c_i` to zero.

IMU :math:`i` is marked invalid when :math:`c_i \geq P`. Once marked invalid, it is
excluded from the output average:

.. math::

   \bar{\boldsymbol{m}}_2 = \frac{1}{n-1} \sum_{i \,:\, c_i < P} \boldsymbol{m}_i

If no IMU is invalid, :math:`\bar{\boldsymbol{m}}` is returned directly.

**Difference magnitudes in the output**

Each vote's difference array (``gyro.imuDifferenceMag`` for the gyro, ``accel.imuDifferenceMag`` for
the accel) is always populated with the values :math:`\delta_i` for all sensors.

**Valid-count invariant**

The number of invalid IMUs can only be **0** (no fault) or **1** (single outlier).

-----------------------------------
Standalone Algorithm (C++ API)
-----------------------------------

``MimuMajorityVoteAlgorithm`` is a plain C++ class with no Xmera dependency.

**Inputs to** ``update()``

.. list-table::
    :widths: 30 15 60
    :header-rows: 1

    * - Parameter
      - Type
      - Description
    * - ``imuOmegas_BN_B``
      - ``std::array<Eigen::Vector3f, kMimuCount>``
      - Array of exactly 3 IMU angular velocity measurements (rad/s).
    * - ``imuAccels_B``
      - ``std::array<Eigen::Vector3f, kMimuCount>``
      - Array of exactly 3 IMU apparent acceleration measurements (m/s²).

**Output** ``MimuMajorityVoteOutput``

.. list-table::
    :widths: 35 15 55
    :header-rows: 1

    * - Field
      - Type
      - Description
    * - ``gyro`` / ``accel``
      - ``MimuVoteResult``
      - Independent vote result for the angular velocity / apparent acceleration. Each bundles the
        four fields below.
    * - ``gyro.average`` / ``accel.average``
      - ``Eigen::Vector3f``
      - Best-estimate body-frame angular velocity (rad/s) / apparent acceleration (m/s²). Equals the
        full average when all agree, or the 2-sensor average when one is invalid.
    * - ``gyro.faultDetected`` / ``accel.faultDetected``
      - ``bool``
      - ``true`` if any IMU has been marked invalid for that quantity's vote.
    * - ``gyro.imuDifferenceMag`` / ``accel.imuDifferenceMag``
      - ``std::array<float, kMimuCount>``
      - Per-IMU difference magnitude (rad/s / m/s²) as described above. Always populated.
    * - ``gyro.imuValid`` / ``accel.imuValid``
      - ``std::array<bool, kMimuCount>``
      - Per-IMU validity flag for that quantity's vote. ``true`` means the sensor is trusted.

**Configuration**

Configuration is supplied as an immutable ``MimuMajorityVoteConfig``, built via the validating
factory ``MimuMajorityVoteConfig::create(omegaThreshold, gyroFaultPersistenceLimit, accelThreshold,
accelFaultPersistenceLimit)``:

- ``omegaThreshold`` (rad/s) and ``accelThreshold`` (m/s²) must be finite and strictly positive.
- ``gyroFaultPersistenceLimit`` and ``accelFaultPersistenceLimit`` must be strictly positive; a value
  of 1 triggers that vote's fault immediately on first detection.

``create()`` throws ``fsw::invalid_argument`` on any invalid parameter. Constructing the algorithm
from a config validates and arms it; ``reInitialize()`` clears both votes' persistence counters.

.. code-block:: cpp

    // omega threshold 0.05 rad/s (3 detections), accel threshold 0.5 m/s² (1 detection)
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(0.05F, 3U, 0.5F, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{omega0, omega1, omega2};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{accel0, accel1, accel2};

    MimuMajorityVoteOutput out = alg.update(imuOmegas_BN_B, imuAccels_B);
    // out.gyro.average / out.accel.average             — best-estimate rate / acceleration
    // out.gyro.faultDetected / out.accel.faultDetected — true if an IMU was rejected for that vote
    // out.gyro.imuValid / out.accel.imuValid           — per-sensor validity flags per vote
    // out.gyro.imuDifferenceMag / out.accel.imuDifferenceMag — per-sensor residuals per vote

-----------------------------------
Module Assumptions and Limitations
-----------------------------------

- Exactly ``kMimuCount`` (3) ``addImuInput()`` calls must be made before ``reset()``;
  any other count throws ``std::invalid_argument``. Attempting to add more than
  ``kMimuCount`` IMUs throws ``fsw::invalid_argument``.
- The threshold :math:`T` must be chosen to be meaningfully larger than the float32
  rounding error in the average computation (~3 × :math:`\varepsilon_\text{mach}` ×
  :math:`|\boldsymbol{\omega}|`). For angular rates up to 1000 rad/s this is
  approximately 3.6 × 10\ :sup:`−4` rad/s.
- When two sensors are exactly equidistant from the full average (a tie), the
  algorithm rejects the one with the lower array index. The returned average therefore
  depends on sensor ordering when ties occur; however, the fault detection outcome and
  invalid count are index-order independent.
- The module does not perform sensor health monitoring over time or persist fault
  history between update cycles. Each call to ``update()`` is stateless with respect
  to fault detection.

----------
User Guide
----------

The Xmera module is instantiated and configured from Python as follows::

    module = mimuMajorityVoteF32.MimuMajorityVote()
    module.modelTag = "mimuMajorityVote"

    # Set the detection parameters (validated when the module is reset)
    module.omegaThreshold = omegaThresholdRadPerSec   # rad/s, must be > 0
    module.gyroFaultPersistenceLimit = 3              # must be >= 1
    module.accelThreshold = accelThresholdMPerSec2    # m/s^2, must be > 0
    module.accelFaultPersistenceLimit = 1             # must be >= 1

    # Connect exactly 3 ImuMessage objects
    for imu_msg in imu_input_messages:  # must have exactly 3 entries
        imu_entry = mimuMajorityVoteF32.ImuMessage()
        imu_entry.imuSensorBodyInMsg.subscribeTo(imu_msg)
        module.addImuInput(imu_entry)

    scSim.AddModelToTask(taskName, module)

``reset()`` builds the validated configuration from the public parameters (raising
``fsw::invalid_argument`` if a parameter is invalid) and constructs the algorithm. ``reConfigure()``
re-applies the current public parameters to the running algorithm, and ``reInitialize()`` clears both
votes' persistence counters.

The voted angular velocity (``AngVelBody``) and acceleration (``AccelBody``) are available on
``module.imuSensorBodyOutMsg`` and the fault status on ``module.mimuFaultMsg``. The ``mimuFaultMsg``
payload fields are:

- ``gyroFaultDetected`` / ``accelFaultDetected`` — ``bool`` per vote.
- ``gyroImuValid[3]`` / ``accelImuValid[3]`` — per-IMU validity flags per vote; index matches the
  order sensors were added via ``addImuInput()``.
- ``gyroImuDifferenceMag[3]`` (rad/s) / ``accelImuDifferenceMag[3]`` (m/s²) — per-IMU difference
  magnitudes per vote; same ordering.
