==================
MIMU Majority Vote
==================

-----------------
Executive Summary
-----------------

The MIMU Majority Vote module combines the angular velocity measurements from exactly three
Inertial Measurement Units (IMUs) into a single best-estimate body-frame angular velocity.
It uses an outlier rejection algorithm: a single faulty sensor is identified by comparing
each measurement against the full-sensor average, and the remaining two sensors are averaged
to form the best estimate.
The module outputs the best-estimate average, a per-IMU validity array, and the angular
velocity difference magnitudes for every sensor — providing downstream fault management with
the information needed to take further action.

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
      - Output message containing the majority-voted angular velocity estimate in the
        spacecraft body frame (``AngVelBody``).
    * - ``mimuFaultMsg``
      - :ref:`MimuFaultMsgPayload`
      - Output message containing fault status, per-IMU validity flags
        (``validImus[3]``), and per-IMU difference magnitudes
        (``omegaDifferencesMag[3]``).

----------------------------
Algorithm Description
----------------------------

The voting algorithm operates on :math:`n = 3` IMU measurements
:math:`\boldsymbol{\omega}_i \in \mathbb{R}^3` (body-frame angular velocity, rad/s),
a scalar threshold :math:`T > 0` (rad/s), and a persistence limit :math:`P \geq 1`
(number of consecutive detections required to trigger a fault).

**Outlier detection and averaging**

Compute the average of all :math:`n` measurements:

.. math::

   \bar{\boldsymbol{\omega}} = \frac{1}{n} \sum_{i=0}^{n-1} \boldsymbol{\omega}_i

Compute the Euclidean distance of each measurement from the full average:

.. math::

   \delta_i = \lVert \boldsymbol{\omega}_i - \bar{\boldsymbol{\omega}} \rVert_2, \quad i = 0, \ldots, n-1

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

   \bar{\boldsymbol{\omega}}_2 = \frac{1}{n-1} \sum_{i \,:\, c_i < P} \boldsymbol{\omega}_i

If no IMU is invalid, :math:`\bar{\boldsymbol{\omega}}` is returned directly.

**Difference magnitudes in the output**

The ``omegaDifferencesMag`` array is always populated with the values
:math:`\delta_i` for all sensors.

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
      - Array of exactly 3 IMU angular velocity measurements
        (:math:`\boldsymbol{\omega}_i`, rad/s).

**Output** ``MimuMajorityVoteOutput``

.. list-table::
    :widths: 35 15 55
    :header-rows: 1

    * - Field
      - Type
      - Description
    * - ``avgOmega_BN_B``
      - ``Eigen::Vector3f``
      - Best-estimate body-frame angular velocity (rad/s). Equals
        :math:`\bar{\boldsymbol{\omega}}` when all agree, or
        :math:`\bar{\boldsymbol{\omega}}_2` when one is invalid.
    * - ``faultDetected``
      - ``bool``
      - ``true`` if any IMU has been marked invalid.
    * - ``omegaDifferencesMag``
      - ``std::array<float, kMimuCount>``
      - Per-IMU difference magnitude (rad/s) as described above. Always populated.
    * - ``validImus``
      - ``std::array<bool, kMimuCount>``
      - Per-IMU validity flag. ``true`` means the sensor is trusted.

**Configuration**

Configuration is supplied as an immutable ``MimuMajorityVoteConfig``, built via the validating
factory ``MimuMajorityVoteConfig::create(omegaThreshold, faultPersistenceLimit)``:

- ``omegaThreshold`` (rad/s) must be finite and strictly positive.
- ``faultPersistenceLimit`` must be strictly positive; a value of 1 triggers the fault immediately
  on first detection.

``create()`` throws ``fsw::invalid_argument`` on any invalid parameter. Constructing the algorithm
from a config validates and arms it; ``reInitialize()`` clears the fault persistence counters.

.. code-block:: cpp

    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(0.05F, 3U)};  // 0.05 rad/s, 3 detections

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = omega0;
    imuOmegas_BN_B.at(1) = omega1;
    imuOmegas_BN_B.at(2) = omega2;

    MimuMajorityVoteOutput out = alg.update(imuOmegas_BN_B);
    // out.avgOmega_BN_B  — best-estimate rate
    // out.faultDetected  — true if any IMU was rejected
    // out.validImus      — per-sensor validity flags
    // out.omegaDifferencesMag — per-sensor residuals

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
    module.faultPersistenceLimit = 3                  # must be >= 1

    # Connect exactly 3 ImuMessage objects
    for imu_msg in imu_input_messages:  # must have exactly 3 entries
        imu_entry = mimuMajorityVoteF32.ImuMessage()
        imu_entry.imuSensorBodyInMsg.subscribeTo(imu_msg)
        module.addImuInput(imu_entry)

    scSim.AddModelToTask(taskName, module)

``reset()`` builds the validated configuration from the public parameters (raising
``fsw::invalid_argument`` if a parameter is invalid) and constructs the algorithm. ``reConfigure()``
re-applies the current public parameters to the running algorithm, and ``reInitialize()`` clears its
fault persistence counters.

The voted angular velocity is available on ``module.imuSensorBodyOutMsg`` and the
fault status on ``module.mimuFaultMsg``. The ``mimuFaultMsg`` payload fields are:

- ``faultDetected`` — ``bool``, mirrors ``MimuMajorityVoteOutput.faultDetected``.
- ``validImus[3]`` — per-IMU validity flags; index matches the order sensors were
  added via ``addImuInput()``.
- ``omegaDifferencesMag[3]`` — per-IMU difference magnitude (rad/s); same ordering.
