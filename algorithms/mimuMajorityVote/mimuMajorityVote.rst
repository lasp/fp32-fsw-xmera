==================
MIMU Majority Vote
==================

-----------------
Executive Summary
-----------------

The MIMU Majority Vote module combines the angular velocity measurements from three or more
Inertial Measurement Units (IMUs) into a single best-estimate body-frame angular velocity.
It uses a two-stage outlier rejection algorithm: a single faulty sensor is first identified
by comparing each measurement against the full-sensor average, then the remaining sensors are
cross-checked against their own average to detect a second level of disagreement.
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
        (``validImus[4]``), and per-IMU difference magnitudes
        (``omegaDifferencesMag[4]``).

----------------------------
Algorithm Description
----------------------------

The voting algorithm operates on :math:`n \geq 3` IMU measurements
:math:`\boldsymbol{\omega}_i \in \mathbb{R}^3` (body-frame angular velocity, rad/s)
and a scalar threshold :math:`T > 0` (rad/s). It runs in two sequential stages every
update cycle.

**Stage 1 — Full-average outlier detection**

Compute the average of all :math:`n` measurements:

.. math::

   \bar{\boldsymbol{\omega}} = \frac{1}{n} \sum_{i=0}^{n-1} \boldsymbol{\omega}_i

Compute the Euclidean distance of each measurement from the full average:

.. math::

   \delta_i = \lVert \boldsymbol{\omega}_i - \bar{\boldsymbol{\omega}} \rVert_2, \quad i = 0, \ldots, n-1

Identify the worst offender:

.. math::

   k = \arg\max_i \; \delta_i

If :math:`\delta_k \geq T`, IMU :math:`k` is flagged as invalid and removed.
The average of the remaining :math:`n-1` sensors is recomputed:

.. math::

   \bar{\boldsymbol{\omega}}_2 = \frac{1}{n-1} \sum_{i \neq k} \boldsymbol{\omega}_i

If no measurement exceeds the threshold (:math:`\delta_k < T`), all sensors are
considered valid and :math:`\bar{\boldsymbol{\omega}}` is returned directly.

**Stage 2 — Reduced-average cross-check**

After removing the outlier, the remaining sensors are re-checked against the new
reduced average. For each remaining valid sensor :math:`i \neq k`:

.. math::

   \delta_i^{(2)} = \lVert \boldsymbol{\omega}_i - \bar{\boldsymbol{\omega}}_2 \rVert_2

If any :math:`\delta_i^{(2)} \geq T`, the remaining sensors disagree with each other.
All :math:`n` IMUs are marked invalid. The returned average is still
:math:`\bar{\boldsymbol{\omega}}_2` (the best estimate obtainable), but the caller is
informed via the validity array that no sensor can be trusted.

**Difference magnitudes in the output**

The ``omegaDifferencesMag`` array is always populated. After Stage 1 the excluded IMU
retains its original Stage 1 difference :math:`\delta_k`; the remaining sensors are
updated to their Stage 2 values :math:`\delta_i^{(2)}`. If no fault is detected all
entries hold the Stage 1 values.

**Valid-count invariant**

Because Stage 2 either leaves the count unchanged or marks all sensors invalid, the
number of invalid IMUs for a 3-sensor configuration can only be **0, 1, or 3** — never
exactly 2.

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
    * - ``imuInputs``
      - ``std::array<MimuInput, 4>``
      - Array of IMU measurements. Only the first ``numberOfImus`` entries are used.
        Each entry contains ``angVelBody`` (:math:`\boldsymbol{\omega}_i`, rad/s).

The number of active IMUs is fixed via ``setNumberOfImus()`` before the first
``update()`` call. It must be in :math:`[3, \texttt{MAX\_IMU\_VEH\_COUNT}]`; a value
outside this range throws ``fs::invalid_argument``.

**Output** ``MimuMajorityVoteOutput``

.. list-table::
    :widths: 35 15 55
    :header-rows: 1

    * - Field
      - Type
      - Description
    * - ``avgAngVelBody``
      - ``Eigen::Vector3f``
      - Best-estimate body-frame angular velocity (rad/s). Equals
        :math:`\bar{\boldsymbol{\omega}}` when all agree, or
        :math:`\bar{\boldsymbol{\omega}}_2` when one or more are invalid.
    * - ``faultDetected``
      - ``bool``
      - ``true`` if any IMU has been marked invalid.
    * - ``omegaDifferencesMag``
      - ``std::array<float, 4>``
      - Per-IMU difference magnitude (rad/s) as described above. Always populated.
    * - ``validImus``
      - ``std::array<bool, 4>``
      - Per-IMU validity flag. ``true`` means the sensor is trusted. Entries beyond
        ``numberOfImus`` are ``false``.

**Configuration**

The detection threshold is set via the ``setOmegaThreshold()`` / ``getOmegaThreshold()``
accessor pair. The threshold must be strictly positive; a zero or negative value throws
``fs::invalid_argument``.

.. code-block:: cpp

    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(0.05F);  // rad/s
    alg.setNumberOfImus(3U);       // fixed for the lifetime of the object

    std::array<MimuInput, MAX_IMU_VEH_COUNT> imuInputs{};
    imuInputs.at(0).angVelBody = omega0;
    imuInputs.at(1).angVelBody = omega1;
    imuInputs.at(2).angVelBody = omega2;

    MimuMajorityVoteOutput out = alg.update(imuInputs);
    // out.avgAngVelBody  — best-estimate rate
    // out.faultDetected  — true if any IMU was rejected
    // out.validImus      — per-sensor validity flags
    // out.omegaDifferencesMag — per-sensor residuals

-----------------------------------
Module Assumptions and Limitations
-----------------------------------

- ``setNumberOfImus()`` must be called before ``reset()``, with a value in
  :math:`[3, \texttt{MAX\_IMU\_VEH\_COUNT}]`. The number of ``addImuInput()`` calls
  must equal the configured value; a mismatch throws ``std::invalid_argument``.
- The maximum number of connected IMUs is ``MAX_IMU_VEH_COUNT`` (currently 4).
- The algorithm is designed for the 3-sensor case. With 4 sensors the same two-stage
  logic applies: one outlier is removed and the remaining three are cross-checked.
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

    # Set the detection threshold (rad/s)
    module.omegaThreshold = omegaThresholdRadPerSec

    # Set the expected number of IMUs (must match the number of addImuInput() calls)
    module.numberOfImus = len(imu_input_messages)

    # Connect one ImuMessage per sensor
    for imu_msg in imu_input_messages:
        imu_entry = mimuMajorityVoteF32.ImuMessage()
        imu_entry.imuSensorBodyInMsg.subscribeTo(imu_msg)
        module.addImuInput(imu_entry)

    scSim.AddModelToTask(taskName, module)

The voted angular velocity is available on ``module.imuSensorBodyOutMsg`` and the
fault status on ``module.mimuFaultMsg``. The ``mimuFaultMsg`` payload fields are:

- ``faultDetected`` — ``bool``, mirrors ``MimuMajorityVoteOutput.faultDetected``.
- ``validImus[4]`` — per-IMU validity flags; index matches the order sensors were
  added via ``addImuInput()``.
- ``omegaDifferencesMag[4]`` — per-IMU difference magnitude (rad/s); same ordering.
