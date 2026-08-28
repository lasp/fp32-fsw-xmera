.. raw:: latex

    {\LARGE \textbf{dvAccumulation}}

Executive Summary
-----------------
``dvAccumulation`` integrates a single body-frame acceleration sample into a running Delta-V
accumulator. Each ``updateState()`` reads one ``IMUSensorBodyMsgF32Payload`` and takes its
``AccelBody`` field as the body-frame non-gravitational acceleration :math:`\ddot{\mathbf{r}}_{B}`.
The accelerometer bias :math:`\mathbf{b}` supplied on the call is subtracted and the remainder integrated over
the configured control period :math:`\Delta t`, so the sample contributes
:math:`\Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \, (\ddot{\mathbf{r}}_{B} - \mathbf{b})`. The
module outputs the running accumulator, time-tagged with the module call time.

Message Connection Descriptions
-------------------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Variable Name
      - Type
      - Description
    * - imuInMsg
      - ``IMUSensorBodyMsgF32Payload``
      - Input IMU body message. Only ``AccelBody`` (m/s^2, float[3], the body-frame non-gravitational
        acceleration) is consumed.
    * - dvAccumulationOutMsg
      - ``NavTransMsgF32Payload``
      - Output navigation message. The adapter populates ``timeTag`` (seconds, double, the module
        call time) and ``vehAccumDV`` (m/s, float[3]); the position and velocity fields are left zero.

Module Parameters
-----------------
.. list-table:: Module Parameters
    :widths: 30 15 10 10 40 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - controlPeriod (required)
      - float
      - [s]
      - 0
      - Control period used as the integration step (time between two ``updateState()`` calls,
        i.e. 1/fsw_rate)
      - Must be finite and greater than zero
    * - accelBias_B
      - Eigen::Vector3f
      - [m/s^2]
      - zeros
      - Accelerometer bias present in the measured body-frame acceleration, subtracted from every
        sample. Zero disables the correction. Passed to the algorithm on every ``updateState()``, so
        an edit takes effect on the next call without ``reconfigure()``.
      - None. Not validated -- a non-finite bias propagates to a non-finite Delta-V

Bounds are enforced by ``DvAccumulationConfig::create`` at ``reset()``, which throws
``fsw::invalid_argument`` on a violation.

Module Assumptions and Limitations
----------------------------------
- ``accelBias_B`` is the additive offset **present in the measurement**, and it is subtracted from
  every sample. A value supplied as a *correction* rather than a bias must be negated before it is
  set, or the error doubles instead of cancelling.
- ``accelBias_B`` is an argument to ``update()``, not configuration. The caller owns it, so the
  algorithm holds no calibration state and a changed bias needs no ``reconfigure()``. It is **not
  validated**: a bias integrates, so a unit error or a g-level entry corrupts an entire burn, and a
  non-finite bias propagates to a non-finite Delta-V with no exception.
- ``accelBias_B`` is a constant offset only. An accelerometer mounted off the center of mass also
  measures rate-dependent terms
  :math:`\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r}) +
  \dot{\boldsymbol{\omega}} \times \mathbf{r}`, which no fixed vector can remove; a bias fit to
  include them is correct only at the rate it was fit at. Lever-arm compensation is out of scope.
- The bias is a lumped body-frame vector applied after ``averageMimuData`` has combined the devices,
  so it represents the aggregate of the per-device biases. If the contributing device set changes
  (e.g. a MIMU dropout), the aggregate changes and the configured value no longer matches.
- The algorithm does not see time. It integrates over the configured ``controlPeriod`` on every call,
  so the caller must drive it once per control period; ``dvAccumulation`` runs at the same cadence as
  its upstream producer and immediately after it, so the acceleration sample is fresh on every call.
- The first ``update()`` after construction or ``reInitialize()`` **starts the accumulation window**
  rather than integrating. N samples bound N-1 intervals, so this is the correct interval count, not a
  dropped sample: the accumulated Delta-V equals the acceleration integrated over the elapsed time
  since that first call. It introduces no bias.
- ``reInitialize()`` zeroes the accumulator and restarts the accumulation window together. They must
  move together: zeroing the accumulator while leaving the window open would integrate a full step
  into a fresh window and put the accumulated Delta-V one interval ahead of the elapsed time. These
  two are the algorithm's entire runtime state, so there is no partial-reset entry point.
- Lifecycle: the adapter constructs the algorithm in ``reset()`` (startup only). State-transition
  hooks call ``reInitialize()``; ``reset()`` is not re-invoked on transitions. ``reconfigure()``
  installs edited parameters without re-arming the accumulation window.
- The accumulator is float-precision (``Eigen::Vector3f``), as is ``controlPeriod``, so the whole
  integration is single precision. ``timeTag`` stays double in the output message.

Accumulation precision
~~~~~~~~~~~~~~~~~~~~~~

Quadrature, not float32, is the real accuracy limit. The rectangle rule holds
:math:`\ddot{\mathbf{r}}_{B}` constant across each control period, so a changing acceleration leaves
an :math:`O(\Delta t \, \Delta\ddot{r})` residual per step, which dominates the rounding bound below.

The accumulator is single precision. For an example small acceleration of 0.005 m/s^2 over a bounding
1.5 hour burn with a 0.2 s ``controlPeriod`` -- 27,000 updates accumulating 27 m/s -- every ``+=``
rounds to the nearest representable value, bounding the accumulated error by :math:`N \cdot 2^{-25}`
relative for ``float`` and :math:`N \cdot 2^{-54}` for ``double``:

.. list-table:: Accumulation drift, example 1.5 hour burn at 0.005 m/s^2
    :widths: 35 25 25
    :header-rows: 1

    * - Accumulator type
      - Worst-case drift
      - Fraction of 27 m/s
    * - ``float`` (as built)
      - 2.2e-2 m/s
      - 0.080%
    * - ``double`` (reference only)
      - 4.0e-11 m/s
      - 1.5e-10%

Single precision is adequate with margin; no ``double`` accumulator or compensated summation is
warranted.

Range and saturation
~~~~~~~~~~~~~~~~~~~~

For an example acceleration of 0.5 m/s^2 with a 0.2 s ``controlPeriod``, the per-update increment is
:math:`\delta = \Delta t \, a = 0.1` m/s and the accumulator has two ceilings:

.. list-table:: Accumulator range limits, example acceleration 0.5 m/s^2
    :widths: 40 20 35
    :header-rows: 1

    * - Limit
      - Value
      - Reached at 0.5 m/s^2
    * - Stagnation (increments stop registering)
      - 2.1e6 m/s
      - ~49 days of continuous thrust
    * - IEEE ``float`` overflow to infinity
      - 3.4e38 m/s
      - ~2e31 years

Stagnation binds first: an increment is lost once :math:`\delta < \mathrm{ULP}(M)/2`, i.e. above
:math:`M \approx \delta \cdot 2^{24}` (here :math:`2^{21}` m/s). Past that point the accumulator
freezes silently -- no trap, no infinity. Neither limit is reachable in operation.

Module Architecture
-------------------
Three-layer split:

- **Adapter (``dvAccumulation.h/.cpp``, ``class DvAccumulation : SysModel``).** Reads the input
  message, converts ``AccelBody`` to an ``Eigen::Vector3f`` via ``utilities/fsw/eigenSupport.h``,
  drives the algorithm with the current ``callTime``, and writes the output message. The adapter owns
  the algorithm via ``std::unique_ptr`` and constructs it inside ``reset()`` after validating that
  ``imuInMsg`` is linked.
- **Algorithm (``dvAccumulationAlgorithm.h/.cpp``, ``class DvAccumulationAlgorithm``).** Pure
  algorithm — no SysModel, no messaging, no time. ``update(rDDotNoGravity_BN_B, accelBias_B)`` takes
  two ``Eigen::Vector3f`` and returns the accumulated ``vehAccumDV_B`` (``Eigen::Vector3f``, m/s),
  subtracting the supplied ``accelBias_B`` and integrating over the ``controlPeriod`` held in its
  validated ``DvAccumulationConfig``. Time-tagging the output message is the adapter's job.
- **C shim (``dvAccumulationAlgorithm_c.h/.cpp``).** Pure-C interface for Ada FFI: opaque handle
  plus ``DvAccumulationAlgorithm_create``/``_destroy``/``_validateConfig``/``_setConfig``/
  ``_reInitialize``/``_update``. ``_update`` takes a ``Vector3f_c`` acceleration and a
  ``Vector3f_c`` bias and returns a ``Vector3f_c`` Delta-V, using the shared ``Vector3f_c``
  from ``utilities/fsw/plainCAlgorithmDataTypes.h``.

Algorithm Layer
---------------
Given the configured control period :math:`\Delta t` (``controlPeriod``), and the per-call arguments
:math:`\mathbf{b}` (``accelBias_B``) and the body-frame acceleration ``rDDotNoGravity_BN_B``:

1. On the first ``update()`` after construction or ``reInitialize()``, start the accumulation window
   and return without integrating: there is no elapsed interval yet.
2. On every later call, subtract the bias and integrate one control period:

   .. math::

      \Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \, (\ddot{\mathbf{r}}_{B} - \mathbf{b})

3. Return ``vehAccumDV_B`` = :math:`\Delta\mathbf{v}`. The adapter tags the output message with
   ``timeTag = callTime * kNano2Sec``.

After :math:`N` calls the accumulator therefore holds the bias-corrected acceleration integrated
over :math:`(N-1)\,\Delta t`, the elapsed time since the window opened.

User Guide
----------
The required module configuration is::

    module = dvAccumulationF32.DvAccumulation()
    module.modelTag = "dvAccumulation"
    module.controlPeriod = 0.2      # [s] integration step; required (> 0), must match the task rate
    module.accelBias_B = [0., 0., 0.]  # [m/s^2] measured bias, subtracted per sample; optional
    module.imuInMsg.subscribeTo(mimuMajorityVote.imuSensorBodyOutMsg)
    # Subscribe a downstream consumer (e.g. dvExecuteGuidance) to module.dvAccumulationOutMsg.

``controlPeriod`` must be set to the rate at which the adapter is driven before ``reset()``. Call
``reset(callTime)`` once before the first ``updateState(callTime)``; ``reset`` throws
``std::invalid_argument`` if ``imuInMsg`` is not linked, and ``fsw::invalid_argument`` if
``controlPeriod`` is not positive. Editing either property after
``reset()`` takes effect on the next ``reconfigure()``, which installs parameters without re-arming
the accumulation window. On a state transition the flight software calls ``reInitialize()`` rather
than ``reset()``.
