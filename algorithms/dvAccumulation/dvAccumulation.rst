.. raw:: latex

    {\LARGE \textbf{dvAccumulation}}

Executive Summary
-----------------
``dvAccumulation`` integrates a single body-frame acceleration sample into a running Delta-V
accumulator. Each ``updateState()`` reads one ``IMUSensorBodyMsgF32Payload`` and takes its
``AccelBody`` field as the body-frame non-gravitational acceleration :math:`\ddot{\mathbf{r}}_{B}`.
The time step is the configured control period :math:`\Delta t`, so the sample is integrated via
:math:`\Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \cdot \ddot{\mathbf{r}}_{B}`. The module
outputs the running accumulator, time-tagged with the module call time.

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

Bounds are enforced by ``DvAccumulationConfig::create`` at ``reset()``, which throws
``fsw::invalid_argument`` on a violation.

Module Assumptions and Limitations
----------------------------------
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

Module Architecture
-------------------
Three-layer split:

- **Adapter (``dvAccumulation.h/.cpp``, ``class DvAccumulation : SysModel``).** Reads the input
  message, converts ``AccelBody`` to an ``Eigen::Vector3f`` via ``utilities/fsw/eigenSupport.h``,
  drives the algorithm with the current ``callTime``, and writes the output message. The adapter owns
  the algorithm via ``std::unique_ptr`` and constructs it inside ``reset()`` after validating that
  ``imuInMsg`` is linked.
- **Algorithm (``dvAccumulationAlgorithm.h/.cpp``, ``class DvAccumulationAlgorithm``).** Pure
  algorithm — no SysModel, no messaging, no time. ``update(rDDotNoGravity_BN_B)`` takes an
  ``Eigen::Vector3f`` and returns the accumulated ``vehAccumDV_B`` (``Eigen::Vector3f``, m/s),
  integrating over the ``controlPeriod`` held in its validated ``DvAccumulationConfig``. Time-tagging
  the output message is the adapter's job.
- **C shim (``dvAccumulationAlgorithm_c.h/.cpp``).** Pure-C interface for Ada FFI: opaque handle
  plus ``DvAccumulationAlgorithm_create``/``_destroy``/``_validateConfig``/``_setConfig``/
  ``_reInitialize``/``_update``. ``_update`` takes a
  ``Vector3f_c`` acceleration and returns a ``Vector3f_c`` Delta-V, using the shared ``Vector3f_c``
  from ``utilities/fsw/plainCAlgorithmDataTypes.h``.

Algorithm Layer
---------------
Given the configured control period :math:`\Delta t` (``controlPeriod``) and the body-frame
acceleration ``rDDotNoGravity_BN_B``:

1. On the first ``update()`` after construction or ``reInitialize()``, start the accumulation window
   and return without integrating: there is no elapsed interval yet.
2. On every later call, integrate one control period:

   .. math::

      \Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \cdot \ddot{\mathbf{r}}_{B}

3. Return ``vehAccumDV_B`` = :math:`\Delta\mathbf{v}`. The adapter tags the output message with
   ``timeTag = callTime * kNano2Sec``.

After :math:`N` calls the accumulator therefore holds the acceleration integrated over
:math:`(N-1)\,\Delta t`, the elapsed time since the window opened.

User Guide
----------
The required module configuration is::

    module = dvAccumulationF32.DvAccumulation()
    module.modelTag = "dvAccumulation"
    module.controlPeriod = 0.2      # [s] integration step; required (> 0), must match the task rate
    module.imuInMsg.subscribeTo(mimuMajorityVote.imuSensorBodyOutMsg)
    # Subscribe a downstream consumer (e.g. dvExecuteGuidance) to module.dvAccumulationOutMsg.

``controlPeriod`` must be set to the rate at which the adapter is driven before ``reset()``. Call
``reset(callTime)`` once before the first ``updateState(callTime)``; ``reset`` throws
``std::invalid_argument`` if ``imuInMsg`` is not linked, and ``fsw::invalid_argument`` if
``controlPeriod`` is not positive. Editing ``controlPeriod`` after ``reset()`` takes effect on the
next ``reconfigure()``. On a state transition the flight software calls ``reInitialize()`` rather
than ``reset()``.
