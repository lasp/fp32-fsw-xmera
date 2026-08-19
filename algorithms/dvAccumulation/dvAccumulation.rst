.. raw:: latex

    {\LARGE \textbf{dvAccumulation}}

Executive Summary
-----------------
``dvAccumulation`` integrates a single body-frame acceleration sample into a running Delta-V
accumulator. Each ``updateState()`` reads one ``IMUSensorBodyMsgF32Payload`` and takes its
``AccelBody`` field as the body-frame non-gravitational acceleration :math:`\ddot{\mathbf{r}}_{B}`.
The time step comes from the module call time,
:math:`\Delta t = (\text{callTime} - \text{previousTime})`, so the sample is integrated via
:math:`\Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \cdot \ddot{\mathbf{r}}_{B}`. The module
outputs the running accumulator plus the time-tag of the most-recently-ingested sample.

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
``dvAccumulation`` has no tunable parameters, so there is no Config class — the algorithm is
default-constructed by the adapter's ``reset()``.

Module Assumptions and Limitations
----------------------------------
- ``callTime`` is in nanoseconds and is assumed to advance once per new sample. ``dvAccumulation`` runs
  at the same cadence as its upstream producer and immediately after it, so the acceleration sample is
  fresh on every call and the call-time delta is the correct integration step.
- The algorithm holds running state split into **non-persistent** (``vehAccumDV_B``) and
  **persistent** (``previousTime``). ``reInitialize()`` resets all of it;
  ``reInitializeExceptPersistentStates()`` resets only ``vehAccumDV_B``, keeping the time reference so
  a continuously-running module keeps integrating across the boundary.
- Lifecycle: the adapter constructs the algorithm in ``reset()`` (startup only). State-transition
  hooks call ``reInitialize()`` / ``reInitializeExceptPersistentStates()``; ``reset()`` is not
  re-invoked on transitions.
- Time reference: on the first ``update()`` after ``reInitialize()`` (``previousTime == 0``), the call
  only sets the time reference ``previousTime = callTime`` (no integration), so ``dt`` does not blow up
  against a zero baseline. ``previousTime == 0`` doubles as the "time reference not yet set" marker, which
  relies on ``callTime`` being non-zero on the first call after a re-initialization (always true for the
  flight mission clock).
- A call whose ``callTime`` is not strictly greater than ``previousTime`` is ignored (no integration),
  so a repeated or non-advancing call time does not double-count.
- The accumulator is float-precision (``Eigen::Vector3f``). ``dt`` is computed in float using
  ``kNano2SecF``. ``timeTag`` stays double in the output message.

Module Architecture
-------------------
Three-layer split:

- **Adapter (``dvAccumulation.h/.cpp``, ``class DvAccumulation : SysModel``).** Reads the input
  message, converts ``AccelBody`` to an ``Eigen::Vector3f`` via ``utilities/fsw/eigenSupport.h``,
  drives the algorithm with the current ``callTime``, and writes the output message. The adapter owns
  the algorithm via ``std::unique_ptr`` and constructs it inside ``reset()`` after validating that
  ``imuInMsg`` is linked.
- **Algorithm (``dvAccumulationAlgorithm.h/.cpp``, ``class DvAccumulationAlgorithm``).** Pure
  algorithm — no SysModel, no messaging. ``update(callTime, rDDotNoGravity_BN_B)`` takes the call time
  and an ``Eigen::Vector3f`` and returns the accumulated ``vehAccumDV_B`` (``Eigen::Vector3f``, m/s).
  Time-tagging the output message is the adapter's job. Default-constructed — no configuration.
- **C shim (``dvAccumulationAlgorithm_c.h/.cpp``).** Pure-C interface for Ada FFI: opaque handle
  plus ``DvAccumulationAlgorithm_create``/``_destroy``/``_reInitialize``/
  ``_reInitializeExceptPersistentStates``/``_update``. ``_update`` takes the call time and a
  ``Vector3f_c`` acceleration and returns a ``Vector3f_c`` Delta-V, using the shared ``Vector3f_c``
  from ``utilities/fsw/plainCAlgorithmDataTypes.h``.

Algorithm Layer
---------------
Given the call time ``callTime``, the body-frame acceleration ``rDDotNoGravity_BN_B``, and the
previously-seen call time ``previousTime``:

1. On the first ``update()`` after ``reInitialize()`` (``previousTime == 0``), latch
   ``previousTime = callTime`` and return without integrating.
2. Otherwise, if ``callTime > previousTime``, integrate the elapsed step:

   .. math::

      \Delta t = (\text{callTime} - \text{previousTime}) \cdot \mathtt{kNano2SecF}
      \quad,\quad
      \Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \cdot \ddot{\mathbf{r}}_{B}
      \quad,\quad
      \text{previousTime} \leftarrow \text{callTime}

3. A ``callTime`` that does not advance past ``previousTime`` is ignored (no integration).
4. Return ``vehAccumDV_B = \Delta\mathbf{v}``. The adapter tags the output message with
   ``timeTag = callTime * kNano2Sec``.

User Guide
----------
The required module configuration is::

    module = dvAccumulationF32.DvAccumulation()
    module.modelTag = "dvAccumulation"
    module.imuInMsg.subscribeTo(mimuMajorityVote.imuSensorBodyOutMsg)
    # Subscribe a downstream consumer (e.g. dvExecuteGuidance) to module.dvAccumulationOutMsg.

There is no further setup — no parameters to set, no validators to satisfy. Call
``reset(callTime)`` once before the first ``updateState(callTime)``; ``reset`` throws
``std::invalid_argument`` if ``imuInMsg`` is not linked. On a state transition the flight software
calls ``reInitialize()`` (or ``reInitializeExceptPersistentStates()``) rather than ``reset()``.
