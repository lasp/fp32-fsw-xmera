.. raw:: latex

    {\LARGE \textbf{dvAccumulation}}

Executive Summary
-----------------
``dvAccumulation`` integrates body-frame accelerometer measurements into a running Delta-V
accumulator. Each input snapshot carries up to ``MAX_ACC_BUF_PKT`` (120) ``AccPktDataMsgF32Payload``
slots — every slot has a per-packet ``measTime`` (ns) and a ``accel_B`` (3-component float). The
algorithm sorts the snapshot by ``measTime``, integrates every packet whose timestamp is *strictly*
greater than the previously-seen latest time via
:math:`\Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \cdot \mathbf{a}_B`, and outputs the running
accumulator plus the time-tag of the most-recently-ingested sample.

Message Connection Descriptions
-------------------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Variable Name
      - Type
      - Description
    * - accPktInMsg
      - ``AccDataMsgF32Payload``
      - Input snapshot of up to 120 accelerometer packets, each carrying a ``measTime`` (uint64 ns)
        and a body-frame ``accel_B`` (float[3]).
    * - dvAccumulationOutMsg
      - ``NavTransMsgF32Payload``
      - Output navigation message. The algorithm populates ``timeTag`` (seconds, double) and
        ``vehAccumDV`` (m/s, float[3]); the position and velocity fields are left zero.

Module Parameters
-----------------
``dvAccumulation`` has no tunable parameters, so there is no Config class — the algorithm is
default-constructed by the adapter's ``reset()``.

Module Assumptions and Limitations
----------------------------------
- ``measTime`` is in nanoseconds.
- The algorithm holds running state split into **non-persistent** (``vehAccumDV_B``) and
  **persistent** (``previousTime``, ``dvInitialized``). ``reInitialize()`` resets all of it;
  ``reInitializeExceptPersistentStates()`` resets only ``vehAccumDV_B``, keeping the integration
  bookkeeping so a continuously-running module ignores the backlog already ingested.
- Lifecycle: the adapter constructs the algorithm in ``reset()`` (startup only). State-transition
  hooks call ``reInitialize()`` / ``reInitializeExceptPersistentStates()``; ``reset()`` is not
  re-invoked on transitions.
- Bootstrap: on the first ``update()`` after ``reInitialize()`` (``dvInitialized == 0``,
  ``previousTime == 0``), the first packet with ``measTime`` greater than ``previousTime`` is
  *consumed* by the bootstrap (it becomes the new ``previousTime``, no integration), and only
  subsequent packets contribute. This is faithful to the original Xmera module semantics.
- Packets with ``measTime`` not strictly greater than ``previousTime`` are dropped at ingest, so
  out-of-order or repeated input is safe (sorting is done inside the algorithm).
- If the ring is empty or no packet beats ``previousTime`` on a given snapshot, the output
  vector and time-tag are zero.
- The accumulator is float-precision (``Eigen::Vector3f``). ``dt`` is computed in float using
  ``kNano2SecF``. ``timeTag`` stays double in the output message.

Module Architecture
-------------------
Three-layer split:

- **Adapter (``dvAccumulation.h/.cpp``, ``class DvAccumulation : SysModel``).** Reads the input
  message, drives the algorithm, writes the output message. The adapter owns the algorithm via
  ``std::unique_ptr`` and constructs it inside ``reset()`` after validating that
  ``accPktInMsg`` is linked.
- **Algorithm (``dvAccumulationAlgorithm.h/.cpp``, ``class DvAccumulationAlgorithm``).** Pure
  algorithm — no SysModel, no messaging. Takes ``AccDataMsgF32Payload`` and returns a
  ``DvAccumulationOutput`` carrying ``timeTag`` (double, s) and ``vehAccumDV_B``
  (``Eigen::Vector3f``, m/s). Default-constructed — no configuration.
- **C shim (``dvAccumulationAlgorithm_c.h/.cpp``, ``dvAccumulationTypes.h``).** Pure-C interface
  for Ada FFI: opaque handle plus ``DvAccumulationAlgorithm_create``/``_destroy``/
  ``_reInitialize``/``_reInitializeExceptPersistentStates``/``_update``. ``dvAccumulationTypes.h``
  (pure C) declares ``DvAccumulationOutput_c``, which is
  the POD mirror of the C++ output using the shared ``Vector3f_c`` from
  ``utilities/fsw/plainCAlgorithmDataTypes.h``. The shim exposes
  ``DvAccumulationAlgorithm_getMaxAccBufPkt()`` for Ada elaboration-time validation against
  ``MAX_ACC_BUF_PKT``.

Algorithm Layer
---------------
Given an input snapshot ``localPkts`` and the previously-seen latest time ``previousTime``:

1. Sort ``localPkts.accPkts`` in place (a local copy) by ascending ``measTime`` using an
   iterative quicksort (a hand-written sort is used rather than ``std::ranges::sort`` to avoid a
   libstdc++ ``clang-analyzer-security.ArrayBound`` false positive on the raw C array).
2. On the first ``update()`` after ``reInitialize()`` (``dvInitialized == 0``), scan the sorted
   buffer for the first packet with ``measTime > previousTime`` and latch it as the new
   ``previousTime`` (consuming that packet — it doesn't integrate). Mark ``dvInitialized = 1``.
3. Iterate the sorted buffer. For each packet with ``measTime > previousTime``:

   .. math::

      \Delta t = (\text{measTime} - \text{previousTime}) \cdot \mathtt{kNano2SecF}
      \quad,\quad
      \Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \cdot \mathbf{a}_B
      \quad,\quad
      \text{previousTime} \leftarrow \text{measTime}

4. Emit ``timeTag = previousTime * kNano2Sec`` and ``vehAccumDV_B = \Delta\mathbf{v}``.

User Guide
----------
The required module configuration is::

    module = dvAccumulationF32.DvAccumulation()
    module.modelTag = "dvAccumulation"
    module.accPktInMsg.subscribeTo(accPktSource)
    # Subscribe a downstream consumer to module.dvAccumulationOutMsg.

There is no further setup — no parameters to set, no validators to satisfy. Call
``reset(callTime)`` once before the first ``updateState(callTime)``; ``reset`` throws
``std::invalid_argument`` if ``accPktInMsg`` is not linked.
