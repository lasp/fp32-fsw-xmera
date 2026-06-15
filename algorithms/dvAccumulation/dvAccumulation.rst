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
    * - dvAcumOutMsg
      - ``NavTransMsgF32Payload``
      - Output navigation message. The algorithm populates ``timeTag`` (seconds, double) and
        ``vehAccumDV`` (m/s, float[3]); the position and velocity fields are left zero.

Module Parameters
-----------------
``dvAccumulation`` has no tunable parameters. The Config class
(``DvAccumulationConfig``) is intentionally empty — its only role is to keep the algorithm
shape uniform with every other ported algorithm (two-phase init: ``Config::create()`` in
the adapter's ``reset()``, passed to the algorithm constructor).

Module Assumptions and Limitations
----------------------------------
- ``measTime`` is in nanoseconds.
- The algorithm holds running state (``vehAccumDV_B``, ``previousTime``, ``dvInitialized``). The
  state is zeroed on ``reset()``; outside of that boundary the accumulator only grows.
- Reset semantics: ``resetState(accData)`` seeds ``previousTime`` to the latest non-zero
  ``measTime`` in the input snapshot so the first ``update()`` after reset integrates only
  packets that arrived after that latch.
- Bootstrap: on the first ``update()`` after a reset, the first packet with ``measTime`` greater
  than the seeded ``previousTime`` is *consumed* by the bootstrap (it becomes the new
  ``previousTime``, no integration), and only subsequent packets contribute. This is faithful
  to the original Basilisk module semantics.
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
  (``Eigen::Vector3f``, m/s). The empty ``DvAccumulationConfig`` flows through the constructor.
- **C shim (``dvAccumulationAlgorithm_c.h/.cpp``, ``dvAccumulationTypes.h``).** Pure-C interface
  for Ada FFI: opaque handle plus ``DvAccumulationAlgorithm_create``/``_destroy``/``_resetState``/
  ``_update``. ``dvAccumulationTypes.h`` (pure C) declares ``DvAccumulationOutput_c``, which is
  the POD mirror of the C++ output using the shared ``Vector3f_c`` from
  ``utilities/algorithmCShimTypes.h``. The shim exposes
  ``DvAccumulationAlgorithm_getMaxAccBufPkt()`` for Ada elaboration-time validation against
  ``MAX_ACC_BUF_PKT``.

Algorithm Layer
---------------
Given an input snapshot ``localPkts`` and the previously-seen latest time ``previousTime``:

1. Sort ``localPkts.accPkts`` in place (well, a local copy) by ascending ``measTime`` using
   ``std::ranges::sort``.
2. On the first ``update()`` after a reset (``dvInitialized == 0``), scan the sorted buffer
   for the first packet with ``measTime > previousTime`` and latch it as the new
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
    # Subscribe a downstream consumer to module.dvAcumOutMsg.

There is no further setup — no parameters to set, no validators to satisfy. Call
``reset(callTime)`` once before the first ``updateState(callTime)``; ``reset`` throws
``std::invalid_argument`` if ``accPktInMsg`` is not linked.
