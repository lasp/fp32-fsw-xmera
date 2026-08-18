Executive Summary
-----------------

This module reads in the Reaction Wheel (RW) speeds, determines the net RW angular momentum, and then determines the
amount of angular momentum that must be dumped. The output is a desired angular momentum change
:math:`{}^{B}\Delta\bm{H}` expressed in body frame components.

The momentum check runs on **every** update, so the requested change tracks the RW speeds as they evolve.

A separate thruster firing logic module, ``thrMomentumDumping``, later computes the thruster on-cycling. The
intermediate ``thrForceMapping`` module maps the requested momentum change into thruster impulse requests: that
module maps a control torque vector into thruster forces, and because multiplying both the input torque and the
output force set by time preserves the relation, the same module also maps a desired angular momentum change into a
set of thruster impulse requests.

All numeric computation is single-precision (``float`` / fp32).

Module Architecture
-------------------

The **algorithm** (``ThrMomentumManagementAlgorithm``) is framework-free and Eigen-typed. It holds a validated
``ThrMomentumManagementConfig`` and implements the dumping law described under `Mathematical Formulation`_. Its
``update()`` never throws, carries no runtime state, and returns the requested momentum change as an
``Eigen::Vector3f``.

The **Xmera adapter** (``ThrMomentumManagement``) inherits from ``SysModel`` and owns all messaging concerns. It
converts between the message payloads' C arrays and the algorithm's Eigen types, and writes the output message on
every update. Configuration uses two-phase initialization: the caller sets the public properties, then ``reset()``
validates the input links, builds the configuration, and constructs the algorithm.

The **Adamant adapter** is a C shim (``thrMomentumManagementAlgorithm_c.h`` / ``.cpp``) exposing the algorithm
through an opaque handle for Ada FFI. ``update()`` returns the requested momentum change as a ``Vector3f_c`` POD. A
non-throwing ``validateConfig()`` lets Ada pre-check a configuration before calling the throwing ``create()`` /
``setConfig()``.

Message Connection Descriptions
-------------------------------

The following table lists all the module input and output messages. The module msg connection is set by the user
from python. The msg type contains a link to the message structure definition, while the description provides
information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - deltaHOutMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Output message with the requested angular momentum change :math:`{}^{B}\Delta\bm{H}` [Nms], written every
        update. The payload is a torque-shaped carrier reused here for angular momentum; the units are Nms, not Nm.
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgF32Payload`
      - Reaction wheel speed input message [r/s], read every update.
    * - rwConfigDataInMsg
      - :ref:`RWArrayConfigMsgF32Payload`
      - RW array configuration input message, read during ``reset()`` and ``reconfigure()``.

Mathematical Formulation
------------------------

Assume the spacecraft contains :math:`N_\text{RW}` reaction wheels. The net RW angular momentum is

.. math::

    \bm{h}_{s} = \sum_{i=1}^{N_\text{RW}} \hat{\bm{g}}_{s_{i}} J_{s_{i}} \Omega_{i}

where :math:`\hat{\bm{g}}_{s_{i}}` is the RW spin axis, :math:`J_{s_{i}}` the spin axis RW inertia, and
:math:`\Omega_{i}` the RW speed about that axis.

Because the inertial attitude of the spacecraft is assumed to be held nominally steady, the body-relative RW cluster
angular momentum rate can be approximated as

.. math::

    \dot{\bm{h}}_{s} = \frac{{}^{B}\text{d}\bm{h}_{s}}{\text{d}t} + \bm{\omega}_{B/N} \times \bm{h}_{s}
                     \approx \frac{{}^{B}\text{d}\bm{h}_{s}}{\text{d}t}

Let :math:`h_{s,\text{min}}` be the lower bound the momentum dumping strategy should achieve. The desired net change
in angular momentum is

.. math::

    {}^{B}\Delta\bm{H} = -\, {}^{B}\bm{h}_{s} \, \frac{|\bm{h}_{s}| - h_{s,\text{min}}}{|\bm{h}_{s}|}

so the requested change is anti-parallel to the stored momentum and leaves exactly :math:`h_{s,\text{min}}` behind.
When :math:`|\bm{h}_{s}| < h_{s,\text{min}}` no dumping is required and :math:`{}^{B}\Delta\bm{H}` is zero.

The magnitude :math:`|\bm{h}_{s}|` appears in the denominator, so the implementation additionally treats a cluster
momentum below :math:`10^{-6}` Nms as zero. That branch is only reachable when :math:`h_{s,\text{min}}` is itself
zero — for any positive threshold the :math:`|\bm{h}_{s}| < h_{s,\text{min}}` comparison short-circuits first — and
it prevents a :math:`0/0` division from producing NaN.

Module Parameters
-----------------

Configuration parameters are validated when ``reset()`` builds the algorithm configuration; an out-of-range value
raises ``fsw::invalid_argument`` and the module is not constructed.

.. list-table:: Module Configuration Parameters
    :widths: 20 15 30 35
    :header-rows: 1

    * - Parameter
      - Type
      - Valid range
      - Description
    * - hsMin
      - float
      - finite, :math:`\ge 0`
      - [Nms] Minimum RW cluster momentum for dumping. Zero is permitted and means "dump all stored momentum".
    * - rwConfigDataInMsg payload
      - message
      - see below
      - RW spin axes and spin-axis inertias, read from the input message rather than set as a property.

The reaction-wheel configuration read from ``rwConfigDataInMsg`` must satisfy: ``numRW`` no greater than the
compile-time maximum ``THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW`` (36, which must match ``RW_EFF_CNT``); the spin axis
matrix and spin-axis inertias finite; and each of the first ``numRW`` spin axes a unit vector to within
:math:`10^{-3}`. Valid axes are normalized exactly on construction, so the momentum sum can rely on unit vectors.
Columns beyond ``numRW`` describe no real wheel and are ignored.

User Guide
----------

The module uses two-phase initialization: set the public configuration properties, connect the input messages, then
``reset()`` builds and validates the configuration.

.. code-block:: python

    from xmera.fp32 import thrMomentumManagementF32

    module = thrMomentumManagementF32.ThrMomentumManagement()
    module.modelTag = "thrMomentumManagement"

    # Phase 1: configuration properties, set before reset()
    module.hsMin = 100.0 / 6000.0 * 100.0  # [Nms] lower ceiling of the RW cluster momentum

    # Connect the required input messages
    module.rwSpeedsInMsg.subscribeTo(rw_speed_in_msg)
    module.rwConfigDataInMsg.subscribeTo(rw_config_in_msg)

    # Phase 2: reset() validates the links and builds the config
    sim.AddModelToTask(task_name, module)

Both input messages are required; ``reset()`` raises if either is unconnected.

To push edited configuration properties onto a running algorithm, call ``reconfigure()``. It raises
``XmeraLifecycleException`` if called before ``reset()``.

Module Assumptions and Limitations
----------------------------------

- The spacecraft is assumed to hold a steady inertial orientation during the momentum dumping maneuver, which is
  what justifies neglecting the :math:`\bm{\omega}_{B/N} \times \bm{h}_{s}` transport term.
- :math:`{}^{B}\Delta\bm{H}` is recomputed from scratch on every update and carries no memory of what has already
  been dumped, so the downstream firing logic is responsible for tracking delivery.
- The RW configuration is sampled at ``reset()`` / ``reconfigure()``, not per update, so it is treated as static
  for the life of the configuration.
- Single-precision arithmetic limits the achievable accuracy to roughly seven significant figures. Against the
  original double-precision implementation the observed error is at float epsilon (~4e-7 absolute on momentum
  changes of order 10 Nms).
