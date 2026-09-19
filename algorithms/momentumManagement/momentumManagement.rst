Executive Summary
-----------------

This module reads in the Reaction Wheel (RW) speeds, determines the net RW angular momentum, and requests the
torque that dumps it. A configured threshold sets the momentum at which a dump starts and ends. The output is
a commanded torque :math:`{}^{B}\bm{L}_r` expressed in body frame components.

The momentum check runs on **every** update, so the requested torque tracks the RW speeds as they evolve.

A downstream mapping module (``forceTorqueThrForceMapping``) converts the commanded torque into per-thruster
forces, and a thruster firing module converts those into thruster on-times.

All numeric computation is single-precision (``float`` / fp32).

Module Architecture
-------------------

The **algorithm** (``MomentumManagementAlgorithm``) is framework-free and Eigen-typed. It holds a validated
``MomentumManagementConfig`` and implements the dumping law described under `Mathematical Formulation`_. Its
``update()`` never throws, returns the requested torque as an ``Eigen::Vector3f``, and advances the momentum
integrator, which is the module's only runtime state; ``reInitialize()`` re-seeds it.

The **Xmera adapter** (``MomentumManagement``) inherits from ``SysModel`` and owns all messaging concerns. It
converts between the message payloads' C arrays and the algorithm's Eigen types, and writes the output message on
every update. Configuration uses two-phase initialization: the caller sets the public properties, then ``reset()``
validates the input links, builds the configuration, and constructs the algorithm.

The **Adamant adapter** is a C shim (``momentumManagementAlgorithm_c.h`` / ``.cpp``) exposing the algorithm
through an opaque handle for Ada FFI. ``update()`` returns the requested torque as a ``Vector3f_c`` POD, and the
configuration crosses the boundary as flattened scalars. A non-throwing ``validateConfig()`` lets Ada pre-check a
configuration before calling the throwing ``create()`` / ``setConfig()``.

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
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Output message with the requested body-frame dumping torque :math:`{}^{B}\bm{L}_r` [Nm], written every
        update.
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgF32Payload`
      - Reaction wheel speed input message [r/s], read every update.
    * - rwConfigDataInMsg
      - :ref:`RWArrayConfigMsgF32Payload`
      - RW array configuration input message, read during ``reset()`` and ``reconfigure()``.
    * - rwAvailInMsg
      - :ref:`RWAvailabilityMsgPayload`
      - Optional per-wheel availability input message, read during ``reset()`` and ``reconfigure()``. Without
        it every wheel counts as available.

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

The effectors cannot always produce torque about every direction. A single gimbaled thruster, for example,
produces the torque :math:`\bm{r}_{M/C} \times \bm{F}`, which is always perpendicular to the moment arm
:math:`\bm{r}_{M/C}` from the center of mass to the thrust point. Cluster momentum along that arm can never be
dumped. Let :math:`[P]` be the orthogonal projector onto the directions the effectors can dump about. The law
acts on that part of the cluster momentum alone,

.. math::

    {}^{B}\bm{h}_{s,\text{dmp}} = [P] \, {}^{B}\bm{h}_{s} .

For that thruster the undumpable direction is the unit moment arm, and the projector removes it:

.. math::

    \hat{\bm{n}} = \frac{\bm{r}_{M/C}}{|\bm{r}_{M/C}|},
    \qquad
    [P] = [I] - \hat{\bm{n}} \hat{\bm{n}}^{T} .

This projector removes the component along :math:`\hat{\bm{n}}`. It does not change the plane that is
perpendicular to :math:`\hat{\bm{n}}`. Effectors that can produce torque about every direction give
:math:`[P] = [I]`. If the effectors cannot dump about more than one direction, :math:`[N]` holds those
directions as its orthonormal columns. The projector is then :math:`[P] = [I] - [N][N]^{T}`.

Let :math:`h_{s,\text{min}}` be the momentum at which a dump starts. This threshold gates the whole control
law. At or above it the commanded torque opposes the dumpable momentum,

.. math::

    {}^{B}\bm{L}_r = -K \, {}^{B}\bm{h}_{s,\text{dmp}} - K_i \, {}^{B}\bm{H}_{s,\text{dmp}}
    \qquad |\bm{h}_{s,\text{dmp}}| \ge h_{s,\text{min}},

with :math:`K` the proportional gain of the dumping loop and :math:`K_i` the integral gain acting on

.. math::

    {}^{B}\bm{H}_{s,\text{dmp}} = \int_{t_0}^{t} {}^{B}\bm{h}_{s,\text{dmp}} \,\text{d}t,

the accumulated dumpable momentum. The integral is advanced with a trapezoidal rule using the configured
``controlPeriod`` as a fixed step (the module is expected to run at that rate), and every component of
:math:`\bm{H}_{s,\text{dmp}}` is then clamped to :math:`\pm` ``integralLimit``, preserving its sign, so a
sustained momentum cannot wind the integral term up without bound.

Below the threshold the dump is over. The module then requests no torque and clears the integral,

.. math::

    {}^{B}\bm{L}_r = \bm{0}, \quad {}^{B}\bm{H}_{s,\text{dmp}} = \bm{0}
    \qquad |\bm{h}_{s,\text{dmp}}| < h_{s,\text{min}} .

The integral is cleared rather than held for two reasons. A held integral is the only term left inside the
deadband, so it would go on commanding a constant torque with no proportional term to oppose it. That torque
drives the cluster momentum through zero and out of the deadband on the opposite side, where the law engages
again, which makes the module limit-cycle across the threshold and waste propellant. A held integral also
outlives its own dump: it measures the impulse the effectors failed to deliver during that dump, which says
nothing about a dump that starts hours later.

Momentum outside the dumpable subspace must not reach the law, which is why :math:`[P]` is applied before
everything else. The effectors cannot remove that momentum, so it would hold the deadband open and make the
integral grow along a direction no torque can act on. The anti-windup clamp bounds each body component of the
integral separately, and a clamp in body components does not preserve the direction of the vector it clamps.
It therefore turns that growth into a request the effectors *can* deliver, and the module commands a dump of
momentum that is not there. Projecting first removes the cause: the integral only ever holds momentum the
effectors can remove.

The law drives the dumpable momentum to zero, not to the threshold. The threshold only sets the momentum at
which a dump starts and ends. A threshold of :math:`h_{s,\text{min}} = 0` holds the gate open at all times,
because the momentum magnitude is never negative.

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
      - [Nms] RW cluster momentum at which a dump starts, and below which it ends. Zero is permitted and makes
        the module dump at all times.
    * - K
      - float
      - finite, :math:`\ge 0`
      - [1/s] Proportional gain :math:`K` mapping the stored momentum onto the requested torque. Zero switches
        the proportional term off and leaves the dump to the integral term. A negative gain is rejected,
        because it would drive the wheels away from the threshold. The reciprocal of :math:`K` is the time
        constant of the dump, so size :math:`K` from the torque the effectors can deliver: a stored momentum of
        10 Nms with :math:`K = 0.05` :math:`\text{s}^{-1}` asks for 0.5 Nm.
    * - Ki
      - float
      - finite, :math:`\ge 0`
      - [1/s2] Integral gain :math:`K_i` on the accumulated stored momentum. Zero switches the integral term
        off.
    * - integralLimit
      - float
      - finite, :math:`\ge 0`, and :math:`> 0` when ``Ki`` :math:`> 0`
      - [Nms2] Anti-windup clamp applied to each body-frame component of the momentum integral. A zero
        limit is rejected while the integral is active, because it would silently pin the integral term to zero
        instead of disabling it -- set ``Ki`` to zero for that.
    * - controlPeriod
      - float
      - finite, :math:`\ge 0`, and :math:`> 0` when ``Ki`` :math:`> 0`
      - [s] Integration step for the integral term, i.e. the rate at which the module is scheduled. Only the
        integral term consumes it, so a purely proportional configuration (``Ki`` = 0) may leave it at zero. It
        must stay finite either way, since a non-finite step would make the request non-finite even with the
        integral switched off.
    * - dumpableProjection_B
      - 3x3 matrix
      - finite, symmetric, idempotent, rank :math:`\ge 1`
      - [-] Orthogonal projector :math:`[P]` onto the directions the effectors can dump about. The identity,
        the default, says that every direction can be dumped, which is correct for a thruster array with full
        torque authority about all three body axes. For a single gimbaled thruster use
        :math:`[I] - \hat{\bm{n}} \hat{\bm{n}}^{T}`, where :math:`\hat{\bm{n}}` is the unit moment arm from
        the center of mass to the thrust point. The zero matrix is rejected: it is a valid projector, but it
        would make the module request nothing for ever without reporting anything, and it is what a caller that
        zero-fills this parameter rather than setting it would supply.
    * - rwConfigDataInMsg payload
      - message
      - see below
      - RW spin axes and spin-axis inertias, read from the input message rather than set as a property.

Every wheel slot is configured: the array holds ``RW_EFF_CNT`` wheels, from ``mission/parameters.h``. The
reaction-wheel configuration read from ``rwConfigDataInMsg`` must satisfy: the spin axis matrix and spin-axis
inertias finite, and every spin axis a unit vector to within :math:`10^{-3}`. Valid axes are normalized exactly
on construction, so the momentum sum can rely on unit vectors. A slot that carries no wheel takes a zero
spin-axis inertia, which contributes no momentum.

An unavailable wheel reports no usable speed, so the module leaves it out of the momentum sum. The dumping
law then sees the cluster as if that wheel were not spinning. Mark a slot UNAVAILABLE through
``rwAvailInMsg`` when its wheel carries no usable speed.

User Guide
----------

The module uses two-phase initialization: set the public configuration properties, connect the input messages, then
``reset()`` builds and validates the configuration.

.. code-block:: python

    from xmera.fp32 import momentumManagementF32

    module = momentumManagementF32.MomentumManagement()
    module.modelTag = "momentumManagement"

    # Phase 1: configuration properties, set before reset()
    module.hsMin = 100.0 / 6000.0 * 100.0  # [Nms] RW cluster momentum at which a dump starts
    module.K = 0.05                        # [1/s] dumping loop proportional gain (0 disables it)
    module.Ki = 0.01                       # [1/s2] integral gain (0 disables the integral term)
    module.integralLimit = 1000.0          # [Nms2] anti-windup clamp per integral component
    module.controlPeriod = 0.5             # [s] task rate; only needed when Ki > 0

    # The directions the effectors can dump about. The identity keeps the whole cluster momentum.
    module.dumpableProjection_B = np.identity(3)

    # For a single gimbaled thruster, remove the moment arm direction, about which it makes no torque:
    #     arm = r_MB_B - CoM_B
    #     axis = arm / np.linalg.norm(arm)
    #     module.dumpableProjection_B = np.identity(3) - np.outer(axis, axis)

    # Connect the required input messages
    module.rwSpeedsInMsg.subscribeTo(rw_speed_in_msg)
    module.rwConfigDataInMsg.subscribeTo(rw_config_in_msg)

    # Optional: mark a wheel unavailable so its momentum is left out of the sum
    module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)

    # Phase 2: reset() validates the links and builds the config
    sim.AddModelToTask(task_name, module)

``rwSpeedsInMsg`` and ``rwConfigDataInMsg`` are required; ``reset()`` raises if either is unconnected.
``rwAvailInMsg`` is optional.

To push edited configuration properties onto a running algorithm without disturbing the integrator, call
``reconfigure()``. To re-seed the integrator itself, call ``reInitialize()``. The module also clears the
integrator on its own whenever the cluster momentum falls below ``hsMin``. Both methods raise
``XmeraLifecycleException`` if called before ``reset()``.

Module Assumptions and Limitations
----------------------------------

- The spacecraft is assumed to hold a steady inertial orientation during the momentum dumping maneuver, which is
  what justifies neglecting the :math:`\bm{\omega}_{B/N} \times \bm{h}_{s}` transport term.
- The integral is advanced with a fixed ``controlPeriod`` step rather than a measured elapsed time, so the
  module must actually be scheduled at that rate; a mismatch scales the integral term proportionally.
- :math:`{}^{B}\bm{L}_r` carries no memory of how much momentum has already been dumped -- only of how long the
  momentum has stayed above the threshold -- so the downstream firing logic is responsible for tracking delivery.
- The request is discontinuous at the threshold. It steps between zero and :math:`K \, h_{s,\text{min}}` as the
  cluster momentum crosses :math:`h_{s,\text{min}}`, so a cluster that hovers on the threshold makes the module
  switch its request on and off. There is no hysteresis on the threshold.
- The integral carries no state across a dump, because falling below the threshold clears it. A dump that ends
  and restarts therefore rebuilds its integral term from zero.
- The RW configuration is sampled at ``reset()`` / ``reconfigure()``, not per update, so it is treated as static
  for the life of the configuration.
- ``dumpableProjection_B`` is a configuration parameter, so the module cannot tell whether it still matches the
  effectors. Refresh it through ``reconfigure()`` whenever the geometry it came from changes -- a center-of-mass
  update, or a switch to a different thruster -- and call ``reInitialize()`` with it, because the accumulated
  integral belongs to the previous dumpable subspace. A stale projector mis-dumps silently.
- Momentum outside the dumpable subspace is left to the reaction wheels. The module does not report it, so it
  can grow to wheel saturation without any indication from this module.
- Single-precision arithmetic limits the achievable accuracy to roughly seven significant figures. Against a
  double-precision reference the observed error is at float epsilon (~4e-7 absolute on cluster momenta of order
  10 Nms), scaled by the gain :math:`K`.
