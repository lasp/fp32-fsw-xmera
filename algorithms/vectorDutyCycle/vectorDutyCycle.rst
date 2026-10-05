Executive Summary
-----------------

This module gates a three-component vector on and off in a fixed duty cycle. During the on window it passes the
input vector through unchanged; during the off window it outputs a zero vector. The cycle is counted in
control periods and runs freely, independent of the vector it carries. The algorithm makes no assumption on what
the vector describes; the Xmera adapter carries a commanded body force or torque, selected by ``vectorType``.

It performs no arithmetic on the vector it carries: a passed-through vector is identical to its input, and all
three components are gated together, so the output is always either the input or zero.

All numeric computation in this module's neighbours is single-precision (``float`` / fp32); the payloads carried
here are ``float`` arrays.

Use Case: Momentum Desaturation with On/Off Thrusters
-----------------------------------------------------

The module can impose a duty cycle on thrusters that dump reaction wheel momentum. It is placed between the momentum
management and the thruster force mapping::

    momentumManagement              requested dumping torque   [Nm]
      -> vectorDutyCycle              gated dumping torque       [Nm]
        -> forceTorqueThrForceMapping   per-thruster force         [N]
          -> thrFiringRemainder / thrFiringSchmitt   thruster on-time  [s]
            -> thrusters

An on/off fixed-thrust thruster cannot produce an arbitrarily small torque: the smallest action available to it is
one minimum-fire-time pulse at full thrust. A dumping torque therefore arrives as a train of coarse kicks that
disturb the attitude loop. The duty cycle modulates the *time* the thrusters fire rather than the amplitude, and the
off window leaves the reaction wheels quiet control periods in which to recover the pointing. Size it by the
number of control periods the wheels need to null the attitude error that one on window injects.

Gating the torque ahead of the mapping gives the same thruster commands as gating the per-thruster force after it:
the mapping keeps no state between calls and maps a zero torque to exactly zero force on every thruster. That holds
only while the mapping receives no body force request, which this module does not see and would not withhold.

This use only makes sense for on/off thrusters:

- A magnetorquer or a throttleable electric thruster can produce a small continuous torque, so no duty cycle is
  needed. Choose the upstream gains low enough that the dumping torque stays inside the attitude controller's
  rejection authority.
- ``thrFiringRemainder`` already cycles on its own for small requests: it banks any on-time below
  ``thrMinFireTime`` into a pulse remainder and emits one minimum pulse every few cycles. That only holds while

  .. math::

      \frac{F}{F_\text{max}} < \frac{t_\text{min fire}}{T_\text{control}},

  i.e. while a proportional on-time would fall below the minimum pulse. Above that ratio the thruster fires every
  cycle and there are no quiet windows. With a sufficiently small gain in ``momentumManagement``, this module is
  not needed.
- The module must **not** be placed on an attitude-control path, where withholding a commanded torque for whole
  control periods would degrade the very loop it is meant to protect.

In this chain the duty ratio reduces the effective gain of the desaturation loop, an integrating
``momentumManagement`` winds up during off windows, and the first pulse of a new request may wait up to
``offPeriods`` control periods; see `Module Behaviour Notes`_.

Module Architecture
-------------------

The **algorithm** (``VectorDutyCycleAlgorithm``) is framework-free. It holds a validated
``VectorDutyCycleConfig`` and implements the cadence described under `Cadence`_. Its ``update()`` never throws
and returns the gated vector. The cadence counter is the module's only runtime state, and all of it is
non-persistent, so ``reInitialize()`` restarts the cycle outright and there is no
``reInitializeExceptPersistentStates()``.

The **Xmera adapter** (``VectorDutyCycle``) inherits from ``SysModel`` and owns all messaging concerns. The
adapter-only property ``vectorType`` selects whether the force or the torque message pair is gated; ``reset()``
fixes the selection. The adapter reads only the selected input message, maps between its C array and the
algorithm's ``Eigen::Vector3f``, and writes only the selected output message on every update. Configuration uses
two-phase initialization: the caller sets the public properties, then ``reset()`` validates the selected input
link, builds the configuration, and constructs the algorithm. The whole configuration lives in
module properties, so no input message is read to build it.

The **Adamant adapter** is a C shim (``vectorDutyCycleAlgorithm_c.h`` / ``.cpp``) exposing the algorithm through
an opaque handle for Ada FFI. The input vector crosses the boundary as a ``Vector3f_c`` and the configuration as
flattened scalars. A non-throwing ``validateConfig()`` lets Ada pre-check a configuration before calling the throwing ``create()`` / ``setConfig()``.

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
    * - cmdForceInMsg
      - :ref:`CmdForceBodyMsgF32Payload`
      - Commanded body-frame force [N]. Required and read every update when ``vectorType`` is ``Force``;
        otherwise ignored.
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Commanded body-frame torque [Nm]. Required and read every update when ``vectorType`` is ``Torque``;
        otherwise ignored.
    * - cmdForceOutMsg
      - :ref:`CmdForceBodyMsgF32Payload`
      - Gated body-frame force [N]: the input during an on period, zero during an off period. Written every
        update when ``vectorType`` is ``Force``; otherwise never written.
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Gated body-frame torque [Nm]: the input during an on period, zero during an off period. Written every
        update when ``vectorType`` is ``Torque``; otherwise never written.

Cadence
-------

One duty cycle is :math:`N_\text{on} +  N_\text{off}` control periods long, where :math:`N_\text{on}` is ``onPeriods`` and
:math:`N_\text{off}` is ``offPeriods``. The on window occupies the leading slots of the cycle, so for the :math:`n`-th update
since the last restart the gate passes the input through when

.. math::

    n \bmod (N_\text{on} + N_\text{off}) < N_\text{on}

and outputs a zero vector otherwise. Writing :math:`\boldsymbol{v}` for the input vector, the output is

.. math::

    \boldsymbol{v}_\text{out} = \begin{cases}
    \boldsymbol{v}, & n \bmod (N_\text{on} + N_\text{off}) < N_\text{on}\\
    \boldsymbol{0}, & \text{otherwise}
    \end{cases}

Three properties of this cadence are worth stating explicitly.

**It is free-running.** The counter advances on every update regardless of the input, so the on
windows sit at a fixed phase rather than being retriggered by the arrival of a new input. A new input can therefore
wait up to :math:`N_\text{off}` control periods before it is first passed through.

**It is all-or-nothing across the axes.** Within one update every vector component is gated identically, so the
output vector never points in a direction that the input did not.

**The vector is passed through, not scaled up.** The average output over a cycle is therefore

.. math::

    \bar{\boldsymbol{v}} = \frac{N_\text{on}}{N_\text{on} + N_\text{off}} \, \boldsymbol{v},

so the duty ratio acts as a gain reduction on any loop closed through the module, which the upstream gain must
account for (see `Module Behaviour Notes`_).

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
    * - onPeriods
      - uint32
      - :math:`\ge 1`
      - [-] Number of consecutive control periods, at the start of each cycle, for which the gate passes the
        input vector through. Zero is rejected because it would hold the vector at zero forever, silently
        disabling the input rather than configuring it.
    * - offPeriods
      - uint32
      - any value with ``onPeriods + offPeriods`` :math:`\le` ``UINT32_MAX``
      - [-] Number of consecutive control periods for which the gate outputs a zero vector. Zero is
        permitted and holds the gate fully open, which is how duty
        cycling is disabled. The only rejected values are those whose sum with ``onPeriods`` would wrap
        around, since a wrapped cycle length would come out shorter than its own on window.
    * - vectorType
      - ``VectorType``
      - ``Force`` or ``Torque``
      - Adapter only. Selects the message pair that the adapter gates. Defaults to ``Torque``. ``reset()`` fixes
        the selection; a later change takes effect at the next ``reset()``.

``onPeriods`` and ``offPeriods`` are counted in **control periods**, not seconds, so the module needs no ``controlPeriod``
parameter and no measured time step: it counts its own invocations. This makes the cadence exact — there is no
rounding of a duration onto a schedule — but it also means the wall-clock length of a cycle is set by the rate at
which the module is scheduled.

User Guide
----------

The module uses two-phase initialization: set the public configuration properties, connect the input message, then
``reset()`` builds and validates the configuration.

.. code-block:: python

    from xmera.fp32 import vectorDutyCycleF32

    module = vectorDutyCycleF32.VectorDutyCycle()
    module.modelTag = "vectorDutyCycle"

    # Phase 1: configuration properties, set before reset()
    module.onPeriods = 1     # [-] pass through for one control period ...
    module.offPeriods = 4   # [-] ... then hold off for four, giving a 1-in-5 duty cycle
    module.vectorType = vectorDutyCycleF32.VectorType_Torque  # gate the torque message pair (default)

    # Connect the input message of the selected vector type
    module.cmdTorqueInMsg.subscribeTo(cmd_torque_in_msg)

    # Phase 2: reset() validates the link and builds the config
    sim.AddModelToTask(task_name, module)

The input message of the selected vector type is required; ``reset()`` raises if it is unconnected. The other
input message is not read and its output message is not written.

To push edited configuration properties onto a running algorithm without restarting the cadence, call
``reconfigure()``. To restart the cadence at its on window, call ``reInitialize()``. Both raise
``XmeraLifecycleException`` if called before ``reset()``.

Module Assumptions and Limitations
----------------------------------

- **The cadence is counted in invocations.** The module must actually be scheduled at the intended control rate;
  the wall-clock duty cycle scales with the task period.
- **The cadence is free-running**, so a new command may be held at zero for up to ``offPeriods`` control
  periods before it is first passed through.

Module Behaviour Notes
----------------------

- **The upstream gain must be sized for the duty ratio.** Because the torque is passed through rather than scaled,
  the average delivered torque is :math:`N_\text{on} / (N_\text{on} + N_\text{off})` of the command. A cadence change therefore rescales the
  effective loop gain of the upstream controller.
- **An upstream integral term winds up during off windows.** The gate withholds torque while the upstream error
  persists, so an integrating controller keeps accumulating with no effect. Its integral limit and this module's
  ``offPeriods`` must be tuned together; a long off window with a generous integral limit produces an
  overshooting command when the gate reopens.
- **Withheld commands are discarded, not banked.** A command withheld during a off window is not carried
  forward. This is deliberate: an upstream integral term is already the accumulator, and banking the command here
  as well would integrate the same error twice. It does mean the module is only correct downstream of a closed-loop
  command, not of a fixed impulse budget.
