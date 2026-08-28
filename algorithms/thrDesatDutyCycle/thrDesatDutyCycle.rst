Executive Summary
-----------------

This module gates a commanded thruster force on and off in a fixed duty cycle. During the firing window it passes
the commanded per-thruster force through unchanged; during the settling window it commands zero force, leaving the
reaction wheels quiet control periods in which to re-stabilize the attitude between desaturation pulses.

The module sits between the thruster force mapping and the thruster firing logic in the momentum desaturation
chain::

    momentumManagement            requested dumping torque   [Nm]
      -> forceTorqueThrForceMapping   per-thruster force     [N]
        -> thrDesatDutyCycle          gated per-thruster force  [N]
          -> thrFiringRemainder / thrFiringSchmitt   thruster on-time  [s]
            -> thrusters

It performs no arithmetic on the force it carries: a passed-through command is bit-identical to its input. The
force magnitude, the minimum fire time, the control period and the conversion to an on-time are all the concern
of the downstream firing module.

All numeric computation in this module's neighbours is single-precision (``float`` / fp32); the payloads carried
here are ``float`` arrays.

Why an Explicit Duty Cycle
--------------------------

The binding constraint is the **minimum impulse bit**, not thrusters as such. An on/off fixed-thrust thruster
cannot produce an arbitrarily small torque: the smallest action available to it is one minimum-fire-time pulse at
full thrust. A desaturation torque therefore arrives as a train of coarse kicks that disturb the attitude loop, so
the strategy is to modulate the *time* the thrusters fire rather than the amplitude, and to leave quiet windows in
which the wheels can recover the pointing. The settling window is what buys that recovery time: it should be sized
by the number of control periods the wheels need to null the attitude error one firing window injects.

The module is consequently **specific to momentum desaturation with on/off thrusters**:

- A magnetorquer or a throttleable electric thruster can produce a small continuous torque, so neither problem
  arises and no gate is needed. Choose the upstream gains low enough that the dumping torque stays inside the
  attitude controller's rejection authority.
- The gate must **not** be placed on an RCS attitude-control path, where deliberately withholding a commanded
  force for whole control periods would degrade the very loop it is meant to protect.

Note also that ``thrFiringRemainder`` already self-cadences for small requests: it banks any on-time below
``thrMinFireTime`` into a pulse remainder and emits one minimum pulse every few cycles, which delivers the
requested average force with quiet cycles in between and no counter at all. That free duty cycling only holds while

.. math::

    \frac{F}{F_\text{max}} < \frac{t_\text{min fire}}{T_\text{control}},

i.e. while the request is small enough that a proportional on-time would fall below the minimum pulse. Above that
ratio the thruster fires every cycle and there are no quiet windows, which is the regime this module exists to
serve. Thus, with a sufficiently small feedback gain K in ``momentumManagement`` resulting in a small requested
torque, this module does not have to be included in the desat chain.

Module Architecture
-------------------

The **algorithm** (``ThrDesatDutyCycleAlgorithm``) is framework-free. It holds a validated
``ThrDesatDutyCycleConfig`` and implements the cadence described under `Cadence`_. Its ``update()`` never throws
and returns the gated force command. The cadence counter is the module's only runtime state, and all of it is
non-persistent, so ``reInitialize()`` restarts the cycle outright and there is no
``reInitializeExceptPersistentStates()``.

The **Xmera adapter** (``ThrDesatDutyCycle``) inherits from ``SysModel`` and owns all messaging concerns. It maps
between the message payload's C array and the algorithm's ``std::array`` and writes the output message on every
update. Configuration uses two-phase initialization: the caller sets the public properties, then ``reset()``
validates the input link, builds the configuration, and constructs the algorithm. The whole configuration lives in
module properties, so no input message is read to build it.

The **Adamant adapter** is a C shim (``thrDesatDutyCycleAlgorithm_c.h`` / ``.cpp``) exposing the algorithm through
an opaque handle for Ada FFI. The force command crosses the boundary as a bounded-array POD
(``ThrDesatDutyCycleForceCmd_c``) and the configuration as flattened scalars. A non-throwing ``validateConfig()``
lets Ada pre-check a configuration before calling the throwing ``create()`` / ``setConfig()``.

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
    * - thrForceInMsg
      - :ref:`THRArrayCmdForceMsgF32Payload`
      - Commanded per-thruster desaturation force [N], read every update.
    * - thrForceOutMsg
      - :ref:`THRArrayCmdForceMsgF32Payload`
      - Gated per-thruster force [N]: the input during a firing period, zero during a settling period. Written
        every update.

Cadence
-------

One duty cycle is :math:`N_f +  N_s` control periods long, where :math:`N_f` is ``firingPeriods`` and
:math:`N_s` is ``settlingPeriods``. Firing occupies the leading slots of the cycle, so for the :math:`n`-th update
since the last restart the gate passes the command through when

.. math::

    n \bmod (N_f + N_s) < N_f

and commands zero force otherwise. Writing :math:`\boldsymbol{F}` for the commanded per-thruster force, the output is

.. math::

    \boldsymbol{F}_\text{out} = \begin{cases}
    \boldsymbol{F}, & n \bmod (N_f + N_s) < N_f\\
    \boldsymbol{0}, & \text{otherwise}
    \end{cases}

Three properties of this cadence are worth stating explicitly.

**It is free-running.** The counter advances on every update regardless of what is commanded, so the firing
windows sit at a fixed phase rather than being retriggered by the arrival of a request. A new desaturation request
can therefore wait up to :math:`N_s` control periods before its first pulse.

**It is all-or-nothing across the array.** Within one update every thruster is gated identically, so a
desaturation torque is never delivered by a partial subset of the cluster, which would apply a torque in a
direction the mapping stage never solved for.

**The force is passed through, not scaled up.** The average delivered force over a cycle is therefore

.. math::

    \bar{\boldsymbol{F}} = \frac{N_f}{N_f + N_s} \, \boldsymbol{F},

so the duty ratio acts as a gain reduction on the desaturation loop, which the upstream gain must account for
(see `Module Assumptions and Limitations`_).

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
    * - firingPeriods
      - uint32
      - :math:`\ge 1`
      - [-] Number of consecutive control periods, at the start of each cycle, for which the gate passes the
        commanded force through. Zero is rejected because it would hold the thrusters off forever, silently
        disabling desaturation rather than configuring it.
    * - settlingPeriods
      - uint32
      - any value with ``firingPeriods + settlingPeriods`` :math:`\le` ``UINT32_MAX``
      - [-] Number of consecutive control periods for which the gate commands zero force, letting the reaction
        wheels re-stabilize the attitude. Zero is permitted and holds the gate fully open, which is how duty
        cycling is disabled. The only rejected values are those whose sum with ``firingPeriods`` would wrap
        around, since a wrapped cycle length would come out shorter than its own firing window.

Both parameters are counted in **control periods**, not seconds, so the module needs no ``controlPeriod``
parameter and no measured time step: it counts its own invocations. This makes the cadence exact — there is no
rounding of a duration onto a schedule — but it also means the wall-clock length of a cycle is set by the rate at
which the module is scheduled.

User Guide
----------

The module uses two-phase initialization: set the public configuration properties, connect the input message, then
``reset()`` builds and validates the configuration.

.. code-block:: python

    from xmera.fp32 import thrDesatDutyCycleF32

    module = thrDesatDutyCycleF32.ThrDesatDutyCycle()
    module.modelTag = "thrDesatDutyCycle"

    # Phase 1: configuration properties, set before reset()
    module.firingPeriods = 1     # [-] fire for one control period ...
    module.settlingPeriods = 4   # [-] ... then hold off for four, giving a 1-in-5 duty cycle

    # Connect the required input message
    module.thrForceInMsg.subscribeTo(thr_force_in_msg)

    # Phase 2: reset() validates the link and builds the config
    sim.AddModelToTask(task_name, module)

The input message is required; ``reset()`` raises if it is unconnected.

To push edited configuration properties onto a running algorithm without restarting the cadence, call
``reconfigure()``. To restart the cadence at its firing window, call ``reInitialize()``. Both raise
``XmeraLifecycleException`` if called before ``reset()``.

Module Assumptions and Limitations
----------------------------------

- **The cadence is counted in invocations.** The module must actually be scheduled at the intended control rate;
  the wall-clock duty cycle scales with the task period.
- **The cadence is free-running**, so the first pulse of a new request may be delayed by up to ``settlingPeriods``
  control periods.

Module Behaviour Notes
----------------------

- **The upstream gain must be sized for the duty ratio.** Because the force is passed through rather than scaled,
  the average delivered force is :math:`N_f / (N_f + N_s)` of the command. A cadence change therefore rescales the
  effective loop gain of the desaturation controller.
- **The upstream integral term winds up during settling windows.** The gate withholds force while the momentum
  error persists, so an integrating upstream controller keeps accumulating with no effect. ``momentumManagement``'s
  ``integralLimit`` and this module's ``settlingPeriods`` must be tuned together; a long settling window with a
  generous integral limit produces an overshooting pulse when the gate reopens.
- **Suppressed requests are discarded, not banked.** A command withheld during a settling window is not carried
  forward. This is deliberate: the upstream integral term is already the accumulator, and banking the request here
  as well would integrate the same error twice. It does mean the module is only correct downstream of a closed-loop
  command, not of a fixed impulse budget.
