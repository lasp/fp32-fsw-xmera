.. raw:: latex

    {\LARGE \textbf{stepperMotorController}}

Executive Summary
-----------------
This module drives a stepper motor toward a reference angle by issuing integer step commands. Internally the controller
runs a small state machine (OFF / IDLE / MOVING / STOPPING / SETTLING) that decides when to command a new move, when a
changed reference should interrupt the current move, and how long to wait after a stop before accepting new commands.
The controller always commands the shortest path around a full revolution, so a reference on the opposite side of the
motor is reached via the smaller of the two directions. The current motor position and motion status are read each tick
from a stepper motor dynamics module via the ``stepperMotorInMsg`` input. All quantities are computed in single
precision (float angles, integer step counts).

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. Message connections are set by the user from
Python.

.. list-table:: Module I/O Messages
    :widths: 30 30 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - motorRefAngleInMsg
      - :ref:`MotorAngleRefMsgF32Payload`
      - Reference motor angle input message (uses ``theta``, [rad])
    * - stepperMotorInMsg
      - :ref:`StepperMotorMsgPayload`
      - Stepper motor dynamics feedback (uses ``motorPosition`` and ``isMotorMoving``)
    * - motorStepCommandOutMsg
      - :ref:`MotorStepCommandMsgPayload`
      - Commanded motor steps output message. Written on a ``MOVE`` (sets ``stepsCommanded``,
        ``stopMotorCommand=false``) or a ``STOP`` (sets ``stepsCommanded=0``, ``stopMotorCommand=true``).

Algorithm Parameters
-------------------------------
The following table lists the configuration parameters. They are held in the validated immutable
``StepperMotorControllerConfig`` (built and validated at ``reset()``); the Xmera adapter exposes them as same-named
public properties that are set before ``reset()``. The integer steps-per-revolution and the full-circle flag are
*derived* from ``stepAngle`` and the angle range by the algorithm — they are not configuration inputs.

.. list-table:: Algorithm Parameters
    :widths: 32 15 10 10 40 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - stepAngle (required)
      - float
      - [rad/step]
      - 0 (must be set)
      - Angle traversed per motor step
      - Must be in :math:`[2\pi / 100000,\, 2\pi]` (validated at reset(); the lower bound caps
        ``stepsPerRev`` at 100k so the fp32 round-trip in ``angleToSteps`` stays within rounding
        tolerance)
    * - minAngle
      - float
      - [rad]
      - 0
      - Lower bound of the motor's travel range; reference angles below this are rejected
      - Must be in :math:`[-2\pi,\, 2\pi]` and strictly less than ``maxAngle`` (validated at reset())
    * - maxAngle
      - float
      - [rad]
      - :math:`2\pi`
      - Upper bound of the motor's travel range; reference angles above this are rejected
      - Must be in :math:`[-2\pi,\, 2\pi]` and strictly greater than ``minAngle`` (validated at reset())
    * - settleCountMax
      - uint32_t
      - [ticks]
      - 10
      - Number of control ticks to remain in ``SETTLING`` before returning to ``IDLE``
      - Any value accepted (no constraint beyond the uint32_t type)
    * - minStepCommand
      - uint32_t
      - [steps]
      - 1
      - Minimum step delta magnitude that triggers a ``MOVE`` from ``IDLE`` or a
        ``STOP``-and-replan from ``MOVING``. Step deltas with magnitude below this value are
        treated as too small to act on.
      - Must be greater than 0 (validated at reset())

Module Architecture
-------------------------------
The module is split into a pure algorithm (``StepperMotorControllerAlgorithm``, free of any messaging or framework
dependencies) and an Xmera adapter (``StepperMotorController``, a ``SysModel``) that owns the message I/O. The adapter
follows a two-phase initialization pattern: the public configuration properties (``stepAngle``, ``minAngle``,
``maxAngle``, ``settleCountMax``, ``minStepCommand``) and the input messages are set first, then ``reset()`` builds and
validates the immutable ``StepperMotorControllerConfig`` and constructs the algorithm. An invalid parameter throws
``fsw::invalid_argument`` at ``reset()``. The adapter exposes no parameters of its own; the motor's absolute position is
read each tick from ``stepperMotorInMsg.motorPosition``, so it needs no initial-angle configuration. A C shim
(``stepperMotorControllerAlgorithm_c.h``) exposes the algorithm to Ada via ``extern "C"`` bindings.

Algorithm Input/Output
-------------------------------
The following tables list the inputs and outputs of the pure algorithm ``update()`` method, independent of the Xmera
messaging layer.

.. list-table:: Algorithm Inputs
    :widths: 25 20 10 45
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Description
    * - currentPosition
      - int
      - [steps]
      - Current motor step position, tracked by the caller
    * - referenceAngle
      - float
      - [rad]
      - Reference motor angle; converted to steps internally via ``angleToSteps()``
    * - isMotorMoving
      - bool
      - [-]
      - Indicates whether the motor is currently moving (used to gate the ``STOPPING`` → ``SETTLING`` transition)

.. list-table:: Algorithm Outputs (StepperMotorControllerOutput)
    :widths: 25 35 10 40
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Description
    * - commandType
      - StepperMotorCommandType
      - [-]
      - ``NONE``, ``MOVE``, or ``STOP``
    * - stepsToMove
      - int
      - [steps]
      - Wrapped (shortest-path) step delta; valid only when ``commandType == MOVE``

Algorithm Description
---------------------
The stepper motor controller operates in integer step counts. Angles are converted to steps using the motor's fixed
step size,

.. math::

    n = \text{round}\!\left( \frac{\theta}{\Delta\theta} \right)

where :math:`\theta` is an angle in radians and :math:`\Delta\theta` is ``stepAngle`` (the angle per motor step).
A derived integer :math:`N = \text{round}(2\pi / \Delta\theta)` is cached internally for the wrap-around math; the
controller never reasons in radians otherwise: both the current and desired positions are integer step counts.

Motor Angle Range
^^^^^^^^^^^^^^^^^
The motor's mechanical travel is bounded by ``[minAngle, maxAngle]``, set together via
``setMotorAngleRange`` (defaults: :math:`[0,\, 2\pi]`). For a **partial range**, any reference angle
outside ``[minAngle, maxAngle]`` is rejected: the controller leaves its internal ``desiredPosition``
unchanged for that tick and the state machine emits no ``MOVE`` command. This keeps the motor
quiescent on out-of-range commands rather than driving it across the forbidden seam.

For a **full-circle range** (``maxAngle - minAngle`` within ``kMinStepAngle`` (:math:`2\pi / 100000`) of :math:`2\pi`), every
reference angle is accepted regardless of sign or magnitude — the existing shortest-path wrap
math reduces it to an equivalent step delta, so e.g. :math:`-30^\circ` and :math:`+330^\circ`
command the same motion.

Step-Delta Selection
^^^^^^^^^^^^^^^^^^^^
Define :math:`\Delta = n_d - n_c` (linear delta in step counts). The controller computes the
commanded step delta either as a shortest-path wrap or as the linear value, depending on the
configured range:

- **Full circle** (``maxAngle - minAngle`` within ``kMinStepAngle`` (:math:`2\pi / 100000`) of :math:`2\pi`): the delta is
  wrapped into the interval :math:`[-\tfrac{N}{2},\, +\tfrac{N}{2}]` via

  .. math::

      \text{wrap}(\Delta) = \begin{cases}
          \Delta - N & \text{if } \Delta > \tfrac{N}{2} \\
          \Delta + N & \text{if } \Delta < -\tfrac{N}{2} \\
          \Delta & \text{otherwise}
      \end{cases}

  applied after :math:`\Delta \bmod N`. A motor at -170° asked to move to +170° commands a 20°
  forward step, not 340° backward.

- **Partial range**: the delta is used unchanged (``stepDelta = Δ``). Wrapping across the
  out-of-range region is unsafe because the motor cannot physically traverse the forbidden seam,
  so the controller takes the linear path within the bounded travel even when it is the longer
  path on the conceptual circle.

State Machine
^^^^^^^^^^^^^
Let :math:`n_c` be the caller-supplied ``currentPosition``, :math:`n_d` the desired position (derived from the reference
angle), and :math:`n_m` the position most recently commanded (stored internally). Let :math:`\tau` be the
``minStepCommand``.

- ``OFF`` — no transitions; the algorithm stays in this state until the caller reassigns it (used to hold the motor
  quiescent).

- ``IDLE`` — the motor is quiescent. If
  :math:`\left| \text{stepDelta}(n_d - n_c) \right| \ge \tau`,
  the algorithm emits a ``MOVE`` with ``stepsToMove = stepDelta(n_d - n_c)``, stores :math:`n_m := n_d`, and transitions to
  ``MOVING``.

- ``MOVING`` — the motor is executing a commanded move. Two conditions are checked each tick, in priority order:

  1. *Reference changed.* If :math:`\left| \text{stepDelta}(n_m - n_d) \right| \ge \tau`, the algorithm emits a ``STOP``
     command and transitions to ``STOPPING``. The Xmera adapter forwards the ``STOP`` to the motor dynamics
     (``stopMotorCommand=true``); because a stepper motor cannot stop mid-step, the motor finishes its current step
     before halting. The algorithm waits in ``STOPPING`` for the motor to report ``isMotorMoving == false``, then
     re-plans from the resulting final position.
  2. *Move complete.* Otherwise, if the caller reports ``isMotorMoving == false``, the algorithm transitions directly
     to ``SETTLING`` and resets the settle counter (skipping ``STOPPING``, since no ``STOP`` command needs to be
     issued — the motor has already come to rest at the commanded target).

- ``STOPPING`` — entered only after a ``STOP`` command was emitted (mid-move interrupt). The algorithm transitions to
  ``SETTLING`` and resets the settle counter as soon as the caller reports ``isMotorMoving == false``.

- ``SETTLING`` — a counter runs for up to ``settleCountMax`` ticks. Once the counter reaches the limit the algorithm
  returns to ``IDLE`` and is ready to issue the next move.

``minStepCommand`` is the smallest step delta magnitude the controller will act on. From ``IDLE`` it gates whether a
new ``MOVE`` is issued (small position errors are left alone — tuned to the motor's step-level resolution and
positioning repeatability). From ``MOVING`` the same threshold gates whether a changed reference is large enough to
interrupt the in-progress move (tuned to avoid interrupting on noise in the reference signal). Move completion in
``MOVING`` is detected via the caller's ``isMotorMoving`` signal rather than this threshold.

Position Tracking via Feedback (Xmera Adapter)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The adapter reads the motor's absolute step position directly from ``stepperMotorInMsg.motorPosition`` each tick
(:math:`n_c = \text{motorPosition}_{\text{feedback}}`). The dynamics module maintains this as a cumulative net signed
step count from the angular zero (``motorPosition * stepAngle`` ≈ :math:`\theta`) and never resets it across commands,
so the adapter does not need to track command-start baselines or initialize a position. ``isMotorMoving`` is read
directly from the same message.

Algorithm Assumptions and Limitations
-------------------------------------
- Motor motion is assumed to be quantized at the step level; reference angles that fall between steps are rounded to
  the nearest step.
- The caller is responsible for tracking the motor's current position and passing it to ``update()`` each tick.
- When a ``STOP`` is issued mid-move, the Xmera adapter forwards the stop to the motor; the motor completes its
  current step before halting, and the algorithm re-plans from the resulting final position.
- The ``OFF`` state is entered only via caller intervention; there is no transition into ``OFF`` from within the
  state machine.
- The motor's travel range ``[minAngle, maxAngle]`` must satisfy :math:`-2\pi \le \text{minAngle} < \text{maxAngle} \le 2\pi`.
  References outside this range produce no movement; the controller does not clamp them. Shortest-path
  wrap-around is enabled only when the configured range is the full circle (within ``kMinStepAngle``
  (:math:`2\pi / 100000` rad) of :math:`2\pi` span).

Module Description (Xmera Usage)
--------------------------------
The ``StepperMotorController`` Xmera adapter provides the simulation integration layer for the algorithm. It:

- Reads ``motorRefAngleInMsg`` to obtain the reference angle each tick.
- Reads ``stepperMotorInMsg`` to obtain the motor's absolute ``motorPosition`` and ``isMotorMoving`` each tick.
- On ``reset()``, builds and validates the immutable configuration from the public properties, constructs the
  algorithm, requires both input messages to be linked, and re-initializes the algorithm's state machine.
- On each ``updateState()``, calls the algorithm with the feedback ``motorPosition``, the reference angle, and the
  feedback ``isMotorMoving``.
- On a ``MOVE`` output, writes ``motorStepCommandOutMsg`` with the commanded step delta and ``stopMotorCommand=false``.
- On a ``STOP`` output, writes ``motorStepCommandOutMsg`` with ``stopMotorCommand=true`` and ``stepsCommanded=0``; the
  motor halts after finishing its current step. The algorithm transitions to ``STOPPING`` and waits for
  ``isMotorMoving == false``.

User Guide
----------
Typical usage in Python is (the configuration properties and message connections must be set before ``reset()``)::

    module = stepperMotorControllerF32.StepperMotorController()
    module.modelTag = "stepperMotorController"
    module.stepAngle = math.radians(1.0)             # [rad/step] (1 deg/step = 360 steps/rev)
    module.minAngle = 0.0                            # full circle (default); use a partial range for bounded steppers
    module.maxAngle = 2 * math.pi
    module.settleCountMax = 2
    module.minStepCommand = 1

    module.motorRefAngleInMsg.subscribeTo(ref_msg)
    module.stepperMotorInMsg.subscribeTo(stepper_motor.stepperMotorOutMsg)

The commanded step delta is available on ``motorStepCommandOutMsg`` each time a new ``MOVE`` is issued.

``minStepCommand`` is the smallest step delta magnitude the controller will command. From ``IDLE`` it gates whether a
new ``MOVE`` is issued for an incoming reference. From ``MOVING`` it separately gates whether a changed reference is
large enough — relative to the currently-commanded target — to interrupt the in-progress move. (Move completion in
``MOVING`` is detected via the caller's ``isMotorMoving`` signal, not via this threshold.)
