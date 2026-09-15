Executive Summary
-----------------
This module calculates the direction of a thruster that gives a requested torque on the vehicle. The thrust has a
constant magnitude, and its line of action goes through a fixed point :math:`M` on the hub. The direction is the
only freedom that is left, and the module calculates it in closed form.

A zero request aligns the line of action with the center of mass, which gives no torque. The module gives the
thrust direction in body-frame coordinates. A downstream module calculates the gimbal angles that turn the
thruster to that direction.

:ref:`momentumManagement` usually gives the requested torque. That module calculates the torque from the momentum
on the reaction wheels, so that the thrust removes that momentum. This module calculates only the direction for
the torque that it receives.

All numeric calculation is single-precision (``float`` / fp32). The module has one algorithm
(``ThrustVectoringAlgorithm``) and two interface adapters. A ``SysModel`` adapter connects it to the Xmera system
with messages. A C shim connects it to the Adamant system through the C/Ada FFI.

Module Architecture
-------------------
The **algorithm** (``ThrustVectoringAlgorithm``) is framework-free and Eigen-typed. It contains the mathematics
below and keeps no cycle-to-cycle state. It never receives a message payload. Its ``update()`` takes the requested
torque, the only quantity that changes each cycle, and returns the thrust unit direction. The geometry comes from
the ``ThrustVectoringConfig``, which ``create()`` examines before it accepts the values.

The algorithm keeps two quantities that it calculates from the configuration: the unit moment arm and the largest
torque that the geometry can give. Both stay constant while the module runs, so ``setConfig()`` calculates them
one time.

The **Xmera adapter** (``ThrustVectoring``) inherits from ``SysModel`` and holds all messaging. Configuration
parameters are public member variables (two-phase initialization). The caller sets them and then calls
``reset()``. That method makes sure that the necessary input messages are connected, and it builds the
configuration from the current property values **and from the vehicle and thruster configuration messages**. These
two messages give the properties of the spacecraft, not its state, so the adapter reads them one time at
``reset()``. Each cycle, ``updateState()`` reads only ``cmdTorqueInMsg``, calls the algorithm, and writes the
results to the output payloads. ``reconfigure()`` reads the two configuration messages again and sends the current
properties to the running algorithm. This is how a new center of mass reaches the module. There is no
``reInitialize()``, because the algorithm keeps no state to start again.

The **Adamant adapter** is a C shim (``thrustVectoringAlgorithm_c.h`` / ``.cpp``). It gives the algorithm an
``extern "C"`` interface, so that Adamant components can call it through the C/Ada FFI bindings.

Message Connection Descriptions
-------------------------------
The following table gives all the module input and output messages. The user sets the module message variable name
from Python. The message type contains a link to the message structure definition, and the description tells what
the message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgF32Payload`
      - Input vehicle configuration message that contains the position of the center of mass of the system. **Read
        one time at** ``reset()``.
    * - thrusterConfigInMsg
      - :ref:`THRConfigMsgF32Payload`
      - Input thruster configuration message. The module takes only ``maxThrust`` from it, because the thrust
        magnitude is the one property of the thruster that the solve uses. The module does not read the other
        fields, whose frame is a property of the mechanism. **Read one time at** ``reset()``.
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Input message that contains the torque [Nm] that the thruster must give to the vehicle about the center of
        mass of the system, in body-frame coordinates. A zero request aligns the thrust line of action with the
        center of mass. :ref:`momentumManagement` usually writes this message. It is the only message that the
        module reads each cycle, and the only argument to the ``update()`` of the algorithm.
    * - bodyHeadingOutMsg
      - :ref:`BodyHeadingMsgF32Payload`
      - Output message that contains the unit direction vector of the thruster, in body-frame coordinates.
    * - thrusterConfigOutMsg
      - :ref:`THRConfigMsgF32Payload`
      - Output thruster configuration message that contains the thrust direction vector and the magnitude, in
        **body frame coordinates**. The entry ``rThrust_B`` here is the position of the thrust application point
        with respect to the origin of the body frame, in body-frame coordinates
        (:math:`{}^\mathcal{B}\boldsymbol{r}_{T/B}`).

Module Parameters
-----------------
``reset()`` examines the configuration parameters when it builds the algorithm configuration. A value that is out
of range causes an ``fsw::invalid_argument``.

.. list-table:: Module Parameters
    :widths: 20 15 20 45
    :header-rows: 1

    * - Parameter
      - Default
      - Valid Range
      - Description
    * - ``r_MB_B``
      - [0, 0, 0]
      - all components finite
      - position of the point :math:`M` with respect to the point :math:`B`, in :math:`\mathcal{B}`-frame
        coordinates. :math:`M` is the point that the thrust line of action goes through
    * - ``armLength``
      - 0
      - :math:`\geq 0`
      - distance [m] from :math:`M` to the thrust application point, against the thrust. The thruster is thus
        behind :math:`M`, and the exhaust goes away from the vehicle. The value changes only the application
        point in the output message, never the force or the torque

The module reads the remainder of the configuration from the input messages at ``reset()``. The center of mass
``CoM_B`` must be finite. The thrust magnitude ``maxThrust`` must be finite and more than zero, because a zero
thrust gives no torque about any point.

The center of mass must also be farther than ``kMinR_CM`` (:math:`10^{-3}` m) from :math:`M`, thus
:math:`\|\boldsymbol{r}_{M/C}\| > 10^{-3}`. A center of mass on :math:`M` gives no moment arm and no direction for
the thrust. The difference :math:`\boldsymbol{r}_{M/B} - \boldsymbol{r}_{C/B}` must also be finite. Two positions
that are each finite can be far enough apart that their difference is not.

The product :math:`F\|\boldsymbol{r}_{M/C}\|` must be finite and more than zero. This is the largest torque that
the geometry can give, and the solution divides by it. The thrust and the moment arm can each stay in range while
their product becomes zero or infinite in single precision.

The module does not examine ``rThrust_B`` and ``tHatThrust_B`` of ``thrusterConfigInMsg``. Those two fields give
the thruster in a frame of the mechanism, and this module has no such frame. The mathematics below holds for a
thrust line of action that goes through :math:`M`, which is a statement about the hardware (see *Module
Assumptions and Limitations*).

Mathematical Formulation
------------------------

Thrust point and moment arm
^^^^^^^^^^^^^^^^^^^^^^^^^^^
A mechanism turns the thruster about the hub-fixed point :math:`M`. The thruster stays on the thrust line at the
distance ``armLength`` from :math:`M`, thus the line of action goes through :math:`M` for every direction that the
mechanism can give.

A force is a sliding vector, so the application point can move to :math:`M` without a change to the torque about
any reference point. The torque about the center of mass :math:`C` is

.. math::
    \boldsymbol{L}_C = \boldsymbol{r}_{M/C} \times F\,\hat{\boldsymbol{t}}.

:math:`\boldsymbol{r}_{M/C}` is hub-fixed. The mechanism therefore changes the torque **only** through the thrust
direction :math:`\hat{\boldsymbol{t}}`, and not through the application point. ``armLength`` does not occur in the
equation above. It changes only the application point that the module writes to its output message.

The module thus needs no description of the mechanism. The center of mass, the point :math:`M` and the requested
torque are all body-frame quantities, and the module calculates the direction directly in body-frame coordinates.
It uses no other frame.

Requested torque
^^^^^^^^^^^^^^^^
``cmdTorqueInMsg`` gives the requested torque :math:`\boldsymbol{L}_\text{req}` in body-frame coordinates. It is
the torque that the thruster must give to the vehicle about the center of mass of the system. The module uses no
control law of its own. It calculates only the direction, and it does this in the frame that the request arrives
in.

Calculating the thrust direction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Let :math:`b = \|\boldsymbol{r}_{M/C}\|`. Divide the thrust direction into the part at a right angle to
:math:`\boldsymbol{r}_{M/C}` and the part along it,

.. math::
    \hat{\boldsymbol{t}} = \boldsymbol{t}_\perp
        + t_\parallel\,\hat{\boldsymbol{r}}_{M/C}, \qquad
    \boldsymbol{t}_\perp \cdot \hat{\boldsymbol{r}}_{M/C} = 0.

Only :math:`\hat{\boldsymbol{t}}` is a unit vector here. :math:`\boldsymbol{t}_\perp` and :math:`t_\parallel` are
its components.

The part along :math:`\boldsymbol{r}_{M/C}` gives no torque, because
:math:`\boldsymbol{r}_{M/C} \times \hat{\boldsymbol{r}}_{M/C} = \boldsymbol{0}`. The perpendicular part therefore
carries all of the torque,

.. math::
    \boldsymbol{L}_C = F b \left( \hat{\boldsymbol{r}}_{M/C} \times \boldsymbol{t}_\perp \right),
        \qquad
    \|\boldsymbol{L}_C\| = F b \,\|\boldsymbol{t}_\perp\|.

:math:`\hat{\boldsymbol{t}}` is a unit vector, thus :math:`\|\boldsymbol{t}_\perp\| \leq 1`. The second identity
then shows that :math:`\|\boldsymbol{t}_\perp\|` **is the torque as a fraction of the largest torque that this
geometry can give**, :math:`F b`. The calculation has three steps.

**Step 1** -- turn the request into a perpendicular component:

.. math::
    \boldsymbol{t}_{\perp,\text{req}} = \boldsymbol{L}_\text{req} \times \hat{\boldsymbol{r}}_{M/C}.

The cross product also removes the component of the request along :math:`\boldsymbol{r}_{M/C}`, which no direction
can give. No separate step is necessary for it.

**Step 2** -- take the length as a fraction of :math:`F b`, and limit it to one. A larger fraction asks for more
torque than the geometry can give, so the limit saturates at the largest torque in the requested direction:

.. math::
    m = \min\left(\frac{\left\|\boldsymbol{t}_{\perp,\text{req}}\right\|}{F b},\, 1\right), \qquad
    \boldsymbol{t}_\perp = m\,
        \frac{\boldsymbol{t}_{\perp,\text{req}}}{\left\|\boldsymbol{t}_{\perp,\text{req}}\right\|}.

**Step 3** -- complete the direction. The unit-length condition gives the remaining component from :math:`m`, so
that a saturated request gives exactly zero. Both signs give the same torque, because this component gives none.
The module takes the sign that fires the thrust from :math:`M` towards the center of mass:

.. math::
    t_\parallel = -\sqrt{1 - m^2}.

This keeps the thrust on the vehicle from the outside. The opposite sign would put the thruster between :math:`M`
and the center of mass, thus inside the vehicle.

The two components are at a right angle and their lengths square to one, so their sum is already a unit vector.
The module normalizes it to remove the rounding that the two components carry:

.. math::
    \hat{\boldsymbol{t}} = \frac{\boldsymbol{t}_\perp + t_\parallel\,\hat{\boldsymbol{r}}_{M/C}}
        {\left\| \boldsymbol{t}_\perp + t_\parallel\,\hat{\boldsymbol{r}}_{M/C} \right\|}.

A zero request gives :math:`\boldsymbol{t}_\perp = \boldsymbol{0}` and
:math:`\hat{\boldsymbol{t}} = -\hat{\boldsymbol{r}}_{M/C}`. The line of action then goes through :math:`M` and the
center of mass, which is the alignment condition.

Reachable torques
^^^^^^^^^^^^^^^^^
The same relations read in the other direction show what this geometry can give. The torques that are available
make a **disk** at a right angle to :math:`\boldsymbol{r}_{M/C}`, with a radius of :math:`F b`. The steps above
hold both of its limits, and nothing more is necessary. The cross product of Step 1 removes what is off the plane
of the disk, and the limit on :math:`m` in Step 2 removes what is outside its edge. The result is therefore the
*nearest available torque* to the request.

:math:`F b` changes with the center-of-mass offset from :math:`M`, and **not** with ``armLength``. A movement of
the thruster along its own line of action changes neither the force nor the torque.

Body-frame outputs
^^^^^^^^^^^^^^^^^^
The calculation already uses body-frame coordinates. The module therefore writes the thrust direction directly to
``bodyHeadingOutMsg`` and to ``thrusterConfigOutMsg``, with no rotation, and gives the magnitude as configured.

The thruster sits ``armLength`` behind :math:`M` along the thrust, thus the application point is

.. math::
    {}^\mathcal{B}\boldsymbol{r}_{T/B} = {}^\mathcal{B}\boldsymbol{r}_{M/B}
        - \ell\,{}^\mathcal{B}\hat{\boldsymbol{t}},

with :math:`\ell` the arm length. The module writes it to ``thrusterConfigOutMsg`` as ``rThrust_B``. This is the
only use of ``armLength``. The torque that the thruster gives to the system follows from these outputs as
:math:`{}^\mathcal{B}\boldsymbol{r}_{T/C} \times F\,{}^\mathcal{B}\hat{\boldsymbol{t}}`. The module does not give
it.

User Guide
----------
The module uses two-phase initialization. Set the public configuration properties, connect the input messages, and
then add the module to the simulation task. ``reset()`` examines the values and builds the configuration. The
vehicle and thruster configuration messages must already hold their final values when ``reset()`` runs, because
that is when the module reads them::

    thrustVectoring = thrustVectoringF32.ThrustVectoring()
    thrustVectoring.modelTag = "thrustVectoring"
    thrustVectoring.r_MB_B = r_MB_B
    thrustVectoring.armLength = armLength

    thrustVectoring.vehConfigInMsg.subscribeTo(vehConfigMsg)
    thrustVectoring.thrusterConfigInMsg.subscribeTo(thrConfigMsg)
    thrustVectoring.cmdTorqueInMsg.subscribeTo(cmdTorqueMsg)

    scSim.AddModelToTask(simTaskName, thrustVectoring)

If the center of mass or the thruster configuration changes later in the mission, call ``reconfigure()``. It reads
both messages again and builds the configuration again.

Module Assumptions and Limitations
----------------------------------
**Assumption.** The thrust line of action goes through the point :math:`M`. This is a statement about the
hardware, and the module cannot examine the thruster. A nozzle with an offset, or one that is canted, breaks the
assumption, and the direction is then incorrect. The module gives no warning for such a thruster. The mechanism
that turns the thruster must keep the line of action through :math:`M`.

**Assumption.** The vehicle configuration and the thruster configuration do not change while the module runs. The
module reads both messages one time, at ``reset()``.

**Assumption.** The requested torque is a number. The module does not examine ``cmdTorqueInMsg``, thus a value
that is not a number goes through to the two output messages. The module that writes the request must give a
value that is a number.

**Limitation.** The torques that this geometry can give are a disk. The disk is at a right angle to
:math:`\boldsymbol{r}_{M/C}` and has a radius of :math:`F\|\boldsymbol{r}_{M/C}\|` (see *Reachable torques*). Thus
the module can give no torque about the direction of the center-of-mass offset. That offset also sets the largest
torque, and not the length of the arm from :math:`M` to the thruster.

**Limitation.** The module cannot always give the requested torque. For a request outside the disk, it gives the
nearest torque on the disk.

**Limitation.** The module applies no travel limit to the direction that it gives. A large request can move the
thrust far from the neutral direction of the mechanism. A downstream module must hold the deflection inside the
range of the mechanism.

**Limitation.** The module does not follow a center of mass that moves as the propellant decreases. The error in
the direction increases with the movement, and the available torque decreases. Call ``reconfigure()`` when a new
estimate is available.
