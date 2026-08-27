Executive Summary
-----------------
This module computes a reference orientation for a platform connected to the main hub, on which a thruster is
mounted whose direction is known in platform-frame coordinates. The goal of this module is to compute a reference
orientation for the platform which offsets the thrust direction with respect to the center of mass so the thruster
produces a **requested** torque on the vehicle, read from an input message. When the requested torque is zero, this
reduces to aligning the thruster line of action with the system's center of mass, zeroing the net torque produced by
the thruster on the spacecraft. The module reports the resulting thruster direction in body-frame coordinates; a
downstream module is responsible for computing the platform gimbal angles that realize it.

The requested torque is produced upstream by a control law: :ref:`momentumManagement` derives it from the momentum
accumulated on the reaction wheels, so that the thruster offset dumps that momentum. This module solves only the
pointing problem for whatever torque it is handed.

The thruster is mounted so its line of action runs through the platform joint: it fires along the platform
frame's :math:`-z` axis from a point on that axis. That makes the reference a **closed-form solve** -- no
iteration, no state, and no dependence on the previous cycle.

All numeric computation is single-precision (``float`` / fp32). The module is a single algorithm
(``ThrustVectoringAlgorithm``) with two interface adapters: a ``SysModel`` adapter that connects it to
the Xmera system via messages, and a C shim that connects it to the Adamant system via the C/Ada FFI.

Module Architecture
-------------------
The **algorithm** (``ThrustVectoringAlgorithm``) is framework-free and Eigen-typed; it implements the
mathematics below and holds no runtime state at all. It never sees a message payload: ``update()`` takes the
requested torque, the only quantity that changes from cycle to cycle, and returns a ``ThrustVectoringOutput``
struct. Everything else lives in the validated ``ThrustVectoringConfig``.

The **Xmera adapter** (``ThrustVectoring``) inherits from ``SysModel`` and owns all messaging concerns.
Configuration parameters are exposed as public member variables (two-phase initialization): the caller sets them,
then calls ``reset()``, which validates that the required input messages are connected and builds a validated
``ThrustVectoringConfig`` from the current property values **and from the vehicle and thruster configuration
messages**. Those two describe the spacecraft, not its state, so they are read once at ``reset()`` rather than on
every cycle. ``updateState()`` therefore reads only ``cmdTorqueInMsg``, invokes the algorithm, and packs the
results back into the output payloads. ``reconfigure()`` re-reads both configuration messages and re-pushes the
current properties into the running algorithm -- the way to pick up a changed center of mass. There is no
``reInitialize()``: the algorithm has no runtime state to re-seed.

The **Adamant adapter** is a C shim (``thrustVectoringAlgorithm_c.h`` / ``.cpp``) that exposes the
algorithm through an ``extern "C"`` interface so Adamant components can call it via the C/Ada FFI bindings.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg variable name is set by the
user from python. The msg type contains a link to the message structure definition, while the description provides
information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - Input vehicle configuration message containing the position of the center of mass of the system. **Read
        once at** ``reset()``.
    * - thrusterConfigFInMsg
      - :ref:`THRConfigMsgPayload`
      - Input thruster configuration message containing the thrust direction vector and magnitude in **platform
        frame coordinates**. The entry ``rThrust_B`` here is the position of the thrust application point, with
        respect to the origin of the platform frame, in platform-frame coordinates
        (:math:`{}^\mathcal{F}\boldsymbol{r}_{T/F}`). **Read once at** ``reset()``.
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - Input message containing the torque [Nm] the thruster is requested to produce on the vehicle about the
        system's center of mass, in body-frame coordinates. A zero request points the thruster line of action
        through the center of mass. Typically produced by :ref:`momentumManagement`. This is the only message read
        on every update, and the only argument to the algorithm's ``update()``.
    * - bodyHeadingOutMsg
      - :ref:`BodyHeadingMsgPayload`
      - Output message containing the unit direction vector of the thruster in body-frame coordinates.
    * - thrusterConfigBOutMsg
      - :ref:`THRConfigMsgPayload`
      - Output thruster configuration message containing the thrust direction vector and magnitude in **reference
        body frame coordinates**. The entry ``rThrust_B`` here is the position of the thrust application point,
        with respect to the origin of the body frame, in body-frame coordinates
        (:math:`{}^\mathcal{B}\boldsymbol{r}_{T/B}`).

Mathematical Formulation
------------------------

Frames and mounting
^^^^^^^^^^^^^^^^^^^
The platform pivots about the hub-fixed joint :math:`M`. The mount frame :math:`\mathcal{M}` is defined with its
:math:`-z` axis along the **un-deflected thrust direction**, so ``sigma_MB`` carries the thruster's mounting
orientation on the hub. The platform frame :math:`\mathcal{F}` shares that convention: the thrust acts along
:math:`-z_\mathcal{F}`, applied at the point ``armLength`` behind :math:`M` along that axis.

Resolved into body-frame coordinates, that axis is written

.. math::
    \hat{\boldsymbol{t}}_0 = -[\mathcal{MB}]^T\,\hat{\boldsymbol{z}}_\mathcal{M},

i.e. the negated third row of :math:`[\mathcal{MB}]`. It is the only quantity the mount frame contributes at run
time, and is computed once when the configuration is set. The deflection cone is measured from it, and it settles
the choice between the two solutions the pointing solve admits.

The consequence is the one the whole module rests on: **the line of action runs through** :math:`M` **for every
platform orientation.** A force is a sliding vector, so the application point may be taken at :math:`M` without
changing the torque about any reference point, and the torque about the center of mass :math:`C` is

.. math::
    \boldsymbol{L}_C = \boldsymbol{r}_{M/C} \times F\,\hat{\boldsymbol{t}}.

:math:`\boldsymbol{r}_{M/C}` is hub-fixed. The platform therefore enters the torque **only** through the thrust
direction :math:`\hat{\boldsymbol{t}}` -- not through the application point -- and the reference follows in
closed form. ``armLength`` does not appear above at all: it affects only where the module reports the thruster to
be, never the force or the torque.

Every quantity the solve needs -- the center of mass, the joint position, the requested torque and
:math:`\hat{\boldsymbol{t}}_0` -- is therefore a body-frame quantity, and the relations below are rotation
equivariant, so they hold in whichever frame the inputs are given in. The reference is computed directly in body
coordinates, with nothing rotated per cycle.

Requested torque
^^^^^^^^^^^^^^^^
The requested torque :math:`\boldsymbol{L}_\text{req}` is an input to this module, supplied in body-frame
coordinates by ``cmdTorqueInMsg``, and is the torque the thruster is asked to produce on the vehicle about the
system's center of mass. The module applies no control law of its own; it only solves the pointing problem, and
it does so in the frame the request already arrives in.

Solving for the thrust direction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Write :math:`b = \|\boldsymbol{r}_{M/C}\|` and split the thrust direction into the part perpendicular to
:math:`\boldsymbol{r}_{M/C}` and the part along it,

.. math::
    \hat{\boldsymbol{t}} = \boldsymbol{t}_\perp
        + t_\parallel\,\hat{\boldsymbol{r}}_{M/C}, \qquad
    \boldsymbol{t}_\perp \cdot \hat{\boldsymbol{r}}_{M/C} = 0.

Only :math:`\hat{\boldsymbol{t}}` is a unit vector here; :math:`\boldsymbol{t}_\perp` and :math:`t_\parallel` are
its components.

The part along :math:`\boldsymbol{r}_{M/C}` produces no torque, because
:math:`\boldsymbol{r}_{M/C} \times \hat{\boldsymbol{r}}_{M/C} = \boldsymbol{0}`. Substituting into the torque
relation from *Frames and mounting* therefore leaves the perpendicular part carrying all of it,

.. math::
    \boldsymbol{L}_C = F b \left( \hat{\boldsymbol{r}}_{M/C} \times \boldsymbol{t}_\perp \right),
        \qquad
    \|\boldsymbol{L}_C\| = F b \,\|\boldsymbol{t}_\perp\|.

Because :math:`\hat{\boldsymbol{t}}` is a unit vector, :math:`\|\boldsymbol{t}_\perp\| \leq 1`, so the second
identity says that :math:`\|\boldsymbol{t}_\perp\|` **is the delivered torque as a fraction of the largest this
geometry can deliver**, :math:`F b`. The solve is then three steps.

**Step 1** -- invert the magnitude relation for the requested torque:

.. math::
    \boldsymbol{t}_{\perp,\text{req}} = \left( \frac{\boldsymbol{L}_\text{req}}{F b} \right)
        \times \hat{\boldsymbol{r}}_{M/C}.

The cross product also discards the component of the request along :math:`\boldsymbol{r}_{M/C}`, which no
orientation can produce, so no separate projection is needed.

**Step 2** -- read off the magnitude and limit it to one. A longer one asks for more torque than :math:`F b`, so
clamping saturates at the largest available in the requested direction:

.. math::
    m = \min\left(\left\|\boldsymbol{t}_{\perp,\text{req}}\right\|,\, 1\right), \qquad
    \boldsymbol{t}_\perp = m\,
        \frac{\boldsymbol{t}_{\perp,\text{req}}}{\left\|\boldsymbol{t}_{\perp,\text{req}}\right\|}.

**Step 3** -- complete the direction. The remaining component follows from the unit-length condition, taken from
:math:`m` so that a saturated request gives exactly zero. Both signs deliver the same torque, since this
component produces none, so take the one leaving the thrust nearer its un-deflected direction:

.. math::
    t_\parallel = \pm\sqrt{1 - m^2}, \qquad
    \operatorname{sign}\left(t_\parallel\right)
        = \operatorname{sign}\left(\hat{\boldsymbol{r}}_{M/C} \cdot \hat{\boldsymbol{t}}_0\right).

The two components are then assembled and normalized, which removes the rounding left by combining them:

.. math::
    \hat{\boldsymbol{t}} = \frac{\boldsymbol{t}_\perp + t_\parallel\,\hat{\boldsymbol{r}}_{M/C}}
        {\left\| \boldsymbol{t}_\perp + t_\parallel\,\hat{\boldsymbol{r}}_{M/C} \right\|}.

A zero request gives :math:`\boldsymbol{t}_\perp = \boldsymbol{0}` and
:math:`\hat{\boldsymbol{t}} = \pm\hat{\boldsymbol{r}}_{M/C}`: the line of action through both :math:`M` and the
center of mass, which is the pure alignment case.

Reachable torques
^^^^^^^^^^^^^^^^^
Reading the same relations the other way shows exactly what this geometry can deliver: the achievable torques
form a **disk**, perpendicular to :math:`\boldsymbol{r}_{M/C}` and of radius :math:`F b`. Both of its limits are
enforced by the steps above rather than by anything extra -- the cross product of Step 1 removes what lies off
the disk's plane, and the clamp of :math:`m` in Step 2 removes what lies beyond its rim. The result is therefore the *closest
achievable torque* to the request, its orthogonal projection onto the disk.

Note that :math:`F b` depends on the center-of-mass offset from the joint and **not** on ``armLength``: moving the
thruster along its own line of action changes neither the force nor the torque.

Deflection cone limit
^^^^^^^^^^^^^^^^^^^^^
The solved direction is finally limited so the thrust stays within a cone of half-angle
:math:`\theta_\text{max}` (``thetaMax``) about its un-deflected direction
:math:`\hat{\boldsymbol{t}}_0`. The deflection is
:math:`\delta = \arccos\left(\hat{\boldsymbol{t}} \cdot \hat{\boldsymbol{t}}_0\right)`; when
:math:`\delta \leq \theta_\text{max}` the direction is left alone, and otherwise it is rebuilt on the cone from
its axial and perpendicular parts,

.. math::
    \hat{\boldsymbol{t}}' = \cos\theta_\text{max}\,\hat{\boldsymbol{t}}_0
        + \sin\theta_\text{max}\,\hat{\boldsymbol{e}}_\perp, \qquad
    \hat{\boldsymbol{e}}_\perp = \frac{\hat{\boldsymbol{t}}
        - \hat{\boldsymbol{t}}_0\left(\hat{\boldsymbol{t}} \cdot \hat{\boldsymbol{t}}_0\right)}
        {\left\|\cdot\right\|}.

This keeps the thrust in the plane the two directions span while pulling it back to the cone. When the solved and
un-deflected directions are nearly antiparallel that plane is ill-defined and an arbitrary perpendicular
direction is used. A clamped reference no longer produces the requested torque.

Body-frame outputs
^^^^^^^^^^^^^^^^^^
The solve already works in body-frame coordinates, so the thrust direction is written straight to
``bodyHeadingOutMsg`` and to ``thrusterConfigBOutMsg`` with no rotation, and the magnitude is reported exactly as
configured.

The thruster sits ``armLength`` behind the joint along the thrust, so the application point is

.. math::
    {}^\mathcal{B}\boldsymbol{r}_{T/B} = {}^\mathcal{B}\boldsymbol{r}_{M/B}
        - \ell\,{}^\mathcal{B}\hat{\boldsymbol{t}},

with :math:`\ell` the arm length (written to ``thrusterConfigBOutMsg`` as ``rThrust_B``). This is the only place
``armLength`` is used. The net torque the thruster produces on the system follows from these outputs as
:math:`{}^\mathcal{B}\boldsymbol{r}_{T/C} \times F\,{}^\mathcal{B}\hat{\boldsymbol{t}}`; the module does not
report it.

Module Parameters
-----------------
Configuration parameters are validated when ``reset()`` builds the algorithm configuration; an out-of-range value
raises ``fsw::invalid_argument``.

.. list-table:: Module Parameters
    :widths: 20 15 20 45
    :header-rows: 1

    * - Parameter
      - Default
      - Valid Range
      - Description
    * - ``sigma_MB``
      - [0, 0, 0]
      - finite
      - MRP relative rotation between body-fixed frames :math:`\mathcal{M}` and :math:`\mathcal{B}`. The
        :math:`\mathcal{M}` frame's :math:`-z` axis is the un-deflected thrust direction, so this carries the
        thruster's mounting orientation
    * - ``r_MB_B``
      - [0, 0, 0]
      - finite
      - relative position of point :math:`M` with respect to point :math:`B`, in :math:`\mathcal{B}`-frame
        coordinates
    * - ``armLength``
      - 0
      - :math:`\geq 0`
      - distance [m] from the joint :math:`M` to the thrust application point, along the thrust. Affects only the
        reported application point, never the force or the torque
    * - ``thetaMax``
      - 0
      - :math:`(0, \pi)`
      - half-angle [rad] of the cone limiting the thrust deflection from its neutral direction (mandatory)

The remaining configuration is read from the input messages at ``reset()``: the center of mass ``CoM_B`` must be
finite, and the thrust magnitude ``maxThrust`` must be finite and positive -- a zero thrust leaves no line of
action to point and is rejected.

The other two fields of ``thrusterConfigFInMsg`` are not free parameters but a **mounting contract**, and are
checked rather than used: ``rThrust_B`` must be zero and ``tHatThrust_B`` must be :math:`[0, 0, -1]`, because the
module's whole formulation rests on the line of action running through the joint. A thruster description that
says otherwise -- an offset nozzle, or a canted one -- is rejected at ``reset()`` rather than silently pointed as
if the assumption held.

The un-deflected thrust must point **from the joint towards the center of mass**,
:math:`\hat{\boldsymbol{r}}_{M/C} \cdot \hat{\boldsymbol{t}}_0 < 0`, and a mounting that does not is rejected at
``reset()``. The thruster sits ``armLength`` behind the joint along the thrust, so this is what places the
thruster *outboard* of the joint; the opposite mounting would put it between the joint and the center of mass,
i.e. inside the vehicle. A commanded deflection keeps it there as long as the cone does not reach a right angle.

The center of mass must also sit farther than ``kMinR_CM`` (:math:`10^{-3}` m) from the platform joint
:math:`M`, i.e. :math:`\|\boldsymbol{r}_{M/C}\| > 10^{-3}`. The reference rotation turns the center of mass
about :math:`M`, so a center of mass on the joint leaves the pointing undefined however the platform is oriented
(see *Thruster pointing*). Rejecting it at ``reset()`` keeps that degenerate geometry out of the algorithm
entirely rather than leaving it to produce a meaningless reference every cycle.

User Guide
----------
The module uses two-phase initialization: set the public configuration properties, connect the input messages,
then add the module to the simulation task (``reset()`` validates and builds the configuration). The vehicle and
thruster configuration messages must already hold their final values when ``reset()`` runs, because that is when
they are read::

    platformReference = thrustVectoringF32.ThrustVectoring()
    platformReference.modelTag = "platformReference"
    platformReference.sigma_MB = sigma_MB
    platformReference.r_MB_B = r_MB_B
    platformReference.armLength = armLength
    platformReference.thetaMax = thetaMax

    platformReference.vehConfigInMsg.subscribeTo(vehConfigMsg)
    platformReference.thrusterConfigFInMsg.subscribeTo(thrConfigFMsg)
    platformReference.cmdTorqueInMsg.subscribeTo(cmdTorqueMsg)

    scSim.AddModelToTask(simTaskName, platformReference)

If the center of mass or the thruster configuration does change later in the mission, call ``reconfigure()`` to
re-read both messages and rebuild the configuration; the runtime state is preserved.

Module Assumptions and Limitations
----------------------------------
**Assumption.** The thrust line of action goes through the platform joint :math:`M`. The thruster fires along
:math:`-z_\mathcal{F}` from a point on that axis. This is a statement about the hardware. The module cannot
examine the thruster, thus it cannot identify a nozzle with an offset or a different direction. For such a
thruster the reference orientation is incorrect. ``reset()`` can compare only the thruster description in the
input message with this assumption (see *Module Parameters*).

**Assumption.** The vehicle configuration and the thruster configuration do not change while the module runs.
The module reads both messages one time, at ``reset()``.

**Limitation.** The torques that this geometry can give are a disk. The disk is at a right angle to
:math:`\boldsymbol{r}_{M/C}` and has a radius of :math:`F\|\boldsymbol{r}_{M/C}\|` (see *Reachable torques*).
Thus the module can give no torque about the direction of the center-of-mass offset. That offset also sets the
largest torque, and not the length of the platform arm.

**Limitation.** The module cannot always give the requested torque. For a request outside the disk, it gives the
nearest torque on the disk. For a solution that moves the thrust more than :math:`\theta_\text{max}` from the
neutral direction, it keeps the thrust on the cone and the thruster gives a different torque.

**Limitation.** The module does not follow a center of mass that moves as the propellant decreases. The error in
the pointing increases with the movement, and the available torque decreases. Call ``reconfigure()`` when a new
estimate is available.
