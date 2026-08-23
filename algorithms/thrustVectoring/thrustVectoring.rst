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

All numeric computation is single-precision (``float`` / fp32). The module is a single algorithm
(``ThrustVectoringAlgorithm``) with two interface adapters: a ``SysModel`` adapter that connects it to
the Xmera system via messages, and a C shim that connects it to the Adamant system via the C/Ada FFI.

Module Architecture
-------------------
The **algorithm** (``ThrustVectoringAlgorithm``) is framework-free and Eigen-typed; it implements the
mathematics below and holds the previous cycle's reference pointing, used to convert the requested torque into the
platform frame. It never sees a message payload: its inputs and outputs are the ``ThrustVectoringInputs`` and
``ThrustVectoringOutput`` structs. Two interface adapters wrap it.

The **Xmera adapter** (``ThrustVectoring``) inherits from ``SysModel`` and owns all messaging concerns.
Configuration parameters are exposed as public member variables (two-phase initialization): the caller sets them,
then calls ``reset()``, which validates that the required input messages are connected and builds a validated
``ThrustVectoringConfig`` from the current property values.
``updateState()`` reads the input messages, converts the payload ``float[3]`` arrays to Eigen types via
``eigenSupport.h``, invokes the algorithm, and packs the results back into the output payloads. ``reconfigure()``
re-pushes the current properties into the running algorithm without disturbing its runtime state, and
``reInitialize()`` re-seeds that runtime state.

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
      - Input vehicle configuration message containing the position of the center of mass of the system.
    * - thrusterConfigFInMsg
      - :ref:`THRConfigMsgPayload`
      - Input thruster configuration message containing the thrust direction vector and magnitude in **platform
        frame coordinates**. The entry ``rThrust_B`` here is the position of the thrust application point, with
        respect to the origin of the platform frame, in platform-frame coordinates
        (:math:`{}^\mathcal{F}\boldsymbol{r}_{T/F}`).
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - Input message containing the torque [Nm] the thruster is requested to produce on the vehicle about the
        system's center of mass, in body-frame coordinates. A zero request points the thruster line of action
        through the center of mass. Typically produced by :ref:`momentumManagement`.
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
A detailed mathematical derivation of the equations implemented in this module can be found in
`R. Calaon, L. Kiner, C. Allard and H. Schaub, "Momentum Management of a Spacecraft equipped with a Dual-Gimballed
Electric Thruster" <http://hanspeterschaub.info/Papers/Calaon2023a.pdf>`__.

The algorithm computes a direction cosine matrix :math:`[\mathcal{FM}]` that describes the reference rotation
between the platform frame :math:`\mathcal{F}` and the mount frame :math:`\mathcal{M}`, chosen so that the thruster
produces a requested torque about the system's center of mass (a zero requested torque points the thruster line of
action through the center of mass). The input parameters allow specifying offsets between the origin :math:`M` of the
hub-fixed mount frame :math:`\mathcal{M}` and the origin :math:`F` of the platform-fixed frame :math:`\mathcal{F}`,
the application point of the thruster force in the :math:`\mathcal{F}` frame, and the direction, in
:math:`\mathcal{F}`-frame coordinates, of the thrust vector.

Platform reference rotation
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The reference DCM :math:`[\mathcal{FM}]` is the single rotation that makes the thruster produce the requested torque
about the center of mass (derived below). The relevant geometry is first assembled in the mount and platform frames:

.. math::
    {}^\mathcal{M}\boldsymbol{r}_{C/M} = [\mathcal{MB}]\left( {}^\mathcal{B}\boldsymbol{r}_{C/B}
        - {}^\mathcal{B}\boldsymbol{r}_{M/B} \right), \qquad
    {}^\mathcal{F}\boldsymbol{r}_{T/M} = {}^\mathcal{F}\boldsymbol{r}_{F/M} + {}^\mathcal{F}\boldsymbol{r}_{T/F},
        \qquad
    {}^\mathcal{F}\boldsymbol{t} = F\,{}^\mathcal{F}\hat{\boldsymbol{t}},

with :math:`[\mathcal{MB}]` built from ``sigma_MB`` and :math:`F` the thrust magnitude.

Thruster pointing
^^^^^^^^^^^^^^^^^
The thruster is fixed in the platform frame, so the torque it produces about the center of mass,
:math:`\boldsymbol{L} = {}^\mathcal{F}\boldsymbol{r}_{T/C} \times {}^\mathcal{F}\boldsymbol{t}` (moment arm times
force), depends on the reference rotation only through where the center of mass falls in that frame,
:math:`{}^\mathcal{F}\boldsymbol{r}_{C/M} = [\mathcal{FM}]\,{}^\mathcal{M}\boldsymbol{r}_{C/M}`. The algorithm
therefore solves for the platform-frame coordinates the center of mass must have to produce a requested torque
:math:`{}^\mathcal{F}\boldsymbol{L}_\text{req}` (zero for pure center-of-mass alignment), then takes
:math:`[\mathcal{FM}]` to be the rotation that carries :math:`{}^\mathcal{M}\boldsymbol{r}_{C/M}` there. Two
conditions pin that target position :math:`{}^\mathcal{F}\boldsymbol{r}_{Ct/M}`.

First, the requested-torque condition

.. math::
    \left( {}^\mathcal{F}\boldsymbol{r}_{T/M} - {}^\mathcal{F}\boldsymbol{r}_{Ct/M} \right)
        \times {}^\mathcal{F}\boldsymbol{t}
        = {}^\mathcal{F}\boldsymbol{L}_\text{req}

confines the target to a line parallel to the thrust (a zero torque gives the thrust line itself; a non-zero torque
shifts it sideways), whose foot perpendicular to the thrust from the joint :math:`M` is

.. math::
    {}^\mathcal{F}\boldsymbol{r}_\perp =
        \frac{ {}^\mathcal{F}\boldsymbol{t} \times \left( {}^\mathcal{F}\boldsymbol{r}_{T/M}
        \times {}^\mathcal{F}\boldsymbol{t} - {}^\mathcal{F}\boldsymbol{L}_\text{req} \right)}
        {\|{}^\mathcal{F}\boldsymbol{t}\|^2}.

Second, a rotation cannot change the center of mass's distance from the joint, so the target lies at distance
:math:`b = \|\boldsymbol{r}_{C/M}\|` from :math:`M`; it is therefore the point of the line at that distance,

.. math::
    {}^\mathcal{F}\boldsymbol{r}_{Ct/M} = {}^\mathcal{F}\boldsymbol{r}_\perp
        + \sqrt{b^2 - \|{}^\mathcal{F}\boldsymbol{r}_\perp\|^2}\, {}^\mathcal{F}\hat{\boldsymbol{t}}.

A real intersection exists whenever :math:`b \geq \|\boldsymbol{r}_\perp\|`, i.e. when the requested torque is
achievable given the available moment arm; for :math:`\boldsymbol{L}_\text{req} = 0` this reduces to the thrust line
through the center of mass and :math:`\|\boldsymbol{r}_\perp\|` is the thrust moment arm about :math:`M`. Only the
component of :math:`\boldsymbol{L}_\text{req}` perpendicular to the thrust is achievable (a force produces no torque
about its own line of action); the parallel component is dropped by the construction.

The reference rotation :math:`[\mathcal{FM}]` then carries :math:`\hat{\boldsymbol{r}}_{C/M}` onto
:math:`\hat{\boldsymbol{r}}_{Ct/M}` through the separation angle
:math:`\phi = \arccos\left(\hat{\boldsymbol{r}}_{C/M}\cdot\hat{\boldsymbol{r}}_{Ct/M}\right)` about the axis
:math:`\hat{\boldsymbol{e}} = \hat{\boldsymbol{r}}_{Ct/M}\times\hat{\boldsymbol{r}}_{C/M}`, encoded as the principal
rotation vector :math:`\phi\,\hat{\boldsymbol{e}}`. When the two directions are nearly antiparallel the cross
product is ill-defined and any axis orthogonal to :math:`\hat{\boldsymbol{r}}_{C/M}` is used for the
:math:`180^\circ` rotation. When the center of mass coincides with the joint (:math:`b \approx 0`) the pointing is
undefined and the identity rotation is returned.

Requested torque
^^^^^^^^^^^^^^^^
The requested torque :math:`\boldsymbol{L}_\text{req}` is an input to this module, supplied in body-frame
coordinates by ``cmdTorqueInMsg``, and is the torque the thruster is asked to produce on the vehicle about the
system's center of mass. The module applies no control law of its own: it converts the request into the platform
frame and hands it to the *Thruster pointing* solve above. Because that solve reaches the requested torque exactly,
the thruster produces :math:`\boldsymbol{L}_\text{req}` (up to the component along the thrust, which no thruster
force can produce, and subject to the cone limit below).

Converting the torque to the platform frame,

.. math::
    {}^\mathcal{F}\boldsymbol{L}_\text{req} = [\mathcal{FM}][\mathcal{MB}]\,
        {}^\mathcal{B}\boldsymbol{L}_\text{req},

needs :math:`[\mathcal{FM}]`, which is the very quantity being solved for. The module breaks this circularity by
reusing the **previous cycle's** reference DCM as the conversion estimate; on the first cycle, where no prior
exists, it is seeded once with the nominal (zero-torque) pointing. The resulting one-cycle staleness is a small,
bounded error absorbed by the upstream feedback loop that generates the request, and it lets each cycle run a
single pointing solve instead of two. ``reInitialize()`` discards the stored pointing, so the next cycle starts
again from the seeded estimate.

Deflection cone limit
^^^^^^^^^^^^^^^^^^^^^
The reference rotation is finally limited so that the thruster direction stays within a cone of half-angle
:math:`\theta_\text{max}` (``thetaMax``) about its neutral, un-rotated direction. Let
:math:`\hat{\boldsymbol{t}}_0 = {}^\mathcal{F}\hat{\boldsymbol{t}}` be the thrust direction at zero deflection
(:math:`[\mathcal{FM}] = [I]`) and :math:`\hat{\boldsymbol{t}}_r = [\mathcal{FM}]^T {}^\mathcal{F}\hat{\boldsymbol{t}}`
the thrust direction at the reference orientation, both in mount-frame coordinates. The deflection is

.. math::
    \delta = \arccos\left( \hat{\boldsymbol{t}}_0 \cdot \hat{\boldsymbol{t}}_r \right).

When :math:`\delta \leq \theta_\text{max}` the reference rotation is left unchanged. Otherwise the aligned thrust
direction is projected onto the cone by rotating it back toward :math:`\hat{\boldsymbol{t}}_0`, in the plane the two
directions span, until the deflection equals :math:`\theta_\text{max}`. This is applied to the reference rotation as

.. math::
    [\mathcal{FM}]' = [\mathcal{FM}]\,[C]^T, \qquad
    [C] = \text{PRV}\!\left( (\delta - \theta_\text{max})\,
        \frac{\hat{\boldsymbol{t}}_0 \times \hat{\boldsymbol{t}}_r}{\|\hat{\boldsymbol{t}}_0 \times \hat{\boldsymbol{t}}_r\|} \right),

where :math:`\text{PRV}(\cdot)` denotes the direction cosine matrix of a principal rotation vector. When the aligned
and neutral directions are nearly antiparallel the rotation plane is ill-defined and an arbitrary axis orthogonal to
:math:`\hat{\boldsymbol{t}}_0` is used. Because the projection reduces the platform rotation, a clamped reference no
longer points the thruster exactly through the center of mass, and the resulting net thruster torque is non-zero.

Body-frame outputs
^^^^^^^^^^^^^^^^^^
The body-frame outputs are resolved from the platform frame through the composite direction cosine matrix
:math:`[\mathcal{FB}] = [\mathcal{FM}][\mathcal{MB}]`, where :math:`[\mathcal{MB}]` is built from ``sigma_MB`` and
:math:`[\mathcal{FM}]` is the reference rotation derived above. The thrust unit direction in body-frame
coordinates is :math:`{}^\mathcal{B}\hat{\boldsymbol{t}} = [\mathcal{FB}]^T {}^\mathcal{F}\hat{\boldsymbol{t}}`
(written to ``bodyHeadingOutMsg`` and to ``thrusterConfigBOutMsg``), and the thrust application point relative to
the body-frame origin is :math:`{}^\mathcal{B}\boldsymbol{r}_{T/B} = {}^\mathcal{B}\boldsymbol{r}_{C/B} +
[\mathcal{FB}]^T {}^\mathcal{F}\boldsymbol{r}_{T/C}` (written to ``thrusterConfigBOutMsg`` as ``rThrust_B``), where
:math:`{}^\mathcal{F}\boldsymbol{r}_{T/C}` is the thrust application point relative to the center of mass expressed
in platform-frame coordinates. The net torque the thruster produces on the system follows from these outputs as
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
      - MRP relative rotation between body-fixed frames :math:`\mathcal{M}` and :math:`\mathcal{B}`
    * - ``r_MB_B``
      - [0, 0, 0]
      - finite
      - relative position of point :math:`M` with respect to point :math:`B`, in :math:`\mathcal{B}`-frame
        coordinates
    * - ``r_FM_F``
      - [0, 0, 0]
      - finite
      - relative position of point :math:`F` with respect to point :math:`M`, in :math:`\mathcal{F}`-frame
        coordinates
    * - ``thetaMax``
      - 0
      - :math:`(0, \pi)`
      - half-angle [rad] of the cone limiting the thrust deflection from its neutral direction (mandatory)

User Guide
----------
The module uses two-phase initialization: set the public configuration properties, connect the input messages,
then add the module to the simulation task (``reset()`` validates and builds the configuration)::

    platformReference = thrustVectoringF32.ThrustVectoring()
    platformReference.modelTag = "platformReference"
    platformReference.sigma_MB = sigma_MB
    platformReference.r_MB_B = r_MB_B
    platformReference.r_FM_F = r_FM_F
    platformReference.thetaMax = thetaMax

    platformReference.vehConfigInMsg.subscribeTo(vehConfigMsg)
    platformReference.thrusterConfigFInMsg.subscribeTo(thrConfigFMsg)
    platformReference.cmdTorqueInMsg.subscribeTo(cmdTorqueMsg)

    scSim.AddModelToTask(simTaskName, platformReference)

Module Assumptions and Limitations
----------------------------------
The requested torque is achievable only when the target center-of-mass line reaches the sphere of radius :math:`b`
about the joint :math:`M`, i.e. when :math:`b \geq \|\boldsymbol{r}_\perp\|` (see *Thruster pointing*); for the
zero-torque case this is just the requirement that the thruster line of action can reach the center of mass. When the
center of mass coincides with the joint the pointing is undefined and the identity rotation is returned. When the
solution would require deflecting the thruster beyond the configured cone half-angle :math:`\theta_\text{max}`, the
reference is clamped to the cone (see *Deflection cone limit*); in that case the thruster does not produce the
requested torque exactly.

The module assumes it is run at a small control period so that the platform moves little between successive calls,
because the requested torque is converted into the platform frame using the *previous* cycle's pointing (see
*Requested torque*). Larger inter-cycle motion does not break the pointing solve, but it degrades that
approximation; the resulting residual is bounded and absorbed by the upstream feedback loop that generates the
request.
