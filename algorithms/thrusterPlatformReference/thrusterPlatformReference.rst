Executive Summary
-----------------
This module computes a reference orientation for a platform connected to the main hub, on which a thruster is
mounted whose direction is known in platform-frame coordinates. The goal of this module is to compute a reference
orientation for the platform which aligns the thruster line of action with the system's center of mass, to zero the
net torque produced by the thruster on the spacecraft. Alternatively, the module can offset the thrust direction
with respect to the center of mass to produce a net torque that dumps the momentum accumulated on the reaction
wheels. The module reports the resulting thruster direction in body-frame coordinates; a downstream module is
responsible for computing the platform gimbal angles that realize it.

All numeric computation is single-precision (``float`` / fp32). The module is a single algorithm
(``ThrusterPlatformReferenceAlgorithm``) with two interface adapters: a ``SysModel`` adapter that connects it to
the Xmera system via messages, and a C shim that connects it to the Adamant system via the C/Ada FFI.

Module Architecture
-------------------
The **algorithm** (``ThrusterPlatformReferenceAlgorithm``) is framework-free and Eigen-typed; it implements the
mathematics below and holds the reaction-wheel momentum integrator state. It never sees a message payload: its
inputs and outputs are the ``ThrusterPlatformReferenceInputs`` and ``ThrusterPlatformReferenceOutput`` structs. Two
interface adapters wrap it.

The **Xmera adapter** (``ThrusterPlatformReference``) inherits from ``SysModel`` and owns all messaging concerns.
Configuration parameters are exposed as public member variables (two-phase initialization): the caller sets them,
then calls ``reset()``, which validates that the required input messages are connected and builds a validated
``ThrusterPlatformReferenceConfig`` from the current property values (deriving the momentum-dumping flag from the
reaction-wheel message link state and reading the reaction-wheel configuration). ``updateState()`` reads the input
messages, converts the payload ``float[3]`` arrays to Eigen types via ``eigenSupport.h``, invokes the algorithm,
and packs the results back into the output payloads. ``reconfigure()`` re-pushes the current properties into the
running algorithm without disturbing its runtime state, and ``reInitialize()`` re-seeds that runtime state.

The **Adamant adapter** is a C shim (``thrusterPlatformReferenceAlgorithm_c.h`` / ``.cpp``) that exposes the
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
    * - rwConfigDataInMsg
      - :ref:`RWArrayConfigMsgPayload`
      - Optional input message containing the number of reaction wheels, their spin-axis inertias and orientations
        with respect to the body frame. Momentum dumping is enabled only when this message and ``rwSpeedsInMsg``
        are both connected.
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - Optional input message containing the speeds of the reaction wheels relative to the hub.
    * - bodyHeadingOutMsg
      - :ref:`BodyHeadingMsgPayload`
      - Output message containing the unit direction vector of the thruster in body-frame coordinates.
    * - thrusterTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - Output message containing the opposite of the net torque produced by the thruster on the system.
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
line of action passes through the system's center of mass. The input parameters allow specifying offsets between the
origin :math:`M` of the hub-fixed mount frame :math:`\mathcal{M}` and the origin :math:`F` of the platform-fixed
frame :math:`\mathcal{F}`, the application point of the thruster force in the :math:`\mathcal{F}` frame, and the
direction, in :math:`\mathcal{F}`-frame coordinates, of the thrust vector.

Platform reference rotation
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The reference DCM :math:`[\mathcal{FM}]` is the single rotation that aligns the thruster line of action with the
system center of mass (derived below). The relevant geometry is first assembled in the mount and platform frames:

.. math::
    {}^\mathcal{M}\boldsymbol{r}_{C/M} = [\mathcal{MB}]\,{}^\mathcal{B}\boldsymbol{r}_{C/B}
        + {}^\mathcal{M}\boldsymbol{r}_{B/M}, \qquad
    {}^\mathcal{F}\boldsymbol{r}_{T/M} = {}^\mathcal{F}\boldsymbol{r}_{F/M} + {}^\mathcal{F}\boldsymbol{r}_{T/F},
        \qquad
    {}^\mathcal{F}\boldsymbol{t} = F\,{}^\mathcal{F}\hat{\boldsymbol{t}},

with :math:`[\mathcal{MB}]` built from ``sigma_MB`` and :math:`F` the thrust magnitude.

Thrust-line alignment
^^^^^^^^^^^^^^^^^^^^^^
The thrust line of action is fixed in the platform frame as the ray
:math:`{}^\mathcal{F}\boldsymbol{r}_{T/M} + c\,{}^\mathcal{F}\hat{\boldsymbol{t}}`, with :math:`c \geq 0`. The
alignment rotation is length preserving, so the rotated center of mass lies on a sphere of radius
:math:`b = \|\boldsymbol{r}_{C/M}\|` about the joint :math:`M`. Aligning the thrust line through the center of mass
therefore reduces to intersecting the ray with that sphere,

.. math::
    \left\| {}^\mathcal{F}\boldsymbol{r}_{T/M} + c\,{}^\mathcal{F}\hat{\boldsymbol{t}} \right\|^2 = b^2,

which, since :math:`\hat{\boldsymbol{t}}` is a unit vector, is a quadratic in :math:`c`,

.. math::
    c^2 + 2\left(\boldsymbol{r}_{T/M}\cdot\hat{\boldsymbol{t}}\right) c
        + \left( \|\boldsymbol{r}_{T/M}\|^2 - b^2 \right) = 0,

whose relevant root (taken with the positive square root) is

.. math::
    c = -\boldsymbol{r}_{T/M}\cdot\hat{\boldsymbol{t}}
        + \sqrt{\left(\boldsymbol{r}_{T/M}\cdot\hat{\boldsymbol{t}}\right)^2 - \|\boldsymbol{r}_{T/M}\|^2 + b^2}.

The discriminant equals :math:`b^2 - r_\text{arm}^2`, where
:math:`r_\text{arm}^2 = \|\boldsymbol{r}_{T/M}\|^2 - (\boldsymbol{r}_{T/M}\cdot\hat{\boldsymbol{t}})^2` is the
squared moment arm of the thrust line about :math:`M`; a real intersection exists whenever
:math:`b \geq r_\text{arm}`, i.e. when the center of mass is reachable by the thrust direction. The intersection
point :math:`{}^\mathcal{F}\boldsymbol{r}_{Ct/M} = {}^\mathcal{F}\boldsymbol{r}_{T/M}
+ c\,{}^\mathcal{F}\hat{\boldsymbol{t}}` is the target position of the center of mass in the platform frame.

The reference rotation :math:`[\mathcal{FM}]` carries :math:`\hat{\boldsymbol{r}}_{C/M}` onto
:math:`\hat{\boldsymbol{r}}_{Ct/M}` through the separation angle
:math:`\phi = \arccos\left(\hat{\boldsymbol{r}}_{C/M}\cdot\hat{\boldsymbol{r}}_{Ct/M}\right)` about the axis
:math:`\hat{\boldsymbol{e}} = \hat{\boldsymbol{r}}_{Ct/M}\times\hat{\boldsymbol{r}}_{C/M}`, encoded as the principal
rotation vector :math:`\phi\,\hat{\boldsymbol{e}}`. When the two directions are nearly antiparallel the cross
product is ill-defined and any axis orthogonal to :math:`\hat{\boldsymbol{r}}_{C/M}` is used for the
:math:`180^\circ` rotation. When the center of mass coincides with the joint (:math:`b \approx 0`) the alignment is
undefined and the identity rotation is returned.

Momentum dumping
^^^^^^^^^^^^^^^^
When the optional reaction-wheel input messages are connected the user can specify the gain :math:`\kappa` (``K``),
the proportional gain of a control law that computes an offset with respect to the center of mass; this makes the
thruster apply a torque on the system that dumps the momentum accumulated on the wheels. The control law is:

.. math::
    \boldsymbol{d} = -\frac{1}{t^2} \boldsymbol{t} \times(\kappa \boldsymbol{h}_w + \kappa_I \boldsymbol{H}_w)

where :math:`\boldsymbol{h}_w` is the momentum on the wheels and :math:`\boldsymbol{H}_w` its integral over time:

.. math::
    \boldsymbol{H}_w = \int_{t_0}^t \boldsymbol{h}_w \text{d}t.

The integral is accumulated with a trapezoidal rule using the configured ``controlPeriod`` as the fixed time step
(the module is expected to run at that rate).

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
in platform-frame coordinates.

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
    * - ``r_BM_M``
      - [0, 0, 0]
      - finite
      - relative position of point :math:`B` with respect to point :math:`M`, in :math:`\mathcal{M}`-frame
        coordinates
    * - ``r_FM_F``
      - [0, 0, 0]
      - finite
      - relative position of point :math:`F` with respect to point :math:`M`, in :math:`\mathcal{F}`-frame
        coordinates
    * - ``K``
      - 0
      - :math:`\geq 0`
      - proportional gain of the momentum dumping control loop
    * - ``Ki``
      - 0
      - :math:`\geq 0`
      - integral gain of the momentum dumping control loop
    * - ``controlPeriod``
      - 0
      - :math:`> 0`
      - integration time step [s] for the momentum dumping integral (the module update rate)
    * - ``thetaMax``
      - 0
      - :math:`(0, \pi)`
      - half-angle [rad] of the cone limiting the thrust deflection from its neutral direction (mandatory)

In addition, when momentum dumping is enabled the reaction-wheel configuration read from ``rwConfigDataInMsg`` must
have a wheel count not exceeding the compile-time maximum (``RW_EFF_CNT``) and unit-length spin axes (they are
normalized on construction).

User Guide
----------
The module uses two-phase initialization: set the public configuration properties, connect the input messages,
then add the module to the simulation task (``reset()`` validates and builds the configuration)::

    platformReference = thrusterPlatformReferenceF32.ThrusterPlatformReference()
    platformReference.modelTag = "platformReference"
    platformReference.sigma_MB = sigma_MB
    platformReference.r_BM_M = r_BM_M
    platformReference.r_FM_F = r_FM_F
    platformReference.K = K
    platformReference.Ki = Ki
    platformReference.controlPeriod = controlPeriod
    platformReference.thetaMax = thetaMax

    platformReference.vehConfigInMsg.subscribeTo(vehConfigMsg)
    platformReference.thrusterConfigFInMsg.subscribeTo(thrConfigFMsg)
    # momentum dumping is enabled only when both reaction-wheel messages are connected
    platformReference.rwConfigDataInMsg.subscribeTo(rwConfigMsg)
    platformReference.rwSpeedsInMsg.subscribeTo(rwSpeedsMsg)

    scSim.AddModelToTask(simTaskName, platformReference)

Module Assumptions and Limitations
----------------------------------
The reference rotation exists only when the thruster line of action can reach the center of mass, i.e. when the
thrust moment arm about the joint :math:`M` does not exceed the distance :math:`b` from the joint to the center of
mass (the ray-sphere discriminant in *Thrust-line alignment* is non-negative). When the center of mass coincides
with the joint the reference rotation is undefined and the identity rotation is returned. When the alignment would
require deflecting the thruster beyond the configured cone half-angle :math:`\theta_\text{max}`, the reference is
clamped to the cone (see *Deflection cone limit*); in that case the thruster is not aligned through the center of
mass and the reported net torque is non-zero.
