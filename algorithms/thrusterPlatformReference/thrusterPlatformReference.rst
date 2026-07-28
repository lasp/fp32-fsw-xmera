Executive Summary
-----------------
This module computes a reference orientation for a dual-gimballed platform connected to the main hub. The platform
can only perform a tip-and-tilt type of rotation, and therefore one degree of freedom is blocked. A thruster is
mounted on the platform, whose direction is known in platform-frame coordinates. The goal of this module is to
compute a reference orientation for the platform which aligns the thruster direction with the system's center of
mass, to zero the net torque produced by the thruster on the spacecraft. Alternatively, the module can offset the
thrust direction with respect to the center of mass to produce a net torque that dumps the momentum accumulated on
the reaction wheels.

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
    * - hingedRigidBodyRef1OutMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Output message containing the reference angle (and zero angle rate) for the tip angle.
    * - hingedRigidBodyRef2OutMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Output message containing the reference angle (and zero angle rate) for the tilt angle.
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

The algorithm computes a direction cosine matrix :math:`[\mathcal{FM}]` that describes the rotation between the
platform frame :math:`\mathcal{F}` and the mount frame :math:`\mathcal{M}`. To be compliant with the constraint in
the motion of the platform, i.e. the dual gimbal, such frame must have a zero in the element (2,1). When such
condition is met, the reference angles computed from the DCM allow the thruster to align through the system's
center of mass. The input parameters allow specifying offsets between the origin :math:`M` of the hub-fixed mount
frame :math:`\mathcal{M}` and the origin :math:`F` of the platform-fixed frame :math:`\mathcal{F}`, the application
point of the thruster force in the :math:`\mathcal{F}` frame, and the direction, in :math:`\mathcal{F}`-frame
coordinates, of the thrust vector.

When the optional reaction-wheel input messages are connected the user can specify the gain :math:`\kappa` (``K``),
the proportional gain of a control law that computes an offset with respect to the center of mass; this makes the
thruster apply a torque on the system that dumps the momentum accumulated on the wheels. The control law is:

.. math::
    \boldsymbol{d} = -\frac{1}{t^2} \boldsymbol{t} \times(\kappa \boldsymbol{h}_w + \kappa_I \boldsymbol{H}_w)

where :math:`\boldsymbol{h}_w` is the momentum on the wheels and :math:`\boldsymbol{H}_w` its integral over time:

.. math::
    \boldsymbol{H}_w = \int_{t_0}^t \boldsymbol{h}_w \text{d}t.

The integral is accumulated with a trapezoidal rule using the configured ``controlPeriod`` as the fixed time step
(the module is expected to run at that rate). The inputs ``theta1Max`` and ``theta2Max`` bound the output reference
angles for the platform. If there are no
mechanical bounds, setting these inputs to a non-positive value bypasses the routine that bounds these angles.

The tip and tilt reference angles :math:`\nu_{1R}` and :math:`\nu_{2R}` are extracted from the final DCM according
to:

.. math::
    \begin{align}
        \nu_{1R} &= \arctan \left( \frac{f_{23}}{f_{22}} \right) &
        \nu_{2R} &= \arctan \left( \frac{f_{31}}{f_{11}} \right)
    \end{align}

The body-frame outputs are resolved from the platform frame through the composite direction cosine matrix
:math:`[\mathcal{FB}] = [\mathcal{FM}][\mathcal{MB}]`, where :math:`[\mathcal{MB}]` is built from ``sigma_MB`` and
:math:`[\mathcal{FM}]` is rebuilt from the (bounded) reference angles. The thrust unit direction in body-frame
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
    * - ``theta1Max``
      - 0
      - finite
      - absolute bound on the tip angle; a non-positive value disables bounding
    * - ``theta2Max``
      - 0
      - finite
      - absolute bound on the tilt angle; a non-positive value disables bounding

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
    platformReference.theta1Max = theta1Max
    platformReference.theta2Max = theta2Max

    platformReference.vehConfigInMsg.subscribeTo(vehConfigMsg)
    platformReference.thrusterConfigFInMsg.subscribeTo(thrConfigFMsg)
    # momentum dumping is enabled only when both reaction-wheel messages are connected
    platformReference.rwConfigDataInMsg.subscribeTo(rwConfigMsg)
    platformReference.rwSpeedsInMsg.subscribeTo(rwSpeedsMsg)

    scSim.AddModelToTask(simTaskName, platformReference)

Module Assumptions and Limitations
----------------------------------
As pointed out in the paper referenced above, it is not always guaranteed that a direction cosine matrix exists
that can satisfy both the pointing requirement on the thrust direction and the kinematic constraint on the
dual-gimballed platform. When a solution does not exist, a minimum problem is solved to compute the closest
constraint-incompliant DCM. The tip and tilt reference angles are then extracted from that final DCM without
checking whether :math:`[\mathcal{FM}]` is constraint compliant. As a result, the angles :math:`\nu_{1R}` and
:math:`\nu_{2R}` produce a constraint-compliant reference, which however might not align the thruster with the
desired point in the hub.
