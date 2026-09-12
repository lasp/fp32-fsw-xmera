Executive Summary
-----------------
This module calculates the two gimbal angles :math:`(\alpha, \beta)` that align the gimbal thrust axis with the
thrust direction from the input message. :ref:`thrustVectoring` gives that thrust direction.
:ref:`gimbalAnglesToMotorAngles` receives the two gimbal angles and calculates the stepper motor angles.

The module calculates the two angles directly. It does no iteration, keeps no state, and does not use data from
the previous cycle. The module needs only one configuration parameter, which is the orientation of the gimbal
mount frame on the hub.

All calculations use single-precision floating point (``float`` / fp32). The module has one algorithm class
(``AxisToGimbalAnglesAlgorithm``) and two adapters. The ``SysModel`` adapter connects the algorithm to the Xmera
system with messages. The C shim connects the algorithm to the Adamant system with the C/Ada FFI.

Module Architecture
-------------------
The algorithm (``AxisToGimbalAnglesAlgorithm``) has no framework dependencies and uses Eigen types. It contains
the mathematics that follows and keeps no runtime state. The algorithm does not use message payloads.
``update()`` receives the thrust direction, which is the only quantity that changes each cycle. ``update()``
returns an ``AxisToGimbalAnglesOutput`` structure. The ``AxisToGimbalAnglesConfig`` object holds the mounting
orientation, and the algorithm calculates the DCM from it one time, when the caller sets the configuration.

The Xmera adapter (``AxisToGimbalAngles``) uses ``SysModel`` as its base class and does all message operations.
The configuration parameters are public member variables (two-phase initialization). The caller sets the
parameters and then calls ``reset()``. ``reset()`` makes sure that the input message is connected, and builds
the configuration from the current parameter values. ``updateState()`` reads the thrust direction,
calls the algorithm, and writes the result to the output message. ``reconfigure()`` sends the current parameter
values to the algorithm again.

The module has no ``reInitialize()`` function. The algorithm keeps no runtime state.

The Adamant adapter is a C shim (``axisToGimbalAnglesAlgorithm_c.h`` / ``.cpp``). It makes the algorithm
available through an ``extern "C"`` interface for the C/Ada FFI bindings.

Message Connection Descriptions
-------------------------------
The table that follows gives the module input and output messages. The user sets the message variable name from
Python. The message type contains a link to the message structure definition.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - thrustDirectionInMsg
      - :ref:`BodyHeadingMsgPayload`
      - Input message with the thrust direction in body-frame coordinates
        (:math:`{}^\mathcal{B}\hat{\boldsymbol{t}}`, the field ``rHat_XB_B``). :ref:`thrustVectoring` usually
        gives this message. This is the only argument to ``update()``.
    * - twoAxisGimbalOutMsg
      - :ref:`TwoAxisGimbalMsgPayload`
      - Output message with the two gimbal angles. ``theta1`` is the angle :math:`\alpha`. ``theta2`` is the
        angle :math:`\beta`. :ref:`gimbalAnglesToMotorAngles` receives this message. The module does not write
        the step fields of the payload.

Module Parameters
-----------------
``reset()`` makes sure that the configuration parameters are correct when it builds the algorithm configuration.
An incorrect value causes an ``fsw::invalid_argument`` exception.

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
      - MRP rotation between the body-fixed frames :math:`\mathcal{M}` and :math:`\mathcal{B}`. The :math:`+z`
        axis of the :math:`\mathcal{M}` frame is the neutral gimbal thrust axis, thus this parameter gives the
        mounting orientation of the gimbal
    * - ``thetaMax``
      - 0
      - :math:`(0, \pi/2)`
      - largest deflection [rad] of the thrust axis from the neutral axis, thus the travel of the mechanism. Two
        plane angles cannot describe a deflection of :math:`90^\circ` or more, thus the travel must stay below it

An MRP with a norm of more than one gives the same rotation as its shadow set. The module changes such an MRP to
the shadow set before it stores the value.

``thetaMax`` has no usable default. The module rejects a travel of zero, thus a caller must always give one.

Mathematical Formulation
------------------------

Frames and mounting
^^^^^^^^^^^^^^^^^^^
The gimbal is on the hub-fixed mount frame :math:`\mathcal{M}`. The :math:`+z` axis of this frame is the neutral
thrust direction. A gimbal at its home position thus fires along :math:`+z_\mathcal{M}`. The parameter
``sigma_MB`` gives the mounting orientation of the gimbal on the hub.

The module first changes the input direction to mount-frame coordinates:

.. math::
    {}^\mathcal{M}\boldsymbol{t} = [\mathcal{MB}]\,{}^\mathcal{B}\hat{\boldsymbol{t}}, \qquad
    [\mathcal{MB}] = [\mathcal{MB}](\boldsymbol{\sigma}_{\mathcal{M}/\mathcal{B}}).

The module calculates :math:`[\mathcal{MB}]` one time, when the caller sets the configuration.

.. note::

    The :math:`+z` direction is a convention. The module sets it, and the caller cannot change it, because the
    sign of each angle depends on it.

    Two rotations give a frame whose :math:`-z` axis is along the thrust:

    - a rotation of :math:`180^\circ` about the mount :math:`x` axis, which changes the sign of :math:`\beta`,
    - a rotation of :math:`180^\circ` about the mount :math:`y` axis, which changes the sign of :math:`\alpha`.

    Both rotations are correct. Thus one thrust direction gives two different pairs of angles, and the equations
    below cannot show which pair the mechanism uses. One frame for all modules removes this problem.

    :ref:`thrustVectoring` uses the same :math:`\mathcal{M}` frame, thus one ``sigma_MB`` value is sufficient for
    both modules.

Gimbal kinematics
^^^^^^^^^^^^^^^^^
The gimbal has **two plane angles**. Each angle is the inclination of the thrust axis in one of the
two mount planes that contain the neutral axis:

- the angle :math:`\alpha` (``theta1``) is the inclination in the :math:`y`-:math:`z` plane,
- the angle :math:`\beta` (``theta2``) is the inclination in the :math:`x`-:math:`z` plane.

Neither angle moves the axis of the other angle. Both angles are referenced to fixed mount axes. The thrust axis
in terms of the two angles is:

.. math::
    {}^\mathcal{M}\boldsymbol{T}(\alpha, \beta) \;\propto\; \begin{bmatrix}
        \tan\beta \\ -\tan\alpha \\ 1
    \end{bmatrix}.

The two angles are thus the coordinates of the point where the thrust axis touches the plane
:math:`z_\mathcal{M} = 1`. Lines of constant :math:`\alpha` and constant :math:`\beta` make a square grid on
that plane.

.. note::

    This is **not** a sequential Euler parameterization. In a nested-ring gimbal, the second rotation moves a
    frame that the first rotation turned. Such a gimbal gives
    :math:`{}^\mathcal{M}\hat{\boldsymbol{t}} = [\sin\phi,\; -\cos\phi\sin\psi,\; \cos\phi\cos\psi]^T`, in which
    the second angle is an out-of-plane (latitude) angle.

    The two forms agree when one of the angles is zero. For all other angles they are different, because
    :math:`\tan\beta = \tan\phi / \cos\alpha`. The difference increases with each angle. At
    :math:`\alpha = 18.5^\circ` and :math:`\phi = 27^\circ`, for example, :math:`\beta` is
    :math:`28.25^\circ`, which is :math:`1.25^\circ` more than :math:`\phi`. A given mechanism has one of these
    two forms, and this module uses the plane-angle form.

Solving for the gimbal angles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The module calculates the two angles directly from this relation, with two four-quadrant arctangents:

.. math::
    \alpha = \tan^{-1}\left( \frac{-\,{}^{\mathcal{M}}t_2}{{}^{\mathcal{M}}t_3} \right), \qquad
    \beta  = \tan^{-1}\left( \frac{{}^{\mathcal{M}}t_1}{{}^{\mathcal{M}}t_3} \right).

Both angles are ratios against the mount :math:`+z` axis. This has two results. First, the length of the input
direction has no effect, because it cancels in each ratio. The module also makes the direction a unit vector
before it calculates the two ratios. This keeps the direction for a very short or a very long input. Second, the
two angles are correct only where the denominator is more than zero. This is a deflection of less than
:math:`90^\circ` from the neutral thrust axis, or :math:`{}^{\mathcal{M}}t_3 > 0`. The two angles become very
large as the deflection increases to :math:`90^\circ`. The travel limit below holds the deflection at or below
:math:`\theta_\text{max}`, which is less than :math:`90^\circ`. Both angles therefore stay inside
:math:`\theta_\text{max}`, and the mapping is correct in the two directions.

The travel limit
^^^^^^^^^^^^^^^^
The mechanism can only move the thrust axis inside a cone of half-angle :math:`\theta_\text{max}` about the
neutral axis. The module applies that limit before it calculates the two angles. A request inside the cone stays
as it is. For a request outside the cone, the module keeps the plane that the request and the neutral axis span,
and moves the direction to the edge of the cone in that plane:

.. math::
    {}^{\mathcal{M}}\hat{\boldsymbol{t}}' = \cos\theta_\text{max}\,\hat{\boldsymbol{z}}_\mathcal{M}
        + \sin\theta_\text{max}\,\hat{\boldsymbol{e}}_\perp, \qquad
    \hat{\boldsymbol{e}}_\perp = \frac{{}^{\mathcal{M}}\hat{\boldsymbol{t}}
        - \hat{\boldsymbol{z}}_\mathcal{M}\,{}^{\mathcal{M}}t_3}{\left\|\cdot\right\|}.

The limit is continuous. A request exactly on the cone gives the same direction from both conditions, because the
perpendicular part then has the length :math:`\sin\theta_\text{max}` before the division.

A request exactly opposite the neutral axis leaves no plane, because its perpendicular part has zero length. The
module takes an arbitrary perpendicular direction there. The result is still on the edge of the cone, thus the
gimbal moves as far as it can towards the request.

:math:`\theta_\text{max}` is less than :math:`90^\circ`, thus the limited direction always has
:math:`{}^{\mathcal{M}}t_3 \geq \cos\theta_\text{max} > 0`. The two angles are therefore always well conditioned,
and each one stays inside :math:`\theta_\text{max}`.

Requests that carry no direction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
A request of zero length, or one with a component that is not a number, carries no direction at all. There is
nothing to limit and nothing to point at, thus the module gives the gimbal home position
:math:`(\alpha, \beta) = (0, 0)`. The home position is the neutral thrust axis.

User Guide
----------
The module uses two-phase initialization. Do the steps that follow:

1. Set the public configuration parameters.
2. Connect the input message.
3. Add the module to the simulation task. ``reset()`` then builds the configuration.

::

    gimbalAngles = axisToGimbalAnglesF32.AxisToGimbalAngles()
    gimbalAngles.modelTag = "gimbalAngles"
    gimbalAngles.sigma_MB = sigma_MB
    gimbalAngles.thetaMax = thetaMax

    gimbalAngles.thrustDirectionInMsg.subscribeTo(thrustVectoring.bodyHeadingOutMsg)

    scSim.AddModelToTask(simTaskName, gimbalAngles)

If the mounting orientation changes during the mission, call ``reconfigure()`` to build the configuration again.

To calculate the stepper motor angles, connect the output message to :ref:`gimbalAnglesToMotorAngles`:

::

    motorAngles.twoAxisGimbalInMsg.subscribeTo(gimbalAngles.twoAxisGimbalOutMsg)

Module Assumptions and Limitations
----------------------------------
**Assumption.** The gimbal has two plane angles, as *Gimbal kinematics* shows. The module cannot
examine the mechanism, thus it cannot identify a mechanism that does not agree with this description. If the
mechanism is a nested-ring gimbal, the angles from this module are incorrect. The note in *Gimbal kinematics*
gives the size of the error.

**Limitation.** The module gives the nearest direction that the mechanism can move to, and not the request
itself. For a request outside the cone of half-angle :math:`\theta_\text{max}`, the thrust stays on the edge of
that cone. The vehicle then receives a different torque from the one that the request asks for.

**Assumption.** One cone of half-angle :math:`\theta_\text{max}` gives the travel of the mechanism. A gimbal
with a different limit on each axis, or with a limit that changes with the other angle, needs a different
description. :ref:`gimbalAnglesToMotorAngles` applies the limits of the motors themselves.
