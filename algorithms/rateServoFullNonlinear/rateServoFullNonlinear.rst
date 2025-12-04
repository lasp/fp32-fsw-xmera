.. raw:: latex

    {\LARGE \textbf{rateServoFullNonlinear}}

Executive Summary
-----------------

This module implements a nonlinear rate servo control that uses the attitude steering message from the
:ref:`mrpSteering` module and determines the control torque vector on the spacecraft.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg connection is set by the
user from python. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 30 30 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - commanded torque output message
    * - guidInMsg
      - :ref:`AttGuidMsgPayload`
      - attitude guidance input message
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - vehicle configuration input message
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - (optional) RW speed input message
    * - rwAvailInMsg
      - :ref:`RWAvailabilityMsgPayload`
      - (optional) RW availability input message
    * - rwParamsInMsg
      - :ref:`RWArrayConfigMsgPayload`
      - (optional) RW configuration parameter input message
    * - rateSteeringInMsg
      - :ref:`RateCmdMsgPayload`
      - commanded rate input message

Module Parameters
-------------------------------
The following table lists all the module parameters than can be set. The parameters are optional unless indicated
(if not specified default is used).

.. list-table:: Module Parameters
    :widths: 60 30 30 30
    :header-rows: 1

    * - Parameter Name
      - Default
      - Description
      - Bounds
    * - P
      - 0
      - Rate error feedback gain
      - Must not be negative (checked in setter)
    * - Ki
      - 0
      - Integral feedback gain
      - Must not be negative (checked in setter). If 0, no integral feedback is applied and the corresponding computation is skipped
    * - integralLimit
      - 0
      - Limit for integral feedback term (term will be capped by integralLimit)
      - Must not be negative (checked in setter)
    * - knownTorquePntB_B
      - [0, 0, 0]
      - Known external torque in body frame components
      - None

Module Assumptions and Limitations
----------------------------------
This control assumes the spacecraft is rigid and that the control gains of the rate servo are chosen such that the decay
time of the rate servo response is much faster than that of :ref:`mrpSteering`.

Initialization
--------------
The module is configured by::

    module = rateServoFullNonlinear.RateServoFullNonlinear()
    module.modelTag = "rateServoFullNonlinear"
    module.P = P
    module.Ki = Ki
    module.integralLimit = integral_limit
    module.knownTorquePntB_B = known_torque

Detailed Module Description
---------------------------

Overview
^^^^^^^^
This module computes a commanded control torque vector :math:`\mathbf L_r`
using a rate based steering law that drives a body frame
:math:`{\mathcal B} :\{ \hat{\mathbf B}_{1}, \hat{\mathbf B}_{2}, \hat{\mathbf B}_{3}\}`
towards a time varying reference frame
:math:`{\mathcal R} :\{ \hat{\mathbf R}_{1}, \hat{\mathbf R}_{2}, \hat{\mathbf R}_{3}\}`,
based on a desired reference frame
:math:`{\mathcal B\ast} :\{ \hat{\mathbf B}_{1}, \hat{\mathbf B}_{2}, \hat{\mathbf B}_{3}\}`
(the desired body frame from the kinematic steering law).

The output message is a body-frame
control torque vector. The required attitude guidance message
contains both attitude tracking error rates as well as reference frame
rates. This message is read in with every update cycle. The vehicle
configuration message is only read in on reset and contains the
spacecraft inertia tensor about the vehicle’s center of mass location.
The commanded body rates are read in from the steering module output
message.

The reaction wheel (RW) configuration message is optional. If the
message name is specified, then the RW message is read in,
otherwise the inertia of the wheels is not considered. If the
optional RW availability message is present, then the control will only
use the RWs that are marked available.

The servo rate feedback control can compensate for Reaction Wheel (RW)
gyroscopic effects as well. This is an optional input message where the
RW configuration array message contains the RW spin axis
:math:`\hat{g}_{s,i}` information and the RW polar inertia about the
spin axis IWs,i . This is only read in on reset. The RW speed message
contains the RW speed :math:`\Omega_i` and is read in every time step.
The optional RW availability message can be used to include or not
include RWs in the MRP feedback. This allows the module to selectively
turn off some RWs. The default is that all RWs are operational and are
included.

Steering Law Goals
^^^^^^^^^^^^^^^^^^
This technical note develops a rate based steering law that drives a
body frame
:math:`{\mathcal B} :\{ \hat{\mathbf B}_{1}, \hat{\mathbf B}_{2}, \hat{\mathbf B}_{3}\}`
towards a time varying reference frame
:math:`{\mathcal R} :\{ \hat{\mathbf R}_{1}, \hat{\mathbf R}_{2}, \hat{\mathbf R}_{3}\}`.
The inertial frame is given by
:math:`{\mathcal N} :\{ \hat{\mathbf N}_{1}, \hat{\mathbf N}_{2}, \hat{\mathbf N}_{3}\}`.
The RW coordinate frame is given by
:math:`\mathcal{W_{i}}:\{ \hat{\mathbf g}_{s_{i}}, \hat{\mathbf g}_{t_{i}}, \hat{\mathbf g}_{g_{i}} \}`.
Using MRPs, the overall control goal is

.. math::

       \mathbf\sigma_{\mathcal{B}/\mathcal{R}} \rightarrow 0

The reference frame orientation
:math:`\mathbf \sigma_{\mathcal{R}/\mathcal{N}}`, angular velocity
:math:`\mathbf\omega_{\mathcal{R}/\mathcal{N}}` and inertial angular
acceleration :math:`\dot{\mathbf \omega}_{\mathcal{R}/\mathcal{N}}` are
assumed to be known.

Rotational Dynamics
^^^^^^^^^^^^^^^^^^^
The rotational equations of motion of a rigid spacecraft with :math:`N`
Reaction Wheels (RWs) attached are given by

.. math::

       [I_{RW}] \dot{\mathbf \omega} = - [\tilde{\mathbf \omega}] \left(
       [I_{RW}] \mathbf\omega + [G_{s}] \mathbf h_{s}
       \right) - [G_{s}] \mathbf u_{s} + \mathbf L

where the inertia tensor :math:`[I_{RW}]` is defined as

.. math::

       [I_{RW}] = [I_{s}] + \sum_{i=1}^{N} \left (J_{t_{i}} \hat{\mathbf g}_{t_{i}} \hat{\mathbf g}_{t_{i}}^{T} + J_{g_{i}} \hat{\mathbf g}_{g_{i}} \hat{\mathbf g}_{g_{i}}^{T}
       \right)

The spacecraft inertia without the :math:`N` RWs is :math:`[I_{s}]`,
while :math:`J_{s_{i}}`, :math:`J_{t_{i}}` and :math:`J_{g_{i}}` are the
RW inertias about the body fixed RW axis :math:`\hat{\mathbf g}_{s_{i}}` (RW
spin axis), :math:`\hat{\mathbf g}_{t_{i}}` and :math:`\hat{\mathbf g}_{g_{i}}`.
The :math:`3\times N` projection matrix :math:`[G_{s}]` is then defined
as

.. math::

        [G_{s}] =
        \begin{bmatrix}
            \cdots {\hat{\mathbf g}}_{s_{i}} \cdots
        \end{bmatrix}

The RW inertial angular momentum vector :math:`\mathbf h_{s}` is defined as

.. math::

       h_{s_{i}} = J_{s_{i}} (\omega_{s_{i}} + \Omega_{i})

Here :math:`\Omega_{i}` is the :math:`i^{\text{th}}` RW spin relative to
the spacecraft, and the body angular velocity is written in terms of
body and RW frame components as

.. math::

       \mathbf\omega = \omega_{1} \hat{\mathbf b}_{1} + \omega_{2} \hat{\mathbf b}_{2} + \omega_{3} \hat{\mathbf b}_{3}
       = \omega_{s_{i}} \hat{\mathbf g}_{s_{i}} +  \omega_{t_{i}} \hat{\mathbf g}_{t_{i}} +  \omega_{g_{i}} \hat{\mathbf g}_{g_{i}}

Angular Velocity Servo Sub-System
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To implement the kinematic steering control, a servo sub-system is
included which will produce the required torques to make the actual body
rates track the desired body rates. The angular velocity tracking error
vector is defined as

.. math::

       \delta \mathbf \omega = \mathbf\omega_{\mathcal{B}/\mathcal{B}^{\ast}} = \mathbf\omega_{\mathcal{B}/\mathcal{N}} - \mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{N}}

where the :math:`\mathcal{B}^{\ast}` frame is the desired body frame
from the kinematic steering law. Note that

.. math::

    \mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{N}} =  \mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{R}} +  \mathbf\omega_{\mathcal{R}/\mathcal{N}}

where :math:`\mathbf\omega_{\mathcal{R}/\mathcal{N}}` is obtained from the
attitude navigation solution, and
:math:`\mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{R}}` is the kinematic
steering rate command. To create a rate-servo system that is robust to
unmodeld torque biases, the state :math:`\mathbf z` is defined as:

.. math::

       \mathbf z = \int_{t_{0}}^{t_{f}} { \delta\mathbf\omega} \textrm{d}t

Let :math:`[P]^{T} = [P]` be a symmetric positive definite rate
feedback gain matrix. The servo rate feedback control is defined as

.. math::

   \begin{gathered}
       [G_{s}]\mathbf u_{s} = [P]\delta\mathbf\omega + [K_{I}]\mathbf z - [\tilde{\mathbf\omega}_{\mathcal{B}^{\ast}/\mathcal{N}}]
       \left( [I_{\text{RW}}] \mathbf\omega_{\mathcal{B}/\mathcal{N}} + [G_{s}] \mathbf h_{s} \right)
       \\
       - [I_{\text{RW}}](\mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{R}} ' +  \dot{\mathbf\omega}_{\mathcal{R}/\mathcal{N}} -  {\mathbf\omega}_{\mathcal{B}/\mathcal{N}} \times  \mathbf\omega_{\mathcal{R}/\mathcal{N}}) + \mathbf L
   \end{gathered}

Defining the right-hand-side as :math:`- \mathbf L_{r}`, this is rewritten in
compact form as

.. math:: [G_{s}]\mathbf u_{s} = -\mathbf L_{r}

The :ref:`rateServoFullNonlinear` module writes the control torque :math:`\mathbf L_{r}` to the :ref:`CmdTorqueBodyMsgPayload`
output message.

Downstream, the :ref:`rwMotorTorque` module can then be used to compute the array of RW motor torques by solving the
typical minimum norm inverse:

.. math:: \mathbf u_{s} = [G_{s}]^{T}\left( [G_{s}][G_{s}]^{T}\right)^{-1} (- \mathbf L_{r})

Additional Information
----------------------
The value of ``integralLimit``, used to limit the degree of integrator
windup and reduce the chance of controller saturation. The integrator
is required to maintain asymptotic tracking in the presence of an
external disturbing torque. For this module, the integration limit is
applied to each element of the integrated error vector :math:`z`, and
any elements greater than the limit are set to the limit instead.
