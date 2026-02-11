.. raw:: latex

    {\LARGE \textbf{mrpFeedback}}

Executive Summary
-----------------
This module provides a general MRP feedback control law.  This 3-axis control can asymptotically track a general
reference attitude trajectory.  The module is setup to work with or without `N` reaction wheels with
general orientation.  If the reaction wheel states are fed into this module, then the resulting RW
gyroscopic terms are compensated for. If the wheel information is not present, then these terms are ignored.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - Control torque output message
    * - intFeedbackTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - Integral feedback control torque output message
    * - guidInMsg
      - :ref:`AttGuidMsgPayload`
      - Attitude guidance input message
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - Vehicle configuration input message
    * - rwParamsInMsg
      - :ref:`RWArrayConfigMsgPayload`
      - Reaction wheel array configuration input message
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - Reaction wheel speeds message
    * - rwAvailInMsg
      - :ref:`RWAvailabilityMsgPayload`
      - Reaction wheel availability message.

Module Parameters
-------------------------------
The following table lists all the module parameters than can be set. The parameters are optional unless indicated
(if not specified default is used).

.. list-table:: Module Parameters
    :widths: 60 30 30 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - K
      - float
      - [rad/s]
      - 0
      - Proportional gain applied to MRP errors
      - Must not be negative (checked in setter)
    * - P
      - float
      - [N m s]
      - 0
      - Rate error feedback gain
      - Must not be negative (checked in setter)
    * - Ki
      - float
      - [N m]
      - 0
      - Integral feedback gain
      - Must not be negative (checked in setter). If 0, no integral feedback is applied and the corresponding computation is skipped
    * - integralLimit
      - float
      - [N m]
      - 0
      - Limit for integral feedback term (term will be capped by integralLimit)
      - Must not be negative (checked in setter)
    * - controlLawType
      - enum ControlLawType
      - [-]
      - 0
      - Considers integral feedback term for momentum contribution if 0, ignores otherwise
      - N/A
    * - knownTorquePntB_B
      - Eigen::Vector3f
      - [N m]]
      - [0, 0, 0]
      - Known external torque in body frame components
      - None

Module Assumptions and Limitations
----------------------------------
This module assumes the main spacecraft is a rigid body.  If RW devices are installed, their wheel speeds are assumed to
be fed into this control solution.

Initialization
--------------
The module is configured by::

    module = mrpFeedback.MrpFeedback()
    module.modelTag = "mrpFeedback"
    module.K = K
    module.P = P
    module.Ki = Ki
    module.integralLimit = integral_limit
    module.controlLawType = control_law_type
    module.knownTorquePntB_B = known_torque

Detailed Module Description
---------------------------
General Function
^^^^^^^^^^^^^^^^
The ``mrpFeedback`` module creates the MRP attitude feedback control torque :math:`{\mathbf L}_{r}` developed in chapter 8
of `Analytical Mechanics of Space Systems <http://doi.org/10.2514/4.105210>`__. The output message is a body-frame
control torque vector. The required attitude guidance message contains both attitude tracking error states as well as
reference frame states. This message is read in with every update cycle. The vehicle configuration message is only read
in on reset and contains the spacecraft inertia tensor about the vehicle's center of mass location.

The MRP feedback control can compensate for Reaction Wheel (RW) gyroscopic effects as well.
This is an optional input message where the RW configuration array message contains the RW spin axis
:math:`\hat{\mathbf g}_{s,i}` information and the RW polar inertia about the spin axis :math:`I_{W_{s,i}}`.
This is only read in on reset. The RW speed message contains the RW speed :math:`\Omega_{i}` and is read in every time
step. The optional RW availability message can be used to include or not include RWs in the MRP feedback.
This allows the module to selectively turn off some RWs. The default is that all RWs are operational and are included.

Initialization
^^^^^^^^^^^^^^
Simply call the module reset function prior to using this control module.  This will reset the prior function call time
variable, and reset the attitude error integral measure. The control update period :math:`\Delta t` is evaluated
automatically.

Algorithm
^^^^^^^^^
This module employs the MRP feedback algorithm of Example (8.14) of
`Analytical Mechanics of Space Systems <http://doi.org/10.2514/4.105210>`__. This nonlinear attitude tracking control
includes an integral measure of the attitude error. Further, we seek to avoid quadratic :math:`\mathbf\omega` terms to
reduce the likelihood of control saturation during a detumbling phase. Let the new nonlinear feedback control be
expressed as

.. math::
    [G_{s}]{\mathbf u}_{s} = -{\mathbf L}_{r}

where

.. math::
    {\mathbf L}_{r} =  -K \mathbf\sigma - [P] \delta\mathbf\omega - [P][K_{I}] {\mathbf z}  - [I_{\text{RW}}](-\dot{\mathbf\omega}_{r} + [\tilde{\mathbf\omega}]\mathbf\omega_{r}) - {\mathbf L}
    \\
    + ([\tilde{\mathbf \omega}_{r}] + [\widetilde{K_{I}{\mathbf z}}])
    \left([I_{\text{RW}}]\mathbf\omega + [G_{s}]{\mathbf h}_{s} \right)

and

.. math::
    h_{s_{i}} = I_{W_{s_{i}}} (\hat{\mathbf g}_{s_{i}}^{T} \mathbf\omega_{B/N} + \Omega_{i})

with :math:`I_{W_{s}}` being the RW spin axis inertia.

The integral attitude error measure :math:`\mathbf z` is defined through

.. math::  {\mathbf z} = K \int_{t_{0}}^{t} \mathbf\sigma \text{d}t + [I_{\text{RW}}] \delta\mathbf\omega

A limit to the magnitude of the :math:`\int_{t_{0}}^{t} \mathbf\sigma \text{d}t` can be specified, which is a scalar
compared to each element of the integral term.

The integral measure :math:`\mathbf z` must be computed to determine :math:`[P][K_{I}] {\mathbf z}`, and the expression
:math:`[\widetilde{K_{I}{\mathbf z}}]` is added to :math:`[\widetilde{\mathbf\omega_{r}}]` term.

User Guide
----------
This module requires the following variables from the required input messages:

- :math:`{\mathbf\sigma}_{B/N}` as ``guidCmdData.sigma_BR``
- :math:`^B{\mathbf\omega}_{B/R}`  as ``guidCmdData.omega_BR_B``
- :math:`^B{\mathbf\omega}_{R/N}` as ``guidCmdData.omega_RN_B``
- :math:`^B\dot{\mathbf\omega}_{R/N}` as ``guidCmdData.domega_RN_B``
- :math:`[I]`, the inertia matrix of the body as ``vehicleConfigOut.ISCPntB_B``

The gains :math:`K` and :math:`P` must be set to positive values.  The integral gain :math:`K_i` is optional, it is 0
by default, in which case the error integration for the controller is disabled, leaving just PD terms. The integrator
is required to maintain asymptotic tracking in the presence of an external disturbing torque.  The ``integralLimit``
is a scalar value applied in an element-wise check to ensure that the value of each element of the
:math:`\int_{t_{0}}^{t} \mathbf\sigma \text{d}t` vector is within the desired limit. If not, the sign of that element
is persevered, but the magnitude is replaced by ``integralLimit``.

If the ``rwParamsInMsg`` is specified, then the associated ``rwSpeedsInMsg`` is required as well.

The ``rwAvailInMsg`` is optional and is used to selectively include RW devices in the control solution.

The ``controlLawType`` is an input that enables the user to choose between two different control laws.
When ``controlLawType = NORMAL``, the control law is that specified above. Otherwise, the control law takes the form:

.. math::

    {\mathbf L}_{r} =  -K \mathbf\sigma - [P] \delta\mathbf\omega - [P][K_{I}] {\mathbf z}  - [I_{\text{RW}}](-\dot{\mathbf\omega}_{r} + [\tilde{\mathbf\omega}]\mathbf\omega_{r}) - {\mathbf L}
    \\
    + [\tilde{\mathbf \omega}]
    \left([I_{\text{RW}}]\mathbf\omega + [G_{s}]{\mathbf h}_{s} \right).

This control law is also asymptotically stable. The advantage when compared to ``controlLawType = NORMAL`` is that in
this one, the integral control feedback, which may contain integration errors, only appears once. On the downside, this
control law depends quadratically on the angular rates of the spacecraft, and could cause a large control torque when
the spacecraft is tumbling at a high rate. When unspecified, this parameter defaults to ``controlLawType = NORMAL``.
