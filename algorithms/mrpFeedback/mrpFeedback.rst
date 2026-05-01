.. raw:: latex

    {\LARGE \textbf{mrpFeedback}}

Executive Summary
-----------------
This module provides a general MRP feedback control law. The 3-axis control can asymptotically track a reference
attitude trajectory. The module works with or without ``N`` reaction wheels with general orientation. If the reaction
wheel state messages are connected, the resulting RW gyroscopic terms are compensated for; otherwise those terms are
ignored.

This is the FP32 port of the Xmera ``mrpFeedback`` module.

Module Architecture
-------------------

The module is split into a thin adapter (``MrpFeedback``) that handles framework integration and an algorithm class
(``MrpFeedbackAlgorithm``) that contains the pure math. The adapter inherits from ``SysModel`` and validates that the
required input messages are connected at ``reset()`` time. It builds a ``MrpFeedbackConfig`` from the public properties
listed below and constructs the algorithm via the two-phase init pattern.

.. list-table:: Module I/O Messages
    :widths: 25 30 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Control torque output
    * - intFeedbackTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Integral feedback control torque output
    * - guidInMsg
      - :ref:`AttGuidMsgF32Payload`
      - Attitude guidance input (required)
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgF32Payload`
      - Vehicle configuration input (required)
    * - rwParamsInMsg
      - :ref:`RWArrayConfigMsgF32Payload`
      - Reaction-wheel array configuration input (optional)
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgF32Payload`
      - Reaction-wheel speeds (required when ``rwParamsInMsg`` is connected)
    * - rwAvailInMsg
      - :ref:`RWAvailabilityMsgPayload`
      - Reaction-wheel availability (optional)

Configuration
~~~~~~~~~~~~~

All configurable parameters are validated at ``reset()`` time via ``MrpFeedbackConfig::create``. A negative gain or
limit causes the factory to throw before the algorithm is constructed.

.. list-table:: Configuration parameters
    :widths: 22 14 14 14 36
    :header-rows: 1

    * - Parameter
      - Type
      - Units
      - Default
      - Description / valid range
    * - ``K``
      - float
      - [N*m]
      - 0
      - Proportional gain on the MRP error. Must be ``>= 0``.
    * - ``P``
      - float
      - [N*m*s]
      - 0
      - Rate-error feedback gain. Must be ``>= 0``.
    * - ``Ki``
      - float
      - [N*m]
      - 0
      - Integral feedback gain. Must be ``>= 0``. ``Ki = 0`` disables the integral term entirely.
    * - ``integralLimit``
      - float
      - [N*m*s]
      - 0
      - Anti-windup clamp applied element-wise to the integrated MRP error. Must be ``>= 0``.
    * - ``controlLawType``
      - ``ControlLawType``
      - --
      - ``NORMAL``
      - ``NORMAL`` includes the integral term in the gyroscopic-compensation cross product;
        ``SIMPLE_INTEGRAL`` uses :math:`\boldsymbol{\omega}_{B/N}` directly.
    * - ``knownTorquePntB_B``
      - ``Eigen::Vector3f``
      - [N*m]
      - ``[0,0,0]``
      - Feedforward known external torque in body-frame components.

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

Subscribe inputs and set the public configuration properties before ``reset()`` is invoked. ::

    module = mrpFeedbackF32.MrpFeedback()
    module.K = 0.15
    module.P = 150.0
    module.Ki = 0.01
    module.integralLimit = 20.0
    module.controlLawType = mrpFeedbackF32.ControlLawType_NORMAL
    module.knownTorquePntB_B = [0.0, 0.0, 0.0]

    module.guidInMsg.subscribeTo(guid_msg)
    module.vehConfigInMsg.subscribeTo(veh_config_msg)
    module.rwParamsInMsg.subscribeTo(rw_params_msg)   # optional
    module.rwSpeedsInMsg.subscribeTo(rw_speeds_msg)   # required when rwParamsInMsg is connected
    module.rwAvailInMsg.subscribeTo(rw_avail_msg)     # optional

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

If a required input message has not been connected when ``reset()`` runs, an ``std::invalid_argument`` is thrown.
If any of the gain or limit parameters fail validation (negative value), ``MrpFeedbackConfig::create`` throws an
``fsw::invalid_argument``. If ``updateState()`` is called before ``reset()``, an ``XmeraLifecycleException`` is thrown.

Mathematical Formulation
------------------------

The ``mrpFeedback`` module implements the MRP attitude feedback control torque :math:`\mathbf{L}_{r}` developed in
chapter 8 of `Analytical Mechanics of Space Systems <http://doi.org/10.2514/4.105210>`__ (Example 8.14). It is a
nonlinear attitude tracking law that includes an integral measure of the attitude error and avoids quadratic
:math:`\mathbf{\omega}` terms to reduce the likelihood of control saturation during a detumbling phase. The output
message is a body-frame control torque vector.

Required guidance inputs (read every cycle via ``guidInMsg``):

- :math:`\boldsymbol{\sigma}_{B/R}` (``sigma_BR``)
- :math:`{}^{B}\boldsymbol{\omega}_{B/R}` (``omega_BR_B``)
- :math:`{}^{B}\boldsymbol{\omega}_{R/N}` (``omega_RN_B``)
- :math:`{}^{B}\dot{\boldsymbol{\omega}}_{R/N}` (``domega_RN_B``)

The spacecraft inertia tensor :math:`[I]` is read once at ``reset()`` from ``vehConfigInMsg``. If ``rwParamsInMsg`` is
connected, the RW spin-axis matrix :math:`[G_s]` and per-wheel spin-axis inertia :math:`I_{W_{s,i}}` are also captured
at ``reset()`` time; the per-cycle wheel speeds :math:`\Omega_i` come from ``rwSpeedsInMsg``.

Control Law (``controlLawType = NORMAL``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. math::

    \boldsymbol{L}_{r} = -K \boldsymbol{\sigma}_{B/R}
        - P \, {}^{B}\boldsymbol{\omega}_{B/R}
        - P K_{I} \boldsymbol{z}
        - [I_{\text{RW}}]\!\left(
            -\dot{\boldsymbol{\omega}}_{R/N}
            + [\tilde{\boldsymbol{\omega}}]\boldsymbol{\omega}_{R/N}
          \right)
        - \boldsymbol{L}
        + \left([\tilde{\boldsymbol{\omega}}_{R/N}] + [\widetilde{K_{I} \boldsymbol{z}}]\right)
          \!\left([I_{\text{RW}}]\boldsymbol{\omega}_{B/N} + [G_{s}]\boldsymbol{h}_{s}\right),

where the per-wheel angular momentum is

.. math::

    h_{s_{i}} = I_{W_{s_{i}}}\!\left( \hat{\boldsymbol{g}}_{s_{i}}^{T} \boldsymbol{\omega}_{B/N} + \Omega_{i} \right),

and :math:`I_{W_{s_{i}}}` is the RW spin-axis inertia.

The integral attitude-error measure :math:`\boldsymbol{z}` is

.. math::

    \boldsymbol{z} = K \int_{t_{0}}^{t} \boldsymbol{\sigma}_{B/R}\, \mathrm{d}t
                   + [I_{\text{RW}}] \boldsymbol{\omega}_{B/R}.

The element-wise magnitude of :math:`\int \boldsymbol{\sigma}_{B/R}\, \mathrm{d}t` is clamped against
``integralLimit``, preserving sign per element. The integral term is computed and applied only when ``Ki > 0``;
setting ``Ki = 0`` skips the integral path entirely.

Control Law (``controlLawType = SIMPLE_INTEGRAL``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. math::

    \boldsymbol{L}_{r} = -K \boldsymbol{\sigma}_{B/R}
        - P \, {}^{B}\boldsymbol{\omega}_{B/R}
        - P K_{I} \boldsymbol{z}
        - [I_{\text{RW}}]\!\left(
            -\dot{\boldsymbol{\omega}}_{R/N}
            + [\tilde{\boldsymbol{\omega}}]\boldsymbol{\omega}_{R/N}
          \right)
        - \boldsymbol{L}
        + [\tilde{\boldsymbol{\omega}}_{B/N}]
          \!\left([I_{\text{RW}}]\boldsymbol{\omega}_{B/N} + [G_{s}]\boldsymbol{h}_{s}\right).

This variant is also asymptotically stable. Compared to ``NORMAL``, the integral feedback (which may contain
integration errors) appears only once; in exchange the law depends quadratically on the body angular rate, which can
generate a large control torque during a high-rate tumble.

Reset
~~~~~

At ``reset()`` the algorithm:

1. Captures :math:`[I]` from ``vehConfigInMsg``.
2. Captures the RW configuration from ``rwParamsInMsg`` if it is connected; otherwise marks the RW count as zero.
3. Zeros the integral state :math:`\int \boldsymbol{\sigma}_{B/R}\, \mathrm{d}t`.
4. Resets the prior-call timestamp; the first ``update`` after a reset uses :math:`\Delta t = 0`.

Assumptions and Limitations
---------------------------

- The spacecraft is treated as a rigid body.
- If RW devices are installed, their wheel speeds must be fed in via ``rwSpeedsInMsg``; otherwise the gyroscopic terms
  are not included in the control torque.
- The reference acceleration ``domega_RN_B`` is included verbatim in the control torque -- discontinuities in the
  reference trajectory will appear directly in the output.
- The output is single-precision FP32. With FP32 inputs and bounded magnitudes, expect relative accuracy on the
  attitude-feedback term on the order of :math:`10^{-7}`.
