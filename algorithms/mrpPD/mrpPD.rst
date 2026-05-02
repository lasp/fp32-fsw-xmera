.. raw:: latex

    {\LARGE \textbf{mrpPD}}

Executive Summary
-----------------
This module provides an MRP-based proportional-derivative attitude control law. It is similar to :ref:`mrpFeedback`,
but without reaction-wheel terms or integral feedback. Given perfect knowledge of the spacecraft inertia and no
unmodeled dynamics, the control law asymptotically tracks a reference attitude. The output is an external body-frame
control torque intended to be produced by a thruster cluster.

This is the FP32 port of the Xmera ``mrpPD`` module.

Module Architecture
-------------------

The module is split into a thin adapter (``MrpPD``) that handles framework integration and an algorithm class
(``MrpPDAlgorithm``) that contains the pure math. The adapter inherits from ``SysModel`` and validates that the
required input messages are connected at ``reset()`` time. It builds an ``MrpPDConfig`` from the public properties
listed below plus the spacecraft inertia read from ``vehConfigInMsg``, then constructs the algorithm via the
two-phase init pattern.

.. list-table:: Module I/O Messages
    :widths: 25 30 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Commanded control torque output
    * - guidInMsg
      - :ref:`AttGuidMsgF32Payload`
      - Attitude guidance input (required)
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgF32Payload`
      - Vehicle configuration input -- supplies the spacecraft inertia (required)

Configuration
~~~~~~~~~~~~~

User-configurable parameters are exposed as public properties on the adapter and validated at ``reset()`` time via
``MrpPDConfig::create``. The spacecraft inertia is not user-set; it is sourced from ``vehConfigInMsg`` at ``reset()``
and passes through the same validator before the algorithm is constructed.

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
      - Proportional gain on the MRP error :math:`\boldsymbol{\sigma}_{B/R}`. Must be ``>= 0``.
    * - ``P``
      - float
      - [N*m*s]
      - 0
      - Rate-error feedback gain on :math:`{}^{B}\boldsymbol{\omega}_{B/R}`. Must be ``>= 0``.
    * - ``knownTorquePntB_B``
      - ``Eigen::Vector3f``
      - [N*m]
      - ``[0,0,0]``
      - Feedforward known external torque about point :math:`B` in body-frame components. Must be finite.
    * - ``spacecraftInertia`` (from ``vehConfigInMsg``)
      - ``Eigen::Matrix3f``
      - [kg*m^2]
      - ``I``
      - Spacecraft inertia tensor about point :math:`B` in body-frame components. Must be a valid inertia tensor
        (symmetric positive-definite, with principal moments satisfying the triangle inequality).

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

Subscribe inputs and set the public configuration properties before ``reset()`` is invoked. ::

    module = mrpPDF32.MrpPD()
    module.modelTag = "mrpPD"
    module.K = 0.15
    module.P = 150.0
    module.knownTorquePntB_B = [0.0, 0.0, 0.0]

    module.guidInMsg.subscribeTo(guid_msg)
    module.vehConfigInMsg.subscribeTo(veh_config_msg)

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

If a required input message has not been connected when ``reset()`` runs, an ``std::invalid_argument`` is thrown.
If any of the configuration parameters fail validation, ``MrpPDConfig::create`` throws an ``fsw::invalid_argument``.
If ``updateState()`` is called before ``reset()``, an ``XmeraLifecycleException`` is thrown.

Mathematical Formulation
------------------------

The ``mrpPD`` module implements the proportional-derivative MRP attitude feedback control law from section 8.4.1 of
`Analytical Mechanics of Space Systems <http://doi.org/10.2514/4.105210>`__. The control torque is

.. math::

    \boldsymbol{L}_{r} = -K\, \boldsymbol{\sigma}_{B/R}
                        - P\, {}^{B}\boldsymbol{\omega}_{B/R}
                        + [I]\, {}^{B}\dot{\boldsymbol{\omega}}_{R/N}
                        - \boldsymbol{L},

where :math:`\boldsymbol{\sigma}_{B/R}` and :math:`{}^{B}\boldsymbol{\omega}_{B/R}` are the attitude and rate
tracking errors of the body frame :math:`\mathcal{B}` relative to the reference frame :math:`\mathcal{R}`,
:math:`{}^{B}\dot{\boldsymbol{\omega}}_{R/N}` is the inertial angular acceleration of the reference frame in
body-frame components, :math:`[I]` is the spacecraft inertia tensor about point :math:`B`, and :math:`\boldsymbol{L}`
is the feedforward known external torque (``knownTorquePntB_B``).

This control output is an external control torque, intended to be produced by a cluster of thrusters. No reaction
wheel terms are present and no gyroscopic terms (no :math:`\boldsymbol{\omega}\times[I]\boldsymbol{\omega}`
contribution) are added. As shown in the reference, the law can asymptotically track an arbitrary reference
trajectory generated by :math:`\mathcal{R}` provided perfect inertia knowledge and no unmodeled dynamics.

Required guidance inputs (read every cycle via ``guidInMsg``):

- :math:`\boldsymbol{\sigma}_{B/R}` (``sigma_BR``)
- :math:`{}^{B}\boldsymbol{\omega}_{B/R}` (``omega_BR_B``)
- :math:`{}^{B}\dot{\boldsymbol{\omega}}_{R/N}` (``domega_RN_B``)

Note that :math:`{}^{B}\boldsymbol{\omega}_{R/N}` is also present on ``AttGuidMsgF32Payload`` but is not consumed by
this module.

Reset
~~~~~

At ``reset()`` the adapter:

1. Verifies ``guidInMsg`` and ``vehConfigInMsg`` are linked; throws ``std::invalid_argument`` otherwise.
2. Reads :math:`[I]` from ``vehConfigInMsg``.
3. Builds an ``MrpPDConfig`` from ``K``, ``P``, ``knownTorquePntB_B``, and the captured inertia.
4. Constructs a fresh ``MrpPDAlgorithm`` with that configuration.

Each ``reset()`` therefore produces a fully revalidated configuration; mid-run changes to public properties take
effect at the next ``reset()`` call.

Assumptions and Limitations
---------------------------

- The spacecraft is treated as a rigid body and the inertia tensor :math:`[I]` does not vary with time.
- The control output does not include any reaction-wheel or gyroscopic terms.
- The reference acceleration ``domega_RN_B`` is included verbatim in the control torque -- discontinuities in the
  reference trajectory will appear directly in the output.
- The output is single-precision FP32. With FP32 inputs and bounded magnitudes, expect relative accuracy on the
  attitude- and rate-feedback terms on the order of :math:`10^{-7}`.
