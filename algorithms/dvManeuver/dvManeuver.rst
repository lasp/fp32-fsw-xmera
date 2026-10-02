Executive Summary
-----------------

The ``dvManeuver`` module controls a Delta-V maneuver. It monitors the Delta-V that the spacecraft accumulates
during the current burn, and it writes a body force command. At burn start, the module latches the accumulated
Delta-V from the :ref:`NavTransMsgF32Payload` message. It then compares the magnitude of the Delta-V that accumulates
after that point with the desired Delta-V magnitude from the :ref:`DvBurnCmdMsgF32Payload` message. The minimum and
maximum burn-time gates also control when the burn completes.

The module writes the body force command at each update. While the burn executes, the command is equal to the
configured ``cmdForce_B``. Before the burn starts and after the burn completes, the command is zero. A downstream
module, for example :ref:`forceTorqueThrForceMapping`, converts the force command into thruster commands.

This is the FP32 port of the Xmera ``dvExecuteGuidance`` module. Inputs and outputs are single-precision (FP32); the
algorithm is single-precision throughout.

Module Architecture
-------------------

The module is split into a thin adapter (``DvManeuver``) that handles framework integration and an algorithm
class (``DvManeuverAlgorithm``) that contains the pure burn state machine.

Adapter Layer
~~~~~~~~~~~~~

The adapter inherits from ``SysModel``. It owns the input / output message hooks, validates that the required inputs
are connected at ``reset()`` time, constructs the algorithm via the two-phase init pattern, converts the message
payloads to and from the algorithm's Eigen types, and writes the algorithm's force command to a
:ref:`CmdForceBodyMsgF32Payload` at each update.

.. list-table:: Module I/O Messages
    :widths: 25 30 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ``navDataInMsg``
      - :ref:`NavTransMsgF32Payload`
      - Navigation message providing the total accumulated Delta-V of the spacecraft.
    * - ``burnDataInMsg``
      - :ref:`DvBurnCmdMsgF32Payload`
      - Commanded burn: the inertial Delta-V vector and the burn start time.
    * - ``cmdForceOutMsg``
      - :ref:`CmdForceBodyMsgF32Payload`
      - Body force command. It is equal to ``cmdForce_B`` while the burn executes. It is zero before the burn
        starts and after the burn completes.
    * - ``burnExecOutMsg``
      - :ref:`DvExecutionDataMsgF32Payload`
      - Burn execution status: whether the burn is executing and whether it has completed.

Configuration
~~~~~~~~~~~~~

The configuration is set through public properties on the adapter before ``reset()`` and validated (via
``DvManeuverConfig``) when the algorithm is constructed.

.. list-table:: Configuration parameters
    :widths: 25 25 50
    :header-rows: 1

    * - Parameter
      - Valid range
      - Description
    * - ``minTime``
      - :math:`\ge 0`, finite
      - [s] Minimum burn time that must elapse before the burn may complete on the Delta-V criterion.
    * - ``maxTime``
      - > 0, finite, and > minTime
      - [s] Maximum burn time. The burn is forced complete once burnTime exceeds maxTime.
    * - ``controlPeriod``
      - :math:`> 0`, finite
      - [s] Flight-software control period, used as the fixed time step for accumulating the burn time. Must be set
        to a positive value before ``reset()``.
    * - ``cmdForce_B``
      - finite
      - [N] Body force that the module commands while the burn executes. A zero force is permitted.

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

The Python usage follows the standard adapter lifecycle: set the configuration properties, subscribe inputs, call
``reset()`` once, then drive ``updateState()`` each cycle. ::

    module = dvManeuverF32.DvManeuver()
    module.controlPeriod = 0.5
    module.minTime = 2.0
    module.maxTime = 10.0
    module.cmdForce_B = [0.0, 0.0, 10.0]

    module.navDataInMsg.subscribeTo(nav_trans_msg)
    module.burnDataInMsg.subscribeTo(dv_burn_cmd_msg)

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

If an input message has not been connected when ``reset()`` runs, an ``std::invalid_argument`` is thrown.
Invalid configuration values cause the configuration validator to throw fsw::invalid_argument. minTime
must be non-negative and finite, maxTime must be positive, finite, and greater than minTime, controlPeriod
must be positive and finite, and cmdForce_B must be finite. If ``updateState()`` is called before ``reset()``, an
``XmeraLifecycleException`` is thrown.

Mathematical Formulation
------------------------

Algorithm Layer
~~~~~~~~~~~~~~~

The algorithm is a burn state machine advanced one step per ``update()`` call. Let :math:`t` be the current call
time, :math:`t_{\text{start}}` the commanded burn start time, :math:`\Delta t` the configured control period,
:math:`\boldsymbol{v}_{\text{accum}}` the accumulated Delta-V from navigation, and
:math:`\Delta\boldsymbol{v}_{\text{cmd}}` the commanded Delta-V.

**Burn start.** The burn begins on the first call at or after the start time, provided it is not already executing and
has not completed. At that instant the accumulated Delta-V is latched as the burn's initial value
:math:`\boldsymbol{v}_{\text{init}}`:

.. math::

   \text{if } t \ge t_{\text{start}}: \quad \boldsymbol{v}_{\text{init}} \leftarrow \boldsymbol{v}_{\text{accum}}.

**Burn time.** While the burn is executing, the elapsed burn time accumulates by the fixed control period each step:

.. math::

   t_{\text{burn}} \leftarrow t_{\text{burn}} + \Delta t.

**Completion.** The Delta-V accumulated since burn start is
:math:`\Delta\boldsymbol{v}_{\text{burn}} = \boldsymbol{v}_{\text{accum}} - \boldsymbol{v}_{\text{init}}`. The burn is
complete when the accumulated magnitude reaches the command and the minimum time has elapsed, or when the maximum time
is exceeded:

.. math::

   \text{complete} =
   \Big( \| \Delta\boldsymbol{v}_{\text{burn}} \| \ge \| \Delta\boldsymbol{v}_{\text{cmd}} \|
   \;\wedge\; t_{\text{burn}} > t_{\min} \Big)
   \;\vee\;
   \big( t_{\text{burn}} > t_{\max} \big).

**Force command.** The algorithm returns the body force command :math:`\boldsymbol{F}_{\text{cmd}}` at each update.
Let :math:`\boldsymbol{F}_{\text{cfg}}` be the configured ``cmdForce_B``:

.. math::

   \boldsymbol{F}_{\text{cmd}} =
   \begin{cases}
   \boldsymbol{F}_{\text{cfg}} & \text{if the burn executes,} \\
   \boldsymbol{0} & \text{before the burn starts and after the burn completes.}
   \end{cases}

Assumptions and Limitations
---------------------------

- The configured ``controlPeriod`` is assumed to match the actual rate at which the module is updated; a mismatch
causes ``burnTime`` to drift from real elapsed time, shifting when the minimum and maximum time gates actually fire.

- Burn-command sequencing is assumed to be handled externally, including providing the appropriate command
when the burn state is reinitialized.

- The accumulated Delta-V provided by ``navDataInMsg`` is assumed to remain continuous and consistently
referenced throughout the burn.

- The attitude guidance is assumed to align ``cmdForce_B`` with the commanded Delta-V direction during the burn.
The module does not compare the two directions.
