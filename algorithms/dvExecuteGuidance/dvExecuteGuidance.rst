Executive Summary
-----------------

The ``dvExecuteGuidance`` module executes a Delta-V maneuver by monitoring the Delta-V accumulated during the
current burn and controlling the thruster on-time command. At burn start, the module latches the accumulated
Delta-V provided by the :ref:`NavTransMsgF32Payload` message. It then compares the magnitude of the Delta-V
accumulated since that point against the desired Delta-V magnitude from the :ref:`DvBurnCmdMsgF32Payload` message.
Before the commanded burn start time is reached the module holds the thrusters off; once the desired Delta-V has
been accumulated, subject to the minimum and maximum burn-time gates, the module commands the thrusters off again.

The module writes the thruster on-time command every update. While the burn is executing, it commands an on-time of
``1.1 * controlPeriod`` for each thruster entry. Before the burn starts and after the burn completes, it commands
zero on-time.

This is the FP32 port of the Xmera ``dvExecuteGuidance`` module. Inputs and outputs are single-precision (FP32); the
algorithm is single-precision throughout.

Module Architecture
-------------------

The module is split into a thin adapter (``DvExecuteGuidance``) that handles framework integration and an algorithm
class (``DvExecuteGuidanceAlgorithm``) that contains the pure burn state machine.

Adapter Layer
~~~~~~~~~~~~~

The adapter inherits from ``SysModel``. It owns the input / output message hooks, validates that the required inputs
are connected at ``reset()`` time, constructs the algorithm via the two-phase init pattern, converts the message
payloads to and from the algorithm's Eigen types, and converts the algorithm's thruster command state into a
:ref:`THRArrayOnTimeCmdMsgF32Payload` written every update. The adapter writes ``1.1 * controlPeriod`` while the burn is
executing and zero otherwise.

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
    * - ``thrCmdOutMsg``
      - :ref:`THRArrayOnTimeCmdMsgF32Payload`
      - Thruster on-time command. Each entry is set to 1.1 * controlPeriod while the burn is
        executing and to zero when the burn is not executing or is complete.
    * - ``burnExecOutMsg``
      - :ref:`DvExecutionDataMsgF32Payload`
      - Burn execution status: whether the burn is executing and whether it has completed.

Configuration
~~~~~~~~~~~~~

The configuration is set through public properties on the adapter before ``reset()`` and validated (via
``DvExecuteGuidanceConfig``) when the algorithm is constructed.

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

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

The Python usage follows the standard adapter lifecycle: set the configuration properties, subscribe inputs, call
``reset()`` once, then drive ``updateState()`` each cycle. ::

    module = dvExecuteGuidanceF32.DvExecuteGuidance()
    module.controlPeriod = 0.5
    module.minTime = 2.0
    module.maxTime = 10.0

    module.navDataInMsg.subscribeTo(nav_trans_msg)
    module.burnDataInMsg.subscribeTo(dv_burn_cmd_msg)

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

If an input message has not been connected when ``reset()`` runs, an ``std::invalid_argument`` is thrown.
Invalid configuration values cause the configuration validator to throw fsw::invalid_argument. minTime
must be non-negative and finite, maxTime must be positive, finite, and greater than minTime, and controlPeriod
must be positive and finite. If ``updateState()`` is called before ``reset()``, an ``XmeraLifecycleException`` is thrown.

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

**Thruster command.** The adapter writes a thruster on-time command every update. While the burn is executing,
every onTimeRequest entry is set to 1.1 * controlPeriod. When the burn is not executing or has completed,
every entry is set to zero.

Assumptions and Limitations
---------------------------

- The configured ``controlPeriod`` is assumed to match the actual rate at which the module is updated; a mismatch
causes ``burnTime`` to drift from real elapsed time, shifting when the minimum and maximum time gates actually fire.

- Burn-command sequencing is assumed to be handled externally, including providing the appropriate command
when the burn state is reinitialized.

- The accumulated Delta-V provided by ``navDataInMsg`` is assumed to remain continuous and consistently
referenced throughout the burn.
