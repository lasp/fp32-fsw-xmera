Executive Summary
-----------------

The ``dvManeuver`` module controls a Delta-V maneuver. It monitors the Delta-V that the spacecraft accumulates
during the current burn, and it writes a body force command. At burn start, the module latches the accumulated
Delta-V from the :ref:`NavTransMsgF32Payload` message. It then compares the magnitude of the Delta-V that accumulates
after that point with the magnitude of the configured ``cmdDv_N``. The burn starts at the configured
``burnStartTime``. The minimum and maximum burn-time gates also control when the burn completes.

The module writes the body force command at each update. While the burn executes, the command is equal to the
configured ``cmdForce_B``. Before the burn starts and after the burn completes, the command is zero. A downstream
module, for example :ref:`forceTorqueThrForceMapping`, converts the force command into thruster commands.

The floating-point inputs and outputs are single precision. All times are integer nanoseconds.

Module Architecture
-------------------

The module is split into a thin adapter (``DvManeuver``) that handles framework integration and an algorithm
class (``DvManeuverAlgorithm``) that contains the pure burn state machine.

Adapter Layer
~~~~~~~~~~~~~

The adapter inherits from ``SysModel``. It owns the input / output message hooks, validates that the required input
is connected at ``reset()`` time, constructs the algorithm via the two-phase init pattern, converts the message
payloads to and from the algorithm's Eigen types, and writes the algorithm's force command to a
:ref:`CmdForceBodyMsgF32Payload` at each update. The adapter also sets the ``burnExecuting`` and ``burnComplete`` flags
of the :ref:`DvExecutionDataMsgF32Payload` from the burn state.

.. list-table:: Module I/O Messages
    :widths: 25 30 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ``navDataInMsg``
      - :ref:`NavTransMsgF32Payload`
      - Navigation message providing the total accumulated Delta-V of the spacecraft.
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
      - :math:`\ge 0`
      - [ns] Minimum burn time that must elapse before the burn can complete on the Delta-V criterion.
    * - ``maxTime``
      - > 0 and > minTime
      - [ns] Maximum burn time. The burn completes when the burn time reaches maxTime.
    * - ``cmdForce_B``
      - finite
      - [N] Body force that the module commands while the burn executes. A zero force is permitted.
    * - ``cmdDv_N``
      - finite
      - [m/s] Commanded Delta-V in inertial frame components. The module compares only its magnitude with the
        accumulated Delta-V. A zero Delta-V is permitted.
    * - ``burnStartTime``
      - any
      - [ns] The burn starts on the first update at or after this time.

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

The Python usage follows the standard adapter lifecycle: set the configuration properties, subscribe the input, call
``reset()`` once, then drive ``updateState()`` each cycle. ::

    module = dvManeuverF32.DvManeuver()
    module.minTime = macros.sec2nano(2.0)
    module.maxTime = macros.sec2nano(10.0)
    module.cmdForce_B = [0.0, 0.0, 10.0]
    module.cmdDv_N = [0.0, 0.0, 5.0]
    module.burnStartTime = macros.sec2nano(1.0)

    module.navDataInMsg.subscribeTo(nav_trans_msg)

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

If ``navDataInMsg`` has not been connected when ``reset()`` runs, an ``std::invalid_argument`` is thrown.
Invalid configuration values cause the configuration validator to throw fsw::invalid_argument. maxTime
must be positive and greater than minTime, and cmdForce_B and cmdDv_N must be finite. If ``updateState()`` is
called before ``reset()``, an ``XmeraLifecycleException`` is thrown.

Mathematical Formulation
------------------------

Algorithm Layer
~~~~~~~~~~~~~~~

The algorithm is a burn state machine advanced one step per ``update()`` call. Let :math:`t` be the current call
time, :math:`t_{\text{start}}` the configured burn start time,
:math:`\boldsymbol{v}_{\text{accum}}` the accumulated Delta-V from navigation, and
:math:`\Delta\boldsymbol{v}_{\text{cmd}}` the configured ``cmdDv_N``.

The state machine has three states: pending, executing, and complete. The burn starts in the pending state. Only
``reInitialize()`` moves the burn out of the complete state.

**Burn start.** The burn moves from pending to executing on the first call at or after the start time. At that
instant the module latches the accumulated Delta-V as the burn's initial value :math:`\boldsymbol{v}_{\text{init}}`,
and the call time as :math:`t_0`:

.. math::

   \text{if } t \ge t_{\text{start}}: \quad \boldsymbol{v}_{\text{init}} \leftarrow \boldsymbol{v}_{\text{accum}},
   \quad t_0 \leftarrow t.

**Burn time.** While the burn is executing, the burn time is the call time since burn start:

.. math::

   t_{\text{burn}} = t - t_0.

All times are integer nanoseconds, so the burn time and the time gates have no rounding error.

**Completion.** While the burn is executing, the Delta-V accumulated since burn start is
:math:`\Delta\boldsymbol{v}_{\text{burn}} = \boldsymbol{v}_{\text{accum}} - \boldsymbol{v}_{\text{init}}`. The burn is
complete when the accumulated magnitude reaches the command and the burn time reaches the minimum time, or when the
burn time reaches the maximum time:

.. math::

   \text{complete} =
   \Big( \| \Delta\boldsymbol{v}_{\text{burn}} \| \ge \| \Delta\boldsymbol{v}_{\text{cmd}} \|
   \;\wedge\; t_{\text{burn}} \ge t_{\min} \Big)
   \;\vee\;
   \big( t_{\text{burn}} \ge t_{\max} \big).

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

- Burn sequencing is assumed to be handled externally. Before the burn state is reinitialized for a new burn, the
operator is assumed to set ``cmdDv_N`` and ``burnStartTime`` for that burn and to call ``reconfigure()``.

- The accumulated Delta-V provided by ``navDataInMsg`` is assumed to remain continuous and consistently
referenced throughout the burn.

- The call time is assumed to increase from one update to the next. If it decreases during a burn, the burn time
decreases too. If it drops below the call time at burn start, the burn completes at that update.

- The attitude guidance is assumed to align ``cmdForce_B`` with the commanded Delta-V direction during the burn.
The module does not compare the two directions.
