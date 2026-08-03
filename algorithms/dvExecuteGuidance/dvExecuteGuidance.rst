Executive Summary
-----------------

The dvExecuteGuidance module executes a Delta-V maneuver by monitoring the accumulated Delta-V and turning the
thrusters off once the commanded Delta-V has been achieved. It compares the magnitude of the accumulated Delta-V from
the :ref:`NavTransMsgF32Payload` message against the desired Delta-V magnitude from the :ref:`DvBurnCmdMsgF32Payload`
message. Before the commanded burn start time is reached the module holds the thrusters off; once the desired Delta-V
has been accumulated (subject to the minimum/maximum burn-time gates) the module commands the thrusters off again.

The module assumes the thrusters are turned on by another module (such as :ref:`thrFiringRemainder`) and only ever
commands them off, by writing a zeroed :ref:`THRArrayOnTimeCmdMsgF32Payload` output. A minimum and a maximum burn time
can be configured: the thrusters are turned off once the desired Delta-V has been accumulated and the burn time is
greater than the minimum time, unless the burn time first exceeds the maximum time (in which case the thrusters are
turned off regardless of the accumulated Delta-V). A maximum time of zero disables the maximum-time criterion, so the
burn is stopped using the accumulated-Delta-V criterion alone.

If the same set of Delta-V thrusters is also used for attitude control (with :ref:`thrForceMapping`,
:ref:`thrFiringRemainder`, or :ref:`thrFiringSchmitt`), those modules turn the thrusters on at the beginning of the
burn and dvExecuteGuidance turns them off at the end. To ensure the :ref:`THRArrayOnTimeCmdMsgF32Payload` output turns
the thrusters off, this module should be updated more frequently than, and with a lower task priority than, the
firing modules.

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
payloads to and from the algorithm's Eigen types, and writes the zeroed thruster on-time command when the algorithm
requests it.

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
      - Thruster on-time command; written as an all-zero payload to turn the thrusters off.
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
      - :math:`\ge 0`, finite
      - [s] Maximum burn time; the burn is forced complete once it is exceeded. A value of ``0`` disables this
        criterion.
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

If an input message has not been connected when ``reset()`` runs, an ``std::invalid_argument`` is thrown. If
``controlPeriod`` is not positive when ``reset()`` runs, the configuration validator throws. If ``updateState()`` is
called before ``reset()``, an ``XmeraLifecycleException`` is thrown.

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

   \text{complete} = \Big( \| \Delta\boldsymbol{v}_{\text{burn}} \| \ge \| \Delta\boldsymbol{v}_{\text{cmd}} \|
   \;\wedge\; t_{\text{burn}} > t_{\min} \Big)
   \;\vee\; \big( t_{\max} \ne 0 \;\wedge\; t_{\text{burn}} > t_{\max} \big).

**Thruster command.** The module writes a zeroed thruster on-time command whenever the burn is complete or not
executing, i.e. whenever the thrusters should be held off.

Assumptions and Limitations
---------------------------

- The thrusters are turned on by a separate firing module; dvExecuteGuidance only ever commands them off.
- The burn time is accumulated using the fixed configured ``controlPeriod`` rather than the wall-clock time between
  calls, so the module must be driven at that period for the burn-time gates to be accurate.
- ``maxTime`` of zero disables the maximum-time criterion. Configuring ``minTime`` greater than a nonzero ``maxTime``
  is contradictory and is not currently rejected; the maximum-time criterion wins in that case.
- All computation is single-precision (FP32).
