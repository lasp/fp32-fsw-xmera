.. raw:: latex

    {\LARGE \textbf{thrFiringSchmitt}}

Executive Summary
-----------------

A Schmitt trigger logic is implemented to map a desired thruster force value into a thruster on command time. All
computation is single-precision (``float``).

The module reads in the attitude control thruster force values for both on- and off-pulsing scenarios, and then maps
this into a time which specifies how long a thruster should be on. The thruster configuration data is read in through a
separate input message in the reset method. The Schmitt trigger allows for an upper and lower bound where the thruster
is either turned on or off. Commands that saturate the control period are over-driven by a configurable
``onTimeSaturationFactor`` (default 1.0) to guarantee thrust for the full duration of the control period.
The paper `Steady-State Attitude and Control Effort Sensitivity Analysis of Discretized
Thruster Implementations <https://doi.org/10.2514/1.A33709>`__ includes a detailed discussion on the Schmitt Trigger
and compares it to other thruster firing methods.

The module layer (``ThrFiringSchmitt``) implements the SysModel interface and manages the module lifecycle through a
two-phase initialization. The algorithm (``ThrFiringSchmittAlgorithm``) contains the stateful Schmitt-trigger logic and
can be used stand-alone given a validated ``ThrFiringSchmittConfig``.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 35 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - thrConfInMsg
      - :ref:`THRArrayConfigMsgF32Payload`
      - Read in ``reset()``. Contains ``numThrusters`` and per-thruster max thrust.
    * - thrForceInMsg
      - :ref:`THRArrayCmdForceMsgF32Payload`
      - Read every ``updateState()``. Provides commanded forces :math:`F_i`. Values may be negative in
        ``OFF_PULSING`` mode; negative forces are clamped to zero after the pulsing adjustment.
    * - onTimeOutMsg
      - :ref:`THRArrayOnTimeCmdMsgF32Payload`
      - Thruster on-time requests :math:`t_i`. Values are non-negative, and may be set to
        ``onTimeSaturationFactor *`` control period when saturation is detected.

Module Architecture
-------------------
The adapter (``ThrFiringSchmitt``, a ``SysModel``) uses a two-phase initialization. The configuration knobs are public
properties set before ``reset()``: ``levelOn``, ``levelOff``, ``thrMinFireTime``, ``controlPeriod``,
``onTimeSaturationFactor``, and ``thrustPulsingRegime``. ``reset(callTime)`` validates that ``thrConfInMsg`` and
``thrForceInMsg`` are connected (an unconnected input throws), reads the thruster configuration message into a
``ThrFiringSchmittThrusterArray`` (per-thruster ``maxThrust``), builds a validated ``ThrFiringSchmittConfig``, and
constructs the owned algorithm instance — so the per-thruster ON/OFF history starts cleared on every reset. Calling
``updateState()`` before ``reset()`` throws an ``XmeraLifecycleException``.

The validated ``ThrFiringSchmittConfig`` is immutable and is constructed via
``ThrFiringSchmittConfig::create(thrusterArray, controlParameters)``, which validates every field (see the bounds in
the table below) and throws ``fsw::invalid_argument`` on any violation. The algorithm's
``reInitialize()`` clears the per-thruster ON/OFF history without rebuilding the configuration (used by the C shim's
reset entry point).

Module Parameters
-------------------------------
The following table lists all the module parameters than can be set. The parameters are optional unless indicated
(if not specified default is used). Bounds are enforced by ``ThrFiringSchmittConfig::create`` at ``reset()``.

.. list-table:: Module Parameters
    :widths: 30 30 10 10 30 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - levelOn, levelOff (required)
      - float, float
      - [-]
      - 0, 0
      - ON and OFF duty cycle fractions
      - 0.0 < levelOn :math:`\le` 1.0, 0.0 :math:`\le` levelOff < 1.0, levelOn :math:`\ge` levelOff
    * - thrMinFireTime (required)
      - float
      - [s]
      - 0
      - Minimum ON time for thrusters
      - Must be greater than zero
    * - thrustPulsingRegime
      - enum ThrustPulsingRegime
      - [-]
      - ON_PULSING (0)
      - Indicates on-pulsing (``ON_PULSING``) or off-pulsing (``OFF_PULSING``)
      - N/A
    * - controlPeriod (required)
      - float
      - [s]
      - 0
      - Control period (time between two update calls, i.e. 1/fsw_rate)
      - Must be greater than zero
    * - onTimeSaturationFactor
      - float
      - [-]
      - 1.0
      - Factor applied to control period when on-time saturates
      - Must be >= 1.0

Module Assumptions and Limitations
----------------------------------

- The incoming forces :math:`F_{i}` can be both positive or negative, depending if an on- or off-pulsing mode is being
  implemented. The particular mode is set through ``thrustPulsingRegime``.
- It is assumed that ``thrMinFireTime`` is less than the control period.
- Max thrust is taken from configuration; must be :math:`\ge 0` per thruster (negative values throw).

Initialization
--------------
The module is configured by::

    module = thrFiringSchmittF32.ThrFiringSchmitt()
    module.modelTag = "thrFiringSchmitt"
    module.levelOn = 0.75
    module.levelOff = 0.25
    module.thrMinFireTime = 0.02
    module.thrustPulsingRegime = thrFiringSchmittF32.ThrustPulsingRegime_ON_PULSING
    module.controlPeriod = 0.5
    module.onTimeSaturationFactor = 1.1

Detailed Module Description
---------------------------

This module implements a Schmitt trigger thruster firing logic. The minimum desired on-time
:math:`\Delta t_{\text{min}}` is specified by the user. If the commanded on-time exceeds this minimum, the thruster
fires for the computed duration. If the on-time falls below the minimum, the Schmitt trigger hysteresis logic
determines whether the thruster should fire for the minimum duration or not fire at all.

For thruster :math:`i` with commanded force :math:`F_i`, maximum thrust :math:`F_{\max,i}`, control period
:math:`\Delta t`, and minimum on-time :math:`\Delta t_{\min}`:

#. ``OFF_PULSING`` adjustment (:math:`F_{i}\le 0`):

   .. math:: F_i = \begin{cases}
      F_i + F_{\max,i}, & \text{if OFF\_PULSING}\\
      F_i, & \text{if ON\_PULSING}
      \end{cases}

#. Clamp negative force:

   .. math:: F_i = \max(F_i, 0)

#. Nominal on-time from force request:

   .. math:: t_i = \frac{F_i}{F_{\max,i}} \, \Delta t

#. Schmitt trigger logic (if :math:`t_i < \Delta t_{\min}`):

   Compute the duty cycle :math:`l = t_i / \Delta t_{\min}`.

   - If :math:`l \ge l_{\text{on}}`: turn thruster on, :math:`t_i = \Delta t_{\min}`
   - If :math:`l \le l_{\text{off}}`: turn thruster off, :math:`t_i = 0`
   - Otherwise: maintain previous state (on :math:`\rightarrow \Delta t_{\min}`, off :math:`\rightarrow 0`)

#. Normal range (if :math:`\Delta t_{\min} \le t_i < \Delta t`):

   :math:`t_i` is used as-is. Thruster state is set to on.

#. Saturation (if :math:`t_i \ge \Delta t`):

   .. math:: t_i = f_{\text{sat}} \, \Delta t

   where :math:`f_{\text{sat}}` is the ``onTimeSaturationFactor`` (default 1.0).

The output stores :math:`t_i` in ``onTimeRequest[i]``.
