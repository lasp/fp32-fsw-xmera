Overview
--------
``thrFiringRemainder`` maps commanded thruster forces to thruster on-times while preserving sub-minimum pulses
through a remainder accumulator. Forces that would normally fall below the minimum on-time are collected and
re-issued once enough residual pulse time has been accumulated. Commands that saturate the control period are over-driven
by a configurable ``onTimeSaturationFactor`` (default 1.0) to guarantee thrust for the full-duration of the control
period.

For example, if the minimum on-time is 20 milli-seconds, an algorithm without a remainder accumulation
calculation would create a deadband about the 20 milli-second control request. With the remainder accumulation logic,
if 5 milli-second on-time requests are computed, these are accumulated such that every
4\ :math:`^{\text{th}}` control step a 20 milli-second burn is
requested. This reduces the deadband behavior of the thruster and
achieves better pointing/precise actuation. In this example the 5 milli-second
un-implemented on-times are accumulated in the variable
:math:`\Delta t_{\text{partial}}`.

The module layer (``ThrFiringRemainder``) implements the SysModel interface, wiring messages and managing module
lifecycle. The algorithm (``ThrFiringRemainderAlgorithm``) contains the stateful remainder accumulation logic that
can be used stand-alone when the caller provides configuration and timing.

The paper
`Steady-State Attitude and Control Effort Sensitivity Analysis of Discretized Thruster Implementations
<https://doi.org/10.2514/1.A33709>`__ details the remainder logic and compares it with alternative firing strategies.

Module Inputs and Outputs
-------------------------
Module inputs:

.. list-table:: Module Input Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - thrConfInMsg
      - :ref:`THRArrayConfigMsgF32Payload`
      - Read in ``reset()``. Contains ``numThrusters`` and per-thruster max thrust. Max thrust must be > 0; the
        algorithm clamps internally to the provided values.
    * - thrForceInMsg
      - :ref:`THRArrayCmdForceMsgF32Payload`
      - Read every ``updateState()``. Provides commanded forces :math:`F_i`. Values may be negative in OFF_PULSING
        mode; negative forces are clamped to zero after the pulsing adjustment.

Module output:

.. list-table:: Module Output Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - onTimeOutMsg
      - :ref:`THRArrayOnTimeCmdMsgF32Payload`
      - Thruster on-time requests :math:`t_i`. Values are non-negative, and may be set to
        ``onTimeSaturationFactor *`` control period when saturation is detected.

Algorithm Configuration
-----------------------
- ``thrMinFireTime`` the minimum thruster on-time (seconds). Must be :math:`\ge 0`; a negative value throws.
- ``controlPeriod`` the control period duration (seconds). Must be :math:`\ge 0`; non-positive values throw.
- ``onTimeSaturationFactor`` multiplier applied to the control period when on-time saturates (default 1.0).
  Must be :math:`\ge 1.0`; values below 1.0 throw.
- ``thrustPulsingRegime`` must be one of ``ON_PULSING`` or ``OFF_PULSING``.
  On-pulsing is assumes a thruster is nominally off and thrust is provided
  by activating the thruster. This regime is commonly used when seeking to
  achieve an attitude control torque onto the spacecraft. Off-pulsing assumes a
  thruster is nominally on, and actuation is achieved by momentarily turning a
  thruster off. This regime is commonly used when performing a delta-V maneuver,
  and the attitude control is achieved by periodically off-pulsing thrusters.
- Max thrust is taken from configuration; if the provided value is :math:`\le 0`, the algorithm still uses it but
  downstream behavior is undefined—callers must provide positive thrust capability.

Algorithm Assumptions
---------------------
- Remainders are tracked per-thruster and persist across calls until a pulse meets or exceeds the minimum on-time.
- In OFF_PULSING mode, a negative commanded force indicates a “turn off” request; the algorithm shifts the force by
  adding the thruster’s maximum thrust before applying clamps.

Initialization
--------------
Module initialization:

- ``reset(callTime)`` checks message wiring, reads ``thrConfInMsg``, and forwards the payload into the algorithm’s
  ``reset`` to initialize ``numThrusters``, per-thruster ``maxThrust``, ``pulseRemainder``, and ``prevCallTime``.

Algorithm initialization:

- ``reset(thrConfigPayload)`` sets ``numThrusters``, copies ``maxThrust[i]`` for each thruster,
  zeros ``pulseRemainder``, and zeros ``prevCallTime``.

Algorithm Description
---------------------
For thruster :math:`i` with commanded force :math:`F_i`, maximum thrust :math:`F_{\max,i}`, control period
:math:`\Delta t`, minimum on-time :math:`\Delta t_{\min}`, and stored remainder :math:`r_i` (in multiples of
:math:`\Delta t_{\min}`):

#. OFF_PULSING adjustment (:math:`F_{i}\le 0`):

   .. math:: F_i \leftarrow \begin{cases}
      F_i + F_{\max,i}, & \text{if OFF\_PULSING}\\
      F_i, & \text{if ON\_PULSING}
      \end{cases}

#. Clamp negative force:

   .. math:: F_i \leftarrow \max(F_i, 0)

#. Nominal on-time from force request:

   .. math:: t_i = \frac{F_i}{F_{\max,i}} \, \Delta t

#. Add accumulated remainder (stored as a fraction of :math:`\Delta t_{\min}`):

   .. math:: t_i \leftarrow t_i + r_i \, \Delta t_{\min}, \quad r_i \leftarrow 0

#. Minimum pulse enforcement:

   .. math:: \text{if } t_i < \Delta t_{\min} \text{ then } r_i \leftarrow \frac{t_i}{\Delta t_{\min}},\; t_i \leftarrow 0

#. Saturation:

   .. math:: \text{if } t_i \ge \Delta t \text{ then } t_i \leftarrow f_{\text{sat}} \, \Delta t

   where :math:`f_{\text{sat}}` is the ``onTimeSaturationFactor`` (default 1.0).

The output message stores :math:`t_i` in ``OnTimeRequest[i]``.

Module vs. Algorithm Usage
--------------------------
- Module (``ThrFiringRemainder``, SysModel): use when operating inside the messaging framework. Connect
  ``thrConfInMsg`` and ``thrForceInMsg``, call ``InitializeSimulation()``/``ExecuteSimulation()``, and read
  ``onTimeOutMsg``.
- Algorithm (``ThrFiringRemainderAlgorithm``): use when you have direct payloads and timing. Call
  ``reset(configPayload)``, set ``thrMinFireTime``, ``controlPeriod``, and ``thrustPulsingRegime`` via setters,
  then call ``update(forcePayload)``.
